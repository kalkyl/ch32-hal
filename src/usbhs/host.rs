//! USBHS host mode: one device on the root port, no hubs.
//!
//! The controller runs one transaction at a time. A transaction is started by writing its
//! token to `UH_EP_PID`; the interrupt on `TRANSFER` records the handshake (`INT_ST`) and
//! the received length, stops the token and wakes the pipe. Pipes take turns per
//! transaction, under one lock, so a pipe that keeps getting NAKs (an interrupt endpoint
//! with nothing to say) doesn't hold up the others between its retries.
//!
//! The packet data goes through two DMA buffers ([`Buffers`]) that the transaction copies
//! to and from while it holds the lock. Data toggles are kept per pipe in software.
//!
//! Register sequence after WCH's EVT host code (`ch32v30x_usbhs_host.c`): the bus reset
//! runs at high speed so the device can chirp, then the speed the port settled on is
//! written back to `USB_CTRL` and SOFs go out, as soon as the reset ends: the SIE sends no
//! SOF until then, and a device that sees none for 3 ms suspends.
//!
//! Needs the host traits of embassy-usb-driver from embassy main (the `usbhs-host`
//! feature).
//!
//! Not supported: isochronous pipes, split transactions (hubs), suspend / resume.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use core::task::Poll;

use ch32_metapac::usbhs::regs::{IntSt, UhEpPid};
use ch32_metapac::usbhs::vals::{SpeedType, Tog};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_driver::host::{
    pipe, AddressSet, DeviceEvent, HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator,
    UsbHostController, UsbPipe,
};
use embassy_usb_driver::{EndpointInfo, EndpointType, Speed};

use super::{DmPin, DpPin, Instance};
use crate::gpio::{AFType, Speed as PinSpeed};
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, Peri};

/// The largest packet a pipe can move (a high-speed bulk packet).
pub const MAX_PACKET: usize = 512;

/// USB 2.0 §7.1.7.5: at least 50 ms for a root port.
const ROOT_RESET_MS: u64 = 50;
/// USB 2.0 §9.2.6.2
const RESET_RECOVERY_MS: u64 = 10;
/// An attach has to hold this long before the port is reset.
const DEBOUNCE_MS: u64 = 100;
/// After the reset, the port has to read attached this many 1 ms samples in a row (WCH: 7).
const ATTACH_STABLE_SAMPLES: u32 = 7;
const ATTACH_TIMEOUT_MS: u64 = 500;
/// A transaction whose interrupt hasn't come by then is stopped. A full-speed transaction
/// can wait up to a frame for room on the bus.
const TRANSACTION_TIMEOUT: Duration = Duration::from_millis(5);
const NO_RESPONSE_RETRIES: u8 = 3;
/// IN packets with the previous toggle (the device missed our ACK and sent it again)
const TOGGLE_RETRIES: u8 = 3;
/// Polls of MIS_ST for the SIE to go free after a token is stopped
const SIE_WAIT: u32 = 2000;

// token and handshake PIDs (USB 2.0 table 8-1)
const PID_OUT: u8 = 0x1;
const PID_IN: u8 = 0x9;
const PID_SETUP: u8 = 0xD;
const PID_ACK: u8 = 0x2;
const PID_NYET: u8 = 0x6;
const PID_NAK: u8 = 0xA;
const PID_STALL: u8 = 0xE;
const PID_DATA0: u8 = 0x3;
const PID_DATA1: u8 = 0xB;

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
/// A connect / disconnect was flagged since the last time the bus task looked
static DETECTED: AtomicBool = AtomicBool::new(false);

static XFER_WAKER: AtomicWaker = AtomicWaker::new();
/// A token is on the bus and its interrupt hasn't been handled
static IN_FLIGHT: AtomicBool = AtomicBool::new(false);
/// The interrupt recorded the transaction's result
static DONE: AtomicBool = AtomicBool::new(false);
static RESULT_ST: AtomicU8 = AtomicU8::new(0);
static RESULT_LEN: AtomicU16 = AtomicU16::new(0);
static OVERFLOW: AtomicBool = AtomicBool::new(false);

/// Bumped by every bus reset and detach: pipes allocated before it fail with
/// `Disconnected`. Written from tasks only.
static GENERATION: AtomicU8 = AtomicU8::new(0);
/// One transaction at a time
static LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

/// the DMA buffers (the DMA address registers read back only their low 16 bits)
static RX_BUF: AtomicU32 = AtomicU32::new(0);
static TX_BUF: AtomicU32 = AtomicU32::new(0);

/// The host's packet buffers, one per direction.
#[repr(C, align(4))]
pub struct Buffers {
    rx: [u8; MAX_PACKET],
    tx: [u8; MAX_PACKET],
}

impl Buffers {
    pub const fn new() -> Self {
        Self {
            rx: [0; MAX_PACKET],
            tx: [0; MAX_PACKET],
        }
    }
}

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let h = T::hregs();
        let flag = r.int_fg().read();

        if flag.detect() {
            r.int_fg().write(|w| w.set_detect(true));
            DETECTED.store(true, Ordering::Release);
            BUS_WAKER.wake();
        }
        if flag.fifo_ov() {
            r.int_fg().write(|w| w.set_fifo_ov(true));
            OVERFLOW.store(true, Ordering::Release);
        }
        if flag.transfer() {
            let st = r.int_st().read();
            let len = r.rx_len().read();
            // stop the token before the release: with INT_BUSY the SIE waits for the flag
            h.ep_pid().write_value(UhEpPid::default());
            r.int_fg().write(|w| w.set_transfer(true));
            if IN_FLIGHT.load(Ordering::Relaxed) {
                IN_FLIGHT.store(false, Ordering::Relaxed);
                RESULT_ST.store(st.0, Ordering::Relaxed);
                RESULT_LEN.store(len, Ordering::Relaxed);
                DONE.store(true, Ordering::Release);
                XFER_WAKER.wake();
            }
        }
    }
}

fn attached<T: Instance>() -> bool {
    T::regs().mis_st().read().dev_attach()
}

fn bump_generation() {
    GENERATION.store(GENERATION.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
    XFER_WAKER.wake();
}

pub struct Driver<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    connected: bool,
}

impl<'d, T: Instance> Driver<'d, T> {
    pub fn new(
        _peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl DpPin<T, 0>>,
        dm: Peri<'d, impl DmPin<T, 0>>,
        buffers: &'d mut Buffers,
    ) -> Self {
        dp.set_as_af_output(AFType::OutputPushPull, PinSpeed::High);
        dm.set_as_af_output(AFType::OutputPushPull, PinSpeed::High);

        T::enable_and_reset();

        let r = T::regs();
        let h = T::hregs();

        r.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });
        embassy_time::block_for(Duration::from_micros(10));
        r.ctrl().modify(|w| w.set_reset_sie(false));

        // high speed until the reset tells otherwise: the device only chirps for a
        // high-speed host
        r.ctrl().write(|w| {
            w.set_host_mode(true);
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_speed_type(SpeedType::HIGHSPEED);
        });
        h.ctrl().write(|w| {
            w.set_phy_suspendm(true);
            w.set_sof_en(true);
        });
        h.config().write(|w| {
            w.set_h_tx_en(true);
            w.set_h_rx_en(true);
        });
        h.ep_type().write_value(Default::default());
        h.rx_max_len().write(|w| w.set_len(MAX_PACKET as u16));

        let rx = buffers.rx.as_mut_ptr() as u32;
        let tx = buffers.tx.as_mut_ptr() as u32;
        h.rx_dma().write_value(rx);
        h.tx_dma().write_value(tx);
        RX_BUF.store(rx, Ordering::Relaxed);
        TX_BUF.store(tx, Ordering::Relaxed);

        IN_FLIGHT.store(false, Ordering::Relaxed);
        DETECTED.store(false, Ordering::Relaxed);
        bump_generation();

        r.int_fg().write(|w| {
            w.set_detect(true);
            w.set_transfer(true);
            w.set_suspend(true);
            w.set_hst_sof(true);
            w.set_fifo_ov(true);
            w.set_setup_act(true);
        });
        r.int_en().write(|w| {
            w.set_detect(true);
            w.set_transfer(true);
            w.set_fifo_ov(true);
        });
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self {
            _phantom: PhantomData,
            connected: false,
        }
    }
}

impl<'d, T: Instance> UsbHostController<'d> for Driver<'d, T> {
    type Allocator = Allocator<'d, T>;

    fn allocator(&self) -> Self::Allocator {
        Allocator { _phantom: PhantomData }
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        if self.connected {
            // a flag while attached: it went and came back before we looked
            poll_fn(|cx| {
                BUS_WAKER.register(cx.waker());
                if DETECTED.load(Ordering::Acquire) || !attached::<T>() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
            self.connected = false;
            bump_generation();
            trace!("usbhs host: detached");
            return DeviceEvent::Disconnected;
        }

        loop {
            DETECTED.store(false, Ordering::Relaxed);
            poll_fn(|cx| {
                BUS_WAKER.register(cx.waker());
                if attached::<T>() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
            Timer::after_millis(DEBOUNCE_MS).await;
            if attached::<T>() && !DETECTED.load(Ordering::Acquire) {
                break;
            }
        }

        self.bus_reset().await;
        self.connected = true;
        let speed = match T::regs().speed_type().read().speed_type() {
            SpeedType::HIGHSPEED => Speed::High,
            SpeedType::LOWSPEED => Speed::Low,
            _ => Speed::Full,
        };
        trace!("usbhs host: attached, {}", speed);
        DeviceEvent::Connected(speed)
    }

    async fn bus_reset(&mut self) {
        let r = T::regs();
        let h = T::hregs();

        r.dev_ad().write(|w| w.set_addr(0));
        r.ctrl().modify(|w| w.set_speed_type(SpeedType::HIGHSPEED));
        h.ctrl().modify(|w| w.set_tx_bus_reset(true));
        Timer::after_millis(ROOT_RESET_MS).await;
        h.ctrl().modify(|w| w.set_tx_bus_reset(false));

        // the speed the chirp (or its absence) settled on during the reset, for the SIE's own
        // timing. at once: the SIE sends no SOF until then, and a device left 3 ms without one
        // suspends (an RP2040 then NAKs its control endpoint for good)
        let speed = r.speed_type().read().speed_type();
        r.ctrl().modify(|w| w.set_speed_type(speed));
        h.ctrl().modify(|w| w.set_sof_en(true));

        let deadline = Instant::now() + Duration::from_millis(ATTACH_TIMEOUT_MS);
        let mut stable = 0;
        while stable < ATTACH_STABLE_SAMPLES && Instant::now() < deadline {
            Timer::after_millis(1).await;
            stable = if attached::<T>() { stable + 1 } else { 0 };
        }
        // the reset's own line change; a real detach still shows in DEV_ATTACH
        DETECTED.store(false, Ordering::Relaxed);

        Timer::after_millis(RESET_RECOVERY_MS).await;
        bump_generation();
    }
}

#[derive(Clone, Copy)]
pub struct Allocator<'d, T: Instance> {
    _phantom: PhantomData<&'d T>,
}

impl<'d, T: Instance> UsbHostAllocator<'d> for Allocator<'d, T> {
    type Pipe<E: pipe::Type, D: pipe::Direction> = Pipe<'d, T, E, D>;

    fn alloc_pipe<E: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<E, D>, HostError> {
        if split.is_some() {
            return Err(HostError::Other("usbhs host: no split transactions"));
        }
        if E::ep_type() == EndpointType::Isochronous {
            return Err(HostError::Other("usbhs host: no isochronous pipes yet"));
        }
        if endpoint.max_packet_size == 0 || endpoint.max_packet_size as usize > MAX_PACKET {
            return Err(HostError::InvalidDescriptor);
        }
        Ok(Pipe {
            _phantom: PhantomData,
            target: Target {
                addr,
                ep: endpoint.addr.index() as u8,
                max_packet_size: endpoint.max_packet_size,
                generation: GENERATION.load(Ordering::Relaxed),
            },
            interval_ms: endpoint.interval_ms,
            tog: false,
            timeout: TimeoutConfig::default(),
        })
    }

    fn device_removed(&self, _addrs: AddressSet) {
        // one device: whichever it was, its pipes go
        bump_generation();
    }
}

/// Where a pipe's transactions go.
#[derive(Clone, Copy)]
struct Target {
    addr: u8,
    ep: u8,
    max_packet_size: u16,
    generation: u8,
}

/// How a transaction ended.
enum Outcome {
    /// IN: the bytes received (copied out); OUT / SETUP: sent
    Ack(usize),
    Nak,
    Stall,
    NoResponse,
    /// IN: the device sent the previous packet again
    ToggleMismatch,
}

/// Stops a token that is still on the bus when the transaction is dropped or timed out.
struct InFlight<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> InFlight<T> {
    fn start(pid: u8, ep: u8) -> Self {
        let r = T::regs();
        let h = T::hregs();
        critical_section::with(|_| {
            DONE.store(false, Ordering::Relaxed);
            OVERFLOW.store(false, Ordering::Relaxed);
            IN_FLIGHT.store(true, Ordering::Relaxed);
            // WCH's order: the token, then the flag release
            h.ep_pid().write(|w| {
                w.set_token(pid);
                w.set_endp(ep);
            });
            r.int_fg().write(|w| w.set_transfer(true));
        });
        Self { _phantom: PhantomData }
    }
}

impl<T: Instance> Drop for InFlight<T> {
    fn drop(&mut self) {
        let r = T::regs();
        let h = T::hregs();
        critical_section::with(|_| {
            if !IN_FLIGHT.load(Ordering::Relaxed) {
                return;
            }
            IN_FLIGHT.store(false, Ordering::Relaxed);
            h.ep_pid().write_value(UhEpPid::default());
            let mut n = 0;
            while !r.mis_st().read().sie_free() && n < SIE_WAIT {
                n += 1;
            }
            r.int_fg().write(|w| w.set_transfer(true));
        });
    }
}

/// One transaction: `pid` with toggle `tog`, sending `out` (OUT / SETUP) or receiving
/// into `rx` (IN).
async fn transaction<T: Instance>(
    t: Target,
    pid: u8,
    tog: bool,
    out: &[u8],
    rx: &mut [u8],
) -> Result<Outcome, PipeError> {
    let _lock = LOCK.lock().await;
    let h = T::hregs();

    if GENERATION.load(Ordering::Relaxed) != t.generation || !attached::<T>() {
        return Err(PipeError::Disconnected);
    }

    if !out.is_empty() {
        // SAFETY: the TX buffer is MAX_PACKET long and the DMA is idle (the lock)
        unsafe {
            core::ptr::copy_nonoverlapping(out.as_ptr(), TX_BUF.load(Ordering::Relaxed) as *mut u8, out.len());
        }
    }
    let tog = if tog { Tog::DATA1 } else { Tog::DATA0 };
    h.dev_ad().write(|w| w.set_addr(t.addr));
    h.rx_max_len().write(|w| w.set_len(t.max_packet_size));
    h.tx_len().write(|w| w.set_len(out.len() as u16));
    h.tx_ctrl().write(|w| w.set_t_tog(tog));
    h.rx_ctrl().write(|w| w.set_r_tog(tog));
    compiler_fence(Ordering::SeqCst);

    let in_flight = InFlight::<T>::start(pid, t.ep);
    let done = embassy_time::with_timeout(
        TRANSACTION_TIMEOUT,
        poll_fn(|cx| {
            XFER_WAKER.register(cx.waker());
            if DONE.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }),
    )
    .await;
    drop(in_flight);
    compiler_fence(Ordering::SeqCst);

    if done.is_err() {
        trace!("usbhs host: transaction {:x} timed out", pid);
        return Ok(Outcome::NoResponse);
    }
    let st = IntSt(RESULT_ST.load(Ordering::Relaxed));
    let len = RESULT_LEN.load(Ordering::Relaxed) as usize;
    let res = st.h_res();

    if pid == PID_IN {
        if st.tog_ok() {
            if OVERFLOW.load(Ordering::Acquire) || len > t.max_packet_size as usize {
                return Err(PipeError::Babble);
            }
            if len > rx.len() {
                return Err(PipeError::BufferOverflow);
            }
            // SAFETY: the RX buffer holds `len` <= MAX_PACKET bytes from the DMA, which
            // is idle (the lock)
            unsafe {
                core::ptr::copy_nonoverlapping(RX_BUF.load(Ordering::Relaxed) as *const u8, rx.as_mut_ptr(), len);
            }
            return Ok(Outcome::Ack(len));
        }
        return match res {
            PID_NAK => Ok(Outcome::Nak),
            PID_STALL => Ok(Outcome::Stall),
            PID_DATA0 | PID_DATA1 => Ok(Outcome::ToggleMismatch),
            0 => Ok(Outcome::NoResponse),
            _ => Err(PipeError::BadResponse),
        };
    }

    match res {
        PID_ACK | PID_NYET => Ok(Outcome::Ack(out.len())),
        PID_NAK => Ok(Outcome::Nak),
        PID_STALL => Ok(Outcome::Stall),
        0 => Ok(Outcome::NoResponse),
        _ => Err(PipeError::BadResponse),
    }
}

/// What a NAK means for the transfer
#[derive(Clone, Copy)]
enum NakRetry {
    /// control: retry until then
    Until(Instant),
    /// bulk: retry until the future is dropped
    Forever,
    /// interrupt: poll again after the endpoint's interval
    Interval(u8),
}

/// Give the other tasks (and pipes) a turn.
async fn yield_now() {
    let mut yielded = false;
    poll_fn(|cx| {
        if yielded {
            Poll::Ready(())
        } else {
            yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
    .await
}

fn embassy_duration(d: core::time::Duration) -> Duration {
    Duration::from_micros(d.as_micros() as u64)
}

pub struct Pipe<'d, T: Instance, E, D> {
    _phantom: PhantomData<(&'d T, fn() -> (E, D))>,
    target: Target,
    interval_ms: u8,
    /// the next bulk / interrupt packet's toggle (DATA1: true)
    tog: bool,
    timeout: TimeoutConfig,
}

impl<'d, T: Instance, E: pipe::Type, D: pipe::Direction> Pipe<'d, T, E, D> {
    /// One packet, retried on NAK as `nak` says and on a missed handshake.
    async fn packet(&self, pid: u8, tog: bool, out: &[u8], rx: &mut [u8], nak: NakRetry) -> Result<usize, PipeError> {
        let mut no_response = 0;
        let mut repeated = 0;
        loop {
            match transaction::<T>(self.target, pid, tog, out, rx).await? {
                Outcome::Ack(n) => return Ok(n),
                Outcome::Stall => return Err(PipeError::Stall),
                Outcome::NoResponse => {
                    no_response += 1;
                    if no_response >= NO_RESPONSE_RETRIES {
                        return Err(PipeError::Timeout);
                    }
                }
                Outcome::ToggleMismatch => {
                    repeated += 1;
                    if repeated >= TOGGLE_RETRIES {
                        return Err(PipeError::DataToggleError);
                    }
                }
                Outcome::Nak => match nak {
                    NakRetry::Until(deadline) => {
                        if Instant::now() >= deadline {
                            return Err(PipeError::Timeout);
                        }
                        yield_now().await;
                    }
                    NakRetry::Forever => yield_now().await,
                    NakRetry::Interval(ms) => Timer::after_millis(ms.max(1) as u64).await,
                },
            }
        }
    }

    fn nak_retry(&self) -> NakRetry {
        match E::ep_type() {
            EndpointType::Interrupt => NakRetry::Interval(self.interval_ms),
            _ => NakRetry::Forever,
        }
    }

    fn control_deadline(&self, has_data: bool) -> NakRetry {
        let t = if has_data {
            self.timeout.data_timeout
        } else {
            self.timeout.no_data_timeout
        };
        NakRetry::Until(Instant::now() + embassy_duration(t))
    }
}

impl<'d, T: Instance, E: pipe::Type, D: pipe::Direction> UsbPipe<E, D> for Pipe<'d, T, E, D> {
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        E: pipe::IsControl,
        D: pipe::IsIn,
    {
        let len = (u16::from_le_bytes([setup[6], setup[7]]) as usize).min(buf.len());
        let nak = self.control_deadline(len > 0);
        let mps = self.target.max_packet_size as usize;

        self.packet(PID_SETUP, false, setup, &mut [], nak).await?;
        let mut n = 0;
        let mut tog = true;
        while n < len {
            let got = self.packet(PID_IN, tog, &[], &mut buf[n..len], nak).await?;
            tog = !tog;
            n += got;
            if got < mps {
                break;
            }
        }
        // status: the opposite direction, DATA1
        if len > 0 {
            self.packet(PID_OUT, true, &[], &mut [], nak).await?;
        } else {
            self.packet(PID_IN, true, &[], &mut [], nak).await?;
        }
        Ok(n)
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        E: pipe::IsControl,
        D: pipe::IsOut,
    {
        let len = (u16::from_le_bytes([setup[6], setup[7]]) as usize).min(buf.len());
        let nak = self.control_deadline(len > 0);
        let mps = self.target.max_packet_size as usize;

        self.packet(PID_SETUP, false, setup, &mut [], nak).await?;
        let mut tog = true;
        for chunk in buf[..len].chunks(mps) {
            self.packet(PID_OUT, tog, chunk, &mut [], nak).await?;
            tog = !tog;
        }
        self.packet(PID_IN, true, &[], &mut [], nak).await?;
        Ok(())
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        let nak = self.nak_retry();
        let mps = self.target.max_packet_size as usize;
        let mut n = 0;
        loop {
            let got = self.packet(PID_IN, self.tog, &[], &mut buf[n..], nak).await?;
            self.tog = !self.tog;
            n += got;
            // an interrupt pipe answers with the one packet it polled
            if E::ep_type() == EndpointType::Interrupt || got < mps || n == buf.len() {
                return Ok(n);
            }
        }
    }

    async fn request_out(&mut self, buf: &[u8], ensure_transaction_end: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        let nak = self.nak_retry();
        let mps = self.target.max_packet_size as usize;
        for chunk in buf.chunks(mps) {
            self.packet(PID_OUT, self.tog, chunk, &mut [], nak).await?;
            self.tog = !self.tog;
        }
        if buf.is_empty() || (ensure_transaction_end && buf.len() % mps == 0) {
            self.packet(PID_OUT, self.tog, &[], &mut [], nak).await?;
            self.tog = !self.tog;
        }
        Ok(())
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig)
    where
        E: pipe::IsControl,
    {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self)
    where
        E: pipe::IsBulkOrInterrupt,
    {
        self.tog = false;
    }
}
