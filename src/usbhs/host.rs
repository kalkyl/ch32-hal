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
//! One isochronous IN pipe runs without a task in the loop: once its first `request_in`
//! arms it, the interrupt starts its IN on every SOF of its period (ahead of any waiting
//! transaction: those start only while no IN is due) and the DMA writes the packet straight
//! into a ring in [`Buffers`]. `request_in` drains the ring. [`iso_in_stats`] counts the
//! packets, the periods whose IN never went out (missed), the packets dropped on a full
//! ring and the INs without a valid answer.
//!
//! Register sequence after WCH's EVT host code (`ch32v30x_usbhs_host.c`): the bus reset
//! runs at high speed so the device can chirp, then the speed the port settled on is
//! written back to `USB_CTRL` and SOFs go out, as soon as the reset ends: the SIE sends no
//! SOF until then, and a device that sees none for 3 ms suspends.
//!
//! Needs the host traits of embassy-usb-driver from embassy main (the `usbhs-host`
//! feature).
//!
//! Not supported: isochronous OUT, high-bandwidth isochronous (more than one transaction per
//! microframe), split transactions (hubs), suspend / resume.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use core::task::Poll;

use ch32_metapac::usbhs::regs::{IntSt, UhEpPid, UhEpType};
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

/// The largest packet a bulk / interrupt / control pipe can move (a high-speed bulk packet).
pub const MAX_PACKET: usize = 512;
/// The largest isochronous packet (one transaction per microframe)
pub const MAX_ISO_PACKET: usize = 1024;

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
/// Slots in the isochronous ring at most (a power of two: the indices are wrapping u8s)
const ISO_MAX_SLOTS: usize = 64;
/// Ahead of each slot's data: its length (u16) and the frame number it arrived in (u16)
const ISO_HEADER: usize = 4;

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
const PID_DATA2: u8 = 0x7;
const PID_MDATA: u8 = 0xF;

/// What the SIE is doing (`ENGINE`)
const IDLE: u8 = 0;
/// a task's transaction
const ASYNC: u8 = 1;
/// the isochronous IN
const ISO: u8 = 2;

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
/// A connect / disconnect was flagged since the last time the bus task looked
static DETECTED: AtomicBool = AtomicBool::new(false);

/// Who has the SIE. Changed in the interrupt and in critical sections.
static ENGINE: AtomicU8 = AtomicU8::new(IDLE);
/// A task waiting for the SIE to go idle
static ENGINE_WAKER: AtomicWaker = AtomicWaker::new();

static XFER_WAKER: AtomicWaker = AtomicWaker::new();
/// The interrupt recorded the task's transaction's result
static DONE: AtomicBool = AtomicBool::new(false);
static RESULT_ST: AtomicU8 = AtomicU8::new(0);
static RESULT_LEN: AtomicU16 = AtomicU16::new(0);
static OVERFLOW: AtomicBool = AtomicBool::new(false);

/// Bumped by every bus reset and detach: pipes allocated before it fail with
/// `Disconnected`. Written from tasks only.
static GENERATION: AtomicU8 = AtomicU8::new(0);
/// One task transaction at a time
static LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

/// the DMA buffers (the DMA address registers read back only their low 16 bits)
static RX_BUF: AtomicU32 = AtomicU32::new(0);
static TX_BUF: AtomicU32 = AtomicU32::new(0);

// the isochronous IN pipe. slot i at ISO_RING + (i % slots) * stride; [tail, head) hold
// packets, and the slot at head is always free: the DMA's target
static ISO_RING: AtomicU32 = AtomicU32::new(0);
static ISO_RING_LEN: AtomicU32 = AtomicU32::new(0);
static ISO_ALLOCATED: AtomicBool = AtomicBool::new(false);
/// An IN every period
static ISO_ACTIVE: AtomicBool = AtomicBool::new(false);
/// This period's IN hasn't gone out
static ISO_DUE: AtomicBool = AtomicBool::new(false);
/// The IN on the bus lands in the head slot but the ring is full: it's dropped
static ISO_DROP: AtomicBool = AtomicBool::new(false);
static ISO_ADDR: AtomicU8 = AtomicU8::new(0);
static ISO_EP: AtomicU8 = AtomicU8::new(0);
static ISO_MPS: AtomicU16 = AtomicU16::new(0);
/// SOFs per IN, and the SOFs left until the next
static ISO_PERIOD: AtomicU8 = AtomicU8::new(1);
static ISO_COUNTDOWN: AtomicU8 = AtomicU8::new(1);
static ISO_STRIDE: AtomicU16 = AtomicU16::new(0);
static ISO_SLOTS: AtomicU8 = AtomicU8::new(0);
static ISO_HEAD: AtomicU8 = AtomicU8::new(0);
static ISO_TAIL: AtomicU8 = AtomicU8::new(0);
static ISO_WAKER: AtomicWaker = AtomicWaker::new();
// counted in the interrupt only
static ISO_PACKETS: AtomicU32 = AtomicU32::new(0);
static ISO_MISSED: AtomicU32 = AtomicU32::new(0);
static ISO_OVERRUNS: AtomicU32 = AtomicU32::new(0);
static ISO_ERRORS: AtomicU32 = AtomicU32::new(0);

/// The host's packet buffers: one per direction for control, bulk and interrupt pipes,
/// and `ISO` bytes for the isochronous IN ring. A slot takes the endpoint's max packet size
/// (rounded up to 4) plus 4; the ring needs at least 2 slots, one of them always free.
#[repr(C, align(4))]
pub struct Buffers<const ISO: usize = 0> {
    rx: [u8; MAX_PACKET],
    tx: [u8; MAX_PACKET],
    iso: [u8; ISO],
}

impl<const ISO: usize> Buffers<ISO> {
    pub const fn new() -> Self {
        Self {
            rx: [0; MAX_PACKET],
            tx: [0; MAX_PACKET],
            iso: [0; ISO],
        }
    }
}

/// The isochronous IN pipe's counters since it was allocated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IsoStats {
    /// packets received
    pub packets: u32,
    /// periods whose IN never went out (the bus was still busy at the next one)
    pub missed: u32,
    /// packets dropped on a full ring
    pub overruns: u32,
    /// INs without a valid answer (no response, a handshake, babble)
    pub errors: u32,
}

/// The isochronous IN pipe's counters.
pub fn iso_in_stats() -> IsoStats {
    IsoStats {
        packets: ISO_PACKETS.load(Ordering::Relaxed),
        missed: ISO_MISSED.load(Ordering::Relaxed),
        overruns: ISO_OVERRUNS.load(Ordering::Relaxed),
        errors: ISO_ERRORS.load(Ordering::Relaxed),
    }
}

fn count(c: &AtomicU32) {
    c.store(c.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
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
        // the transaction before the SOF: an IN that completes in the same entry frees the
        // SIE for the next period's
        if flag.transfer() {
            let st = r.int_st().read();
            let len = r.rx_len().read();
            // stop the token before the release: with INT_BUSY the SIE waits for the flag
            h.ep_pid().write_value(UhEpPid::default());
            r.int_fg().write(|w| w.set_transfer(true));
            match ENGINE.load(Ordering::Relaxed) {
                ASYNC => {
                    RESULT_ST.store(st.0, Ordering::Relaxed);
                    RESULT_LEN.store(len, Ordering::Relaxed);
                    DONE.store(true, Ordering::Release);
                    XFER_WAKER.wake();
                }
                ISO => iso_complete::<T>(st, len),
                _ => {}
            }
            ENGINE.store(IDLE, Ordering::Relaxed);
            if !iso_start_if_due::<T>() {
                ENGINE_WAKER.wake();
            }
        }
        if flag.hst_sof() {
            r.int_fg().write(|w| w.set_hst_sof(true));
            iso_on_sof::<T>();
        }
    }
}

/// A SOF: the iso IN is due once per period.
fn iso_on_sof<T: Instance>() {
    if !ISO_ACTIVE.load(Ordering::Relaxed) {
        return;
    }
    let left = ISO_COUNTDOWN.load(Ordering::Relaxed);
    if left > 1 {
        ISO_COUNTDOWN.store(left - 1, Ordering::Relaxed);
        return;
    }
    ISO_COUNTDOWN.store(ISO_PERIOD.load(Ordering::Relaxed), Ordering::Relaxed);
    if ISO_DUE.load(Ordering::Relaxed) {
        // the last period's IN never got the bus; this one replaces it
        count(&ISO_MISSED);
    }
    ISO_DUE.store(true, Ordering::Relaxed);
    iso_start_if_due::<T>();
}

/// Start the iso IN if one is due and the SIE is idle. Interrupt or critical section.
fn iso_start_if_due<T: Instance>() -> bool {
    if !ISO_ACTIVE.load(Ordering::Relaxed)
        || !ISO_DUE.load(Ordering::Relaxed)
        || ENGINE.load(Ordering::Relaxed) != IDLE
    {
        return false;
    }
    let r = T::regs();
    let h = T::hregs();
    let slots = ISO_SLOTS.load(Ordering::Relaxed);
    let head = ISO_HEAD.load(Ordering::Relaxed);
    let tail = ISO_TAIL.load(Ordering::Acquire);
    // the IN goes out anyway (the device's period is the device's), into the free head slot
    ISO_DROP.store(head.wrapping_sub(tail) >= slots - 1, Ordering::Relaxed);
    let slot = iso_slot(head);

    h.dev_ad().write(|w| w.set_addr(ISO_ADDR.load(Ordering::Relaxed)));
    h.rx_max_len().write(|w| w.set_len(ISO_MPS.load(Ordering::Relaxed)));
    h.rx_dma().write_value(slot + ISO_HEADER as u32);
    h.ep_type().write(|w| w.set_h_rx_type(true));
    h.rx_ctrl().write(|w| w.set_r_tog(Tog::DATA0));
    OVERFLOW.store(false, Ordering::Relaxed);
    ISO_DUE.store(false, Ordering::Relaxed);
    ENGINE.store(ISO, Ordering::Relaxed);
    h.ep_pid().write(|w| {
        w.set_token(PID_IN);
        w.set_endp(ISO_EP.load(Ordering::Relaxed));
    });
    r.int_fg().write(|w| w.set_transfer(true));
    true
}

fn iso_slot(i: u8) -> u32 {
    let slots = ISO_SLOTS.load(Ordering::Relaxed);
    ISO_RING.load(Ordering::Relaxed) + (i & (slots - 1)) as u32 * ISO_STRIDE.load(Ordering::Relaxed) as u32
}

/// The iso IN's answer: a data packet into the ring.
fn iso_complete<T: Instance>(st: IntSt, len: u16) {
    if !ISO_ACTIVE.load(Ordering::Relaxed) {
        return;
    }
    if !matches!(st.h_res(), PID_DATA0 | PID_DATA1 | PID_DATA2 | PID_MDATA)
        || OVERFLOW.load(Ordering::Relaxed)
        || len > ISO_MPS.load(Ordering::Relaxed)
    {
        count(&ISO_ERRORS);
        return;
    }
    if ISO_DROP.load(Ordering::Relaxed) {
        count(&ISO_OVERRUNS);
        return;
    }
    let head = ISO_HEAD.load(Ordering::Relaxed);
    let frame = T::regs().frame_no().read().frame_no();
    // SAFETY: the head slot is the producer's (this interrupt) until head moves
    unsafe {
        let p = iso_slot(head) as *mut u16;
        p.write_volatile(len);
        p.add(1).write_volatile(frame);
    }
    ISO_HEAD.store(head.wrapping_add(1), Ordering::Release);
    count(&ISO_PACKETS);
    ISO_WAKER.wake();
}

/// Stop the iso IN pipe (dropped, or the device went).
fn iso_stop() {
    critical_section::with(|_| {
        ISO_ACTIVE.store(false, Ordering::Relaxed);
        ISO_DUE.store(false, Ordering::Relaxed);
    });
    ISO_WAKER.wake();
}

fn attached<T: Instance>() -> bool {
    T::regs().mis_st().read().dev_attach()
}

fn bump_generation() {
    GENERATION.store(GENERATION.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
    iso_stop();
    XFER_WAKER.wake();
    ENGINE_WAKER.wake();
}

pub struct Driver<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    connected: bool,
}

impl<'d, T: Instance> Driver<'d, T> {
    pub fn new<const ISO: usize>(
        _peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl DpPin<T, 0>>,
        dm: Peri<'d, impl DmPin<T, 0>>,
        buffers: &'d mut Buffers<ISO>,
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
        h.ep_type().write_value(UhEpType::default());
        h.rx_max_len().write(|w| w.set_len(MAX_PACKET as u16));

        let rx = buffers.rx.as_mut_ptr() as u32;
        let tx = buffers.tx.as_mut_ptr() as u32;
        h.rx_dma().write_value(rx);
        h.tx_dma().write_value(tx);
        RX_BUF.store(rx, Ordering::Relaxed);
        TX_BUF.store(tx, Ordering::Relaxed);
        ISO_RING.store(buffers.iso.as_mut_ptr() as u32, Ordering::Relaxed);
        ISO_RING_LEN.store(ISO as u32, Ordering::Relaxed);
        ISO_ALLOCATED.store(false, Ordering::Relaxed);

        ENGINE.store(IDLE, Ordering::Relaxed);
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
            w.set_sof_act(true);
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

        iso_stop();
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
        let iso = E::ep_type() == EndpointType::Isochronous;
        let limit = if iso { MAX_ISO_PACKET } else { MAX_PACKET };
        // high bandwidth (wMaxPacketSize bits 12:11) is over the limit too
        if endpoint.max_packet_size == 0 || endpoint.max_packet_size as usize > limit {
            return Err(HostError::InvalidDescriptor);
        }
        let generation = GENERATION.load(Ordering::Relaxed);
        if iso {
            iso_alloc(addr, endpoint, D::is_out())?;
        }
        Ok(Pipe {
            _phantom: PhantomData,
            target: Target {
                addr,
                ep: endpoint.addr.index() as u8,
                max_packet_size: endpoint.max_packet_size,
                generation,
            },
            interval_ms: endpoint.interval_ms,
            iso,
            tog: false,
            timeout: TimeoutConfig::default(),
        })
    }

    fn device_removed(&self, _addrs: AddressSet) {
        // one device: whichever it was, its pipes go
        bump_generation();
    }
}

/// Claim the isochronous IN pipe and lay out its ring.
fn iso_alloc(addr: u8, endpoint: &EndpointInfo, out: bool) -> Result<(), HostError> {
    if out {
        return Err(HostError::Other("usbhs host: no isochronous OUT"));
    }
    let stride = ISO_HEADER + (endpoint.max_packet_size as usize).next_multiple_of(4);
    let fit = ISO_RING_LEN.load(Ordering::Relaxed) as usize / stride;
    if fit < 2 {
        return Err(HostError::InsufficientMemory);
    }
    // a power of two: the wrapping u8 indices stay continuous
    let slots = 1 << (usize::BITS - 1 - fit.min(ISO_MAX_SLOTS).leading_zeros());
    // bInterval: 2^(n-1) (micro)frames
    let period = 1u8 << (endpoint.interval_ms.clamp(1, 8) - 1);
    critical_section::with(|_| {
        if ISO_ALLOCATED.load(Ordering::Relaxed) {
            return Err(HostError::OutOfPipes);
        }
        ISO_ALLOCATED.store(true, Ordering::Relaxed);
        ISO_ACTIVE.store(false, Ordering::Relaxed);
        ISO_DUE.store(false, Ordering::Relaxed);
        ISO_ADDR.store(addr, Ordering::Relaxed);
        ISO_EP.store(endpoint.addr.index() as u8, Ordering::Relaxed);
        ISO_MPS.store(endpoint.max_packet_size, Ordering::Relaxed);
        ISO_PERIOD.store(period, Ordering::Relaxed);
        ISO_STRIDE.store(stride as u16, Ordering::Relaxed);
        ISO_SLOTS.store(slots as u8, Ordering::Relaxed);
        for c in [&ISO_PACKETS, &ISO_MISSED, &ISO_OVERRUNS, &ISO_ERRORS] {
            c.store(0, Ordering::Relaxed);
        }
        Ok(())
    })
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

/// Stops a task's token that is still on the bus when the transaction is dropped or timed
/// out.
struct InFlight<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Drop for InFlight<T> {
    fn drop(&mut self) {
        let r = T::regs();
        let h = T::hregs();
        critical_section::with(|_| {
            if ENGINE.load(Ordering::Relaxed) != ASYNC {
                return;
            }
            h.ep_pid().write_value(UhEpPid::default());
            let mut n = 0;
            while !r.mis_st().read().sie_free() && n < SIE_WAIT {
                n += 1;
            }
            r.int_fg().write(|w| w.set_transfer(true));
            ENGINE.store(IDLE, Ordering::Relaxed);
            iso_start_if_due::<T>();
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
        // SAFETY: the TX buffer is MAX_PACKET long and only the task holding the lock uses it
        unsafe {
            core::ptr::copy_nonoverlapping(out.as_ptr(), TX_BUF.load(Ordering::Relaxed) as *mut u8, out.len());
        }
    }
    let tog = if tog { Tog::DATA1 } else { Tog::DATA0 };
    compiler_fence(Ordering::SeqCst);

    // the SIE once it's idle and no iso IN is due; the registers in the same critical
    // section, as the interrupt sets them for the iso IN
    let in_flight = poll_fn(|cx| {
        critical_section::with(|_| {
            let due = ISO_ACTIVE.load(Ordering::Relaxed) && ISO_DUE.load(Ordering::Relaxed);
            if ENGINE.load(Ordering::Relaxed) != IDLE || due {
                ENGINE_WAKER.register(cx.waker());
                return Poll::Pending;
            }
            h.dev_ad().write(|w| w.set_addr(t.addr));
            h.rx_max_len().write(|w| w.set_len(t.max_packet_size));
            h.rx_dma().write_value(RX_BUF.load(Ordering::Relaxed));
            h.ep_type().write_value(UhEpType::default());
            h.tx_len().write(|w| w.set_len(out.len() as u16));
            h.tx_ctrl().write(|w| w.set_t_tog(tog));
            h.rx_ctrl().write(|w| w.set_r_tog(tog));
            DONE.store(false, Ordering::Relaxed);
            OVERFLOW.store(false, Ordering::Relaxed);
            ENGINE.store(ASYNC, Ordering::Relaxed);
            // WCH's order: the token, then the flag release
            h.ep_pid().write(|w| {
                w.set_token(pid);
                w.set_endp(t.ep);
            });
            T::regs().int_fg().write(|w| w.set_transfer(true));
            Poll::Ready(InFlight::<T> { _phantom: PhantomData })
        })
    })
    .await;

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
            // SAFETY: the RX buffer holds `len` <= MAX_PACKET bytes from the DMA; only the
            // task holding the lock uses it
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

/// The next packet from the isochronous IN ring, arming the pipe on the first call: its
/// length and the frame number register's value when it arrived.
async fn iso_read<T: Instance>(generation: u8, buf: &mut [u8]) -> Result<(usize, u16), PipeError> {
    let gone = || GENERATION.load(Ordering::Relaxed) != generation || !attached::<T>();
    if gone() {
        return Err(PipeError::Disconnected);
    }
    critical_section::with(|_| {
        if !ISO_ACTIVE.load(Ordering::Relaxed) {
            ISO_HEAD.store(0, Ordering::Relaxed);
            ISO_TAIL.store(0, Ordering::Relaxed);
            ISO_COUNTDOWN.store(1, Ordering::Relaxed);
            ISO_DUE.store(false, Ordering::Relaxed);
            ISO_ACTIVE.store(true, Ordering::Relaxed);
        }
    });
    poll_fn(|cx| {
        ISO_WAKER.register(cx.waker());
        let tail = ISO_TAIL.load(Ordering::Relaxed);
        if ISO_HEAD.load(Ordering::Acquire) != tail {
            let slot = iso_slot(tail);
            // SAFETY: a slot in [tail, head): the interrupt is done with it until tail moves
            let len = unsafe { (slot as *const u16).read_volatile() } as usize;
            let frame = unsafe { (slot as *const u16).add(1).read_volatile() };
            let r = if len > buf.len() {
                Err(PipeError::BufferOverflow)
            } else {
                unsafe {
                    core::ptr::copy_nonoverlapping((slot as usize + ISO_HEADER) as *const u8, buf.as_mut_ptr(), len);
                }
                Ok((len, frame))
            };
            ISO_TAIL.store(tail.wrapping_add(1), Ordering::Release);
            return Poll::Ready(r);
        }
        if gone() || !ISO_ACTIVE.load(Ordering::Relaxed) {
            return Poll::Ready(Err(PipeError::Disconnected));
        }
        Poll::Pending
    })
    .await
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
    /// the isochronous IN pipe (owns the ring)
    iso: bool,
    /// the next bulk / interrupt packet's toggle (DATA1: true)
    tog: bool,
    timeout: TimeoutConfig,
}

impl<'d, T: Instance, E, D> Drop for Pipe<'d, T, E, D> {
    fn drop(&mut self) {
        if self.iso {
            iso_stop();
            ISO_ALLOCATED.store(false, Ordering::Relaxed);
        }
    }
}

impl<'d, T: Instance> Pipe<'d, T, pipe::Isochronous, pipe::In> {
    /// `request_in`, plus the frame number register's value when the packet arrived (the
    /// device's packets against USB time).
    pub async fn read_iso(&mut self, buf: &mut [u8]) -> Result<(usize, u16), PipeError> {
        iso_read::<T>(self.target.generation, buf).await
    }
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
        if self.iso {
            return iso_read::<T>(self.target.generation, buf).await.map(|(len, _)| len);
        }
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
