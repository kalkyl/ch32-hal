//! Independent watchdog (IWDG)
//!
//! Unlike STM32's, the start key doesn't turn the LSI on (after a system reset it is off),
//! and the prescaler and reload only reach the LSI domain with it running: [`IndependentWatchdog::new`]
//! turns the LSI on first. Once unleashed, the watchdog runs until the next reset.
use core::marker::PhantomData;

use embassy_hal_internal::PeripheralType;

use crate::time::Hertz;
use crate::{pac, Peri};

/// Nominal LSI frequency; it varies by tens of percent over parts and temperature.
#[cfg(any(ch32v0, ch32m0))]
pub const LSI_FREQ: Hertz = Hertz(128_000);
/// Nominal LSI frequency; it varies by tens of percent over parts and temperature.
#[cfg(not(any(ch32v0, ch32m0)))]
pub const LSI_FREQ: Hertz = Hertz(40_000);

const KEY_RELOAD: u16 = 0xAAAA;
const KEY_UNLOCK: u16 = 0x5555;
const KEY_START: u16 = 0xCCCC;
// 12-bit counter
const MAX_RL: u16 = 0xFFF;
// LSI cycles take tens of us; a few of them at most
const WAIT_LOOPS: u32 = 1_000_000;

/// Timeout in us for a prescaler and reload value.
const fn timeout_us(prescaler: u32, reload: u16) -> u32 {
    (1_000_000u64 * (reload as u64 + 1) * prescaler as u64 / LSI_FREQ.0 as u64) as u32
}

/// Independent watchdog driver.
pub struct IndependentWatchdog<'d, T: Instance> {
    _wdg: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> IndependentWatchdog<'d, T> {
    /// Configure the watchdog for a timeout in microseconds (at the nominal LSI frequency),
    /// with the LSI turned on. Start it with [`Self::unleash`], then [`Self::pet`] it within
    /// the timeout or the MCU resets.
    pub fn new(_instance: Peri<'d, T>, timeout_us: u32) -> Self {
        // the shortest prescaler (4 to 256) whose longest timeout covers the one asked for
        let pr = unwrap!((0..=6u8).find(|&pr| timeout_us <= self::timeout_us(4 << pr, MAX_RL)));
        let prescaler = 4u32 << pr;
        let reload = ((timeout_us as u64 * LSI_FREQ.0 as u64 / prescaler as u64 / 1_000_000) as u16)
            .saturating_sub(1)
            .min(MAX_RL);

        let rcc = pac::RCC;
        rcc.rstsckr().modify(|w| w.set_lsion(true));
        for _ in 0..WAIT_LOOPS {
            if rcc.rstsckr().read().lsirdy() {
                break;
            }
        }

        let wdg = T::regs();
        wdg.ctlr().write(|w| w.set_key(KEY_UNLOCK));
        wdg.pscr().write(|w| w.set_pr(pr));
        wdg.rldr().write(|w| w.set_rl(reload));
        for _ in 0..WAIT_LOOPS {
            let s = wdg.statr().read();
            if !s.pvu() && !s.rvu() {
                break;
            }
        }

        trace!(
            "IWDG: {}us timeout ({}us asked), PR={}, RL={}",
            self::timeout_us(prescaler, reload),
            timeout_us,
            pr,
            reload
        );

        Self { _wdg: PhantomData }
    }

    /// Start the watchdog. It can't be stopped until the next reset.
    pub fn unleash(&mut self) {
        T::regs().ctlr().write(|w| w.set_key(KEY_START));
        self.pet();
    }

    /// Reload the counter.
    pub fn pet(&mut self) {
        T::regs().ctlr().write(|w| w.set_key(KEY_RELOAD));
    }
}

trait SealedInstance {
    fn regs() -> crate::pac::iwdg::Iwdg;
}

/// IWDG instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

foreach_peripheral!(
    (iwdg, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> crate::pac::iwdg::Iwdg {
                crate::pac::$inst
            }
        }

        impl Instance for crate::peripherals::$inst {}
    };
);
