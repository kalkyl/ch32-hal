//! Quadrature encoder interface: a general-purpose timer counting the edges of a two-phase
//! encoder in hardware (encoder mode 3, both inputs, x4).

use core::marker::PhantomData;

use super::low_level::{InputTISelection, Timer};
use super::{Channel, Channel1Pin, Channel2Pin, GeneralInstance16bit};
use crate::gpio::{AnyPin, Pull};
use crate::pac::timer::vals;
use crate::Peri;

/// Channel 1 marker type.
pub enum Ch1 {}
/// Channel 2 marker type.
pub enum Ch2 {}

/// Encoder input pin.
pub struct QeiPin<'d, T, C> {
    _pin: Peri<'d, AnyPin>,
    phantom: PhantomData<(T, C)>,
}

macro_rules! channel_impl {
    ($new_chx:ident, $channel:ident, $pin_trait:ident) => {
        impl<'d, T: GeneralInstance16bit> QeiPin<'d, T, $channel> {
            #[doc = concat!("Create a new ", stringify!($channel), " encoder input.")]
            pub fn $new_chx<const REMAP: u8>(pin: Peri<'d, impl $pin_trait<T, REMAP>>, pull: Pull) -> Self {
                critical_section::with(|_| {
                    pin.set_as_input(pull);
                    T::set_remap(REMAP);
                });
                QeiPin {
                    _pin: pin.into(),
                    phantom: PhantomData,
                }
            }
        }
    };
}

channel_impl!(new_ch1, Ch1, Channel1Pin);
channel_impl!(new_ch2, Ch2, Channel2Pin);

/// Quadrature encoder driver.
pub struct Qei<'d, T: GeneralInstance16bit> {
    inner: Timer<'d, T>,
}

impl<'d, T: GeneralInstance16bit> Qei<'d, T> {
    /// Create a new encoder driver. `filter` applies to both inputs.
    pub fn new(
        tim: Peri<'d, T>,
        _ch1: QeiPin<'d, T, Ch1>,
        _ch2: QeiPin<'d, T, Ch2>,
        filter: vals::FilterValue,
    ) -> Self {
        let inner = Timer::new(tim);
        for ch in [Channel::Ch1, Channel::Ch2] {
            inner.set_input_ti_selection(ch, InputTISelection::Normal);
            inner.set_input_capture_filter(ch, filter);
        }
        let r = inner.regs_gp16();
        r.smcfgr().modify(|w| w.set_sms(0b011));
        r.atrlr().write_value(u16::MAX);
        inner.start();
        Self { inner }
    }

    /// The counter, wrapping: one count per edge on either input.
    pub fn count(&self) -> u16 {
        self.inner.regs_gp16().cnt().read()
    }
}
