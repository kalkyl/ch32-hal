#![no_std]
#![no_main]

//! Independent watchdog: petted for a while, then left alone -- the MCU resets, and the next
//! boot reports the watchdog as the reset cause.

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::iwdg::IndependentWatchdog;
use hal::{pac, println};
use panic_halt as _;

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let flags = pac::RCC.rstsckr().read();
    println!(
        "reset by: {}",
        if flags.iwdgrstf() {
            "the watchdog"
        } else if flags.porrstf() {
            "power-on"
        } else if flags.sftrstf() {
            "software"
        } else {
            "pin"
        }
    );
    pac::RCC.rstsckr().modify(|w| w.set_rmvf(true));

    let mut wdg = IndependentWatchdog::new(p.IWDG, 2_000_000);
    wdg.unleash();
    for i in 0..5 {
        println!("pet {}", i);
        wdg.pet();
        Timer::after_millis(500).await;
    }
    println!("no more petting: a reset within 2 s");
    loop {
        Timer::after_secs(1).await;
    }
}
