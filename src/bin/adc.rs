#![no_std]
#![no_main]

use core::{any::Any, ops::DerefMut};

use cortex_m::asm::nop;
use defmt::*;
use embassy_executor::Spawner;
use nxp_pac::*;
use {defmt_rtt as _, panic_halt as _};

fn init() {
    info!("Init");

    // Enable IOCON
    // Enable GPIO as well for testing a simple CS
    SYSCON.ahbclkctrl0().modify(|w| {
        w.set_iocon(true);
        w.set_gpio0(true);
    });

    // IOCON Setup
    // All pins are configured according to the standard settings in the SPI config documentation

    // Channel 0
    IOCON.pio0(16).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT0);
        w.set_digimode(iocon::vals::PioDigimode::ANALOG);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
        w.set_asw(iocon::vals::PioAsw::VALUE1);
    });
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init();
    info!("Res: {}", ADC0.verid().read().res().to_bits());
    info!("IADCKI: {}", ADC0.verid().read().iadcki().to_bits());
    loop {
        for _ in 0..1_000_000 {
            nop();
        }
    }
}
