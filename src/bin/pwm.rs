#![no_std]
#![no_main]

use cortex_m::asm::nop;
use defmt::*;
use embassy_executor::Spawner;
use nxp_pac::{FLEXCOMM2, *};
use {defmt_rtt as _, panic_halt as _};

fn init() {
    info!("Initialization");

    // Enable clocks (Syscon is enabled by default)
    info!("Enable clocks");
    SYSCON.ahbclkctrl0().modify(|w| w.set_iocon(true));
    SYSCON.ahbclkctrl1().modify(|w| w.set_sct(true));
    // Reset SCTimer
    info!("Reset SC Timer");
    SYSCON
        .presetctrl1()
        .modify(|w| w.set_sct_rst(syscon::vals::SctRst::ASSERTED));
    SYSCON
        .presetctrl1()
        .modify(|w| w.set_sct_rst(syscon::vals::SctRst::RELEASED));

    // Select the clock source for Flexcomm 2

    info!("Select clock");

    // Depends on the oversampling rate, for this example 16x oversampling is used
    // let source_clock = match config.baudrate {
    //     750_001..=6_000_000 => {
    //         SYSCON
    //             .fcclksel(2)
    //             .modify(|w| w.set_sel(syscon::vals::FcclkselSel::ENUM_0X3)); // 96 MHz
    //         96_000_000
    //     }
    //     1_501..=750_000 => {
    //         SYSCON
    //             .fcclksel(2)
    //             .write(|w| w.set_sel(syscon::vals::FcclkselSel::ENUM_0X2)); // 12 MHz
    //         12_000_000
    //     }
    //     121..=1_500 => {
    //         SYSCON
    //             .fcclksel(2)
    //             .write(|w| w.set_sel(syscon::vals::FcclkselSel::ENUM_0X4)); // 1 MHz
    //         1_000_000
    //     }
    //     _ => {
    //         crate::unreachable!("{} baudrate is not permitted in this mode", config.baudrate);
    //         // Try 32 KHz mode
    //     }
    // };

    // The Fractional Rate Generator can be used to obtain more precise baud rates when the
    // function clock is not a good multiple of standard (or otherwise desirable) baud rates.
    // The FRG is typically set up to produce an integer multiple of the highest required baud
    // rate, or a very close approximation. The BRG is then used to obtain the actual baud rate
    // needed.
    FLEXCOMM2
        .pselid()
        .modify(|w| w.set_persel(flexcomm::vals::Persel::I2C));
    //IOCON Setup
    info!("IOCON Setup");
    IOCON.pio1(24).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT1);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    }); // rx
    IOCON.pio0(27).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT1);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    }); // tx
        // To get the baudrate, we need to backpropagate across the formulas
        // Having baud rate, assume DIV is 256 since it gives more granularity

    // Flexcomm Interface function clock = (clock selected via FCCLKSEL) / (1 + MULT / DIV)

    // Firstly, we find integer division value. BRG = source_clock / (16 * baud_rate). Drop the fraction part. A bigger clock will let chisel it using FRG
    // Secondly, raw_clock = source_clock / (16 * BRG)
    // Thirdly, MULT = raw_clock * 256 / baud_rate - 256
    // Finally, final_clock =  raw_clock / (1 + MULT / DIV)

    info!("Flexcomm clock");
    SYSCON.flexfrgctrl(2).modify(|w| {
        w.set_div(0xFF);
        w.set_mult(0);
    });

    info!("I2C Config");
    // I2C configuration part
    // FIFO Configuration

    info!("Enabling DMA settings for USART");
    USART2.fifocfg().modify(|w| {
        w.set_dmarx(true);
        w.set_dmatx(true);
        // w.set_waketx(true);
        // w.set_wakerx(true);
    });

    // USART Interrupts
    /* usart.intenset.modify(|_, w| {
        w.framerren()
            .set_bit()
            .parityerren()
            .set_bit()
            .rxnoiseen()
            .set_bit()
            .aberren()
            .set_bit()
    });*/

    // Enable FIFO and USART

    info!("After all settings, enable fifo");
    USART2.fifocfg().modify(|w| {
        w.set_enablerx(true);
        w.set_enabletx(true);
    });

    info!("Enable USART");
    USART2.cfg().modify(|w| {
        w.set_enable(true);
    });

    for _ in 0..200_000 {
        nop();
    }

    // FIFO Interrupts
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init();

    info!("Inside the main function");
    loop {
        nop();
    }
}
