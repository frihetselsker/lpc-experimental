#![no_std]
#![no_main]

use cortex_m::asm::nop;
use defmt::*;
use embassy_executor::Spawner;
use nxp_pac::*;
use {defmt_rtt as _, panic_halt as _};

// const DIVIDER: u8 = 96;
const DUTY_CYCLE: u32 = 50;
const TOP: u32 = u32::MAX;
const OUTPUT_PIN: usize = 1;

fn init() {
    info!("Initialization");

    // Enable clocks (Syscon is enabled by default)
    info!("Enable clocks");
    SYSCON.ahbclkctrl0().modify(|w| w.set_iocon(true));
    SYSCON.ahbclkctrl1().modify(|w| w.set_sct(true));

    // IOCON Setup
    info!("IOCON Setup");
    IOCON.pio0(18).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT4);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    }); // SCTimer Output 1

    // Choose the clock for PWM
    SYSCON
        .sctclksel()
        .modify(|w| w.set_sel(syscon::vals::SctclkselSel::ENUM_0X3));

    // SYSCON.sctclkdiv().modify(|w| w.set_div(DIVIDER - 1));

    // Reset SCTimer
    info!("Reset SC Timer");
    SYSCON
        .presetctrl1()
        .modify(|w| w.set_sct_rst(syscon::vals::SctRst::ASSERTED));
    SYSCON
        .presetctrl1()
        .modify(|w| w.set_sct_rst(syscon::vals::SctRst::RELEASED));

    // Configure SCTimer
    SCT0.config().modify(|w| {
        w.set_unify(sct0::vals::Unify::UNIFIED_COUNTER);
        w.set_clkmode(sct0::vals::Clkmode::SYSTEM_CLOCK_MODE);
        w.set_autolimit_l(true);
    });

    SCT0.match0().modify(|w| {
        w.set_matchn_l((TOP & 0xFFFF) as u16);
        w.set_matchn_h((TOP >> 16) as u16);
    });

    SCT0.matchrel0().modify(|w| {
        w.set_reloadn_l((TOP << 16) as u16);
        w.set_reloadn_h((TOP >> 16) as u16);
    });

    // The actual matches

    SCT0.match1().modify(|w| {
        let value: u32 = (TOP / 100) * DUTY_CYCLE;
        w.set_matchn_l((value & 0xFFFF) as u16);
        w.set_matchn_h((value >> 16) as u16);
    });

    // SCT0.matchrel1().modify(|w| {
    //     let value: u32 = (TOP / 100) * DUTY_CYCLE;
    //     w.set_reloadn_l((value << 16) as u16);
    //     w.set_reloadn_h((value >> 16) as u16);
    // });

    SCT0.match2().modify(|w| {
        w.set_matchn_l(0);
        w.set_matchn_h(0);
    });

    // SCT0.matchrel2().modify(|w| {
    //     w.set_reloadn_l(0);
    //     w.set_reloadn_h(0);
    // });

    // Events

    SCT0.ev(0).ev_ctrl().modify(|w| {
        w.set_matchsel(2);
        w.set_combmode(sct0::vals::Combmode::MATCH);
        w.set_stateld(sct0::vals::Stateld::ADD);
        w.set_statev(0);
    });
    SCT0.ev(1).ev_ctrl().modify(|w| {
        w.set_matchsel(1);
        w.set_combmode(sct0::vals::Combmode::MATCH);
        w.set_stateld(sct0::vals::Stateld::ADD);
        w.set_statev(0);
    });

    SCT0.ev(0).ev_state().modify(|w| w.set_statemskn(1 << 0));
    SCT0.ev(1).ev_state().modify(|w| w.set_statemskn(1 << 0));

    SCT0.out(1).out_set().modify(|w| w.set_set(1 << 0));
    SCT0.out(1).out_clr().modify(|w| w.set_clr(1 << 1));

    SCT0.state().modify(|w| w.set_state_l(0));

    SCT0.ctrl().modify(|w| {
        w.set_clrctr_l(true);
        w.set_stop_l(false);
    });
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init();

    info!("Inside the main function");
    loop {
        nop();
    }
}
