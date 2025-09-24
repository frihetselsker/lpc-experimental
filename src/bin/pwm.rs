#![no_std]
#![no_main]

use cortex_m::asm::nop;
use defmt::*;
use embassy_executor::Spawner;
use nxp_pac::*;
use {defmt_rtt as _, panic_halt as _};

const DUTY_CYCLE: u32 = 10;
const TOP: u32 = 200_000;
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

    // Divide clock by 1-256

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
        w.set_unify(sct0::vals::Unify::UNIFIED_COUNTER); // 32-bit counter for now, can be configured so that it has two separate counters
        w.set_clkmode(sct0::vals::Clkmode::SYSTEM_CLOCK_MODE);
        w.set_autolimit_l(true);
    });

    // Match 0 will reset the timer using TOP value

    SCT0.mtch(0).modify(|w| {
        w.set_matchn_l((TOP & 0xFFFF) as u16);
        w.set_matchn_h((TOP >> 16) as u16);
    });

    // It is OBLIGATORY to write the reload value
    // because after each limit match values are cleared

    SCT0.matchrel(0).modify(|w| {
        w.set_reloadn_l((TOP & 0xFFFF) as u16);
        w.set_reloadn_h((TOP >> 16) as u16);
    });

    // The actual matches that are used for event logic

    SCT0.mtch(1).modify(|w| {
        let value: u32 = (TOP / 100) * DUTY_CYCLE;
        w.set_matchn_l((value & 0xFFFF) as u16);
        w.set_matchn_h((value >> 16) as u16);
    });

    // Reload value is OBLIGATORY

    SCT0.matchrel(1).modify(|w| {
        let value: u32 = (TOP / 100) * DUTY_CYCLE;
        w.set_reloadn_l((value & 0xFFFF) as u16);
        w.set_reloadn_h((value >> 16) as u16);
    });

    SCT0.mtch(2).modify(|w| {
        w.set_matchn_l(0);
        w.set_matchn_h(0);
    });

    // Reload value is OBLIGATORY

    SCT0.matchrel(2).modify(|w| {
        w.set_reloadn_l(0);
        w.set_reloadn_h(0);
    });

    // Event configuration

    SCT0.ev(0).ev_ctrl().modify(|w| {
        w.set_matchsel(2);
        w.set_combmode(sct0::vals::Combmode::MATCH);
        w.set_stateld(sct0::vals::Stateld::ADD); // STATE + statev
        w.set_statev(0);
    });
    SCT0.ev(1).ev_ctrl().modify(|w| {
        w.set_matchsel(1);
        w.set_combmode(sct0::vals::Combmode::MATCH);
        w.set_stateld(sct0::vals::Stateld::ADD); // STATE + statev
        w.set_statev(0);
    });

    // Statev can move to another state

    // Assign events to states
    SCT0.ev(0).ev_state().modify(|w| w.set_statemskn(1 << 0));
    SCT0.ev(1).ev_state().modify(|w| w.set_statemskn(1 << 0));

    SCT0.out(OUTPUT_PIN).out_set().modify(|w| w.set_set(1 << 0)); // High when event 0 is active
    SCT0.out(OUTPUT_PIN).out_clr().modify(|w| w.set_clr(1 << 1)); // Low when event 1 is active

    SCT0.state().modify(|w| w.set_state_l(0)); // State 0 by default

    SCT0.ctrl().modify(|w| {
        w.set_clrctr_l(true); // Clear counter before start
        w.set_halt_l(false); // Remove halt and start the actual counter
                             // w.set_pre_l(96 - 1); // Division by prescaled value
    });
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init();

    info!("Inside the main function");

    info!("Counter: {:b}", SCT0.count().read().0);
    for _ in 0..100_000 {
        nop();
    }
    info!("Is unified? {}", SCT0.config().read().unify().to_bits());
    info!("Is counting down? {}", SCT0.ctrl().read().down_l());
    info!("Is halted? {} ", SCT0.ctrl().read().halt_l());
    info!(
        "Is bidirectional? {}",
        SCT0.ctrl().read().bidir_l().to_bits()
    );
    info!("Is stopped? {}", SCT0.ctrl().read().stop_l());
    loop {
        info!("Counter: {:b}", SCT0.count().read().0);
        for _ in 0..100_000 {
            nop();
        }

        info!("Is unified? {}", SCT0.config().read().unify().to_bits());
        info!("Is counting down? {}", SCT0.ctrl().read().down_l());
        info!("Is halted? {} ", SCT0.ctrl().read().halt_l());
        info!(
            "Is bidirectional? {}",
            SCT0.ctrl().read().bidir_l().to_bits()
        );
        info!("Is stopped? {}", SCT0.ctrl().read().stop_l());
    }
}
