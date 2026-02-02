#![no_std]
#![no_main]

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
        w.set_adc(true);
    });
    // Enable Analog Control for powering ADC up
    SYSCON.ahbclkctrl2().modify(|w| w.set_analog_ctrl(true));

    // Choose clock for ADC
    // FRO 96 MHz
    SYSCON.adcclksel().modify(|w| w.set_sel(0x2));
    SYSCON.adcclkdiv().modify(|w| {
        w.set_reset(syscon::vals::AdcclkdivReset::ASSERTED);
        w.set_div(0x0);
        w.set_reset(syscon::vals::AdcclkdivReset::RELEASED);
    });
    // Remark: For proper ADC operation, the `PDEN_AUXBIAS` bit
    // in the `PDRUNCFG0` register must be set to 0 (powered)
    // and `VREF1VENABLE` bit in the `AUX_BIAS` register must be
    // set to 1 (enabled) prior to using ADC peripheral.
    PMC.pdruncfg0()
        .modify(|w| w.set_pden_auxbias(pmc::vals::PdenAuxbias::POWEREDON));
    ANACTRL.aux_bias().modify(|w| w.set_vref1venable(true));

    // IOCON Setup
    // All pins are configured according to the standard settings in the SPI config documentation

    // Channel 0B
    IOCON.pio0(16).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT0);
        w.set_digimode(iocon::vals::PioDigimode::ANALOG);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
        w.set_asw(iocon::vals::PioAsw::VALUE1);
    });

    // Reset ADC
    ADC0.ctrl().modify(|w| {
        w.set_rst(adc0::vals::Rst::RST_1);
        w.set_rst(adc0::vals::Rst::RST_0);
        w.set_rstfifo0(adc0::vals::Rstfifo0::RSTFIFO0_1);
        w.set_rstfifo1(adc0::vals::Rstfifo1::RSTFIFO1_1);
        w.set_rstfifo0(adc0::vals::Rstfifo0::RSTFIFO0_0);
        w.set_rstfifo1(adc0::vals::Rstfifo1::RSTFIFO1_0);
    });

    // Turn off
    ADC0.ctrl().modify(|w| {
        w.set_adcen(adc0::vals::Adcen::ADCEN_0);
    });
    ADC0.cfg()
        .modify(|w| w.set_pwrsel(adc0::vals::Pwrsel::PWRSEL_0));
    ADC0.ctrl().modify(|w| {
        w.set_dozen(adc0::vals::Dozen::DOZEN_0);
        w.set_cal_avgs(adc0::vals::CalAvgs::CAL_AVGS_7);
    });

    ADC0.cfg().modify(|w| {
        w.set_pwren(adc0::vals::Pwren::PWREN_1);
        w.set_pudly(0x80);
        w.set_refsel(adc0::vals::Refsel::REFSEL_1);
        w.set_pwrsel(adc0::vals::Pwrsel::PWRSEL_3);
        w.set_tprictrl(adc0::vals::Tprictrl::TPRICTRL_0);
        w.set_tres(adc0::vals::Tres::TRES_0);
    });

    ADC0.pause().modify(|w| {
        w.set_pausedly(0x0);
        w.set_pauseen(adc0::vals::Pauseen::PAUSEEN_0);
    });

    ADC0.fctrl(0).modify(|w| w.set_fwmark(0));
    ADC0.fctrl(1).modify(|w| w.set_fwmark(0));

    ADC0.ctrl()
        .modify(|w| w.set_adcen(adc0::vals::Adcen::ADCEN_1));

    calibrate();

    // TODO(frihetselsker): Finish startup and perform first ADC conversion
}

fn calibrate() {
    ADC0.ofstrim().modify(|w| {
        w.set_ofstrim_a(0xA);
        w.set_ofstrim_b(0xA);
    });
    ADC0.ctrl()
        .modify(|w| w.set_cal_req(adc0::vals::CalReq::CAL_REQ_1));
    // Wait for auto calibration to be ready
    while (ADC0.gcc(0).read().rdy().to_bits() == 0) || (ADC0.gcc(1).read().rdy().to_bits() == 0) {}
    let gain_a = ADC0.gcc(0).read().gain_cal();
    let gain_b = ADC0.gcc(1).read().gain_cal();
    // Fixed point efficiency
    // I don't understand this formula.
    // Why is it optimizing the calculation?
    // How did he come up with this formula?
    let gcr_a = (((gain_a as u32) << 16) / (0x1FFFFu32 - gain_a as u32)) as u16;
    let gcr_b = (((gain_b as u32) << 16) / (0x1FFFFu32 - gain_b as u32)) as u16;
    ADC0.gcr(0).modify(|w| {
        w.set_gcalr(gcr_a);
        w.set_rdy(adc0::vals::GcrRdy::RDY_1);
    });
    ADC0.gcr(1).modify(|w| {
        w.set_gcalr(gcr_b);
        w.set_rdy(adc0::vals::GcrRdy::RDY_1);
    });

    while ADC0.stat().read().cal_rdy().to_bits() == 0 {}
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
