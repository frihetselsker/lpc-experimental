#![no_std]
#![no_main]

use cortex_m::asm::nop;
use defmt::*;
use embassy_executor::Spawner;
use nxp_pac::*;
use {defmt_rtt as _, panic_halt as _};

pub enum Error {

}

fn init() {
    info!("Init");

    // SPI 7 at FLEXCOMM 7 Setup

    // Enable iocon and flexcomm
    SYSCON.ahbclkctrl0().modify(|w| {
        w.set_iocon(true);
        w.set_gpio0(true);
        w.set_gpio1(true);
        w.set_mux(true);
    });
    SYSCON.ahbclkctrl1().modify(|w| {
        w.set_fc(7, true);
    });

    // Reset Flexcomm 7
    SYSCON.presetctrl1().modify(|w| {
        w.set_fc_rst(7, syscon::vals::FcRst::ASSERTED);
    });
    SYSCON.presetctrl1().modify(|w| {
        w.set_fc_rst(7, syscon::vals::FcRst::RELEASED);
    });

    // CLK SEL
    // Select Main Clock (ENUM_0X2 is FRO 12Mhz)
    SYSCON
        .fcclksel(7)
        .modify(|w| w.set_sel(syscon::vals::FcclkselSel::ENUM_0X2));

    // Set flexcomm to SPI
    FLEXCOMM7.pselid().modify(|w| {
        w.set_persel(flexcomm::vals::Persel::SPI);
    });

    // IOCON Setup
    // All pins are configured according to the standard settings in the SPI config documentation

    // SSEL1 at func 1 iocon
    IOCON.pio1(20).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT0);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    });

    GPIO.dirset(1).write(|w| w.set_dirsetp(1 << 20));

    cs_set_high();

    // MOSI at func 7 iocon
    IOCON.pio0(20).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT7);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    });

    // MISO at func 7 iocon
    IOCON.pio0(19).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT7);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    });

    // SCKL at func 7 iocon
    IOCON.pio0(21).modify(|w| {
        w.set_func(iocon::vals::PioFunc::ALT7);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    });

    // FlexcommFRG divider (div is 0xFF by default in the documentation meaning it can be divided by either 1 or 2)
    SYSCON.flexfrgctrl(7).modify(|w| {
        w.set_div(0xFF);
        w.set_mult(0);
    });

    // SPI clock divider can go from 1 to 255
    SPI7.div().modify(|w| {
        w.set_divval(1);
    });
    // Final Clock is 12 MHz

    SPI7.fifocfg().modify(|w| {
        w.set_enablerx(false);
        w.set_enabletx(false);
    });
    // SPI Master config using ssel_1
    SPI7.cfg().modify(|w| {
        w.set_enable(false);
        w.set_master(spi::vals::Master::MASTER_MODE);
        w.set_lsbf(spi::vals::Lsbf::STANDARD);
        w.set_cpha(spi::vals::Cpha::CAPTURE); // CPHA = 0 CAPTURE = 0, CHANGE = 1
        w.set_cpol(spi::vals::Cpol::LOW); // CPOL = 0
        w.set_loop_(false);
        //w.set_spol1(spi::vals::Spol1::LOW);
    });

    
    SPI7.fifocfg().modify(|w| {
        w.set_dmatx(false);
        w.set_dmarx(false);
        w.set_enabletx(true);
        w.set_enablerx(true);
        // w.set_emptytx(true);
        // w.set_emptyrx(true);
    });

    // Finally, enable the device
    SPI7.cfg().modify(|w| w.set_enable(true));
}

fn cs_set_high() {
    GPIO.set(1).write(|w| w.set_setp(1 << 20));
}

fn cs_set_low() {
    GPIO.clr(1).write(|w| w.set_clrp(1 << 20));
}

pub fn flush() {
    while !SPI7.fifostat().read().txempty(){}
}

pub fn blocking_write(data: &[u8]) -> Result<(), Error> {
    for d in data {
            while !SPI7.fifostat().read().txnotfull(){}
            SPI7.fifowr().write(|w| {
                w.set_rxignore(spi::vals::Rxignore::IGNORE);
                w.set_txdata(*d as u16); // Data to be transferred
                w.set_len(7);
            });
    }
    flush();
    Ok(())
}

pub fn blocking_read(data: &mut [u8]) -> Result<(), Error> {
    for d in data {
        while !SPI7.fifostat().read().txnotfull(){}
        SPI7.fifowr().write(|w| {
            w.set_rxignore(spi::vals::Rxignore::READ);
            w.set_txdata(0u16); // Data to be transferred
            w.set_len(7);
        });
        while !SPI7.fifostat().read().rxnotempty(){}
        *d = SPI7.fiford().read().rxdata() as u8;
    }

    Ok(())
}

pub fn blocking_transfer(read: &mut [u8], write: &[u8]) -> Result<(), Error> {
    let len = read.len().max(write.len());
    for i in 0..len {
        let wb = write.get(i).copied().unwrap_or(0);
        while !SPI7.fifostat().read().txnotfull() {}
        SPI7.fifowr().write(|w| {
            w.set_rxignore(spi::vals::Rxignore::READ);
            w.set_txdata(wb as u16); // Data to be transferred
            w.set_len(7);
        });
        while !SPI7.fifostat().read().rxnotempty(){}
        let rb = SPI7.fiford().read().rxdata() as u8;
        if let Some(r) = read.get_mut(i) {
            *r = rb;
        }
    }
    flush();
    Ok(())
}

pub fn blocking_transfer_in_place(data: &mut [u8]) -> Result<(), Error> {
    for d in data {
        while !SPI7.fifostat().read().txnotfull() {}
        SPI7.fifowr().write(|w| {
            w.set_rxignore(spi::vals::Rxignore::READ);
            w.set_txdata(*d as u16); // Data to be transferred
            w.set_len(7);
        });
        while !SPI7.fifostat().read().rxnotempty(){}
        *d = SPI7.fiford().read().rxdata() as u8;
    }
    flush();
    Ok(())
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init();

    let data: [u8; 2] = [28, 06];

    loop {
        info!("Start of transfer");
        cs_set_low();
        for (i, d) in data.iter().enumerate() {
            SPI7.fifowr().write(|w| {
                w.set_rxignore(spi::vals::Rxignore::IGNORE);
                w.set_txdata(*d as u16); // Data to be transferred
                w.set_len(7);
            });
        }
        cs_set_high();

        let tx_level = SPI7.fifostat().read().txlvl();
        let rx_level = SPI7.fifostat().read().rxlvl();
        info!("Current TX level after transfer: {}", tx_level);
        info!("Current RX level after transfer: {}", rx_level);

        for _ in 0..1_000_000 {
            nop();
        }
    }
}
