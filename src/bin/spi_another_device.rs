#![no_std]
#![no_main]

use core::any::Any;

use cortex_m::asm::nop;
use defmt::*;
use embassy_executor::Spawner;
use nxp_pac::*;
use {defmt_rtt as _, panic_halt as _};

const REG_ADDRESS: u8 = 0x75;

fn init() {
    info!("Init");

    // SPI 7 at FLEXCOMM 7 Setup

    // Enable iocon and flexcomm
    // Enable GPIO as well for testing a simple CS
    SYSCON.ahbclkctrl0().modify(|w| {
        w.set_iocon(true);
        w.set_gpio1(true);
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
        w.set_func(iocon::vals::PioFunc::ALT1);
        w.set_digimode(iocon::vals::PioDigimode::DIGITAL);
        w.set_slew(iocon::vals::PioSlew::STANDARD);
        w.set_mode(iocon::vals::PioMode::INACTIVE);
        w.set_invert(false);
        w.set_od(iocon::vals::PioOd::NORMAL);
    });

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
        w.set_divval(0);
    });
    // Final Clock is 12 MHz

    // SPI Master config using ssel_1
    SPI7.cfg().modify(|w| {
        w.set_master(spi::vals::Master::MASTER_MODE);
        w.set_lsbf(spi::vals::Lsbf::STANDARD);
        w.set_cpha(spi::vals::Cpha::CHANGE);
        w.set_cpol(spi::vals::Cpol::LOW);
        w.set_loop_(false);
        w.set_spol1(spi::vals::Spol1::HIGH);
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

// pub fn blocking_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
//         let p = self.inner.regs();
//         let len = read.len().max(write.len());
//         for i in 0..len {
//             let wb = write.get(i).copied().unwrap_or(0);
//             while !p.sr().read().tnf() {}
//             p.dr().write(|w| w.set_data(wb as _));
//             while !p.sr().read().rne() {}
//             let rb = p.dr().read().data() as u8;
//             if let Some(r) = read.get_mut(i) {
//                 *r = rb;
//             }
//         }
//         self.flush()?;
//         Ok(())
//     }

pub fn blocking_transfer(read: &mut [u8], write: &[u8]) {
    let len = read.len().max(write.len());
    let mut counter: u8 = 0;
    for i in 0..len {
        let wb = write.get(i).copied().unwrap_or(0);
        SPI7.fifowr().write(|w| {
            w.set_txdata(wb as u16); // Data to be transferred
            w.set_len(0x7); // 8bit length
            w.set_txssel1_n(spi::vals::Txssel1N::ASSERTED);
            if counter == len as u8 - 1 {
                w.set_eot(true);
                // GPIO.set(1).write(|w| w.set_setp(1 << 20));
            }
        });
        while !SPI7.fifostat().read().rxnotempty() {}
        let rb = SPI7.fiford().read().rxdata();
        info!("Data read: {}", rb);
        if let Some(r) = read.get_mut(i) {
            *r = rb as u8;
        }
        counter += 1;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init();
    // Set up the GPIO pin to be active high
    // GPIO.set(1).write(|w| w.set_setp(1 << 20));
    // let mut data = [0u16; 8];
    let tx_data = [(1 << 7) | REG_ADDRESS, 0x00];
    let mut rx_data = [0u8; 2];
    loop {
        // Assert the SSEL you are transferring data to
        // SPI7.fifowr().write(|w| {
        //     w.set_txssel1_n(spi::vals::Txssel1N::ASSERTED);
        //     w.set_len(0xF); // !! IMPORTANT !! If length isn't specified data won't be shifted out
        // });

        info!("Start of transfer");
        // info!("Is slave asserted? {}", SPI7.stat().read().ssa());
        // GPIO.clr(1).write(|w| w.set_clrp(1 << 20));

        blocking_transfer(&mut rx_data, &tx_data);

        info!("The array content is {}", rx_data);

        for _ in 0..1_000_000 {
            nop();
        }
    }
}
