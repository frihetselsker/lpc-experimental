#![no_std]
#![no_main]

use cortex_m::asm::nop;
use defmt::*;
use embassy_executor::Spawner;
use nxp_pac::{FLEXCOMM2, *};
use {defmt_rtt as _, panic_halt as _};


/// Serial error
#[derive(Format, Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Triggered when the FIFO (or shift-register) is overflowed.
    Overrun,
    /// Triggered when a break is received
    Break,
    /// Triggered when there is a parity mismatch between what's received and
    /// our settings.
    Parity,
    /// Triggered when the received character didn't have a valid stop bit.
    Framing,
    Noise,
}

/// Word length.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataBits {
    /// 7 bits.
    #[doc = "7 bit Data length."]
    DataBits7,
    /// 8 bits.
    #[doc = "8 bit Data length."]
    DataBits8,
    /// 9 bits.
    #[doc = "9 bit data length. The 9th bit is commonly used for addressing in multidrop mode."]
    DataBits9,
}


/// Parity bit.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Parity {
    /// No parity.
    #[doc = "No parity."]
    ParityNone,
    /// Even parity.
    #[doc = "Even parity."]
    ParityEven,
    /// Odd parity.
    #[doc = "Odd parity."]
    ParityOdd,
}

/// Stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopBits {
    #[doc = "1 stop bit."]
    STOP1,
    #[doc = "2 stop bits. This setting should only be used for asynchronous communication."]
    STOP2,
}


/// UART config.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// Baud rate.
    pub baudrate: u32,
    /// Word length.
    pub data_bits: DataBits,
    /// Stop bits.
    pub stop_bits: StopBits,
    /// Parity bit.
    pub parity: Parity,
    /// Invert the tx pin output
    pub invert_tx: bool,
    /// Invert the rx pin input
    pub invert_rx: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 5000,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
            invert_rx: false,
            invert_tx: false,
        }
    }
}

fn dma_init(){
    // Start clock for DMA
    SYSCON.ahbclkctrl0().modify(|w| w.set_dma0(true));
    // Reset DMA
    SYSCON.presetctrl0().modify(|w| w.set_dma0_rst(syscon::vals::Dma0Rst::ASSERTED));
    SYSCON.presetctrl0().modify(|w| w.set_dma0_rst(syscon::vals::Dma0Rst::RELEASED));

    //
    DMA0.ctrl().modify(|w| {
        w.set_enable(true);
        w.set_enable(false);
    });
}

fn init(config: Config) {
    info!("Initialization");
    dma_init();

    // Enable clocks (Syscon is enabled by default)
    info!("Enable clocks");
    SYSCON.ahbclkctrl0().modify(|w| w.set_iocon(true));
    SYSCON.ahbclkctrl1().modify(|w| w.set_fc(2, true));

    // Reset Flexcomm 2
    info!("Reset Flexcomm");
    SYSCON.presetctrl1().modify(|w| w.set_fc_rst(2, syscon::vals::FcRst::ASSERTED));
    SYSCON.presetctrl1().modify(|w| w.set_fc_rst(2, syscon::vals::FcRst::RELEASED));

    // Select the clock source for Flexcomm 2

    info!("Select clock");

    // Depends on the oversampling rate, for this example 16x oversampling is used
    let source_clock = match config.baudrate {
        750_001..=6_000_000 => {
            SYSCON.fcclksel(2).modify(|w| w.set_sel(syscon::vals::FcclkselSel::ENUM_0X3)); // 96 MHz
            96_000_000
        }
        1_501..=750_000 => {
            SYSCON.fcclksel(2).write(|w| w.set_sel(syscon::vals::FcclkselSel::ENUM_0X2)); // 12 MHz
            12_000_000
        }
        121..=1_500 => {
            SYSCON.fcclksel(2).write(|w| w.set_sel(syscon::vals::FcclkselSel::ENUM_0X4)); // 1 MHz
            1_000_000
        }
        _ => {
            crate::unreachable!("{} baudrate is not permitted in this mode", config.baudrate);
            // Try 32 KHz mode
        }
    };

    // The Fractional Rate Generator can be used to obtain more precise baud rates when the
    // function clock is not a good multiple of standard (or otherwise desirable) baud rates.
    // The FRG is typically set up to produce an integer multiple of the highest required baud
    // rate, or a very close approximation. The BRG is then used to obtain the actual baud rate
    // needed.
    FLEXCOMM2.pselid().modify(|w| w.set_persel(flexcomm::vals::Persel::USART));
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

    // Baud rate setup
    let mut brg_value = source_clock / (16 * config.baudrate);
    info!("BRG Value {}", brg_value);
    if brg_value >= 256 {
        brg_value = 255;
    }
    let raw_clock = source_clock / (16 * brg_value);
    info!("Raw clock {}", raw_clock);
    let mult_value: u32 = (raw_clock * 256 / config.baudrate) - 256;
    info!("Mult value {}", mult_value);
    let final_clock = raw_clock as f32 / (1.0 + mult_value as f32 / 256.0);
    info!("Final clock is {}", final_clock);
    let error: f32 = (final_clock - config.baudrate as f32) as f32 / config.baudrate as f32;
    info!("Error {}", error);

    crate::assert!(mult_value < 256);
    crate::assert!(brg_value < 256);
    info!("Flexcomm clock");
    SYSCON.flexfrgctrl(2)
        .modify(|w|{
            w.set_div(0xFF);
            w.set_mult(mult_value as u8);
        } );
    info!("Baud rate config");
    USART2
        .brg()
        .modify(|w| { 
            w.set_brgval((brg_value - 1) as u16);
        }); // Baud rate = Flexcomm Interface fucntion clock / (BRGVAL + 1)

    // The clock is divided by 16 afterwards

    info!("USART Config");
    // USART configuration part
    USART2.cfg().modify(|w| {
            // LIN break mode enable
            // Disabled. Break detect and generate is configured for normal operation.
            w.set_linmode(false);
            //CTS Enable. Determines whether CTS is used for flow control. CTS can be from the
            //input pin, or from the USART’s own RTS if loopback mode is enabled.
            // No flow control. The transmitter does not receive any automatic flow control signal.
            w.set_ctsen(false);
            // Selects synchronous or asynchronous operation.
            w.set_syncen(nxp_pac::usart::vals::Syncen::ASYNCHRONOUS_MODE);
            // Selects the clock polarity and sampling edge of received data in synchronous mode.
            w.set_clkpol(nxp_pac::usart::vals::Clkpol::RISING_EDGE);
            // Synchronous mode Master select.
            // When synchronous mode is enabled, the USART is a master.
            w.set_syncmst(nxp_pac::usart::vals::Syncmst::MASTER);
            // Selects data loopback mode
            w.set_loop_(nxp_pac::usart::vals::Loop::NORMAL);
            // Output Enable Turnaround time enable for RS-485 operation.
            // Disabled. If selected by OESEL, the Output Enable signal deasserted at the end of
            // the last stop bit of a transmission.
            w.set_oeta(false);
            // Output enable select.
            // Standard. The RTS signal is used as the standard flow control function.
            w.set_oesel(nxp_pac::usart::vals::Oesel::STANDARD);
            // Automatic address matching enable.
            // Disabled. When addressing is enabled by ADDRDET, address matching is done by
            // software. This provides the possibility of versatile addressing (e.g. respond to more
            // than one address)
            w.set_autoaddr(false);
            // Output enable polarity.
            // Low. If selected by OESEL, the output enable is active low.
            w.set_oepol(nxp_pac::usart::vals::Oepol::LOW);
        });

        // Configurations based on the config written by a user
        USART2.cfg().modify(|w| {
            w.set_datalen(match config.data_bits {
                DataBits::DataBits7 => nxp_pac::usart::vals::Datalen::BIT_7,
                DataBits::DataBits8 => nxp_pac::usart::vals::Datalen::BIT_8,
                DataBits::DataBits9 => nxp_pac::usart::vals::Datalen::BIT_9,
            });
            w.set_paritysel(match config.parity {
                Parity::ParityNone => nxp_pac::usart::vals::Paritysel::NO_PARITY,
                Parity::ParityEven => nxp_pac::usart::vals::Paritysel::EVEN_PARITY,
                Parity::ParityOdd => nxp_pac::usart::vals::Paritysel::ODD_PARITY,
            });
            w.set_stoplen(match config.stop_bits {
                StopBits::STOP1 => nxp_pac::usart::vals::Stoplen::BIT_1,
                StopBits::STOP2 => nxp_pac::usart::vals::Stoplen::BITS_2,
            });
            w.set_rxpol(match config.invert_rx {
                false => nxp_pac::usart::vals::Rxpol::STANDARD,
                true => nxp_pac::usart::vals::Rxpol::INVERTED,
            });
            w.set_txpol(match config.invert_tx {
                false => nxp_pac::usart::vals::Txpol::STANDARD,
                true => nxp_pac::usart::vals::Txpol::INVERTED,
            });
        });


    info!("Oversampling setup");
    USART2.osr().modify(|w| {w.set_osrval(0xF); }); // 16x Oversampling
                                                              // 5x Oversampling is minimal

    // By default, the oversampling rate is 16x

    // FIFO Configuration

    info!("Disabling DMA");
    USART2
        .fifocfg()
        .modify(|w| {
            w.set_dmarx(false);
            w.set_dmatx(false);
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
    USART2
        .fifocfg()
        .modify(|w| {
            w.set_enablerx(true);
            w.set_enabletx(true);
        });

    for _ in 0..200_000 {
        nop();
    }

    info!("Enable USART");
    USART2.cfg().modify(|w| {
        w.set_enable(true);
    });

    for _ in 0..200_000 {
        nop();
    }

    // critical_section::with(|_cs| {
    // blocking_write(&[2u8, 3u8, 4u8]).unwrap();
    // });

    // usart.fifocfg.modify(|_, w| {
    //     w.emptytx()
    //         .set_bit()
    //         .emptyrx()
    //         .set_bit()
    //         .emptytx()
    //         .clear_bit()
    //         .emptyrx()
    //         .clear_bit()
    // });

    // FIFO Interrupts

        USART2.fifotrig().modify(|w| {
            w.set_txlvl(3);
            w.set_txlvlena(true);
            w.set_rxlvl(3);
            w.set_rxlvlena(true);
        });
        USART2.fifointenset().modify(|w| {
            w.set_rxerr(true);
            w.set_txerr(true);
            w.set_txlvl(false);
            w.set_rxlvl(false);
        });

}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init(Config::default());

    info!("Inside the main function");

    let letter = b'a';

    loop {
        info!("Started sending");
        for i in 0..5 {
            info!("Letter {}", i);
            USART2
                .fifowr()
                .write(|w| { 
                    w.set_txdata(letter as u16);
                });
            for _ in 0..500_000 {
                nop();
            }
        }
        // usart.fifocfg.modify(|_, w| w.emptyrx().set_bit());

        info!("Finished sending");
        for _ in 0..200_000 {
            nop();
        }

        info!("Started reading");
        for i in 0..5 {
            info!("Letter {}", i);
            info!("{}", USART2.fiford().read().rxdata());
            for _ in 0..500_000 {
                nop();
            }
        }
        info!("Finished reading");
        for _ in 0..1_000_000 {
            nop();
        }
    }
}
