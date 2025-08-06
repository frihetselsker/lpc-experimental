#![no_std]
#![no_main]

use core::any::Any;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use cortex_m::asm::nop;
use cortex_m::peripheral;
use defmt::*;
use embassy_executor::Spawner;
use embassy_hal_internal::atomic_ring_buffer::RingBuffer;
use embassy_hal_internal::interrupt::{InterruptExt, Priority};
use embassy_sync::waitqueue::AtomicWaker;
use lpc55_pac::{interrupt, FLEXCOMM2, IOCON, SYSCON, USART2};
use portable_atomic::AtomicU8;
use {defmt_rtt as _, panic_halt as _};

static COUNTER: AtomicU8 = AtomicU8::new(0);

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

impl DataBits {
    fn bits(&self) -> u8 {
        match self {
            Self::DataBits7 => 0b00,
            Self::DataBits8 => 0b01,
            Self::DataBits9 => 0b10,
        }
    }
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

impl Parity {
    fn bits(&self) -> u8 {
        match self {
            Self::ParityNone => 0b00,
            Self::ParityEven => 0b10,
            Self::ParityOdd => 0b11,
        }
    }
}

/// Stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopBits {
    #[doc = "1 stop bit."]
    STOP1,
    #[doc = "2 stop bits. This setting should only be used for asynchronous communication."]
    STOP2,
}

impl StopBits {
    fn bits(&self) -> bool {
        return match self {
            Self::STOP1 => false,
            Self::STOP2 => true,
        };
    }
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
            baudrate: 9600,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
            invert_rx: false,
            invert_tx: false,
        }
    }
}

pub struct State {
    tx_waker: AtomicWaker,
    tx_buf: RingBuffer,
    rx_waker: AtomicWaker,
    rx_buf: RingBuffer,
    rx_error: AtomicU8,
}

impl State {
    pub const fn new() -> Self {
        Self {
            rx_buf: RingBuffer::new(),
            tx_buf: RingBuffer::new(),
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            rx_error: AtomicU8::new(0),
        }
    }
}

pub fn init_buffers<'d>(
    state: &State,
    tx_buffer: Option<&'d mut [u8]>,
    rx_buffer: Option<&'d mut [u8]>,
) {
    if let Some(tx_buffer) = tx_buffer {
        let len = tx_buffer.len();
        unsafe { state.tx_buf.init(tx_buffer.as_mut_ptr(), len) };
    }

    if let Some(rx_buffer) = rx_buffer {
        let len = rx_buffer.len();
        unsafe { state.rx_buf.init(rx_buffer.as_mut_ptr(), len) };
    }
}

fn init(config: Config) {
    info!("Initialization");

    // Pointers to the registers used

    let syscon = unsafe { &*SYSCON::ptr() };
    let flexcomm = unsafe { &*FLEXCOMM2::ptr() };
    let iocon = unsafe { &*IOCON::ptr() };
    let usart = unsafe { &*USART2::ptr() };

    // Enable clocks (Syscon is enabled by default)
    info!("Enable clocks");
    syscon.ahbclkctrl0.modify(|_, w| w.iocon().enable());
    syscon.ahbclkctrl1.modify(|_, w| w.fc2().enable());

    // Reset Flexcomm 2
    info!("Reset Flexcomm");
    syscon.presetctrl1.modify(|_, w| w.fc2_rst().set_bit());
    syscon.presetctrl1.modify(|_, w| w.fc2_rst().clear_bit());

    // Select the clock source for Flexcomm 2

    info!("Select clock");

    // Depends on the oversampling rate, for this example 16x oversampling is used
    let source_clock = match config.baudrate {
        750_001..=6_000_000 => {
            syscon.fcclksel2().write(|w| w.sel().enum_0x3()); // 96 MHz
            96_000_000
        }
        1_501..=750_000 => {
            syscon.fcclksel2().write(|w| w.sel().enum_0x2()); // 12 MHz
            12_000_000
        }
        121..=1_500 => {
            syscon.fcclksel2().write(|w| w.sel().enum_0x4()); // 1 MHz
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
    flexcomm.pselid.modify(|_, w| w.persel().usart());
    //IOCON Setup
    info!("IOCON Setup");
    iocon.pio1_24.modify(|_, w| {
        w.func()
            .alt1()
            .digimode()
            .digital()
            .slew()
            .standard()
            .mode()
            .inactive()
            .invert()
            .disabled()
            .od()
            .normal()
    }); // rx
    iocon.pio0_27.modify(|_, w| {
        w.func()
            .alt1()
            .digimode()
            .digital()
            .slew()
            .standard()
            .mode()
            .inactive()
            .invert()
            .disabled()
            .od()
            .normal()
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
    syscon
        .flexfrg2ctrl()
        .modify(|_, w| unsafe { w.div().bits(0xFF).mult().bits(mult_value as u8) });

    info!("Baud rate config");
    usart
        .brg
        .modify(|_, w| unsafe { w.brgval().bits((brg_value - 1) as u16) }); // Baud rate = Flexcomm Interface fucntion clock / (BRGVAL + 1)

    // The clock is divided by 16 afterwards

    info!("USART Config");
    // USART configuration part
    usart.cfg.modify(|_, w| {
        w.linmode()
            .disabled()
            .ctsen()
            .disabled()
            .syncen()
            .asynchronous_mode()
            .clkpol()
            .rising_edge()
            .syncmst()
            .master()
            .loop_()
            .normal()
            .oeta()
            .disabled()
            .oesel()
            .standard()
            .autoaddr()
            .disabled()
            .oepol()
            .low()
    });

    // Based on configuration
    usart.cfg.modify(|_, w| unsafe {
        w.datalen()
            .bits(config.data_bits.bits())
            .paritysel()
            .bits(config.parity.bits())
            .stoplen()
            .bit(config.stop_bits.bits())
            .rxpol()
            .bit(config.invert_rx)
            .txpol()
            .bit(config.invert_tx)
    });

    info!("Oversampling setup");
    usart.osr.modify(|_, w| unsafe { w.osrval().bits(0xF) }); // 16x Oversampling
                                                              // 5x Oversampling is minimal

    // By default, the oversampling rate is 16x

    // FIFO Configuration

    info!("Disabling DMA");
    usart
        .fifocfg
        .modify(|_, w| w.dmatx().disabled().dmarx().disabled());

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
    usart
        .fifocfg
        .modify(|_, w| w.enabletx().enabled().enablerx().enabled());

    for _ in 0..200_000 {
        nop();
    }

    info!("Enable USART");
    usart.cfg.modify(|_, w| w.enable().enabled());

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
    critical_section::with(|_cs| {
        usart.fifotrig.modify(|_, w| unsafe {
            w.txlvl()
                .bits(1)
                .txlvlena()
                .disabled()
                .rxlvl()
                .bits(1)
                .rxlvlena()
                .disabled()
        });
        usart.fifointenset.modify(|_, w| {
            w.txerr()
                .enabled()
                .rxerr()
                .enabled()
                .txlvl()
                .disabled()
                .rxlvl()
                .enabled()
        });
        // Enable interrupts

        unsafe {
            interrupt::FLEXCOMM2.set_priority(Priority::from(3));
            interrupt::FLEXCOMM2.enable();
        }
    });
}

/// Transmit the provided buffer blocking execution until done.
fn blocking_write(buffer: &[u8]) -> Result<(), Error> {
    let usart = unsafe { &*USART2::ptr() };
    // usart.fifointenset.modify(|_, w| w.txlvl().enabled());
    for &b in buffer {
        while usart.fifostat.read().txnotfull().bit_is_clear() {}
        usart
            .fifowr
            .modify(|_, w| unsafe { w.txdata().bits(b as u16) });
        let data = usart.fifostat.read().txlvl().bits();
        info!("TX FIFO: {}", data);
    }
    // usart.fifointenclr.modify(|_, w| w.txlvl().set_bit().txlvl().clear_bit());
    Ok(())
}

/// Flush UART TX blocking execution until done.
fn blocking_flush() -> Result<(), Error> {
    let usart = unsafe { &*USART2::ptr() };
    while usart.fifostat.read().txempty().bit_is_clear() {}
    Ok(())
}

/// Check if UART is busy transmitting.
fn busy() -> bool {
    let usart = unsafe { &*USART2::ptr() };
    usart.fifostat.read().txempty().bit_is_clear()
}

/// Read from UART RX blocking execution until done.

fn blocking_read(mut buffer: &mut [u8]) -> Result<(), Error> {
    let usart = unsafe { &*USART2::ptr() };
    usart.fifotrig.modify(|_, w| w.rxlvlena().enabled());
    while !buffer.is_empty() {
        match drain_fifo(buffer) {
            Ok(0) => continue, // Wait for more data
            Ok(n) => buffer = &mut buffer[n..],
            Err((_, err)) => return Err(err),
        }
    }
    usart.fifotrig.modify(|_, w| w.rxlvlena().disabled());
    Ok(())
}
/// Returns Ok(len) if no errors occurred. Returns Err((len, err)) if an error was
/// encountered. in both cases, `len` is the number of *good* bytes copied into
/// `buffer`.
fn drain_fifo(buffer: &mut [u8]) -> Result<usize, (usize, Error)> {
    let usart = unsafe { &*USART2::ptr() };
    for (i, b) in buffer.iter_mut().enumerate() {
        let data = usart.fifostat.read().rxlvl().bits();
        info!("RX FIFO: {}", data);
        while usart.fifostat.read().rxnotempty().bit_is_clear() {}

        if usart.fifostat.read().rxerr().bit_is_set() {
            return Err((i, Error::Overrun));
        } else if usart.fifordnopop.read().parityerr().bit_is_set() {
            return Err((i, Error::Parity));
        } else if usart.fifordnopop.read().framerr().bit_is_set() {
            return Err((i, Error::Framing));
        } else if usart.fifordnopop.read().rxnoise().bit_is_set() {
            return Err((i, Error::Noise));
        }

        let dr = usart.fiford.read().bits() as u8;
        *b = dr;
    }
    Ok(buffer.len())
}

#[cortex_m_rt::interrupt]
fn FLEXCOMM2() {
    COUNTER.add(1, core::sync::atomic::Ordering::Relaxed);
    let usart = unsafe { &*USART2::ptr() };
    let tx_level = usart.fifostat.read().txlvl().bits();
    let rx_level = usart.fifostat.read().rxlvl().bits();

    let tx_error = usart.fifostat.read().txerr().bit_is_set();
    let rx_error = usart.fifostat.read().rxerr().bit_is_set();
    let peripheral_error = usart.fifostat.read().perint().bit_is_set();

    let tx_empty = usart.fifostat.read().txempty().bit_is_set();
    let tx_not_full = usart.fifostat.read().txnotfull().bit_is_set();
    let rx_not_empty = usart.fifostat.read().rxnotempty().bit_is_set();
    let rx_full = usart.fifostat.read().rxfull().bit_is_set();

    warn!("On interrupt");
    warn!(
        "COUNTER: {}",
        COUNTER.load(core::sync::atomic::Ordering::Relaxed)
    );

    info!("TX FIFO {}", tx_level);
    info!("RX FIFO {}", rx_level);

    info!("Errors: ");
    info!("TX Error raised: {}", tx_error);
    info!("RX Error raised: {}", rx_error);
    info!("Peripheral error raised: {}", peripheral_error);

    info!("Other flags:");
    info!("TX Empty raised: {}", tx_empty);
    info!("TX not full raised: {}", tx_not_full);
    info!("RX not empty raised: {}", rx_not_empty);
    info!("RX full raised: {}", rx_full);

    usart.fifointenclr.modify(|_, w| w.rxlvl().set_bit());

    for _ in 0..1_000_000 {
        nop();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init(Config::default());
    info!("Inside the main function");
    loop {
        let tx: &[u8; 10] = b"hello,geo!";
        match blocking_write(tx) {
            Ok(_) => {
                info!("The data was sent successfully");
            }
            _ => {
                crate::panic!("Error at TX");
            }
        }

        for _ in 0..250_000 {
            nop();
        }

        blocking_flush().unwrap();

        let mut rx = [0u8; 10];
        match blocking_read(&mut rx) {
            Ok(_) => info!("The data was read successully"),
            Err(e) => info!("Error {:?}", e),
        }
        match core::str::from_utf8_mut(&mut rx) {
            Ok(s) => info!("The message: {}", s),
            Err(_e) => info!("UTF8 Error"),
        }

        info!("Inside the loop");

        for _ in 0..250_000 {
            nop();
        }
    }
}
