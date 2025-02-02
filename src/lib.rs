#![no_std]
#![doc = include_str!("../README.md")]

use core::{
    future::Future,
    ptr::NonNull,
    sync::atomic::{fence, Ordering},
};

use bitflags::bitflags;
use embedded_io::ErrorType;
use spin_on::spin_on;
use tock_registers::{
    fields::FieldValue, interfaces::*, register_bitfields, register_structs, registers::*,
};

#[derive(Debug)]
pub enum ErrorKind {
    Framing,
    Parity,
    Break,
    Overrun,
}

register_bitfields! [
    // First parameter is the register width. Can be u8, u16, u32, or u64.
    u32,

    Data [
        DATA  OFFSET(0) NUMBITS(8) [],
        FRAMING_ERROR    OFFSET(8) NUMBITS(1) [],
        PARITY_ERROR OFFSET(9) NUMBITS(1) [],
        BREAK_ERROR OFFSET(10) NUMBITS(1) [],
        OVERRUN_ERROR OFFSET(11) NUMBITS(1) [],
    ],

    Fr [
        CLEAR_TO_SEND OFFSET(0) NUMBITS(1) [],
        DATA_SET_READY OFFSET(1) NUMBITS(1) [],
        DATA_CARRIER_DETECT OFFSET(2) NUMBITS(1) [],
        BUSY OFFSET(3) NUMBITS(1) [],
        RX_FIFO_EMPTY OFFSET(4) NUMBITS(1) [],
        TX_FIFO_FULL OFFSET(5) NUMBITS(1) [],
        RX_FIFO_FULL OFFSET(6) NUMBITS(1) [],
        TX_FIFO_EMPTY OFFSET(7) NUMBITS(1) [],
    ],

    LineControlRegister [
        SEND_BREAK OFFSET(0) NUMBITS(1) [],
        PARITY_ENABLE OFFSET(1) NUMBITS(1) [],
        EVEN_PARITY_SELECT OFFSET(2) NUMBITS(1) [
            Odd = 0,
            Even = 1,
        ],
        TWO_STOP_BITS_SELECT OFFSET(3) NUMBITS(1) [],
        ENABLE_FIFO OFFSET(4) NUMBITS(1) [],
        WORD_LEN OFFSET(5) NUMBITS(2) [
            B5 = 0b00,
            B6 = 0b01,
            B7 = 0b10,
            B8 = 0b11,
        ],
        STICK_PARITY_SELECT OFFSET(7) NUMBITS(1)[],
    ],

    ControlRegister [
        ENABLE OFFSET(0) NUMBITS(1) [],
        SIR_ENABLE OFFSET(1) NUMBITS(1) [],
        SIR_LP OFFSET(2) NUMBITS(1) [],
        LOOPBACK_ENABLE OFFSET(7) NUMBITS(1) [],
        TX_ENABLE OFFSET(8) NUMBITS(1) [],
        RX_ENABLE OFFSET(9) NUMBITS(1) [],
        DATA_TRANSMIT_READY OFFSET(10) NUMBITS(1) [],
        REQUEST_TO_SEND OFFSET(11) NUMBITS(1) [],
        OUT1 OFFSET(12) NUMBITS(1) [],
        OUT2 OFFSET(13) NUMBITS(1) [],
        RTS_ENABLE OFFSET(14) NUMBITS(1) [],
        CTS_ENABLE OFFSET(15) NUMBITS(1) [],
    ],
    IFLS [
        TXIFLSEL OFFSET(0) NUMBITS(2) [],
        RXIFLSEL OFFSET(3) NUMBITS(2) [],
    ],
];

register_structs! {
    Registers {
        (0x00 => dr: ReadWrite<u32, Data::Register>),
        (0x04 => _reserved0),
        /// Flag Register.
        (0x18 => fr: ReadOnly<u32, Fr::Register>),
        (0x1c => _reserved1),
        (0x24 => ibrd: ReadWrite<u32>),
        (0x28 => fbrd: ReadWrite<u32>),
        (0x2c => lcr_h: ReadWrite<u32, LineControlRegister::Register>),
        /// Control register.
        (0x30 => cr: ReadWrite<u32, ControlRegister::Register>),
        /// Interrupt FIFO Level Select Register.
        (0x34 => ifls: ReadWrite<u32, IFLS::Register>),
        /// Interrupt Mask Set Clear Register.
        (0x38 => imsc: ReadWrite<u32>),
        /// Raw Interrupt Status Register.
        (0x3c => ris: ReadOnly<u32>),
        /// Masked Interrupt Status Register.
        (0x40 => mis: ReadOnly<u32>),
        /// Interrupt Clear Register.
        (0x44 => icr: WriteOnly<u32>),
        (0x48 => @END),
    }
}

/// Word length.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataBits {
    Bits5,
    Bits6,
    Bits7,
    Bits8,
}

impl From<DataBits> for FieldValue<u32, LineControlRegister::Register> {
    fn from(val: DataBits) -> Self {
        match val {
            DataBits::Bits5 => LineControlRegister::WORD_LEN::B5,
            DataBits::Bits6 => LineControlRegister::WORD_LEN::B6,
            DataBits::Bits7 => LineControlRegister::WORD_LEN::B7,
            DataBits::Bits8 => LineControlRegister::WORD_LEN::B8,
        }
    }
}

/// Parity bit.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Parity {
    None,
    Even,
    Odd,
}

/// Stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1,
    #[doc = "2 stop bits"]
    STOP2,
}

pub struct Config {
    pub baud_rate: u32,
    pub clock_freq: u64,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
}

pub struct Pl011 {
    base: usize,
}

impl Pl011 {
    pub async fn new(base: NonNull<u8>, config: Option<Config>) -> Self {
        let mut s = Self {
            base: base.as_ptr() as usize,
        };
        s.set_config(config).await;
        s
    }

    pub fn new_sync(base: NonNull<u8>, config: Option<Config>) -> Self {
        spin_on(Self::new(base, config))
    }

    async fn flush_all(&mut self) {
        self.wait_expect(|reg| reg.fr.matches_all(Fr::TX_FIFO_EMPTY::SET + Fr::BUSY::CLEAR))
            .await;
    }

    pub async fn set_config(&mut self, config: Option<Config>) {
        self.flush_all().await;

        // 1. Disable the UART.
        self.reg().cr.modify(ControlRegister::ENABLE::CLEAR);

        // 2. Wait for the end of transmission or reception of the current character.

        // 3. Flush the transmit FIFO by setting the FEN bit to 0 in the Line Control Register, UARTLCR_H.
        self.reg()
            .lcr_h
            .modify(LineControlRegister::ENABLE_FIFO::CLEAR);

        fence(Ordering::Release);

        if let Some(config) = config {
            let pen;
            let eps;
            let sps;

            match config.parity {
                Parity::None => {
                    pen = LineControlRegister::PARITY_ENABLE::CLEAR;
                    eps = LineControlRegister::EVEN_PARITY_SELECT::CLEAR;
                    sps = LineControlRegister::STICK_PARITY_SELECT::CLEAR;
                }
                Parity::Even => {
                    pen = LineControlRegister::PARITY_ENABLE::SET;
                    eps = LineControlRegister::EVEN_PARITY_SELECT::SET;
                    sps = LineControlRegister::STICK_PARITY_SELECT::CLEAR;
                }
                Parity::Odd => {
                    pen = LineControlRegister::PARITY_ENABLE::SET;
                    eps = LineControlRegister::EVEN_PARITY_SELECT::CLEAR;
                    sps = LineControlRegister::STICK_PARITY_SELECT::CLEAR;
                }
            }
            let word_len = config.data_bits.into();
            let stp2 = match config.stop_bits {
                StopBits::STOP1 => LineControlRegister::TWO_STOP_BITS_SELECT::CLEAR,
                StopBits::STOP2 => LineControlRegister::TWO_STOP_BITS_SELECT::SET,
            };

            let baud_rate_div = (8 * config.clock_freq) / config.baud_rate as u64;
            let mut baud_ibrd = baud_rate_div >> 7;
            let mut baud_fbrd = ((baud_rate_div & 0x7f) + 1) / 2;

            if baud_ibrd == 0 {
                baud_ibrd = 1;
                baud_fbrd = 0;
            } else if baud_ibrd >= 65535 {
                baud_ibrd = 65535;
                baud_fbrd = 0;
            }

            self.reg().ibrd.set(baud_ibrd as u32);
            self.reg().fbrd.set(baud_fbrd as u32);

            self.reg()
                .lcr_h
                .write(LineControlRegister::ENABLE_FIFO::SET + word_len + pen + eps + sps + stp2);
        } else {
            self.reg()
                .lcr_h
                .modify(LineControlRegister::ENABLE_FIFO::SET);
        }

        self.reg().cr.write(
            ControlRegister::ENABLE::SET
                + ControlRegister::TX_ENABLE::SET
                + ControlRegister::RX_ENABLE::SET,
        );
    }
    pub fn set_config_sync(&mut self, config: Option<Config>) {
        spin_on(self.set_config(config));
    }

    async fn read(&self) -> Result<u8, ErrorKind> {
        self.wait_expect(|reg| reg.fr.read(Fr::RX_FIFO_EMPTY) == 0)
            .await;

        let val = self.reg().dr.get();
        let b = Data::DATA.read(val) as u8;

        if Data::FRAMING_ERROR.is_set(val) {
            return Err(ErrorKind::Framing);
        }
        if Data::PARITY_ERROR.is_set(val) {
            return Err(ErrorKind::Parity);
        }
        if Data::OVERRUN_ERROR.is_set(val) {
            return Err(ErrorKind::Overrun);
        }
        if Data::BREAK_ERROR.is_set(val) {
            return Err(ErrorKind::Break);
        }

        Ok(b)
    }

    async fn write(&self, data: u8) {
        self.wait_expect(|reg| reg.fr.read(Fr::TX_FIFO_FULL) == 0)
            .await;

        self.reg().dr.write(Data::DATA.val(data as u32));
    }

    fn reg(&self) -> &Registers {
        unsafe { &*(self.base as *const Registers) }
    }

    async fn wait_expect<F>(&self, expect: F)
    where
        F: Fn(&Registers) -> bool,
    {
        WaitExpectFuture {
            base: self.base,
            expect,
        }
        .await;
    }

    pub fn set_tx_fifo_threadhold(&mut self, threshold: Threshold) {
        self.reg().ifls.modify(IFLS::TXIFLSEL.val(threshold as _));
    }

    pub fn set_rx_fifo_threadhold(&mut self, threshold: Threshold) {
        self.reg().ifls.modify(IFLS::RXIFLSEL.val(threshold as _));
    }

    pub fn write_imsc(&mut self, mask: IMSC) {
        self.reg().imsc.set(mask.bits());
    }

    pub fn read_ris(&self) -> RIS {
        RIS::from_bits_truncate(self.reg().ris.get())
    }
    pub fn read_mis(&self) -> MIS {
        MIS::from_bits_truncate(self.reg().mis.get())
    }

    pub fn write_icr(&mut self, mask: RIS) {
        self.reg().icr.set(mask.bits());
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct IMSC: u32 {
        /// nUARTRI 调制解调中断屏蔽
        const RIMIM = 1 << 0;
        /// nUARTCTS 调制解调中断屏蔽
        const CTSMIM = 1 << 1;
        /// nUARTDCD 调制解调中断屏蔽
        const CDCDMIM = 1 << 2;
        const DSRMIM = 1 << 3;
        /// 接收中断屏蔽
        const RXIM = 1 << 4;
        /// 发送中断屏蔽
        const TXIM = 1 << 5;
        /// 接收超时中断屏蔽
        const RTIM = 1 << 6;
        const FEIM = 1 << 7;
        const PEIM = 1 << 8;
        const BEIM = 1 << 9;
        const OEIM = 1 << 10;
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct RIS: u32 {
        const RIRMIS = 1 << 0;
        const CTSRMIS = 1 << 1;
        const DCDRMIS = 1 << 2;
        const DSRRMIS = 1 << 3;
        const RXRIS = 1 << 4;
        const TXRIS = 1 << 5;
        const RTRIS = 1 << 6;
        const FERIS = 1 << 7;
        const PERIS = 1 << 8;
        const BERIS = 1 << 9;
        const OERIS = 1 << 10;
    }
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct MIS: u32 {
        const RIMMIS = 1 << 0;
        const CTSMMIS = 1 << 1;
        const DCDMMIS = 1 << 2;
        const DSRMMIS = 1 << 3;
        const RXMIS = 1 << 4;
        const TXMIS = 1 << 5;
        const RTMIS = 1 << 6;
        const FEMIS = 1 << 7;
        const PEMIS = 1 << 8;
        const BEMIS = 1 << 9;
        const OEMIS = 1 << 10;
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct ICR: u32 {
        const RIMIC = 1 << 0;
        const CTSMIC = 1 << 1;
        const DCDMIC = 1 << 2;
        const DSRMIC = 1 << 3;
        const RXIC = 1 << 4;
        const TXIC = 1 << 5;
        const RTIC = 1 << 6;
        const FEIC = 1 << 7;
        const PEIC = 1 << 8;
        const BEIC = 1 << 9;
        const OEIC = 1 << 10;
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Threshold {
    /// FIFO ≤ 1/8 full
    Level1 = 000,
    /// FIFO ≤ 1/4 full
    Level2 = 0b001,
    /// FIFO ≤ 1/2 full
    Level3 = 0b010,
    /// FIFO ≤ 3/4 full
    Level4 = 0b011,
    /// FIFO ≤ 7/8 full
    Level5 = 0b100,
}

struct WaitExpectFuture<F>
where
    F: Fn(&Registers) -> bool,
{
    base: usize,
    expect: F,
}

impl<F> Future for WaitExpectFuture<F>
where
    F: Fn(&Registers) -> bool,
{
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let reg = unsafe { &*(self.base as *const Registers) };
        let expect = (self.expect)(reg);
        if expect {
            core::task::Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            core::task::Poll::Pending
        }
    }
}

impl embedded_io::Error for ErrorKind {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            ErrorKind::Framing => embedded_io::ErrorKind::InvalidData,
            ErrorKind::Parity => embedded_io::ErrorKind::InvalidData,
            ErrorKind::Break => embedded_io::ErrorKind::InvalidData,
            ErrorKind::Overrun => embedded_io::ErrorKind::Other,
        }
    }
}

impl ErrorType for Pl011 {
    type Error = ErrorKind;
}

impl embedded_io::Write for Pl011 {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        spin_on(Pl011::write(self, buf[0]));
        Ok(1)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
impl embedded_io::Read for Pl011 {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let b = spin_on(Pl011::read(self))?;
        buf[0] = b;
        Ok(1)
    }
}

impl embedded_io_async::Write for Pl011 {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Pl011::write(self, buf[0]).await;
        Ok(1)
    }
}
impl embedded_io_async::Read for Pl011 {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let b = Pl011::read(self).await?;
        buf[0] = b;
        Ok(1)
    }
}
