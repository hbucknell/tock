//! Implementation of the STM32L4 USART peripheral.
//!
//! Supports UART features only

use core::cell::Cell;
use kernel::common::cells::{OptionalCell, TakeCell};
use kernel::common::registers::{register_bitfields, ReadOnly, ReadWrite, WriteOnly};
use kernel::common::StaticRef;
use kernel::hil;
use kernel::ClockInterface;
use kernel::ReturnCode;

//use crate::dma1;
//use crate::dma1::Dma1Peripheral;
use crate::memory_map;
use crate::rcc;

/// Universal synchronous asynchronous receiver transmitter
#[repr(C)]
struct UsartRegisters {
    /// Control register 1
    cr1: ReadWrite<u32, CR1::Register>,
    /// Control register 2
    cr2: ReadWrite<u32, CR2::Register>,
    /// Control register 3
    cr3: ReadWrite<u32, CR3::Register>,
    /// Baud rate register
    brr: ReadWrite<u32, BRR::Register>,
    /// Guard time and prescaler register
    gtpr: ReadWrite<u32, GTPR::Register>,
    /// Receiver Time Out register
    rtor: ReadWrite<u32, RTOR::Register>,
    /// USART Request register
    rqr: WriteOnly<u32, RQR::Register>,
    /// Interrupt and Status register
    isr: ReadOnly<u32, ISR::Register>,
    /// Interrupt flag clear register
    icr: ReadWrite<u32, ICR::Register>,
    /// Receive data register
    rdr: ReadOnly<u32, RDR::Register>,
    /// Transmit data register
    tdr: ReadWrite<u32, TDR::Register>,
}

register_bitfields![u32,
    CR1 [
        /// Word length - Bit 1
        M1 OFFSET(27) NUMBITS(1) [],
        /// End of Block interrupt enable
        EOBIE OFFSET(26) NUMBITS(1) [],
        /// Receive Time Out interrupt enable
        RTOIE OFFSET(26) NUMBITS(1) [],
        /// Driver Enable Assertion Time
        DEAT OFFSET(21) NUMBITS(5) [],
        /// Driver Enable Deassertion Time
        DEDT OFFSET(16) NUMBITS(5) [],
        /// Oversampling by 8-bit or 16-bit mode
        OVER8 OFFSET(15) NUMBITS(1) [],
        /// Character match interrupt enable
        CMIE OFFSET(14) NUMBITS(1) [],
        /// Mute Mode Enable
        MME OFFSET(13) NUMBITS(1) [],
        /// Word length - Bit 0
        M0 OFFSET(12) NUMBITS(1) [],
        /// Wakeup method
        WAKE OFFSET(11) NUMBITS(1) [],
        /// Parity control enable
        PCE OFFSET(10) NUMBITS(1) [],
        /// Parity selection
        PS OFFSET(9) NUMBITS(1) [],
        /// PE interrupt enable
        PEIE OFFSET(8) NUMBITS(1) [],
        /// TXE interrupt enable
        TXEIE OFFSET(7) NUMBITS(1) [],
        /// Transmission complete interrupt enable
        TCIE OFFSET(6) NUMBITS(1) [],
        /// RXNE interrupt enable
        RXNEIE OFFSET(5) NUMBITS(1) [],
        /// IDLE interrupt enable
        IDLEIE OFFSET(4) NUMBITS(1) [],
        /// Transmitter enable
        TE OFFSET(3) NUMBITS(1) [],
        /// Receiver enable
        RE OFFSET(2) NUMBITS(1) [],
        /// USART Enable in STOP Mode
        UESM OFFSET(1) NUMBITS(1) [],
        /// USART Enable
        UE OFFSET(0) NUMBITS(1) []
    ],
    CR2 [
        /// Address of the USART node
        ADD OFFSET(24) NUMBITS(8) [],
        /// Receiver Time-Out enable
        RTOEN OFFSET(23) NUMBITS(1) [],
        /// Auto Baud-Rate Mode
        ABRMOD OFFSET(21) NUMBITS(2) [],
        /// Auto Baud-Rate Enable
        ABREN OFFSET(20) NUMBITS(1) [],
        /// Most Significant Bit First
        MSBFIRST OFFSET(19) NUMBITS(1) [],
        /// Binary data inversion
        DATAINV OFFSET(18) NUMBITS(1) [],
        /// RX pin active level inversion
        TXINV OFFSET(17) NUMBITS(1) [],
        /// RX pin active level inversion
        RXINV OFFSET(16) NUMBITS(1) [],
        /// SWAP TX/RX pins
        SWAP OFFSET(15) NUMBITS(1) [],
        /// LIN mode enable
        LINEN OFFSET(14) NUMBITS(1) [],
        /// STOP bits
        STOP OFFSET(12) NUMBITS(2) [],
        /// Clock enable
        CLKEN OFFSET(11) NUMBITS(1) [],
        /// Clock polarity
        CPOL OFFSET(10) NUMBITS(1) [],
        /// Clock phase
        CPHA OFFSET(9) NUMBITS(1) [],
        /// Last bit clock pulse
        LBCL OFFSET(8) NUMBITS(1) [],
        /// LIN break detection interrupt enable
        LBDIE OFFSET(6) NUMBITS(1) [],
        /// LIN break detection length
        LBDL OFFSET(5) NUMBITS(1) [],
        /// 7-bit or 4-bit Address Detection
        ADDM7 OFFSET(4) NUMBITS(1) []
    ],
    CR3 [
        /// Transmission complete before guard time interrupt enable
        TCBGTIE OFFSET(24) NUMBITS(1) [],
        /// USART Clock enable in Stop mode
        UCESM OFFSET(23) NUMBITS(1) [],
        /// Wake Up Interrupt Enable
        WUFIE OFFSET(22) NUMBITS(1) [],
        /// Wake UP Interrupt Flag Selection
        WUS OFFSET(20) NUMBITS(2) [
            WAKE_ON_ADDRESS_MATCH   = 0b00,
            WAKE_RESERVED           = 0b01,
            WAKE_ON_START_BIT       = 0b10,
            WAKE_ON_RXNE            = 0b11
        ],
        /// SmartCard Auto-Retry Count
        SCARCNT OFFSET(17) NUMBITS(3) [],
        /// Driver Enable Polarity Selection
        DEP OFFSET(15) NUMBITS(1) [],
        /// Driver Enable Mode
        DEM OFFSET(14) NUMBITS(1) [],
        /// DMA Disable on Reception Error
        DDRE OFFSET(13) NUMBITS(1) [],
        /// Overrun Disable
        OVRDIS OFFSET(12) NUMBITS(1) [],
        /// One sample bit method enable
        ONEBIT OFFSET(11) NUMBITS(1) [],
        /// CTS interrupt enable
        CTSIE OFFSET(10) NUMBITS(1) [],
        /// CTS enable
        CTSE OFFSET(9) NUMBITS(1) [],
        /// RTS enable
        RTSE OFFSET(8) NUMBITS(1) [],
        /// DMA enable transmitter
        DMAT OFFSET(7) NUMBITS(1) [],
        /// DMA enable receiver
        DMAR OFFSET(6) NUMBITS(1) [],
        /// Smartcard mode enable
        SCEN OFFSET(5) NUMBITS(1) [],
        /// Smartcard NACK enable
        NACK OFFSET(4) NUMBITS(1) [],
        /// Half-duplex selection
        HDSEL OFFSET(3) NUMBITS(1) [],
        /// IrDA low-power
        IRLP OFFSET(2) NUMBITS(1) [],
        /// IrDA mode enable
        IREN OFFSET(1) NUMBITS(1) [],
        /// Error interrupt enable
        EIE OFFSET(0) NUMBITS(1) []
    ],
    BRR [
        /// mantissa of USARTDIV
        DIV_Mantissa OFFSET(4) NUMBITS(12) [],
        /// fraction of USARTDIV
        DIV_Fraction OFFSET(0) NUMBITS(4) []
    ],
    GTPR [
        /// Guard time value
        GT OFFSET(8) NUMBITS(8) [],
        /// Prescaler value
        PSC OFFSET(0) NUMBITS(8) []
    ],
    RTOR [
        /// Block Length
        BLEN OFFSET(24) NUMBITS(8) [],
        /// Receiver Time Out Value
        RTO OFFSET(0) NUMBITS(24) []
    ],
    RQR [
        /// Transmit data flush Request
        TXFRQ OFFSET(4) NUMBITS(1) [],
        /// Receive Data flush Request
        RXFRQ OFFSET(3) NUMBITS(1) [],
        /// Mute Mode Request
        MMRQ OFFSET(2) NUMBITS(1) [],
        /// Send Break Request
        SBKRQ OFFSET(1) NUMBITS(1) [],
        /// Auto-Baud Rate Request
        ABRRQ OFFSET(0) NUMBITS(1) []
    ],
    ISR [
        /// Transmission complete before guard time
        TCBGT OFFSET(25) NUMBITS(1) [],
        /// Receive Enable Acknowledge Flag
        REACK OFFSET(22) NUMBITS(1) [],
        /// Transmit Enable Acknowledge Flag
        TEACK OFFSET(21) NUMBITS(1) [],
        /// Wake Up from stop mode Flag
        WUF OFFSET(20) NUMBITS(1) [],
        /// Receive Wake Up from mute mode Flag
        RWU OFFSET(19) NUMBITS(1) [],
        /// Send Break Flag
        SBKF OFFSET(17) NUMBITS(1) [],
        /// Character Match Flag
        CMF OFFSET(17) NUMBITS(1) [],
        /// Busy Flag
        BUSY OFFSET(16) NUMBITS(1) [],
        /// Auto-Baud Rate Flag
        ABRF OFFSET(15) NUMBITS(1) [],
        /// Auto-Baud Rate Error
        ABRE OFFSET(14) NUMBITS(1) [],
        /// End Of Block Flag
        EOBF OFFSET(12) NUMBITS(1) [],
        /// Receiver Time Out
        RTOF OFFSET(11) NUMBITS(1) [],
        /// CTS flag
        CTS OFFSET(10) NUMBITS(1) [],
        /// CTS interrupt flag
        CTSIF OFFSET(9) NUMBITS(1) [],
        /// LIN break detection flag
        LBDF OFFSET(8) NUMBITS(1) [],
        /// Transmit data register empty
        TXE OFFSET(7) NUMBITS(1) [],
        /// Transmission complete
        TC OFFSET(6) NUMBITS(1) [],
        /// Read data register not empty
        RXNE OFFSET(5) NUMBITS(1) [],
        /// IDLE line detected
        IDLE OFFSET(4) NUMBITS(1) [],
        /// Overrun error
        ORE OFFSET(3) NUMBITS(1) [],
        /// Noise detected flag
        NE OFFSET(2) NUMBITS(1) [],
        /// Framing error
        FE OFFSET(1) NUMBITS(1) [],
        /// Parity error
        PE OFFSET(0) NUMBITS(1) []
    ],
    ICR [
        /// Wake Up from stop mode  Clear Flag
        WUCF OFFSET(20) NUMBITS(1) [],
        /// Character Match  Clear Flag
        CMCF OFFSET(17) NUMBITS(1) [],
        /// End Of Block  Clear Flag
        EOBCF OFFSET(12) NUMBITS(1) [],
        /// Receiver Time Out Clear Flag
        RTOCF OFFSET(11) NUMBITS(1) [],
        /// CTS interrupt Clear Flag
        CTSCF OFFSET(9) NUMBITS(1) [],
        /// LIN break detection flag Clear Flag
        LBDCF OFFSET(8) NUMBITS(1) [],
        /// Transmission completed before guard time clear flag
        TCBGTCF OFFSET(7) NUMBITS(1) [],
        /// Transmission complete Clear Flag
        TCCF OFFSET(6) NUMBITS(1) [],
        /// IDLE line detected Clear Flag
        IDLECF OFFSET(4) NUMBITS(1) [],
        /// Overrun error Clear Flag
        ORECF OFFSET(3) NUMBITS(1) [],
        /// Noise detected flag Clear Flag
        NCF OFFSET(2) NUMBITS(1) [],
        /// Framing error Clear Flag
        FECF OFFSET(1) NUMBITS(1) [],
        /// Parity error Clear Flag
        PECF OFFSET(0) NUMBITS(1) []
    ],
    RDR [
        // 9th bit, a better way to handle this?
        BIT9 OFFSET(8) NUMBITS(1) [],
        /// Read Data Register
        DATA OFFSET(0) NUMBITS(8) []
    ],
    TDR [
        // 9th bit, a better way to handle this?
        BIT9 OFFSET(8) NUMBITS(1) [],
        /// Transmit Data Register
        DATA OFFSET(0) NUMBITS(8) []
    ]
];

const _LPUART1_REG: StaticRef<UsartRegisters> =
    unsafe { StaticRef::new(memory_map::LPUART1_BASE as *const UsartRegisters) };
const _USART1_REG: StaticRef<UsartRegisters> =
    unsafe { StaticRef::new(memory_map::USART1_BASE as *const UsartRegisters) };
const _USART2_REG: StaticRef<UsartRegisters> =
    unsafe { StaticRef::new(memory_map::USART2_BASE as *const UsartRegisters) };
const _USART3_REG: StaticRef<UsartRegisters> =
    unsafe { StaticRef::new(memory_map::USART3_BASE as *const UsartRegisters) };
const _UART4_REG: StaticRef<UsartRegisters> =
    unsafe { StaticRef::new(memory_map::UART4_BASE as *const UsartRegisters) };
const _UART5_REG: StaticRef<UsartRegisters> =
    unsafe { StaticRef::new(memory_map::UART5_BASE as *const UsartRegisters) };

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq)]
enum USARTStateRX {
    Unconfigured,
    Idle,
    IRQ_Receiving,
    DMA_Receiving,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq)]
enum USARTStateTX {
    Unconfigured,
    Idle,
    IRQ_Transmitting,
    DMA_Transmitting,
    Transfer_Completing, // DMA finished, but not all bytes sent
}

pub struct Usart<'a> {
    registers: StaticRef<UsartRegisters>,
    clock: rcc::PeripheralClock,

    tx_client: OptionalCell<&'a dyn hil::uart::TransmitClient>,
    rx_client: OptionalCell<&'a dyn hil::uart::ReceiveClient>,

    // tx_dma: OptionalCell<&'a dma1::Stream<'a>>,
    // tx_dma_pid: Dma1Peripheral,
    // rx_dma: OptionalCell<&'a dma1::Stream<'a>>,
    // rx_dma_pid: Dma1Peripheral,
    tx_len: Cell<usize>,
    rx_len: Cell<usize>,
    tx_count: OptionalCell<usize>,
    rx_count: OptionalCell<usize>,

    tx_buffer: TakeCell<'static, [u8]>,
    rx_buffer: TakeCell<'static, [u8]>,

    usart_tx_state: Cell<USARTStateTX>,
    usart_rx_state: Cell<USARTStateRX>,
}

// // for use by `set_dma`
// pub struct TxDMA<'a>(pub &'a dma1::Stream<'a>);
// pub struct RxDMA<'a>(pub &'a dma1::Stream<'a>);

pub static mut USART1: Usart = Usart::new(
    _USART1_REG,
    rcc::PeripheralClock::APB2(rcc::PCLK2::USART1),
    // Dma1Peripheral::USART1_TX,
    // Dma1Peripheral::USART1_RX,
);

pub static mut USART2: Usart = Usart::new(
    _USART2_REG,
    rcc::PeripheralClock::APB1(rcc::PCLK1_1::USART2),
    // Dma1Peripheral::USART2_TX,
    // Dma1Peripheral::USART2_RX,
);

pub static mut USART3: Usart = Usart::new(
    _USART3_REG,
    rcc::PeripheralClock::APB1(rcc::PCLK1_1::USART3),
    // Dma1Peripheral::USART3_TX,
    // Dma1Peripheral::USART3_RX,
);

impl Usart<'a> {
    const fn new(
        base_addr: StaticRef<UsartRegisters>,
        clock: rcc::PeripheralClock,
        // tx_dma_pid: Dma1Peripheral,
        // rx_dma_pid: Dma1Peripheral,
    ) -> Usart<'a> {
        Usart {
            registers: base_addr,
            clock: clock,

            tx_client: OptionalCell::empty(),
            rx_client: OptionalCell::empty(),

            // tx_dma: OptionalCell::empty(),
            // tx_dma_pid: tx_dma_pid,
            // rx_dma: OptionalCell::empty(),
            // rx_dma_pid: rx_dma_pid,
            tx_len: Cell::new(0),
            rx_len: Cell::new(0),
            tx_count: OptionalCell::new(0),
            rx_count: OptionalCell::new(0),

            tx_buffer: TakeCell::empty(),
            rx_buffer: TakeCell::empty(),

            usart_tx_state: Cell::new(USARTStateTX::Unconfigured),
            usart_rx_state: Cell::new(USARTStateRX::Unconfigured),
        }
    }

    // pub fn set_dma(&self, tx_dma: TxDMA<'a>, rx_dma: RxDMA<'a>) {
    //     self.tx_dma.set(tx_dma.0);
    //     self.rx_dma.set(rx_dma.0);
    // }

    // According to section 25.4.13, we need to make sure that USART TC flag is
    // set before disabling the DMA TX on the peripheral side.
    pub fn handle_interrupt(&self) {
        let isr_flags = self.registers.isr.extract();
        let cr1_extract = self.registers.cr1.extract();
        let cr3_extract = self.registers.cr3.extract();

        // isr_flags = isr_flags & (ISR::ORE | ISR::PE | ISR::NE | ISR::FE);
        if (isr_flags.is_set(ISR::ORE)
            || isr_flags.is_set(ISR::PE)
            || isr_flags.is_set(ISR::NE)
            || isr_flags.is_set(ISR::FE))
            && (cr3_extract.is_set(CR3::EIE)
                || cr1_extract.is_set(CR1::RXNEIE)
                || cr1_extract.is_set(CR1::PEIE))
        {
            use hil::uart::Error;
            // Error has occured, handle appropriately
            let _error: Error = Error::None;
        } else {
            // No errors,
            // Process receive data
            if isr_flags.is_set(ISR::RXNE) && cr1_extract.is_set(CR1::RXNEIE) {
                // Recevied character
                // TODO, handle 9bit reception...
                if self.usart_rx_state.get() == USARTStateRX::IRQ_Receiving {
                    // Read bytes
                    self.rx_buffer.map(|buf| {
                        let count = self.rx_count.unwrap_or(0);
                        buf[count] = self.registers.rdr.get() as u8;
                        self.rx_count.replace(count + 1);
                    });
                    if self.rx_count.unwrap_or(0) >= self.rx_len.get() {
                        self.registers.cr1.modify(CR1::RXNEIE::CLEAR);
                        self.registers.cr1.modify(CR1::PEIE::CLEAR);
                        // Disable the USART error interrupt
                        self.registers.cr3.modify(CR3::EIE::CLEAR);
                        // Return state to idle
                        self.usart_rx_state.set(USARTStateRX::Idle);

                        // Get client data
                        let buffer = self.rx_buffer.take();
                        let count = self.rx_count.unwrap_or(0);
                        self.rx_len.set(0);
                        self.rx_count.set(0);

                        // Call client
                        self.rx_client.map(|client| {
                            buffer.map(|buf| {
                                client.received_buffer(
                                    buf,
                                    count,
                                    ReturnCode::SUCCESS,
                                    hil::uart::Error::None,
                                );
                                // TODO receive error reporting, ECANCEL - cancelled,
                                // ESIZE - returning with less that len, FAIL - some error detected
                            });
                        });
                    }
                }
            }
            // Process transmit data
            if isr_flags.is_set(ISR::TXE) && cr1_extract.is_set(CR1::TXEIE) {
                // TODO, handle 9bit trasnmission...
                if self.usart_tx_state.get() == USARTStateTX::IRQ_Transmitting {
                    // Check for empty buffer and proceed to tx complete.
                    if self.tx_count.unwrap_or(0) >= self.tx_len.get() {
                        // Commence complete tx
                        self.disable_transmit_interrupt();
                        self.usart_tx_state.set(USARTStateTX::Transfer_Completing);
                        self.enable_transmit_complete_interrupt();
                    } else {
                        // Send data byte
                        self.tx_buffer.map(|buf| {
                            let count = self.tx_count.unwrap_or(0);
                            self.registers.tdr.set(buf[count].into());
                            self.tx_count.replace(count + 1);
                        });
                    }
                }
            }
            // Process transmit complete
            if isr_flags.is_set(ISR::TC) && cr1_extract.is_set(CR1::TCIE) {
                self.disable_transmit_complete_interrupt();
                self.clear_transmit_complete();

                // Trasnition USARTStateTX from Transfer_Completing to Idle
                self.usart_tx_state.set(USARTStateTX::Idle);

                // get buffer
                let buffer = self.tx_buffer.take();
                let count = self.tx_count.unwrap_or(0);
                self.tx_len.set(0);
                self.tx_count.set(0);

                // alert client
                self.tx_client.map(|client| {
                    buffer.map(|buf| {
                        client.transmitted_buffer(buf, count, ReturnCode::SUCCESS);
                    });
                });
            }
        }
    }

    // // for use by dma1
    // pub fn get_address_dr(&self) -> u32 {
    //     &self.registers.dr as *const ReadWrite<u32> as u32
    // }

    // for use by panic in io.rs
    pub fn send_byte(&self, byte: u8) {
        // loop till TXE (Transmit data register empty) becomes 1
        while !self.registers.isr.is_set(ISR::TXE) {}

        self.registers.tdr.set(byte.into());
    }

    /// Enable IT Transmission
    fn enable_transmit_interrupt(&self) {
        self.registers.cr1.modify(CR1::TXEIE::SET);
    }

    /// Disable IT Transmission
    fn disable_transmit_interrupt(&self) {
        self.registers.cr1.modify(CR1::TXEIE::CLEAR);
    }

    /// Enable IT Reception
    fn enable_receive_interrupt(&self) {
        self.registers.cr3.modify(CR3::EIE::SET);
        self.registers.cr1.modify(CR1::RXNEIE::SET);
    }

    /// Disable IT Reception
    fn disable_receive_interrupt(&self) {
        self.registers.cr1.modify(CR1::RXNEIE::CLEAR);
        self.registers.cr1.modify(CR1::PEIE::CLEAR);
        self.registers.cr3.modify(CR3::EIE::CLEAR);
    }

    /// enable DMA TX from the peripheral side
    fn enable_transmit_dma(&self) {
        self.registers.cr3.modify(CR3::DMAT::SET);
    }

    /// disable DMA TX from the peripheral side
    fn disable_transmit_dma(&self) {
        self.registers.cr3.modify(CR3::DMAT::CLEAR);
    }

    /// enable DMA RX from the peripheral side
    fn enable_receive_dma(&self) {
        self.registers.cr3.modify(CR3::DMAR::SET);
    }

    /// disable DMA RX from the peripheral side
    fn disable_receive_dma(&self) {
        self.registers.cr3.modify(CR3::DMAR::CLEAR);
    }

    // Enable IT transmit complete
    fn enable_transmit_complete_interrupt(&self) {
        self.registers.cr1.modify(CR1::TCIE::SET);
    }

    // Disable IT transmit complete
    fn disable_transmit_complete_interrupt(&self) {
        self.registers.cr1.modify(CR1::TCIE::CLEAR);
    }

    // Clear IT transmit complete
    fn clear_transmit_complete(&self) {
        self.registers.icr.modify(ICR::TCCF::CLEAR);
    }

    fn abort_tx(&self, rcode: ReturnCode) {
        self.disable_transmit_interrupt();
        self.disable_transmit_complete_interrupt();

        if self.registers.cr3.is_set(CR3::DMAT) {
            // DMA Transmit active, cancel
            self.registers.cr3.modify(CR3::DMAT::CLEAR);

            // Abort DMA here...

            // // get buffer
            // let (mut buffer, len) = self.tx_dma.map_or((None, 0), |tx_dma| {
            //     // `abort_transfer` also disables the stream
            //     tx_dma.abort_transfer()
            // });

            // // The number actually transmitted is the difference between
            // // the requested number and the number remaining in DMA transfer.
            // let count = self.tx_len.get() - len as usize;
            // self.tx_len.set(0);
            // self.tx_count.set(0);

            self.usart_tx_state.set(USARTStateTX::Idle);

        // // alert client
        // self.tx_client.map(|client| {
        //     buffer.take().map(|buf| {
        //         client.transmitted_buffer(buf, count, rcode);
        //     });
        // });
        } else {
            // get buffer
            let buffer = self.tx_buffer.take();
            let count = self.tx_count.unwrap_or(0);
            self.tx_len.set(0);
            self.tx_count.set(0);

            self.usart_tx_state.set(USARTStateTX::Idle);

            // alert client
            self.tx_client.map(|client| {
                buffer.map(|buf| {
                    client.transmitted_buffer(buf, count, rcode);
                });
            });
        }
    }

    fn abort_rx(&self, rcode: ReturnCode, error: hil::uart::Error) {
        self.disable_receive_interrupt();

        if self.registers.cr3.is_set(CR3::DMAR) {
            // DMA Transmit active, cancel
            self.registers.cr3.modify(CR3::DMAR::CLEAR);

            // Abort DMA here...

            self.usart_rx_state.set(USARTStateRX::Idle);
            // // get buffer
            // let (mut buffer, len) = self.rx_dma.map_or((None, 0), |rx_dma| {
            //     // `abort_transfer` also disables the stream
            //     rx_dma.abort_transfer()
            // });

            // // Get client data
            // let buffer = self.rx_buffer.take();
            // let count = self.tx_count.unwrap_or(0);
            self.rx_len.set(0);
            self.tx_count.set(0);
        } else {
            self.registers.icr.modify(ICR::ORECF::CLEAR);
            self.registers.icr.modify(ICR::NCF::CLEAR);
            self.registers.icr.modify(ICR::PECF::CLEAR);
            self.registers.icr.modify(ICR::FECF::CLEAR);

            // Flush receive data
            self.registers.rqr.write(RQR::RXFRQ::SET);

            self.usart_rx_state.set(USARTStateRX::Idle);

            // Get client data
            let buffer = self.rx_buffer.take();
            let count = self.tx_count.unwrap_or(0);
            self.tx_len.set(0);
            self.tx_count.set(0);

            // Call client
            self.rx_client.map(|client| {
                buffer.map(|buf| {
                    client.received_buffer(buf, count, rcode, error);
                });
            });
        }
    }
}

impl hil::uart::Configure for Usart<'a> {
    fn configure(&self, params: hil::uart::Parameters) -> ReturnCode {
        if params.baud_rate != 115200
            || params.stop_bits != hil::uart::StopBits::One
            || params.parity != hil::uart::Parity::None
            || params.hw_flow_control != false
            || params.width != hil::uart::Width::Eight
        {
            panic!(
                "Currently we only support uart setting of 115200bps 8N1, no hardware flow control"
            );
        }

        self.clock.enable();

        if self.registers.cr1.is_set(CR1::UE) {
            // while !self.registers.isr.is_set(ISR::TXE) {}
            // self.registers.tdr.set('*' as u32);
            return ReturnCode::EBUSY;
        }

        // Configure the word length - 0: 1 Start bit, 8 Data bits, n Stop bits
        self.registers.cr1.modify(CR1::M1::CLEAR);
        self.registers.cr1.modify(CR1::M0::CLEAR);

        // Set the stop bit length - 00: 1 Stop bits
        self.registers.cr2.modify(CR2::STOP.val(0b00 as u32));

        // Set no parity
        self.registers.cr1.modify(CR1::PCE::CLEAR);

        // Set the baud rate. By default OVER8 is 0 (oversampling by 16) and
        // PCLK1 is at 16Mhz. The desired baud rate is 115.2KBps. So according
        // to Table 149 of reference manual, the value for BRR is 34.722
        // // DIV_Fraction = 0.6875 * 16 = 11 = 0xB
        // // DIV_Mantissa = 8 = 0x8
        // self.registers.brr.modify(BRR::DIV_Fraction.val(0xB as u32));
        // self.registers.brr.modify(BRR::DIV_Mantissa.val(0x8 as u32));
        self.registers.brr.set(34);

        // Enable transmit block
        self.registers.cr1.modify(CR1::TE::SET);

        // Enable receive block
        self.registers.cr1.modify(CR1::RE::SET);

        // Enable USART
        self.registers.cr1.modify(CR1::UE::SET);

        // while !self.registers.isr.is_set(ISR::TXE) {}
        // self.registers.tdr.set('H' as u32);
        // while !self.registers.isr.is_set(ISR::TXE) {}
        // self.registers.tdr.set('e' as u32);
        // while !self.registers.isr.is_set(ISR::TXE) {}
        // self.registers.tdr.set('l' as u32);
        // while !self.registers.isr.is_set(ISR::TXE) {}
        // self.registers.tdr.set('l' as u32);
        // while !self.registers.isr.is_set(ISR::TXE) {}
        // self.registers.tdr.set('o' as u32);
        // while !self.registers.isr.is_set(ISR::TXE) {}
        // self.registers.tdr.set('\n' as u32);

        self.usart_rx_state.set(USARTStateRX::Idle);
        self.usart_tx_state.set(USARTStateTX::Idle);

        ReturnCode::SUCCESS
    }
}

impl hil::uart::Transmit<'a> for Usart<'a> {
    fn set_transmit_client(&self, client: &'a dyn hil::uart::TransmitClient) {
        self.tx_client.set(client);
    }

    fn transmit_buffer(
        &self,
        tx_data: &'static mut [u8],
        tx_len: usize,
    ) -> (ReturnCode, Option<&'static mut [u8]>) {
        // In virtual_uart.rs, transmit is only called when inflight is None. So
        // if the state machine is working correctly, transmit should never
        // abort.

        if self.usart_tx_state.get() == USARTStateTX::Unconfigured {
            // Configure the USART first
            return (ReturnCode::EOFF, Some(tx_data));
        } else if self.usart_tx_state.get() != USARTStateTX::Idle {
            // there is an ongoing transmission, busy
            return (ReturnCode::EBUSY, Some(tx_data));
        }

        if tx_len > tx_data.len() {
            return (ReturnCode::ESIZE, Some(tx_data));
        }

        self.tx_len.set(tx_len);
        self.tx_count.set(0);
        self.tx_buffer.replace(tx_data);

        // // setup and enable dma stream
        // self.tx_dma.map(move |dma| {
        //     self.tx_len.set(tx_len);
        //     dma.do_transfer(tx_data, tx_len);
        // });

        self.usart_tx_state.set(USARTStateTX::IRQ_Transmitting);

        // enable tx on peripheral side
        self.enable_transmit_interrupt();
        (ReturnCode::SUCCESS, None)
    }

    fn transmit_word(&self, _word: u32) -> ReturnCode {
        ReturnCode::FAIL
    }

    fn transmit_abort(&self) -> ReturnCode {
        if self.usart_tx_state.get() == USARTStateTX::IRQ_Transmitting
            || self.usart_tx_state.get() == USARTStateTX::Transfer_Completing
            || self.usart_tx_state.get() == USARTStateTX::DMA_Transmitting
        {
            self.abort_tx(ReturnCode::ECANCEL);
            ReturnCode::SUCCESS
        } else {
            // Either Unconfigured or Idle
            ReturnCode::SUCCESS
        }
    }
}

impl hil::uart::Receive<'a> for Usart<'a> {
    fn set_receive_client(&self, client: &'a dyn hil::uart::ReceiveClient) {
        self.rx_client.set(client);
    }

    fn receive_buffer(
        &self,
        rx_buffer: &'static mut [u8],
        rx_len: usize,
    ) -> (ReturnCode, Option<&'static mut [u8]>) {
        if self.usart_rx_state.get() == USARTStateRX::Unconfigured {
            // Configure the USART first
            return (ReturnCode::EOFF, Some(rx_buffer));
        } else if self.usart_rx_state.get() != USARTStateRX::Idle {
            return (ReturnCode::EBUSY, Some(rx_buffer));
        }

        if rx_len > rx_buffer.len() {
            return (ReturnCode::ESIZE, Some(rx_buffer));
        }

        self.rx_len.set(rx_len);
        self.rx_count.set(0);
        self.rx_buffer.replace(rx_buffer);

        // // setup and enable dma stream
        // self.rx_dma.map(move |dma| {
        //     self.rx_len.set(rx_len);
        //     dma.do_transfer(rx_buffer, rx_len);
        // });

        self.usart_rx_state.set(USARTStateRX::IRQ_Receiving);

        // enable rx on the peripheral side
        self.enable_receive_interrupt();
        (ReturnCode::SUCCESS, None)
    }

    fn receive_word(&self) -> ReturnCode {
        ReturnCode::FAIL
    }

    fn receive_abort(&self) -> ReturnCode {
        self.abort_rx(ReturnCode::ECANCEL, hil::uart::Error::Aborted);
        ReturnCode::SUCCESS
    }
}

impl hil::uart::UartData<'a> for Usart<'a> {}
impl hil::uart::Uart<'a> for Usart<'a> {}

// impl dma1::StreamClient for Usart<'a> {
//     fn transfer_done(&self, pid: dma1::Dma1Peripheral) {
//         if pid == self.tx_dma_pid {
//             self.usart_tx_state.set(USARTStateTX::Transfer_Completing);
//             self.enable_transmit_complete_interrupt();
//         } else if pid == self.rx_dma_pid {
//             // In case of RX, we can call the client directly without having
//             // to trigger an interrupt.
//             if self.usart_rx_state.get() == USARTStateRX::DMA_Receiving {
//                 self.disable_rx();
//                 self.usart_rx_state.set(USARTStateRX::Idle);

//                 // get buffer
//                 let buffer = self.rx_dma.map_or(None, |rx_dma| rx_dma.return_buffer());

//                 let length = self.rx_len.get();
//                 self.rx_len.set(0);

//                 // alert client
//                 self.rx_client.map(|client| {
//                     buffer.map(|buf| {
//                         client.received_buffer(
//                             buf,
//                             length,
//                             ReturnCode::SUCCESS,
//                             hil::uart::Error::None,
//                         );
//                     });
//                 });
//             }
//         }
//     }
// }
