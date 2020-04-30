//! Implementation of the STM32L4 GPIO peripheral.
//!
//! Handles configuration and use of GPIOs
//!
//! TODO:
//! - Add speed control API

// use cortexm4;
// use cortexm4::support::atomic;
use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::common::cells::OptionalCell;
use kernel::common::registers::{register_bitfields, ReadOnly, ReadWrite, WriteOnly};
use kernel::common::StaticRef;
use kernel::debug;
use kernel::hil;
use kernel::ClockInterface;

use crate::exti;
use crate::memory_map;
use crate::rcc;
use crate::syscfg;

/// General-purpose I/Os
#[repr(C)]
struct GpioRegisters {
    /// GPIO port mode register
    moder: ReadWrite<u32, MODER::Register>,
    /// GPIO port output type register
    otyper: ReadWrite<u32, OTYPER::Register>,
    /// GPIO port output speed register
    ospeedr: ReadWrite<u32, OSPEEDR::Register>,
    /// GPIO port pull-up/pull-down register
    pupdr: ReadWrite<u32, PUPDR::Register>,
    /// GPIO port input data register
    idr: ReadOnly<u32, IDR::Register>,
    /// GPIO port output data register
    odr: ReadWrite<u32, ODR::Register>,
    /// GPIO port bit set/reset register
    bsrr: WriteOnly<u32, BSRR::Register>,
    /// GPIO port configuration lock register
    lckr: ReadWrite<u32, LCKR::Register>,
    /// GPIO alternate function low register
    afrl: ReadWrite<u32, AFRL::Register>,
    /// GPIO alternate function high register
    afrh: ReadWrite<u32, AFRH::Register>,
    /// GPIO Bit Reset function high register
    brr: WriteOnly<u32, AFRH::Register>,
    /// GPIO analog switch control function high register
    ascr: ReadWrite<u32, AFRH::Register>,
}

register_bitfields![u32,
    MODER [
        /// Port x configuration bits (y = 0..15)
        MODER15 OFFSET(30) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER14 OFFSET(28) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER13 OFFSET(26) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER12 OFFSET(24) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER11 OFFSET(22) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER10 OFFSET(20) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER9 OFFSET(18) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER8 OFFSET(16) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER7 OFFSET(14) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER6 OFFSET(12) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER5 OFFSET(10) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER4 OFFSET(8) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER3 OFFSET(6) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER2 OFFSET(4) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER1 OFFSET(2) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        MODER0 OFFSET(0) NUMBITS(2) []
    ],
    OTYPER [
        /// Port x configuration bits (y = 0..15)
        OT15 OFFSET(15) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT14 OFFSET(14) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT13 OFFSET(13) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT12 OFFSET(12) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT11 OFFSET(11) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT10 OFFSET(10) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT9 OFFSET(9) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT8 OFFSET(8) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT7 OFFSET(7) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT6 OFFSET(6) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT5 OFFSET(5) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT4 OFFSET(4) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT3 OFFSET(3) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT2 OFFSET(2) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT1 OFFSET(1) NUMBITS(1) [],
        /// Port x configuration bits (y = 0..15)
        OT0 OFFSET(0) NUMBITS(1) []
    ],
    OSPEEDR [
        /// Port x configuration bits (y = 0..15)
        OSPEEDR15 OFFSET(30) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR14 OFFSET(28) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR13 OFFSET(26) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR12 OFFSET(24) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR11 OFFSET(22) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR10 OFFSET(20) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR9 OFFSET(18) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR8 OFFSET(16) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR7 OFFSET(14) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR6 OFFSET(12) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR5 OFFSET(10) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR4 OFFSET(8) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR3 OFFSET(6) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR2 OFFSET(4) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR1 OFFSET(2) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        OSPEEDR0 OFFSET(0) NUMBITS(2) []
    ],
    PUPDR [
        /// Port x configuration bits (y = 0..15)
        PUPDR15 OFFSET(30) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR14 OFFSET(28) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR13 OFFSET(26) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR12 OFFSET(24) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR11 OFFSET(22) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR10 OFFSET(20) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR9 OFFSET(18) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR8 OFFSET(16) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR7 OFFSET(14) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR6 OFFSET(12) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR5 OFFSET(10) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR4 OFFSET(8) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR3 OFFSET(6) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR2 OFFSET(4) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR1 OFFSET(2) NUMBITS(2) [],
        /// Port x configuration bits (y = 0..15)
        PUPDR0 OFFSET(0) NUMBITS(2) []
    ],
    IDR [
        /// Port input data (y = 0..15)
        IDR15 OFFSET(15) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR14 OFFSET(14) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR13 OFFSET(13) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR12 OFFSET(12) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR11 OFFSET(11) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR10 OFFSET(10) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR9 OFFSET(9) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR8 OFFSET(8) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR7 OFFSET(7) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR6 OFFSET(6) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR5 OFFSET(5) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR4 OFFSET(4) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR3 OFFSET(3) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR2 OFFSET(2) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR1 OFFSET(1) NUMBITS(1) [],
        /// Port input data (y = 0..15)
        IDR0 OFFSET(0) NUMBITS(1) []
    ],
    ODR [
        /// Port output data (y = 0..15)
        ODR15 OFFSET(15) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR14 OFFSET(14) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR13 OFFSET(13) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR12 OFFSET(12) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR11 OFFSET(11) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR10 OFFSET(10) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR9 OFFSET(9) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR8 OFFSET(8) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR7 OFFSET(7) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR6 OFFSET(6) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR5 OFFSET(5) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR4 OFFSET(4) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR3 OFFSET(3) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR2 OFFSET(2) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR1 OFFSET(1) NUMBITS(1) [],
        /// Port output data (y = 0..15)
        ODR0 OFFSET(0) NUMBITS(1) []
    ],
    BSRR [
        /// Port x reset bit y (y = 0..15)
        BR15 OFFSET(31) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR14 OFFSET(30) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR13 OFFSET(29) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR12 OFFSET(28) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR11 OFFSET(27) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR10 OFFSET(26) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR9 OFFSET(25) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR8 OFFSET(24) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR7 OFFSET(23) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR6 OFFSET(22) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR5 OFFSET(21) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR4 OFFSET(20) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR3 OFFSET(19) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR2 OFFSET(18) NUMBITS(1) [],
        /// Port x reset bit y (y = 0..15)
        BR1 OFFSET(17) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BR0 OFFSET(16) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS15 OFFSET(15) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS14 OFFSET(14) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS13 OFFSET(13) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS12 OFFSET(12) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS11 OFFSET(11) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS10 OFFSET(10) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS9 OFFSET(9) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS8 OFFSET(8) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS7 OFFSET(7) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS6 OFFSET(6) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS5 OFFSET(5) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS4 OFFSET(4) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS3 OFFSET(3) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS2 OFFSET(2) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS1 OFFSET(1) NUMBITS(1) [],
        /// Port x set bit y (y= 0..15)
        BS0 OFFSET(0) NUMBITS(1) []
    ],
    LCKR [
        /// Port x lock bit y (y= 0..15)
        LCKK OFFSET(16) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK15 OFFSET(15) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK14 OFFSET(14) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK13 OFFSET(13) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK12 OFFSET(12) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK11 OFFSET(11) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK10 OFFSET(10) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK9 OFFSET(9) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK8 OFFSET(8) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK7 OFFSET(7) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK6 OFFSET(6) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK5 OFFSET(5) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK4 OFFSET(4) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK3 OFFSET(3) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK2 OFFSET(2) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK1 OFFSET(1) NUMBITS(1) [],
        /// Port x lock bit y (y= 0..15)
        LCK0 OFFSET(0) NUMBITS(1) []
    ],
    AFRL [
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL7 OFFSET(28) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL6 OFFSET(24) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL5 OFFSET(20) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL4 OFFSET(16) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL3 OFFSET(12) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL2 OFFSET(8) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL1 OFFSET(4) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 0..7)
        AFRL0 OFFSET(0) NUMBITS(4) []
    ],
    AFRH [
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH15 OFFSET(28) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH14 OFFSET(24) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH13 OFFSET(20) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH12 OFFSET(16) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH11 OFFSET(12) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH10 OFFSET(8) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH9 OFFSET(4) NUMBITS(4) [],
        /// Alternate function selection for port x bit y (y = 8..15)
        AFRH8 OFFSET(0) NUMBITS(4) []
    ],
    BRR [
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR15 OFFSET(15) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR14 OFFSET(14) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR13 OFFSET(13) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR12 OFFSET(12) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR11 OFFSET(11) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR10 OFFSET(10) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR9 OFFSET(9) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR8 OFFSET(8) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR7 OFFSET(7) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR6 OFFSET(6) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR5 OFFSET(5) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR4 OFFSET(4) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR3 OFFSET(3) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR2 OFFSET(2) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR1 OFFSET(1) NUMBITS(1) [],
        /// Bit Reset function selection for port x bit y (y= 0..15)
        BR0 OFFSET(0) NUMBITS(1) []
    ],
    ASCR [
        /// Analog switch control for port x bit y (y= 0..15)
        ASC15 OFFSET(15) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC14 OFFSET(14) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC13 OFFSET(13) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC12 OFFSET(12) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC11 OFFSET(11) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC10 OFFSET(10) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC9 OFFSET(9) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC8 OFFSET(8) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC7 OFFSET(7) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC6 OFFSET(6) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC5 OFFSET(5) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC4 OFFSET(4) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC3 OFFSET(3) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC2 OFFSET(2) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC1 OFFSET(1) NUMBITS(1) [],
        /// Analog switch control for port x bit y (y= 0..15)
        ASC0 OFFSET(0) NUMBITS(1) []
    ]
];

const GPIOH_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOH_BASE as *const GpioRegisters) };

const GPIOG_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOG_BASE as *const GpioRegisters) };

const GPIOF_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOF_BASE as *const GpioRegisters) };

const GPIOE_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOE_BASE as *const GpioRegisters) };

const GPIOD_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOD_BASE as *const GpioRegisters) };

const GPIOC_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOC_BASE as *const GpioRegisters) };

const GPIOB_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOB_BASE as *const GpioRegisters) };

const GPIOA_REGS: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(memory_map::GPIOA_BASE as *const GpioRegisters) };

/// STM32F446RE has eight GPIO ports labeled from A-H [^1]. This is represented
/// by three bits.
///
/// [^1]: Figure 3. STM32F446xC/E block diagram, page 16 of the datasheet
// #[repr(u32)]
// pub enum PortId {
//     A = 0b000,
//     B = 0b001,
//     C = 0b010,
//     D = 0b011,
//     E = 0b100,
//     F = 0b101,
//     G = 0b110,
//     H = 0b111,
// }

/// GPIO pin mode [^1]
///
/// [^1]: Section 7.1.4, page 187 of reference manual
enum_from_primitive! {
    #[repr(u32)]
    #[derive(PartialEq)]
    pub enum Mode {
        Input = 0b00,
        GeneralPurposeOutputMode = 0b01,
        AlternateFunctionMode = 0b10,
        AnalogMode = 0b11,
    }
}

/// Alternate functions that may be assigned to a `Pin`.
///
/// GPIO pins on the STM32F446RE may serve multiple functions. In addition to
/// the default functionality, each pin can be assigned up to sixteen different
/// alternate functions. The various functions for each pin are described in
/// "Alternate Function"" section of the STM32F446RE datasheet[^1].
///
/// Alternate Function bit mapping is shown here[^2].
///
/// [^1]: Section 4, Pinout and pin description, Table 11. Alternate function,
///       pages 59-66
///
/// [^2]: Section 7.4.9, page 192 of Reference Manual
#[repr(u32)]
pub enum AlternateFunction {
    AF0 = 0b0000,
    AF1 = 0b0001,
    AF2 = 0b0010,
    AF3 = 0b0011,
    AF4 = 0b0100,
    AF5 = 0b0101,
    AF6 = 0b0110,
    AF7 = 0b0111,
    AF8 = 0b1000,
    AF9 = 0b1001,
    AF10 = 0b1010,
    AF11 = 0b1011,
    AF12 = 0b1100,
    AF13 = 0b1101,
    AF14 = 0b1110,
    AF15 = 0b1111,
}

/// Pin mask for handling EXTI 0 IRQ
const EXTI0_MASK: u32 = 0x00000001;
/// Pin mask for handling EXTI 1 IRQ
const EXTI1_MASK: u32 = 0x00000002;
/// Pin mask for handling EXTI 2 IRQ
const EXTI2_MASK: u32 = 0x00000004;
/// Pin mask for handling EXTI 3 IRQ
const EXTI3_MASK: u32 = 0x00000008;
/// Pin mask for handling EXTI 4 IRQ
const EXTI4_MASK: u32 = 0x00000010;
/// Pin mask for handling EXTI 9-5 IRQ
const EXTI9_5_MASK: u32 = 0x000003E0;
/// Pin mask for handling EXTI 15-10 IRQ
const EXTI15_10_MASK: u32 = 0x0000FC00;

/// GPIO pin internal pull-up and pull-down [^1]
///
/// [^1]: Section 7.4.4, page 189 of reference manual
enum_from_primitive! {
    #[repr(u32)]
    enum PullUpPullDown {
        NoPullUpPullDown = 0b00,
        PullUp = 0b01,
        PullDown = 0b10,
    }
}

/// Name of the GPIO pin on the STM32F446RE.
///
/// The "Pinout and pin description" section [^1] of the STM32F446RE datasheet
/// shows the mapping between the names and the hardware pins on different chip
/// packages.
///
/// The first three bits represent the port and last four bits represent the
/// pin.
///
/// [^1]: Section 4, Pinout and pin description, pages 41-45
#[rustfmt::skip]
#[repr(u8)]
#[derive(Copy, Clone)]
#[derive(Debug)]
pub enum Pin {
    PA00, PA01, PA02, PA03,
    PA04, PA05, PA06, PA07,
    PA08, PA09, PA10, PA11,
    PA12, PA13, PA14, PA15,

    PB00, PB01, PB02, PB03,
    PB04, PB05, PB06, PB07,
    PB08, PB09, PB10, PB11,
    PB12, PB13, PB14, PB15,

    PC00, PC01, PC02, PC03,
    PC04, PC05, PC06, PC07,
    PC08, PC09, PC10, PC11,
    PC12, PC13, PC14, PC15,

    PD00, PD01, PD02, PD03,
    PD04, PD05, PD06, PD07,
    PD08, PD09, PD10, PD11,
    PD12, PD13, PD14, PD15,
    
    PE00, PE01, PE02, PE03,
    PE04, PE05, PE06, PE07,
    PE08, PE09, PE10, PE11,
    PE12, PE13, PE14, PE15,

    PF00, PF01, PF02, PF03,
    PF04, PF05, PF06, PF07,
    PF08, PF09, PF10, PF11,
    PF12, PF13, PF14, PF15,

    PG00, PG01, PG02, PG03,
    PG04, PG05, PG06, PG07,
    PG08, PG09, PG10, PG11,
    PG12, PG13, PG14, PG15,

    PH00, PH01, PH02, PH03,
    PH04, PH05, PH06, PH07,
    PH08, PH09, PH10, PH11,
    PH12, PH13, PH14, PH15,

    // Port I - only on STM32L49x/L4Ax
    // PI00, PI01, PI02, PI03,
    // PI04, PI05, PI06, PI07,
    // PI08, PI09, PI10, PI11,
    // PI12, PI13, PI14, PI15,
}

// Active Pin to carry out operations on
pub struct GpioPin {
    registers: StaticRef<GpioRegisters>,
    mask: u32,
    // exti_lineid: OptionalCell<exti::LineId>,
}

impl GpioPin {
    const fn new(port: StaticRef<GpioRegisters>, mask: u32) -> GpioPin {
        GpioPin {
            registers: port,
            mask: mask,
        }
    }
}

struct ExtiClients {
    client: [OptionalCell<&'static dyn hil::gpio::Client>; 16],
}

static mut EXTI_CLIENTS: ExtiClients = ExtiClients::new();

impl ExtiClients {
    const fn new() -> ExtiClients {
        ExtiClients {
            client: [
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
                OptionalCell::empty(),
            ],
        }
    }
}

// macro_rules! declare_gpio_pins {
//     ($($pin:ident)*) => {
//         [
//             $(Some(Pin::new(GpioPin::$pin)), )*
//         ]
//     }
// }

// macro_rules! declare_gpio_pins {
//     ($($pin:ident)*) => {
//         [
//             match (*$pin as usize & 0xf0) >> 4 {
//                 0 => $(Some(Pin::new(GPIOA_REGS, 1 << (GpioPin::$pin as usize & 0x0f))), )*,
//                 _ => $(Some(Pin::new(GPIOA_REGS, 1 << (GpioPin::$pin as usize & 0x0f))), )*,
//             }
//         ]
//     }
// }

// macro_rules! declare_gpio_pin {
//     ($($pin:ident)*) => {
//         $(Some(Pin::new(PORT[(GpioPin::$pin as usize & 0xf0) >> 4].registers, 1 << (GpioPin::$pin as usize & 0x0f))) )*
//     }
// }

// We need to use `Option<Pin>`, instead of just `Pin` because GPIOH has
// only two pins - PH00 and PH01, rather than the usual sixteen pins.
pub static mut PIN: [Option<GpioPin>; 16 * 8] = [
    // Port A
    Some(GpioPin::new(GPIOA_REGS, 00)),
    Some(GpioPin::new(GPIOA_REGS, 01)),
    Some(GpioPin::new(GPIOA_REGS, 02)),
    Some(GpioPin::new(GPIOA_REGS, 03)),
    Some(GpioPin::new(GPIOA_REGS, 04)),
    Some(GpioPin::new(GPIOA_REGS, 05)),
    Some(GpioPin::new(GPIOA_REGS, 06)),
    Some(GpioPin::new(GPIOA_REGS, 07)),
    Some(GpioPin::new(GPIOA_REGS, 08)),
    Some(GpioPin::new(GPIOA_REGS, 09)),
    Some(GpioPin::new(GPIOA_REGS, 10)),
    Some(GpioPin::new(GPIOA_REGS, 11)),
    Some(GpioPin::new(GPIOA_REGS, 12)),
    Some(GpioPin::new(GPIOA_REGS, 13)),
    Some(GpioPin::new(GPIOA_REGS, 14)),
    Some(GpioPin::new(GPIOA_REGS, 15)),
    // Port B
    Some(GpioPin::new(GPIOB_REGS, 00)),
    Some(GpioPin::new(GPIOB_REGS, 01)),
    Some(GpioPin::new(GPIOB_REGS, 02)),
    Some(GpioPin::new(GPIOB_REGS, 03)),
    Some(GpioPin::new(GPIOB_REGS, 04)),
    Some(GpioPin::new(GPIOB_REGS, 05)),
    Some(GpioPin::new(GPIOB_REGS, 06)),
    Some(GpioPin::new(GPIOB_REGS, 07)),
    Some(GpioPin::new(GPIOB_REGS, 08)),
    Some(GpioPin::new(GPIOB_REGS, 09)),
    Some(GpioPin::new(GPIOB_REGS, 10)),
    Some(GpioPin::new(GPIOB_REGS, 11)),
    Some(GpioPin::new(GPIOB_REGS, 12)),
    Some(GpioPin::new(GPIOB_REGS, 13)),
    Some(GpioPin::new(GPIOB_REGS, 14)),
    Some(GpioPin::new(GPIOB_REGS, 15)),
    // Port C
    Some(GpioPin::new(GPIOC_REGS, 00)),
    Some(GpioPin::new(GPIOC_REGS, 01)),
    Some(GpioPin::new(GPIOC_REGS, 02)),
    Some(GpioPin::new(GPIOC_REGS, 03)),
    Some(GpioPin::new(GPIOC_REGS, 04)),
    Some(GpioPin::new(GPIOC_REGS, 05)),
    Some(GpioPin::new(GPIOC_REGS, 06)),
    Some(GpioPin::new(GPIOC_REGS, 07)),
    Some(GpioPin::new(GPIOC_REGS, 08)),
    Some(GpioPin::new(GPIOC_REGS, 09)),
    Some(GpioPin::new(GPIOC_REGS, 10)),
    Some(GpioPin::new(GPIOC_REGS, 11)),
    Some(GpioPin::new(GPIOC_REGS, 12)),
    Some(GpioPin::new(GPIOC_REGS, 13)),
    Some(GpioPin::new(GPIOC_REGS, 14)),
    Some(GpioPin::new(GPIOC_REGS, 15)),
    // Port D
    None, // 00
    None, // 01
    Some(GpioPin::new(GPIOD_REGS, 02)),
    None, // 03
    None, // 04
    None, // 05
    None, // 06
    None, // 07
    None, // 08
    None, // 09
    None, // 10
    None, // 11
    None, // 12
    None, // 13
    None, // 14
    None, // 15
    // Port E
    None, // 00
    None, // 01
    None, // 02
    None, // 03
    None, // 04
    None, // 05
    None, // 06
    None, // 07
    None, // 08
    None, // 09
    None, // 10
    None, // 11
    None, // 12
    None, // 13
    None, // 14
    None, // 15
    // Port F
    None, // 00
    None, // 01
    None, // 02
    None, // 03
    None, // 04
    None, // 05
    None, // 06
    None, // 07
    None, // 08
    None, // 09
    None, // 10
    None, // 11
    None, // 12
    None, // 13
    None, // 14
    None, // 15
    // Port G
    None, // 00
    None, // 01
    None, // 02
    None, // 03
    None, // 04
    None, // 05
    None, // 06
    None, // 07
    None, // 08
    None, // 09
    None, // 10
    None, // 11
    None, // 12
    None, // 13
    None, // 14
    None, // 15
    // Port H
    Some(GpioPin::new(GPIOH_REGS, 0)),
    Some(GpioPin::new(GPIOH_REGS, 1)),
    None, // 02
    None, // 03
    None, // 04
    None, // 05
    None, // 06
    None, // 07
    None, // 08
    None, // 09
    None, // 10
    None, // 11
    None, // 12
    None, // 13
    None, // 14
    None, // 15
];

impl Pin {
    fn get_pin(&self) -> &GpioPin {
        unsafe {
            match PIN[usize::from_u32(*self as u32).unwrap()] {
                None => {
                    panic!("get_pin: NOT found other");
                }
                _ => {}
            };
        }
        unsafe {
            &PIN[usize::from_u32(*self as u32).unwrap()]
                .as_ref()
                .unwrap()
        }
    }

    // pub fn get_pin_mut(&self) -> &mut GpioPin {
    //     unsafe { &mut PIN[usize::from_u32(*self as u32).unwrap()].as_mut().unwrap() }
    // }
    // fn get_pin_mut(&self) -> &mut Option<Pin> {
    //     let mut port_num: u8 = *self as u8;

    //     // Right shift p by 4 bits, so we can get rid of pin bits
    //     port_num >>= 4;

    //     let mut pin_num: u8 = *self as u8;
    //     // Mask top 3 bits, so can get only the suffix
    //     pin_num &= 0b0001111;

    //     unsafe { &mut PIN[usize::from(port_num)][usize::from(pin_num)] }
    // }

    fn get_port(&self) -> &Port {
        let mut port_num: u8 = *self as u8;

        // Right shift p by 4 bits, so we can get rid of pin bits
        port_num >>= 4;
        unsafe { &PORT[usize::from(port_num)] }
    }

    // extract the last 4 bits. [3:0] is the pin number, [6:4] is the port
    // number
    fn get_pin_number(&self) -> u8 {
        let mut pin_num = *self as u8;

        pin_num = pin_num & 0b00001111;
        pin_num
    }

    // extract bits [6:4], which is the port number
    fn get_port_number(&self) -> u8 {
        let mut port_num: u8 = *self as u8;

        // Right shift p by 4 bits, so we can get rid of pin bits
        port_num >>= 4;
        port_num
    }
}

pub struct Port {
    registers: StaticRef<GpioRegisters>,
    clock: rcc::PeripheralClock,
}

pub static mut PORT: [Port; 8] = [
    Port {
        registers: GPIOA_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOA),
    },
    Port {
        registers: GPIOB_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOB),
    },
    Port {
        registers: GPIOC_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOC),
    },
    Port {
        registers: GPIOD_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOD),
    },
    Port {
        registers: GPIOE_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOE),
    },
    Port {
        registers: GPIOF_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOF),
    },
    Port {
        registers: GPIOG_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOG),
    },
    Port {
        registers: GPIOH_REGS,
        clock: rcc::PeripheralClock::AHB2(rcc::HCLK2::GPIOH),
    },
];

impl Pin {
    // pub fn set_client(&self, client: &'a dyn hil::gpio::Client) {
    //     let index = self.get_pin_number();

    //     EXTI_CLIENTS.client[usize::from(index as u8)].set(client);
    // }

    pub fn get_mode(&self) -> Mode {
        // let maybe = unsafe {
        //     &PIN[usize::from((*self as u16 & 0x00f0) >> 4)][usize::from(*self as u16 & 0x000f)]
        // };

        // let pin = maybe.as_ref().expect("get_mode: invalid pin accessed");

        let pin = self.get_pin();
        // panic!("get_mode: invalid pin accessed {}", *self as u32);
        let val = pin.registers.moder.get() & pin.mask;
        Mode::from_u32(val).unwrap_or(Mode::Input)
    }

    pub fn set_mode(&self, mode: Mode) {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.moder.modify(MODER::MODER0.val(mode as u32)),
            0b0001 => port.registers.moder.modify(MODER::MODER1.val(mode as u32)),
            0b0010 => port.registers.moder.modify(MODER::MODER2.val(mode as u32)),
            0b0011 => port.registers.moder.modify(MODER::MODER3.val(mode as u32)),
            0b0100 => port.registers.moder.modify(MODER::MODER4.val(mode as u32)),
            0b0101 => port.registers.moder.modify(MODER::MODER5.val(mode as u32)),
            0b0110 => port.registers.moder.modify(MODER::MODER6.val(mode as u32)),
            0b0111 => port.registers.moder.modify(MODER::MODER7.val(mode as u32)),
            0b1000 => port.registers.moder.modify(MODER::MODER8.val(mode as u32)),
            0b1001 => port.registers.moder.modify(MODER::MODER9.val(mode as u32)),
            0b1010 => port.registers.moder.modify(MODER::MODER10.val(mode as u32)),
            0b1011 => port.registers.moder.modify(MODER::MODER11.val(mode as u32)),
            0b1100 => port.registers.moder.modify(MODER::MODER12.val(mode as u32)),
            0b1101 => port.registers.moder.modify(MODER::MODER13.val(mode as u32)),
            0b1110 => port.registers.moder.modify(MODER::MODER14.val(mode as u32)),
            0b1111 => port.registers.moder.modify(MODER::MODER15.val(mode as u32)),
            _ => {}
        }
    }

    pub fn set_alternate_function(&self, af: AlternateFunction) {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.afrl.modify(AFRL::AFRL0.val(af as u32)),
            0b0001 => port.registers.afrl.modify(AFRL::AFRL1.val(af as u32)),
            0b0010 => port.registers.afrl.modify(AFRL::AFRL2.val(af as u32)),
            0b0011 => port.registers.afrl.modify(AFRL::AFRL3.val(af as u32)),
            0b0100 => port.registers.afrl.modify(AFRL::AFRL4.val(af as u32)),
            0b0101 => port.registers.afrl.modify(AFRL::AFRL5.val(af as u32)),
            0b0110 => port.registers.afrl.modify(AFRL::AFRL6.val(af as u32)),
            0b0111 => port.registers.afrl.modify(AFRL::AFRL7.val(af as u32)),
            0b1000 => port.registers.afrh.modify(AFRH::AFRH8.val(af as u32)),
            0b1001 => port.registers.afrh.modify(AFRH::AFRH9.val(af as u32)),
            0b1010 => port.registers.afrh.modify(AFRH::AFRH10.val(af as u32)),
            0b1011 => port.registers.afrh.modify(AFRH::AFRH11.val(af as u32)),
            0b1100 => port.registers.afrh.modify(AFRH::AFRH12.val(af as u32)),
            0b1101 => port.registers.afrh.modify(AFRH::AFRH13.val(af as u32)),
            0b1110 => port.registers.afrh.modify(AFRH::AFRH14.val(af as u32)),
            0b1111 => port.registers.afrh.modify(AFRH::AFRH15.val(af as u32)),
            _ => {}
        }
    }

    // pub fn get_pinid(&self) -> GpioPin {
    //     self
    // }

    // pub fn set_exti_lineid(&self, lineid: exti::LineId) {
    //     self.exti_lineid.set(lineid);
    // }

    fn set_mode_output_pushpull(&self) {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.otyper.modify(OTYPER::OT0::CLEAR),
            0b0001 => port.registers.otyper.modify(OTYPER::OT1::CLEAR),
            0b0010 => port.registers.otyper.modify(OTYPER::OT2::CLEAR),
            0b0011 => port.registers.otyper.modify(OTYPER::OT3::CLEAR),
            0b0100 => port.registers.otyper.modify(OTYPER::OT4::CLEAR),
            0b0101 => port.registers.otyper.modify(OTYPER::OT5::CLEAR),
            0b0110 => port.registers.otyper.modify(OTYPER::OT6::CLEAR),
            0b0111 => port.registers.otyper.modify(OTYPER::OT7::CLEAR),
            0b1000 => port.registers.otyper.modify(OTYPER::OT8::CLEAR),
            0b1001 => port.registers.otyper.modify(OTYPER::OT9::CLEAR),
            0b1010 => port.registers.otyper.modify(OTYPER::OT10::CLEAR),
            0b1011 => port.registers.otyper.modify(OTYPER::OT11::CLEAR),
            0b1100 => port.registers.otyper.modify(OTYPER::OT12::CLEAR),
            0b1101 => port.registers.otyper.modify(OTYPER::OT13::CLEAR),
            0b1110 => port.registers.otyper.modify(OTYPER::OT14::CLEAR),
            0b1111 => port.registers.otyper.modify(OTYPER::OT15::CLEAR),
            _ => {}
        }
    }

    fn get_pullup_pulldown(&self) -> PullUpPullDown {
        let port = self.get_port();

        let val = match self.get_pin_number() {
            0b0000 => port.registers.pupdr.read(PUPDR::PUPDR0),
            0b0001 => port.registers.pupdr.read(PUPDR::PUPDR1),
            0b0010 => port.registers.pupdr.read(PUPDR::PUPDR2),
            0b0011 => port.registers.pupdr.read(PUPDR::PUPDR3),
            0b0100 => port.registers.pupdr.read(PUPDR::PUPDR4),
            0b0101 => port.registers.pupdr.read(PUPDR::PUPDR5),
            0b0110 => port.registers.pupdr.read(PUPDR::PUPDR6),
            0b0111 => port.registers.pupdr.read(PUPDR::PUPDR7),
            0b1000 => port.registers.pupdr.read(PUPDR::PUPDR8),
            0b1001 => port.registers.pupdr.read(PUPDR::PUPDR9),
            0b1010 => port.registers.pupdr.read(PUPDR::PUPDR10),
            0b1011 => port.registers.pupdr.read(PUPDR::PUPDR11),
            0b1100 => port.registers.pupdr.read(PUPDR::PUPDR12),
            0b1101 => port.registers.pupdr.read(PUPDR::PUPDR13),
            0b1110 => port.registers.pupdr.read(PUPDR::PUPDR14),
            0b1111 => port.registers.pupdr.read(PUPDR::PUPDR15),
            _ => 0,
        };

        PullUpPullDown::from_u32(val).unwrap_or(PullUpPullDown::NoPullUpPullDown)
    }

    fn set_pullup_pulldown(&self, pupd: PullUpPullDown) {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.pupdr.modify(PUPDR::PUPDR0.val(pupd as u32)),
            0b0001 => port.registers.pupdr.modify(PUPDR::PUPDR1.val(pupd as u32)),
            0b0010 => port.registers.pupdr.modify(PUPDR::PUPDR2.val(pupd as u32)),
            0b0011 => port.registers.pupdr.modify(PUPDR::PUPDR3.val(pupd as u32)),
            0b0100 => port.registers.pupdr.modify(PUPDR::PUPDR4.val(pupd as u32)),
            0b0101 => port.registers.pupdr.modify(PUPDR::PUPDR5.val(pupd as u32)),
            0b0110 => port.registers.pupdr.modify(PUPDR::PUPDR6.val(pupd as u32)),
            0b0111 => port.registers.pupdr.modify(PUPDR::PUPDR7.val(pupd as u32)),
            0b1000 => port.registers.pupdr.modify(PUPDR::PUPDR8.val(pupd as u32)),
            0b1001 => port.registers.pupdr.modify(PUPDR::PUPDR9.val(pupd as u32)),
            0b1010 => port.registers.pupdr.modify(PUPDR::PUPDR10.val(pupd as u32)),
            0b1011 => port.registers.pupdr.modify(PUPDR::PUPDR11.val(pupd as u32)),
            0b1100 => port.registers.pupdr.modify(PUPDR::PUPDR12.val(pupd as u32)),
            0b1101 => port.registers.pupdr.modify(PUPDR::PUPDR13.val(pupd as u32)),
            0b1110 => port.registers.pupdr.modify(PUPDR::PUPDR14.val(pupd as u32)),
            0b1111 => port.registers.pupdr.modify(PUPDR::PUPDR15.val(pupd as u32)),
            _ => {}
        }
    }

    fn set_output_high(&self) {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.bsrr.write(BSRR::BS0::SET),
            0b0001 => port.registers.bsrr.write(BSRR::BS1::SET),
            0b0010 => port.registers.bsrr.write(BSRR::BS2::SET),
            0b0011 => port.registers.bsrr.write(BSRR::BS3::SET),
            0b0100 => port.registers.bsrr.write(BSRR::BS4::SET),
            0b0101 => port.registers.bsrr.write(BSRR::BS5::SET),
            0b0110 => port.registers.bsrr.write(BSRR::BS6::SET),
            0b0111 => port.registers.bsrr.write(BSRR::BS7::SET),
            0b1000 => port.registers.bsrr.write(BSRR::BS8::SET),
            0b1001 => port.registers.bsrr.write(BSRR::BS9::SET),
            0b1010 => port.registers.bsrr.write(BSRR::BS10::SET),
            0b1011 => port.registers.bsrr.write(BSRR::BS11::SET),
            0b1100 => port.registers.bsrr.write(BSRR::BS12::SET),
            0b1101 => port.registers.bsrr.write(BSRR::BS13::SET),
            0b1110 => port.registers.bsrr.write(BSRR::BS14::SET),
            0b1111 => port.registers.bsrr.write(BSRR::BS15::SET),
            _ => {}
        }
    }

    fn set_output_low(&self) {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.bsrr.write(BSRR::BR0::SET),
            0b0001 => port.registers.bsrr.write(BSRR::BR1::SET),
            0b0010 => port.registers.bsrr.write(BSRR::BR2::SET),
            0b0011 => port.registers.bsrr.write(BSRR::BR3::SET),
            0b0100 => port.registers.bsrr.write(BSRR::BR4::SET),
            0b0101 => port.registers.bsrr.write(BSRR::BR5::SET),
            0b0110 => port.registers.bsrr.write(BSRR::BR6::SET),
            0b0111 => port.registers.bsrr.write(BSRR::BR7::SET),
            0b1000 => port.registers.bsrr.write(BSRR::BR8::SET),
            0b1001 => port.registers.bsrr.write(BSRR::BR9::SET),
            0b1010 => port.registers.bsrr.write(BSRR::BR10::SET),
            0b1011 => port.registers.bsrr.write(BSRR::BR11::SET),
            0b1100 => port.registers.bsrr.write(BSRR::BR12::SET),
            0b1101 => port.registers.bsrr.write(BSRR::BR13::SET),
            0b1110 => port.registers.bsrr.write(BSRR::BR14::SET),
            0b1111 => port.registers.bsrr.write(BSRR::BR15::SET),
            _ => {}
        }
    }

    fn is_output_high(&self) -> bool {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.odr.is_set(ODR::ODR0),
            0b0001 => port.registers.odr.is_set(ODR::ODR1),
            0b0010 => port.registers.odr.is_set(ODR::ODR2),
            0b0011 => port.registers.odr.is_set(ODR::ODR3),
            0b0100 => port.registers.odr.is_set(ODR::ODR4),
            0b0101 => port.registers.odr.is_set(ODR::ODR5),
            0b0110 => port.registers.odr.is_set(ODR::ODR6),
            0b0111 => port.registers.odr.is_set(ODR::ODR7),
            0b1000 => port.registers.odr.is_set(ODR::ODR8),
            0b1001 => port.registers.odr.is_set(ODR::ODR9),
            0b1010 => port.registers.odr.is_set(ODR::ODR10),
            0b1011 => port.registers.odr.is_set(ODR::ODR11),
            0b1100 => port.registers.odr.is_set(ODR::ODR12),
            0b1101 => port.registers.odr.is_set(ODR::ODR13),
            0b1110 => port.registers.odr.is_set(ODR::ODR14),
            0b1111 => port.registers.odr.is_set(ODR::ODR15),
            _ => false,
        }
    }

    fn toggle_output(&self) -> bool {
        if self.is_output_high() {
            self.set_output_low();
            false
        } else {
            self.set_output_high();
            true
        }
    }

    fn read_input(&self) -> bool {
        let port = self.get_port();

        match self.get_pin_number() {
            0b0000 => port.registers.idr.is_set(IDR::IDR0),
            0b0001 => port.registers.idr.is_set(IDR::IDR1),
            0b0010 => port.registers.idr.is_set(IDR::IDR2),
            0b0011 => port.registers.idr.is_set(IDR::IDR3),
            0b0100 => port.registers.idr.is_set(IDR::IDR4),
            0b0101 => port.registers.idr.is_set(IDR::IDR5),
            0b0110 => port.registers.idr.is_set(IDR::IDR6),
            0b0111 => port.registers.idr.is_set(IDR::IDR7),
            0b1000 => port.registers.idr.is_set(IDR::IDR8),
            0b1001 => port.registers.idr.is_set(IDR::IDR9),
            0b1010 => port.registers.idr.is_set(IDR::IDR10),
            0b1011 => port.registers.idr.is_set(IDR::IDR11),
            0b1100 => port.registers.idr.is_set(IDR::IDR12),
            0b1101 => port.registers.idr.is_set(IDR::IDR13),
            0b1110 => port.registers.idr.is_set(IDR::IDR14),
            0b1111 => port.registers.idr.is_set(IDR::IDR15),
            _ => false,
        }
    }

    /// Map Pin to LineId
    fn to_lineid(&self) -> exti::LineId {
        // How to handle mapping errors? Panic? Silent? debug?
        // This mapping (pin# -> LineId is implicit), should it be explicit?
        exti::LineId::from_u8(self.get_pin_number()).unwrap()
    }

    /// Map Pin to ExtiPin
    fn to_extipin(&self) -> syscfg::ExtiPin {
        // How to handle mapping errors? Panic? Silent? debug?
        // This mapping (pin# -> LineId is implicit), should it be explicit?
        syscfg::ExtiPin::from_u8(self.get_pin_number()).unwrap()
    }

    /// Map Pin to ExtiPort
    fn to_extiport(&self) -> syscfg::ExtiPort {
        // How to handle mapping errors? Panic? Silent? debug?
        // This mapping (pin# -> LineId is implicit), should it be explicit?
        syscfg::ExtiPort::from_u8(self.get_port_number()).unwrap()
    }

    /// Handle GPIO - EXTI 0 - IRQ
    pub fn handle_exti0_interrupt() {
        unsafe {
            let active = exti::EXTI.handle_exti_lines(EXTI0_MASK);
            if active != 0 {
                EXTI_CLIENTS.client[0].map(|client| client.fired());
            }
        }
        debug!("EXTI 0 fired");
    }
    /// Handle GPIO - EXTI 1 - IRQ
    pub fn handle_exti1_interrupt() {
        unsafe {
            let active = exti::EXTI.handle_exti_lines(EXTI1_MASK);
            if active != 0 {
                EXTI_CLIENTS.client[1].map(|client| client.fired());
            }
        }
        debug!("EXTI 1 fired");
    }
    /// Handle GPIO - EXTI 2 - IRQ
    pub fn handle_exti2_interrupt() {
        unsafe {
            let active = exti::EXTI.handle_exti_lines(EXTI2_MASK);
            if active != 0 {
                EXTI_CLIENTS.client[2].map(|client| client.fired());
            }
        }
        debug!("EXTI 2 fired");
    }
    /// Handle GPIO - EXTI 3 - IRQ
    pub fn handle_exti3_interrupt() {
        unsafe {
            let active = exti::EXTI.handle_exti_lines(EXTI3_MASK);
            if active != 0 {
                EXTI_CLIENTS.client[3].map(|client| client.fired());
            }
        }
        debug!("EXTI 3 fired");
    }
    /// Handle GPIO - EXTI 4 - IRQ
    pub fn handle_exti4_interrupt() {
        unsafe {
            let active = exti::EXTI.handle_exti_lines(EXTI4_MASK);
            if active != 0 {
                EXTI_CLIENTS.client[4].map(|client| client.fired());
            }
        }
        debug!("EXTI 4 fired");
    }
    /// Handle GPIO - EXTI 9-5 - IRQ
    pub fn handle_exti9_5_interrupt() {
        unsafe {
            let mut active = exti::EXTI.handle_exti_lines(EXTI9_5_MASK);
            while active != 0 {
                let pin = active.trailing_zeros() as usize;
                active &= !(1 << pin);
                EXTI_CLIENTS.client[pin].map(|client| client.fired());
            }
        }
        debug!("EXTI 9-5 fired");
    }
    /// Handle GPIO - EXTI 15-10 - IRQ
    pub fn handle_exti15_10_interrupt() {
        unsafe {
            let mut active = exti::EXTI.handle_exti_lines(EXTI15_10_MASK);
            while active != 0 {
                let pin = active.trailing_zeros() as usize;
                active &= !(1 << pin);
                EXTI_CLIENTS.client[pin].map(|client| client.fired());
            }
        }
        debug!("EXTI 15-10 fired");
    }
}

impl hil::gpio::Pin for Pin {}
impl hil::gpio::InterruptPin for Pin {}

impl hil::gpio::Configure for Pin {
    /// Output mode default is push-pull
    fn make_output(&self) -> hil::gpio::Configuration {
        self.get_port().clock.enable();

        self.set_mode(Mode::GeneralPurposeOutputMode);
        self.set_mode_output_pushpull();
        hil::gpio::Configuration::Output
    }

    /// Input mode default is no internal pull-up, no pull-down (i.e.,
    /// floating). Also upon setting the mode as input, the internal schmitt
    /// trigger is automatically activated. Schmitt trigger is deactivated in
    /// AnalogMode.
    fn make_input(&self) -> hil::gpio::Configuration {
        self.get_port().clock.enable();

        self.set_mode(Mode::Input);
        hil::gpio::Configuration::Input
    }

    /// According to AN4899, Section 6.1, setting to AnalogMode, disables
    /// internal schmitt trigger. We do not disable clock to the GPIO port,
    /// because there could be other pins active on the port.
    fn deactivate_to_low_power(&self) {
        self.get_port().clock.enable();

        self.set_mode(Mode::AnalogMode);
    }

    fn disable_output(&self) -> hil::gpio::Configuration {
        self.get_port().clock.enable();

        self.set_mode(Mode::AnalogMode);
        hil::gpio::Configuration::LowPower
    }

    fn disable_input(&self) -> hil::gpio::Configuration {
        self.get_port().clock.enable();

        self.set_mode(Mode::AnalogMode);
        hil::gpio::Configuration::LowPower
    }

    fn set_floating_state(&self, mode: hil::gpio::FloatingState) {
        self.get_port().clock.enable();

        match mode {
            hil::gpio::FloatingState::PullUp => self.set_pullup_pulldown(PullUpPullDown::PullUp),
            hil::gpio::FloatingState::PullDown => {
                self.set_pullup_pulldown(PullUpPullDown::PullDown)
            }
            hil::gpio::FloatingState::PullNone => {
                self.set_pullup_pulldown(PullUpPullDown::NoPullUpPullDown)
            }
        }
    }

    fn floating_state(&self) -> hil::gpio::FloatingState {
        match self.get_pullup_pulldown() {
            PullUpPullDown::PullUp => hil::gpio::FloatingState::PullUp,
            PullUpPullDown::PullDown => hil::gpio::FloatingState::PullDown,
            PullUpPullDown::NoPullUpPullDown => hil::gpio::FloatingState::PullNone,
        }
    }

    fn configuration(&self) -> hil::gpio::Configuration {
        match self.get_mode() {
            Mode::Input => hil::gpio::Configuration::Input,
            Mode::GeneralPurposeOutputMode => hil::gpio::Configuration::Output,
            Mode::AnalogMode => hil::gpio::Configuration::LowPower,
            Mode::AlternateFunctionMode => hil::gpio::Configuration::Function,
        }
    }

    fn is_input(&self) -> bool {
        self.get_mode() == Mode::Input
    }

    fn is_output(&self) -> bool {
        self.get_mode() == Mode::GeneralPurposeOutputMode
    }
}

impl hil::gpio::Output for Pin {
    fn set(&self) {
        self.set_output_high();
    }

    fn clear(&self) {
        self.set_output_low();
    }

    fn toggle(&self) -> bool {
        self.toggle_output()
    }
}

impl hil::gpio::Input for Pin {
    fn read(&self) -> bool {
        self.read_input()
    }
}

impl hil::gpio::Interrupt for Pin {
    fn enable_interrupts(&self, mode: hil::gpio::InterruptEdge) {
        // TODO handle errors in lineid conversion
        let lineid = self.to_lineid();

        unsafe {
            // disable the interrupt
            exti::EXTI.mask_interrupt(lineid);
            exti::EXTI.clear_pending(lineid);

            syscfg::SYSCFG.select_exti_source(self.to_extipin(), self.to_extiport());

            match mode {
                hil::gpio::InterruptEdge::EitherEdge => {
                    exti::EXTI.select_rising_trigger(lineid);
                    exti::EXTI.select_falling_trigger(lineid);
                }
                hil::gpio::InterruptEdge::RisingEdge => {
                    exti::EXTI.select_rising_trigger(lineid);
                    exti::EXTI.deselect_falling_trigger(lineid);
                }
                hil::gpio::InterruptEdge::FallingEdge => {
                    exti::EXTI.deselect_rising_trigger(lineid);
                    exti::EXTI.select_falling_trigger(lineid);
                }
            }

            exti::EXTI.unmask_interrupt(lineid);
        }
    }

    fn disable_interrupts(&self) {
        // TODO handle errors in lineid conversion
        let lineid = self.to_lineid();

        unsafe {
            exti::EXTI.mask_interrupt(lineid);
            exti::EXTI.clear_pending(lineid);

            syscfg::SYSCFG.select_exti_source(self.to_extipin(), syscfg::ExtiPort::PA);

            exti::EXTI.deselect_falling_trigger(lineid);
            exti::EXTI.deselect_rising_trigger(lineid);
        }
    }

    fn set_client(&self, client: &'static dyn hil::gpio::Client) {
        let index = self.get_pin_number();

        unsafe {
            EXTI_CLIENTS.client[usize::from(index as u8)].set(client);
        }
    }

    fn is_pending(&self) -> bool {
        // TODO handle errors in lineid conversion
        let lineid = self.to_lineid();
        unsafe { exti::EXTI.is_pending(lineid) }
        // unsafe {
        //     self.exti_lineid
        //         .map_or(false, |&mut lineid| exti::EXTI.is_pending(lineid))
        // }
    }
}
