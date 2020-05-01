//! Implementation of the STM32L4 GPIO peripheral.
//!
//! Handles configuration and use of GPIOs
//!
//! TODO:
//! - Add lock feature

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
    /// GPIO port output (PP/OD) type register
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
    ascr: ReadWrite<u32, ASCR::Register>,
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

/// Mask applied to 'Pin' enum to extract the pin number
const PIN_NUMBER_MASK: usize = 0x0F;
/// Mask applied to 'Pin' enum to extract the port number
const PORT_NUMBER_MASK: usize = 0xF0;
/// Shift bits applied to 'Pin' enum to extract the port number
const PORT_NUMBER_SHIFT: usize = 4;

/// GPIO pin mode
enum_from_primitive! {
    #[repr(u32)]
    #[derive(PartialEq)]
    pub enum PinNumber {
        PIN0,
        PIN1,
        PIN2,
        PIN3,
        PIN4,
        PIN5,
        PIN6,
        PIN7,
        PIN8,
        PIN9,
        PIN10,
        PIN11,
        PIN12,
        PIN13,
        PIN14,
        PIN15,
    }
}

/// GPIO pin mode
enum_from_primitive! {
    #[repr(u32)]
    #[derive(PartialEq)]
    pub enum Mode {
        Input = 0b00,
        Output = 0b01,
        AlternateFunction = 0b10,
        Analog = 0b11,
    }
}

/// GPIO output pin type (PP/OD)
enum_from_primitive! {
    #[repr(u32)]
    #[derive(PartialEq)]
    pub enum Type {
        PushPull = 0b0,
        OpenDrain = 0b1,
    }
}

/// GPIO pin speed for outputs
enum_from_primitive! {
    #[repr(u32)]
    #[derive(PartialEq)]
    pub enum Speed {
        Low = 0b00,
        Medium = 0b01,
        High = 0b10,
        VeryHigh = 0b11,
    }
}

/// GPIO pin internal pull-up and pull-down
enum_from_primitive! {
    #[repr(u32)]
    enum PullUpPullDown {
        NoPullUpPullDown = 0b00,
        PullUp = 0b01,
        PullDown = 0b10,
    }
}

/// GPIO output pin type (PP/OD)
enum_from_primitive! {
    #[repr(u32)]
    #[derive(PartialEq)]
    pub enum Analog {
        Disconnect = 0b0,
        Connect = 0b1,
    }
}

/// Alternate functions that may be assigned to a `Pin`.
///
/// GPIO pins on the STM32L4xx may serve multiple functions. In addition to
/// the default functionality, each pin can be assigned up to sixteen different
/// alternate functions. The various functions for each pin are described in
/// "Alternate Function" section of the STM32F446RE datasheet[^1].
///
/// Alternate Function bit mapping is shown here.
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

/// Name of the GPIO pins on the STM32L4xx.
///
/// This 'Pin' enum must map correctly against the GpioPin table 'PIN'. 
/// Maybe a macro could help ensure this in future...
/// 
/// This 'Pin' enum describes the maximum pin count that can be handled by 
/// register interface. Whereas the 'PIN' (GpioPin[]) maps from 'Pin' to
/// the actual pins present on this package (LQFP64).
///
/// The lower four bits represent the pin number within the Port, Pins[15..0].
/// The upper three/four bits (7..4) represent the port number, Ports[0..7] -> Ports[A..H].
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

/// Active Pin to carry out operations on
pub struct GpioPin {
    registers: StaticRef<GpioRegisters>,
    mask: u32,
}

impl GpioPin {
    const fn new(port: StaticRef<GpioRegisters>, mask: u32) -> GpioPin {
        GpioPin {
            registers: port,
            mask: mask,
        }
    }
}

/// The STM32L4xx family GPIO interrupt hanlding is somewhat complex.
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

pub struct Port {
    registers: StaticRef<GpioRegisters>,
    clock: rcc::PeripheralClock,
}

/// GPIO Port table
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

/// Name of the GPIO pins actually present on package of LQFP64.
///
/// The 'Pin' enum must map correctly against the GpioPin table 'PIN'.
/// Maybe a macro could help ensure this in future...
///
/// The 'Pin' enum describes the maximum pin count that can be handled by
/// register interface. Whereas the 'PIN' (GpioPin[]) maps from 'Pin' to
/// the actual pins present on this package (LQFP64).
///
/// We need to use `Option<GpioPin>`, instead of just `GpioPin` so that there
/// is a mpping between what the registers can handle and what pins are present.
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
    // /// Retrive the raw GpioPin struct
    // fn get_pin(&self) -> &GpioPin {
    //     unsafe {
    //         &PIN[*self as usize]
    //             .as_ref()
    //             .expect("invalid pin requested, pin not present")
    //     }
    // }

    /// Find the struct Port from the Pin.
    fn get_port(&self) -> &Port {
        let port_num: usize = *self as usize & PORT_NUMBER_MASK;

        unsafe { &PORT[port_num >> PORT_NUMBER_SHIFT] }
    }

    /// Find the pin number, index within the Port.
    fn get_pin_number(&self) -> PinNumber {
        let pin_num = *self as usize;

        PinNumber::from_usize(pin_num & PIN_NUMBER_MASK).expect("get_pin_number: invalid Pin")
    }

    // /// Find the Port number. This is unchecked for out of range errors.
    // fn get_port_number(&self) -> usize {
    //     let mut port_num: usize = *self as usize;

    //     // Right shift p by 4 bits, so we can get rid of pin bits
    //     port_num >>= PORT_NUMBER_SHIFT;
    //     port_num
    // }

    /// Map Pin to LineId
    fn to_lineid(&self) -> exti::LineId {
        let pin_num = *self as usize;
        // How to handle mapping errors? Panic? Silent? debug?
        // This mapping (pin# -> LineId is implicit), should it be explicit?
        exti::LineId::from_usize(pin_num & PIN_NUMBER_MASK).unwrap()
    }

    /// Map Pin to ExtiPin
    fn to_extipin(&self) -> syscfg::ExtiPin {
        let pin_num = *self as usize;
        // How to handle mapping errors? Panic? Silent? debug?
        // This mapping (pin# -> ExtiPin is implicit), should it be explicit?
        syscfg::ExtiPin::from_usize(pin_num & PIN_NUMBER_MASK).unwrap()
    }

    /// Map Pin to ExtiPort
    fn to_extiport(&self) -> syscfg::ExtiPort {
        let port_num: usize = *self as usize;
        // How to handle mapping errors? Panic? Silent? debug?
        // This mapping (pin# -> ExitPort is implicit), should it be explicit?
        syscfg::ExtiPort::from_usize(port_num >> PORT_NUMBER_SHIFT).unwrap()
    }

    /// Get GPIO pin mode
    pub fn get_mode(&self) -> Mode {
        // let pin = self.get_pin();
        // let val = pin.registers.moder.get() & pin.mask;
        // Mode::from_u32(val).unwrap_or(Mode::Input)
        let port = self.get_port();

        let mode = match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.moder.read(MODER::MODER0),
            PinNumber::PIN1 => port.registers.moder.read(MODER::MODER1),
            PinNumber::PIN2 => port.registers.moder.read(MODER::MODER2),
            PinNumber::PIN3 => port.registers.moder.read(MODER::MODER3),
            PinNumber::PIN4 => port.registers.moder.read(MODER::MODER4),
            PinNumber::PIN5 => port.registers.moder.read(MODER::MODER5),
            PinNumber::PIN6 => port.registers.moder.read(MODER::MODER6),
            PinNumber::PIN7 => port.registers.moder.read(MODER::MODER7),
            PinNumber::PIN8 => port.registers.moder.read(MODER::MODER8),
            PinNumber::PIN9 => port.registers.moder.read(MODER::MODER9),
            PinNumber::PIN10 => port.registers.moder.read(MODER::MODER10),
            PinNumber::PIN11 => port.registers.moder.read(MODER::MODER11),
            PinNumber::PIN12 => port.registers.moder.read(MODER::MODER12),
            PinNumber::PIN13 => port.registers.moder.read(MODER::MODER13),
            PinNumber::PIN14 => port.registers.moder.read(MODER::MODER14),
            PinNumber::PIN15 => port.registers.moder.read(MODER::MODER15),
        };
        Mode::from_u32(mode).expect("get_mode translation failed")
    }

    /// Set GPIO pin mode
    pub fn set_mode(&self, mode: Mode) {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.moder.modify(MODER::MODER0.val(mode as u32)),
            PinNumber::PIN1 => port.registers.moder.modify(MODER::MODER1.val(mode as u32)),
            PinNumber::PIN2 => port.registers.moder.modify(MODER::MODER2.val(mode as u32)),
            PinNumber::PIN3 => port.registers.moder.modify(MODER::MODER3.val(mode as u32)),
            PinNumber::PIN4 => port.registers.moder.modify(MODER::MODER4.val(mode as u32)),
            PinNumber::PIN5 => port.registers.moder.modify(MODER::MODER5.val(mode as u32)),
            PinNumber::PIN6 => port.registers.moder.modify(MODER::MODER6.val(mode as u32)),
            PinNumber::PIN7 => port.registers.moder.modify(MODER::MODER7.val(mode as u32)),
            PinNumber::PIN8 => port.registers.moder.modify(MODER::MODER8.val(mode as u32)),
            PinNumber::PIN9 => port.registers.moder.modify(MODER::MODER9.val(mode as u32)),
            PinNumber::PIN10 => port.registers.moder.modify(MODER::MODER10.val(mode as u32)),
            PinNumber::PIN11 => port.registers.moder.modify(MODER::MODER11.val(mode as u32)),
            PinNumber::PIN12 => port.registers.moder.modify(MODER::MODER12.val(mode as u32)),
            PinNumber::PIN13 => port.registers.moder.modify(MODER::MODER13.val(mode as u32)),
            PinNumber::PIN14 => port.registers.moder.modify(MODER::MODER14.val(mode as u32)),
            PinNumber::PIN15 => port.registers.moder.modify(MODER::MODER15.val(mode as u32)),
        }
    }

    /// Get output pin type, push-pull type or open-drain
    fn get_output_type(&self) -> Type {
        let port = self.get_port();

        let otype = match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.otyper.read(OTYPER::OT0),
            PinNumber::PIN1 => port.registers.otyper.read(OTYPER::OT1),
            PinNumber::PIN2 => port.registers.otyper.read(OTYPER::OT2),
            PinNumber::PIN3 => port.registers.otyper.read(OTYPER::OT3),
            PinNumber::PIN4 => port.registers.otyper.read(OTYPER::OT4),
            PinNumber::PIN5 => port.registers.otyper.read(OTYPER::OT5),
            PinNumber::PIN6 => port.registers.otyper.read(OTYPER::OT6),
            PinNumber::PIN7 => port.registers.otyper.read(OTYPER::OT7),
            PinNumber::PIN8 => port.registers.otyper.read(OTYPER::OT8),
            PinNumber::PIN9 => port.registers.otyper.read(OTYPER::OT9),
            PinNumber::PIN10 => port.registers.otyper.read(OTYPER::OT10),
            PinNumber::PIN11 => port.registers.otyper.read(OTYPER::OT11),
            PinNumber::PIN12 => port.registers.otyper.read(OTYPER::OT12),
            PinNumber::PIN13 => port.registers.otyper.read(OTYPER::OT13),
            PinNumber::PIN14 => port.registers.otyper.read(OTYPER::OT14),
            PinNumber::PIN15 => port.registers.otyper.read(OTYPER::OT15),
        };
        Type::from_u32(otype).expect("get_output_type translation failed")
    }

    /// Set output pin to be push-pull/open-drain type
    fn set_output_type(&self, otype: Type) {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.otyper.modify(OTYPER::OT0.val(otype as u32)),
            PinNumber::PIN1 => port.registers.otyper.modify(OTYPER::OT1.val(otype as u32)),
            PinNumber::PIN2 => port.registers.otyper.modify(OTYPER::OT2.val(otype as u32)),
            PinNumber::PIN3 => port.registers.otyper.modify(OTYPER::OT3.val(otype as u32)),
            PinNumber::PIN4 => port.registers.otyper.modify(OTYPER::OT4.val(otype as u32)),
            PinNumber::PIN5 => port.registers.otyper.modify(OTYPER::OT5.val(otype as u32)),
            PinNumber::PIN6 => port.registers.otyper.modify(OTYPER::OT6.val(otype as u32)),
            PinNumber::PIN7 => port.registers.otyper.modify(OTYPER::OT7.val(otype as u32)),
            PinNumber::PIN8 => port.registers.otyper.modify(OTYPER::OT8.val(otype as u32)),
            PinNumber::PIN9 => port.registers.otyper.modify(OTYPER::OT9.val(otype as u32)),
            PinNumber::PIN10 => port.registers.otyper.modify(OTYPER::OT10.val(otype as u32)),
            PinNumber::PIN11 => port.registers.otyper.modify(OTYPER::OT11.val(otype as u32)),
            PinNumber::PIN12 => port.registers.otyper.modify(OTYPER::OT12.val(otype as u32)),
            PinNumber::PIN13 => port.registers.otyper.modify(OTYPER::OT13.val(otype as u32)),
            PinNumber::PIN14 => port.registers.otyper.modify(OTYPER::OT14.val(otype as u32)),
            PinNumber::PIN15 => port.registers.otyper.modify(OTYPER::OT15.val(otype as u32)),
        }
    }

    /// Get GPIO pin speed
    pub fn get_speed(&self) -> Speed {
        let port = self.get_port();

        let mode = match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR0),
            PinNumber::PIN1 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR1),
            PinNumber::PIN2 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR2),
            PinNumber::PIN3 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR3),
            PinNumber::PIN4 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR4),
            PinNumber::PIN5 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR5),
            PinNumber::PIN6 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR6),
            PinNumber::PIN7 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR7),
            PinNumber::PIN8 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR8),
            PinNumber::PIN9 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR9),
            PinNumber::PIN10 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR10),
            PinNumber::PIN11 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR11),
            PinNumber::PIN12 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR12),
            PinNumber::PIN13 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR13),
            PinNumber::PIN14 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR14),
            PinNumber::PIN15 => port.registers.ospeedr.read(OSPEEDR::OSPEEDR15),
        };
        Speed::from_u32(mode).expect("get_speed: translation failed")
    }

    /// Set GPIO pin speed
    pub fn set_speed(&self, speed: Speed) {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR0.val(speed as u32)),
            PinNumber::PIN1 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR1.val(speed as u32)),
            PinNumber::PIN2 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR2.val(speed as u32)),
            PinNumber::PIN3 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR3.val(speed as u32)),
            PinNumber::PIN4 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR4.val(speed as u32)),
            PinNumber::PIN5 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR5.val(speed as u32)),
            PinNumber::PIN6 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR6.val(speed as u32)),
            PinNumber::PIN7 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR7.val(speed as u32)),
            PinNumber::PIN8 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR8.val(speed as u32)),
            PinNumber::PIN9 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR9.val(speed as u32)),
            PinNumber::PIN10 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR10.val(speed as u32)),
            PinNumber::PIN11 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR11.val(speed as u32)),
            PinNumber::PIN12 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR12.val(speed as u32)),
            PinNumber::PIN13 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR13.val(speed as u32)),
            PinNumber::PIN14 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR14.val(speed as u32)),
            PinNumber::PIN15 => port
                .registers
                .ospeedr
                .modify(OSPEEDR::OSPEEDR15.val(speed as u32)),
        }
    }

    /// Get pin pull-up/pull-down setting
    fn get_pullup_pulldown(&self) -> PullUpPullDown {
        let port = self.get_port();

        let val = match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.pupdr.read(PUPDR::PUPDR0),
            PinNumber::PIN1 => port.registers.pupdr.read(PUPDR::PUPDR1),
            PinNumber::PIN2 => port.registers.pupdr.read(PUPDR::PUPDR2),
            PinNumber::PIN3 => port.registers.pupdr.read(PUPDR::PUPDR3),
            PinNumber::PIN4 => port.registers.pupdr.read(PUPDR::PUPDR4),
            PinNumber::PIN5 => port.registers.pupdr.read(PUPDR::PUPDR5),
            PinNumber::PIN6 => port.registers.pupdr.read(PUPDR::PUPDR6),
            PinNumber::PIN7 => port.registers.pupdr.read(PUPDR::PUPDR7),
            PinNumber::PIN8 => port.registers.pupdr.read(PUPDR::PUPDR8),
            PinNumber::PIN9 => port.registers.pupdr.read(PUPDR::PUPDR9),
            PinNumber::PIN10 => port.registers.pupdr.read(PUPDR::PUPDR10),
            PinNumber::PIN11 => port.registers.pupdr.read(PUPDR::PUPDR11),
            PinNumber::PIN12 => port.registers.pupdr.read(PUPDR::PUPDR12),
            PinNumber::PIN13 => port.registers.pupdr.read(PUPDR::PUPDR13),
            PinNumber::PIN14 => port.registers.pupdr.read(PUPDR::PUPDR14),
            PinNumber::PIN15 => port.registers.pupdr.read(PUPDR::PUPDR15),
        };

        PullUpPullDown::from_u32(val).unwrap_or(PullUpPullDown::NoPullUpPullDown)
    }

    /// Set pin pull-up/pull-down setting
    fn set_pullup_pulldown(&self, pupd: PullUpPullDown) {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.pupdr.modify(PUPDR::PUPDR0.val(pupd as u32)),
            PinNumber::PIN1 => port.registers.pupdr.modify(PUPDR::PUPDR1.val(pupd as u32)),
            PinNumber::PIN2 => port.registers.pupdr.modify(PUPDR::PUPDR2.val(pupd as u32)),
            PinNumber::PIN3 => port.registers.pupdr.modify(PUPDR::PUPDR3.val(pupd as u32)),
            PinNumber::PIN4 => port.registers.pupdr.modify(PUPDR::PUPDR4.val(pupd as u32)),
            PinNumber::PIN5 => port.registers.pupdr.modify(PUPDR::PUPDR5.val(pupd as u32)),
            PinNumber::PIN6 => port.registers.pupdr.modify(PUPDR::PUPDR6.val(pupd as u32)),
            PinNumber::PIN7 => port.registers.pupdr.modify(PUPDR::PUPDR7.val(pupd as u32)),
            PinNumber::PIN8 => port.registers.pupdr.modify(PUPDR::PUPDR8.val(pupd as u32)),
            PinNumber::PIN9 => port.registers.pupdr.modify(PUPDR::PUPDR9.val(pupd as u32)),
            PinNumber::PIN10 => port.registers.pupdr.modify(PUPDR::PUPDR10.val(pupd as u32)),
            PinNumber::PIN11 => port.registers.pupdr.modify(PUPDR::PUPDR11.val(pupd as u32)),
            PinNumber::PIN12 => port.registers.pupdr.modify(PUPDR::PUPDR12.val(pupd as u32)),
            PinNumber::PIN13 => port.registers.pupdr.modify(PUPDR::PUPDR13.val(pupd as u32)),
            PinNumber::PIN14 => port.registers.pupdr.modify(PUPDR::PUPDR14.val(pupd as u32)),
            PinNumber::PIN15 => port.registers.pupdr.modify(PUPDR::PUPDR15.val(pupd as u32)),
        }
    }

    /// Get pin analogu configuration, this indicates whether the pin and the
    /// ADC are connected.
    fn get_analog_config(&self) -> Analog {
        let port = self.get_port();

        let val = match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.ascr.read(ASCR::ASC0),
            PinNumber::PIN1 => port.registers.ascr.read(ASCR::ASC1),
            PinNumber::PIN2 => port.registers.ascr.read(ASCR::ASC2),
            PinNumber::PIN3 => port.registers.ascr.read(ASCR::ASC3),
            PinNumber::PIN4 => port.registers.ascr.read(ASCR::ASC4),
            PinNumber::PIN5 => port.registers.ascr.read(ASCR::ASC5),
            PinNumber::PIN6 => port.registers.ascr.read(ASCR::ASC6),
            PinNumber::PIN7 => port.registers.ascr.read(ASCR::ASC7),
            PinNumber::PIN8 => port.registers.ascr.read(ASCR::ASC8),
            PinNumber::PIN9 => port.registers.ascr.read(ASCR::ASC9),
            PinNumber::PIN10 => port.registers.ascr.read(ASCR::ASC10),
            PinNumber::PIN11 => port.registers.ascr.read(ASCR::ASC11),
            PinNumber::PIN12 => port.registers.ascr.read(ASCR::ASC12),
            PinNumber::PIN13 => port.registers.ascr.read(ASCR::ASC13),
            PinNumber::PIN14 => port.registers.ascr.read(ASCR::ASC14),
            PinNumber::PIN15 => port.registers.ascr.read(ASCR::ASC15),
        };

        Analog::from_u32(val).unwrap_or(Analog::Disconnect)
    }

    /// Set pin analog configuration, this controls whether the pin and the
    /// ADC are connected.
    fn set_analog_config(&self, swicth: Analog) {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.ascr.modify(ASCR::ASC0.val(swicth as u32)),
            PinNumber::PIN1 => port.registers.ascr.modify(ASCR::ASC1.val(swicth as u32)),
            PinNumber::PIN2 => port.registers.ascr.modify(ASCR::ASC2.val(swicth as u32)),
            PinNumber::PIN3 => port.registers.ascr.modify(ASCR::ASC3.val(swicth as u32)),
            PinNumber::PIN4 => port.registers.ascr.modify(ASCR::ASC4.val(swicth as u32)),
            PinNumber::PIN5 => port.registers.ascr.modify(ASCR::ASC5.val(swicth as u32)),
            PinNumber::PIN6 => port.registers.ascr.modify(ASCR::ASC6.val(swicth as u32)),
            PinNumber::PIN7 => port.registers.ascr.modify(ASCR::ASC7.val(swicth as u32)),
            PinNumber::PIN8 => port.registers.ascr.modify(ASCR::ASC8.val(swicth as u32)),
            PinNumber::PIN9 => port.registers.ascr.modify(ASCR::ASC9.val(swicth as u32)),
            PinNumber::PIN10 => port.registers.ascr.modify(ASCR::ASC10.val(swicth as u32)),
            PinNumber::PIN11 => port.registers.ascr.modify(ASCR::ASC11.val(swicth as u32)),
            PinNumber::PIN12 => port.registers.ascr.modify(ASCR::ASC12.val(swicth as u32)),
            PinNumber::PIN13 => port.registers.ascr.modify(ASCR::ASC13.val(swicth as u32)),
            PinNumber::PIN14 => port.registers.ascr.modify(ASCR::ASC14.val(swicth as u32)),
            PinNumber::PIN15 => port.registers.ascr.modify(ASCR::ASC15.val(swicth as u32)),
        };
    }

    pub fn set_alternate_function(&self, af: AlternateFunction) {
        let port = self.get_port();

        match self.get_pin_number() {
            // AF register low
            PinNumber::PIN0 => port.registers.afrl.modify(AFRL::AFRL0.val(af as u32)),
            PinNumber::PIN1 => port.registers.afrl.modify(AFRL::AFRL1.val(af as u32)),
            PinNumber::PIN2 => port.registers.afrl.modify(AFRL::AFRL2.val(af as u32)),
            PinNumber::PIN3 => port.registers.afrl.modify(AFRL::AFRL3.val(af as u32)),
            PinNumber::PIN4 => port.registers.afrl.modify(AFRL::AFRL4.val(af as u32)),
            PinNumber::PIN5 => port.registers.afrl.modify(AFRL::AFRL5.val(af as u32)),
            PinNumber::PIN6 => port.registers.afrl.modify(AFRL::AFRL6.val(af as u32)),
            PinNumber::PIN7 => port.registers.afrl.modify(AFRL::AFRL7.val(af as u32)),
            // AF register high
            PinNumber::PIN8 => port.registers.afrh.modify(AFRH::AFRH8.val(af as u32)),
            PinNumber::PIN9 => port.registers.afrh.modify(AFRH::AFRH9.val(af as u32)),
            PinNumber::PIN10 => port.registers.afrh.modify(AFRH::AFRH10.val(af as u32)),
            PinNumber::PIN11 => port.registers.afrh.modify(AFRH::AFRH11.val(af as u32)),
            PinNumber::PIN12 => port.registers.afrh.modify(AFRH::AFRH12.val(af as u32)),
            PinNumber::PIN13 => port.registers.afrh.modify(AFRH::AFRH13.val(af as u32)),
            PinNumber::PIN14 => port.registers.afrh.modify(AFRH::AFRH14.val(af as u32)),
            PinNumber::PIN15 => port.registers.afrh.modify(AFRH::AFRH15.val(af as u32)),
        }
    }

    fn set_output_high(&self) {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.bsrr.write(BSRR::BS0::SET),
            PinNumber::PIN1 => port.registers.bsrr.write(BSRR::BS1::SET),
            PinNumber::PIN2 => port.registers.bsrr.write(BSRR::BS2::SET),
            PinNumber::PIN3 => port.registers.bsrr.write(BSRR::BS3::SET),
            PinNumber::PIN4 => port.registers.bsrr.write(BSRR::BS4::SET),
            PinNumber::PIN5 => port.registers.bsrr.write(BSRR::BS5::SET),
            PinNumber::PIN6 => port.registers.bsrr.write(BSRR::BS6::SET),
            PinNumber::PIN7 => port.registers.bsrr.write(BSRR::BS7::SET),
            PinNumber::PIN8 => port.registers.bsrr.write(BSRR::BS8::SET),
            PinNumber::PIN9 => port.registers.bsrr.write(BSRR::BS9::SET),
            PinNumber::PIN10 => port.registers.bsrr.write(BSRR::BS10::SET),
            PinNumber::PIN11 => port.registers.bsrr.write(BSRR::BS11::SET),
            PinNumber::PIN12 => port.registers.bsrr.write(BSRR::BS12::SET),
            PinNumber::PIN13 => port.registers.bsrr.write(BSRR::BS13::SET),
            PinNumber::PIN14 => port.registers.bsrr.write(BSRR::BS14::SET),
            PinNumber::PIN15 => port.registers.bsrr.write(BSRR::BS15::SET),
        }
    }

    fn set_output_low(&self) {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.bsrr.write(BSRR::BR0::SET),
            PinNumber::PIN1 => port.registers.bsrr.write(BSRR::BR1::SET),
            PinNumber::PIN2 => port.registers.bsrr.write(BSRR::BR2::SET),
            PinNumber::PIN3 => port.registers.bsrr.write(BSRR::BR3::SET),
            PinNumber::PIN4 => port.registers.bsrr.write(BSRR::BR4::SET),
            PinNumber::PIN5 => port.registers.bsrr.write(BSRR::BR5::SET),
            PinNumber::PIN6 => port.registers.bsrr.write(BSRR::BR6::SET),
            PinNumber::PIN7 => port.registers.bsrr.write(BSRR::BR7::SET),
            PinNumber::PIN8 => port.registers.bsrr.write(BSRR::BR8::SET),
            PinNumber::PIN9 => port.registers.bsrr.write(BSRR::BR9::SET),
            PinNumber::PIN10 => port.registers.bsrr.write(BSRR::BR10::SET),
            PinNumber::PIN11 => port.registers.bsrr.write(BSRR::BR11::SET),
            PinNumber::PIN12 => port.registers.bsrr.write(BSRR::BR12::SET),
            PinNumber::PIN13 => port.registers.bsrr.write(BSRR::BR13::SET),
            PinNumber::PIN14 => port.registers.bsrr.write(BSRR::BR14::SET),
            PinNumber::PIN15 => port.registers.bsrr.write(BSRR::BR15::SET),
        }
    }

    fn is_output_high(&self) -> bool {
        let port = self.get_port();

        match self.get_pin_number() {
            PinNumber::PIN0 => port.registers.odr.is_set(ODR::ODR0),
            PinNumber::PIN1 => port.registers.odr.is_set(ODR::ODR1),
            PinNumber::PIN2 => port.registers.odr.is_set(ODR::ODR2),
            PinNumber::PIN3 => port.registers.odr.is_set(ODR::ODR3),
            PinNumber::PIN4 => port.registers.odr.is_set(ODR::ODR4),
            PinNumber::PIN5 => port.registers.odr.is_set(ODR::ODR5),
            PinNumber::PIN6 => port.registers.odr.is_set(ODR::ODR6),
            PinNumber::PIN7 => port.registers.odr.is_set(ODR::ODR7),
            PinNumber::PIN8 => port.registers.odr.is_set(ODR::ODR8),
            PinNumber::PIN9 => port.registers.odr.is_set(ODR::ODR9),
            PinNumber::PIN10 => port.registers.odr.is_set(ODR::ODR10),
            PinNumber::PIN11 => port.registers.odr.is_set(ODR::ODR11),
            PinNumber::PIN12 => port.registers.odr.is_set(ODR::ODR12),
            PinNumber::PIN13 => port.registers.odr.is_set(ODR::ODR13),
            PinNumber::PIN14 => port.registers.odr.is_set(ODR::ODR14),
            PinNumber::PIN15 => port.registers.odr.is_set(ODR::ODR15),
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
            PinNumber::PIN0 => port.registers.idr.is_set(IDR::IDR0),
            PinNumber::PIN1 => port.registers.idr.is_set(IDR::IDR1),
            PinNumber::PIN2 => port.registers.idr.is_set(IDR::IDR2),
            PinNumber::PIN3 => port.registers.idr.is_set(IDR::IDR3),
            PinNumber::PIN4 => port.registers.idr.is_set(IDR::IDR4),
            PinNumber::PIN5 => port.registers.idr.is_set(IDR::IDR5),
            PinNumber::PIN6 => port.registers.idr.is_set(IDR::IDR6),
            PinNumber::PIN7 => port.registers.idr.is_set(IDR::IDR7),
            PinNumber::PIN8 => port.registers.idr.is_set(IDR::IDR8),
            PinNumber::PIN9 => port.registers.idr.is_set(IDR::IDR9),
            PinNumber::PIN10 => port.registers.idr.is_set(IDR::IDR10),
            PinNumber::PIN11 => port.registers.idr.is_set(IDR::IDR11),
            PinNumber::PIN12 => port.registers.idr.is_set(IDR::IDR12),
            PinNumber::PIN13 => port.registers.idr.is_set(IDR::IDR13),
            PinNumber::PIN14 => port.registers.idr.is_set(IDR::IDR14),
            PinNumber::PIN15 => port.registers.idr.is_set(IDR::IDR15),
        }
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

        self.set_analog_config(Analog::Disconnect);
        self.set_output_type(Type::PushPull);
        self.set_speed(Speed::High);
        self.set_mode(Mode::Output);
        hil::gpio::Configuration::Output
    }

    /// Input mode default is no internal pull-up, no pull-down (i.e.,
    /// floating). Also upon setting the mode as input, the internal schmitt
    /// trigger is automatically activated. Schmitt trigger is deactivated in
    /// Analog Mode.
    fn make_input(&self) -> hil::gpio::Configuration {
        self.get_port().clock.enable();

        self.set_analog_config(Analog::Disconnect);
        self.set_mode(Mode::Input);
        hil::gpio::Configuration::Input
    }

    fn deactivate_to_low_power(&self) {
        self.get_port().clock.enable();

        self.set_mode(Mode::Analog);
        self.set_analog_config(Analog::Disconnect);
    }

    fn disable_output(&self) -> hil::gpio::Configuration {
        self.get_port().clock.enable();

        self.set_mode(Mode::Analog);
        self.set_analog_config(Analog::Disconnect);
        hil::gpio::Configuration::LowPower
    }

    fn disable_input(&self) -> hil::gpio::Configuration {
        self.get_port().clock.enable();

        self.set_mode(Mode::Analog);
        self.set_analog_config(Analog::Disconnect);
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
            Mode::Output => hil::gpio::Configuration::Output,
            Mode::Analog => hil::gpio::Configuration::LowPower,
            Mode::AlternateFunction => hil::gpio::Configuration::Function,
        }
    }

    fn is_input(&self) -> bool {
        self.get_mode() == Mode::Input
    }

    fn is_output(&self) -> bool {
        self.get_mode() == Mode::Output
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
