//! Implementation of the STM32L4 External interrupts controller.
//!
//! Handles generation, masking, rising/falling selection, status
//! and software emulation of event/interrupt requests,

use cortexm4::support::atomic;
use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::common::cells::OptionalCell;
use kernel::common::registers::{register_bitfields, ReadWrite};
use kernel::common::StaticRef;
use kernel::hil;

use crate::gpio;
use crate::memory_map;

/// External interrupt/event controller
#[repr(C)]
struct ExtiRegisters {
    /// Interrupt mask register 1
    imr1: ReadWrite<u32, IMR1::Register>,
    /// Event mask register 1
    emr1: ReadWrite<u32, EMR1::Register>,
    /// Rising Trigger selection register 1
    rtsr1: ReadWrite<u32, RTSR1::Register>,
    /// Falling Trigger selection register 1
    ftsr1: ReadWrite<u32, FTSR1::Register>,
    /// Software interrupt event register 1
    swier1: ReadWrite<u32, SWIER1::Register>,
    /// Pending register 1
    pr1: ReadWrite<u32, PR1::Register>,
    /// Interrupt mask register 2
    imr2: ReadWrite<u32, IMR2::Register>,
    /// Event mask register 2
    emr2: ReadWrite<u32, EMR2::Register>,
    /// Rising Trigger selection register 2
    rtsr2: ReadWrite<u32, RTSR2::Register>,
    /// Falling Trigger selection register 2
    ftsr2: ReadWrite<u32, FTSR2::Register>,
    /// Software interrupt event register 2
    swier2: ReadWrite<u32, SWIER2::Register>,
    /// Pending register 2
    pr2: ReadWrite<u32, PR2::Register>,
}

register_bitfields![u32,
    IMR1 [
        /// Interrupt Mask on line 0
        IM0 OFFSET(0) NUMBITS(1) [],
        /// Interrupt Mask on line 1
        IM1 OFFSET(1) NUMBITS(1) [],
        /// Interrupt Mask on line 2
        IM2 OFFSET(2) NUMBITS(1) [],
        /// Interrupt Mask on line 3
        IM3 OFFSET(3) NUMBITS(1) [],
        /// Interrupt Mask on line 4
        IM4 OFFSET(4) NUMBITS(1) [],
        /// Interrupt Mask on line 5
        IM5 OFFSET(5) NUMBITS(1) [],
        /// Interrupt Mask on line 6
        IM6 OFFSET(6) NUMBITS(1) [],
        /// Interrupt Mask on line 7
        IM7 OFFSET(7) NUMBITS(1) [],
        /// Interrupt Mask on line 8
        IM8 OFFSET(8) NUMBITS(1) [],
        /// Interrupt Mask on line 9
        IM9 OFFSET(9) NUMBITS(1) [],
        /// Interrupt Mask on line 10
        IM10 OFFSET(10) NUMBITS(1) [],
        /// Interrupt Mask on line 11
        IM11 OFFSET(11) NUMBITS(1) [],
        /// Interrupt Mask on line 12
        IM12 OFFSET(12) NUMBITS(1) [],
        /// Interrupt Mask on line 13
        IM13 OFFSET(13) NUMBITS(1) [],
        /// Interrupt Mask on line 14
        IM14 OFFSET(14) NUMBITS(1) [],
        /// Interrupt Mask on line 15
        IM15 OFFSET(15) NUMBITS(1) [],
        /// Interrupt Mask on line 16
        IM16 OFFSET(16) NUMBITS(1) [],
        /// Interrupt Mask on line 17
        IM17 OFFSET(17) NUMBITS(1) [],
        /// Interrupt Mask on line 18
        IM18 OFFSET(18) NUMBITS(1) [],
        /// Interrupt Mask on line 19
        IM19 OFFSET(19) NUMBITS(1) [],
        /// Interrupt Mask on line 20
        IM20 OFFSET(20) NUMBITS(1) [],
        /// Interrupt Mask on line 21
        IM21 OFFSET(21) NUMBITS(1) [],
        /// Interrupt Mask on line 22
        IM22 OFFSET(22) NUMBITS(1) []
    ],
    EMR1 [
        /// Event Mask on line 0
        EM0 OFFSET(0) NUMBITS(1) [],
        /// Event Mask on line 1
        EM1 OFFSET(1) NUMBITS(1) [],
        /// Event Mask on line 2
        EM2 OFFSET(2) NUMBITS(1) [],
        /// Event Mask on line 3
        EM3 OFFSET(3) NUMBITS(1) [],
        /// Event Mask on line 4
        EM4 OFFSET(4) NUMBITS(1) [],
        /// Event Mask on line 5
        EM5 OFFSET(5) NUMBITS(1) [],
        /// Event Mask on line 6
        EM6 OFFSET(6) NUMBITS(1) [],
        /// Event Mask on line 7
        EM7 OFFSET(7) NUMBITS(1) [],
        /// Event Mask on line 8
        EM8 OFFSET(8) NUMBITS(1) [],
        /// Event Mask on line 9
        EM9 OFFSET(9) NUMBITS(1) [],
        /// Event Mask on line 10
        EM10 OFFSET(10) NUMBITS(1) [],
        /// Event Mask on line 11
        EM11 OFFSET(11) NUMBITS(1) [],
        /// Event Mask on line 12
        EM12 OFFSET(12) NUMBITS(1) [],
        /// Event Mask on line 13
        EM13 OFFSET(13) NUMBITS(1) [],
        /// Event Mask on line 14
        EM14 OFFSET(14) NUMBITS(1) [],
        /// Event Mask on line 15
        EM15 OFFSET(15) NUMBITS(1) [],
        /// Event Mask on line 16
        EM16 OFFSET(16) NUMBITS(1) [],
        /// Event Mask on line 17
        EM17 OFFSET(17) NUMBITS(1) [],
        /// Event Mask on line 18
        EM18 OFFSET(18) NUMBITS(1) [],
        /// Event Mask on line 19
        EM19 OFFSET(19) NUMBITS(1) [],
        /// Event Mask on line 20
        EM20 OFFSET(20) NUMBITS(1) [],
        /// Event Mask on line 21
        EM21 OFFSET(21) NUMBITS(1) [],
        /// Event Mask on line 22
        EM22 OFFSET(22) NUMBITS(1) []
    ],
    RTSR1 [
        /// Rising trigger event configuration of line 0
        RT0 OFFSET(0) NUMBITS(1) [],
        /// Rising trigger event configuration of line 1
        RT1 OFFSET(1) NUMBITS(1) [],
        /// Rising trigger event configuration of line 2
        RT2 OFFSET(2) NUMBITS(1) [],
        /// Rising trigger event configuration of line 3
        RT3 OFFSET(3) NUMBITS(1) [],
        /// Rising trigger event configuration of line 4
        RT4 OFFSET(4) NUMBITS(1) [],
        /// Rising trigger event configuration of line 5
        RT5 OFFSET(5) NUMBITS(1) [],
        /// Rising trigger event configuration of line 6
        RT6 OFFSET(6) NUMBITS(1) [],
        /// Rising trigger event configuration of line 7
        RT7 OFFSET(7) NUMBITS(1) [],
        /// Rising trigger event configuration of line 8
        RT8 OFFSET(8) NUMBITS(1) [],
        /// Rising trigger event configuration of line 9
        RT9 OFFSET(9) NUMBITS(1) [],
        /// Rising trigger event configuration of line 10
        RT10 OFFSET(10) NUMBITS(1) [],
        /// Rising trigger event configuration of line 11
        RT11 OFFSET(11) NUMBITS(1) [],
        /// Rising trigger event configuration of line 12
        RT12 OFFSET(12) NUMBITS(1) [],
        /// Rising trigger event configuration of line 13
        RT13 OFFSET(13) NUMBITS(1) [],
        /// Rising trigger event configuration of line 14
        RT14 OFFSET(14) NUMBITS(1) [],
        /// Rising trigger event configuration of line 15
        RT15 OFFSET(15) NUMBITS(1) [],
        /// Rising trigger event configuration of line 16
        RT16 OFFSET(16) NUMBITS(1) [],
        /// Rising trigger event configuration of line 18
        RT18 OFFSET(18) NUMBITS(1) [],
        /// Rising trigger event configuration of line 19
        RT19 OFFSET(19) NUMBITS(1) [],
        /// Rising trigger event configuration of line 20
        RT20 OFFSET(20) NUMBITS(1) [],
        /// Rising trigger event configuration of line 21
        RT21 OFFSET(21) NUMBITS(1) [],
        /// Rising trigger event configuration of line 22
        RT22 OFFSET(22) NUMBITS(1) []
    ],
    FTSR1 [
        /// Falling trigger event configuration of line 0
        FT0 OFFSET(0) NUMBITS(1) [],
        /// Falling trigger event configuration of line 1
        FT1 OFFSET(1) NUMBITS(1) [],
        /// Falling trigger event configuration of line 2
        FT2 OFFSET(2) NUMBITS(1) [],
        /// Falling trigger event configuration of line 3
        FT3 OFFSET(3) NUMBITS(1) [],
        /// Falling trigger event configuration of line 4
        FT4 OFFSET(4) NUMBITS(1) [],
        /// Falling trigger event configuration of line 5
        FT5 OFFSET(5) NUMBITS(1) [],
        /// Falling trigger event configuration of line 6
        FT6 OFFSET(6) NUMBITS(1) [],
        /// Falling trigger event configuration of line 7
        FT7 OFFSET(7) NUMBITS(1) [],
        /// Falling trigger event configuration of line 8
        FT8 OFFSET(8) NUMBITS(1) [],
        /// Falling trigger event configuration of line 9
        FT9 OFFSET(9) NUMBITS(1) [],
        /// Falling trigger event configuration of line 10
        FT10 OFFSET(10) NUMBITS(1) [],
        /// Falling trigger event configuration of line 11
        FT11 OFFSET(11) NUMBITS(1) [],
        /// Falling trigger event configuration of line 12
        FT12 OFFSET(12) NUMBITS(1) [],
        /// Falling trigger event configuration of line 13
        FT13 OFFSET(13) NUMBITS(1) [],
        /// Falling trigger event configuration of line 14
        FT14 OFFSET(14) NUMBITS(1) [],
        /// Falling trigger event configuration of line 15
        FT15 OFFSET(15) NUMBITS(1) [],
        /// Falling trigger event configuration of line 16
        FT16 OFFSET(16) NUMBITS(1) [],
        /// Falling trigger event configuration of line 18
        FT18 OFFSET(18) NUMBITS(1) [],
        /// Falling trigger event configuration of line 19
        FT19 OFFSET(19) NUMBITS(1) [],
        /// Falling trigger event configuration of line 20
        FT20 OFFSET(20) NUMBITS(1) [],
        /// Falling trigger event configuration of line 21
        FT21 OFFSET(21) NUMBITS(1) [],
        /// Falling trigger event configuration of line 22
        FT22 OFFSET(22) NUMBITS(1) []
    ],
    SWIER1 [
        /// Software Interrupt on line 0
        SWI0 OFFSET(0) NUMBITS(1) [],
        /// Software Interrupt on line 1
        SWI1 OFFSET(1) NUMBITS(1) [],
        /// Software Interrupt on line 2
        SWI2 OFFSET(2) NUMBITS(1) [],
        /// Software Interrupt on line 3
        SWI3 OFFSET(3) NUMBITS(1) [],
        /// Software Interrupt on line 4
        SWI4 OFFSET(4) NUMBITS(1) [],
        /// Software Interrupt on line 5
        SWI5 OFFSET(5) NUMBITS(1) [],
        /// Software Interrupt on line 6
        SWI6 OFFSET(6) NUMBITS(1) [],
        /// Software Interrupt on line 7
        SWI7 OFFSET(7) NUMBITS(1) [],
        /// Software Interrupt on line 8
        SWI8 OFFSET(8) NUMBITS(1) [],
        /// Software Interrupt on line 9
        SWI9 OFFSET(9) NUMBITS(1) [],
        /// Software Interrupt on line 10
        SWI10 OFFSET(10) NUMBITS(1) [],
        /// Software Interrupt on line 11
        SWI11 OFFSET(11) NUMBITS(1) [],
        /// Software Interrupt on line 12
        SWI12 OFFSET(12) NUMBITS(1) [],
        /// Software Interrupt on line 13
        SWI13 OFFSET(13) NUMBITS(1) [],
        /// Software Interrupt on line 14
        SWI14 OFFSET(14) NUMBITS(1) [],
        /// Software Interrupt on line 15
        SWI15 OFFSET(15) NUMBITS(1) [],
        /// Software Interrupt on line 16
        SWI16 OFFSET(16) NUMBITS(1) [],
        /// Software Interrupt on line 18
        SWI18 OFFSET(18) NUMBITS(1) [],
        /// Software Interrupt on line 19
        SWI19 OFFSET(19) NUMBITS(1) [],
        /// Software Interrupt on line 20
        SWI20 OFFSET(20) NUMBITS(1) [],
        /// Software Interrupt on line 21
        SWI21 OFFSET(21) NUMBITS(1) [],
        /// Software Interrupt on line 22
        SWI22 OFFSET(22) NUMBITS(1) []
    ],
    PR1 [
        /// Pending interrupt flag 0
        PIF0 OFFSET(0) NUMBITS(1) [],
        /// Pending interrupt flag 1
        PIF1 OFFSET(1) NUMBITS(1) [],
        /// Pending interrupt flag 2
        PIF2 OFFSET(2) NUMBITS(1) [],
        /// Pending interrupt flag 3
        PIF3 OFFSET(3) NUMBITS(1) [],
        /// Pending interrupt flag 4
        PIF4 OFFSET(4) NUMBITS(1) [],
        /// Pending interrupt flag 5
        PIF5 OFFSET(5) NUMBITS(1) [],
        /// Pending interrupt flag 6
        PIF6 OFFSET(6) NUMBITS(1) [],
        /// Pending interrupt flag 7
        PIF7 OFFSET(7) NUMBITS(1) [],
        /// Pending interrupt flag 8
        PIF8 OFFSET(8) NUMBITS(1) [],
        /// Pending interrupt flag 9
        PIF9 OFFSET(9) NUMBITS(1) [],
        /// Pending interrupt flag 10
        PIF10 OFFSET(10) NUMBITS(1) [],
        /// Pending interrupt flag 11
        PIF11 OFFSET(11) NUMBITS(1) [],
        /// Pending interrupt flag 12
        PIF12 OFFSET(12) NUMBITS(1) [],
        /// Pending interrupt flag 13
        PIF13 OFFSET(13) NUMBITS(1) [],
        /// Pending interrupt flag 14
        PIF14 OFFSET(14) NUMBITS(1) [],
        /// Pending interrupt flag 15
        PIF15 OFFSET(15) NUMBITS(1) [],
        /// Pending interrupt flag 16
        PIF16 OFFSET(16) NUMBITS(1) [],
        /// Pending interrupt flag 17
        PIF17 OFFSET(17) NUMBITS(1) [],
        /// Pending interrupt flag 18
        PIF18 OFFSET(18) NUMBITS(1) [],
        /// Pending interrupt flag 19
        PIF19 OFFSET(19) NUMBITS(1) [],
        /// Pending interrupt flag 20
        PIF20 OFFSET(20) NUMBITS(1) [],
        /// Pending interrupt flag 21
        PIF21 OFFSET(21) NUMBITS(1) [],
        /// Pending interrupt flag 22
        PIF22 OFFSET(22) NUMBITS(1) []
    ],
    IMR2 [
        /// Interrupt Mask on line 32
        IM32 OFFSET(0) NUMBITS(1) [],
        /// Interrupt Mask on line 33
        IM33 OFFSET(1) NUMBITS(1) [],
        /// Interrupt Mask on line 34
        IM34 OFFSET(2) NUMBITS(1) [],
        /// Interrupt Mask on line 35
        IM35 OFFSET(3) NUMBITS(1) [],
        /// Interrupt Mask on line 36
        IM36 OFFSET(4) NUMBITS(1) [],
        /// Interrupt Mask on line 37
        IM37 OFFSET(5) NUMBITS(1) [],
        /// Interrupt Mask on line 38
        IM38 OFFSET(6) NUMBITS(1) [],
        /// Interrupt Mask on line 39
        IM39 OFFSET(7) NUMBITS(1) [],
        /// Interrupt Mask on line 40, only applicable to STM32L49x/L4Ax
        IM40 OFFSET(8) NUMBITS(1) []
    ],
    EMR2 [
        /// Event Mask on line 32
        EM32 OFFSET(0) NUMBITS(1) [],
        /// Event Mask on line 33
        EM33 OFFSET(1) NUMBITS(1) [],
        /// Event Mask on line 34
        EM34 OFFSET(2) NUMBITS(1) [],
        /// Event Mask on line 35
        EM35 OFFSET(3) NUMBITS(1) [],
        /// Event Mask on line 36
        EM36 OFFSET(4) NUMBITS(1) [],
        /// Event Mask on line 37
        EM37 OFFSET(5) NUMBITS(1) [],
        /// Event Mask on line 38
        EM38 OFFSET(6) NUMBITS(1) [],
        /// Event Mask on line 39
        EM39 OFFSET(7) NUMBITS(1) [],
        /// Event Mask on line 40, only applicable to STM32L49x/L4Ax
        EM40 OFFSET(8) NUMBITS(1) []
    ],
    RTSR2 [
        /// Rising trigger event configuration of line 32
        RT32 OFFSET(0) NUMBITS(1) [],
        /// Rising trigger event configuration of line 33
        RT33 OFFSET(1) NUMBITS(1) [],
        /// Rising trigger event configuration of line 34
        RT34 OFFSET(2) NUMBITS(1) [],
        /// Rising trigger event configuration of line 35
        RT35 OFFSET(3) NUMBITS(1) [],
        /// Rising trigger event configuration of line 36
        RT36 OFFSET(4) NUMBITS(1) [],
        /// Rising trigger event configuration of line 37
        RT37 OFFSET(5) NUMBITS(1) [],
        /// Rising trigger event configuration of line 38
        RT38 OFFSET(6) NUMBITS(1) [],
        /// Rising trigger event configuration of line 39
        RT39 OFFSET(7) NUMBITS(1) [],
        /// Rising trigger event configuration of line 40, only applicable to STM32L49x/L4Ax
        RT40 OFFSET(8) NUMBITS(1) []
    ],
    FTSR2 [
        /// Falling trigger event configuration of line 32
        FT32 OFFSET(0) NUMBITS(1) [],
        /// Falling trigger event configuration of line 33
        FT33 OFFSET(1) NUMBITS(1) [],
        /// Falling trigger event configuration of line 34
        FT34 OFFSET(2) NUMBITS(1) [],
        /// Falling trigger event configuration of line 35
        FT35 OFFSET(3) NUMBITS(1) [],
        /// Falling trigger event configuration of line 36
        FT36 OFFSET(4) NUMBITS(1) [],
        /// Falling trigger event configuration of line 37
        FT37 OFFSET(5) NUMBITS(1) [],
        /// Falling trigger event configuration of line 38
        FT38 OFFSET(6) NUMBITS(1) [],
        /// Falling trigger event configuration of line 39
        FT39 OFFSET(7) NUMBITS(1) [],
        /// Falling trigger event configuration of line 40, only applicable to STM32L49x/L4Ax
        FT40 OFFSET(8) NUMBITS(1) []
    ],
    SWIER2 [
        /// Software Interrupt on line 32
        SWI32 OFFSET(0) NUMBITS(1) [],
        /// Software Interrupt on line 33
        SWI33 OFFSET(1) NUMBITS(1) [],
        /// Software Interrupt on line 34
        SWI34 OFFSET(2) NUMBITS(1) [],
        /// Software Interrupt on line 35
        SWI35 OFFSET(3) NUMBITS(1) [],
        /// Software Interrupt on line 36
        SWI36 OFFSET(4) NUMBITS(1) [],
        /// Software Interrupt on line 37
        SWI37 OFFSET(5) NUMBITS(1) [],
        /// Software Interrupt on line 38
        SWI38 OFFSET(6) NUMBITS(1) [],
        /// Software Interrupt on line 39
        SWI39 OFFSET(7) NUMBITS(1) [],
        /// Software Interrupt on line 40, only applicable to STM32L49x/L4Ax
        SWI40 OFFSET(8) NUMBITS(1) []
    ],
    PR2 [
        /// Pending interrupt flag 32
        PIF32 OFFSET(0) NUMBITS(1) [],
        /// Pending interrupt flag 33
        PIF33 OFFSET(1) NUMBITS(1) [],
        /// Pending interrupt flag 34
        PIF34 OFFSET(2) NUMBITS(1) [],
        /// Pending interrupt flag 35
        PIF35 OFFSET(3) NUMBITS(1) [],
        /// Pending interrupt flag 36
        PIF36 OFFSET(4) NUMBITS(1) [],
        /// Pending interrupt flag 37
        PIF37 OFFSET(5) NUMBITS(1) [],
        /// Pending interrupt flag 38
        PIF38 OFFSET(6) NUMBITS(1) [],
        /// Pending interrupt flag 39
        PIF39 OFFSET(7) NUMBITS(1) [],
        /// Pending interrupt flag 40, only applicable to STM32L49x/L4Ax
        PIF40 OFFSET(8) NUMBITS(1) []
    ]
];

const EXTI_REGS: StaticRef<ExtiRegisters> =
    unsafe { StaticRef::new(memory_map::EXTI_BASE as *const ExtiRegisters) };

/// EXTI block has 40 lines going into NVIC for STM32L47x/L48x and 41 lines going
/// into NVIC for STM32L49x/L4Ax. This arrangement is described in the Refernce Manual.
/// A summary is provided here.
///
/// The event/interrupt lines going into NVIC are mapped to the following NVIC IRQs.
/// Note there is *no* one-to-one mapping between the EXTI lines and the NVIC IRQs.
/// The 23 lines going into NVIC translates to 14 IRQs on NVIC.
///
///  EXTI line (NVIC IRQ)
///  - EXTI0 (6)
///  - EXTI1 (7)
///  - EXTI2 (8)
///  - EXTI3 (9)
///  - EXTI4 (10)
///  - EXTI9_5 (23)
///  - EXTI15_10 (40)
///
///  - EXTI16 -> PVD (1)
///  - EXTI17 -> OTG_FS (67)
///  - EXTI18 -> RTC_Alarm (41)
///  - EXTI19 -> TAMP_STAMP (2)?
///  - EXTI20 -> RTC_WKUP (3)
///  - EXTI21 -> COMP1 (64)
///  - EXTI22 -> COMP2 ()?
///  - EXTI23 -> I2C1_EV (31)
///
///  - EXTI24 -> I2C2_EV (33)
///  - EXTI24 -> I2C3_EV (72)
///  - EXTI[24..30] -> USART/UART[1..5] (37, 38, 39, 52, 53)
///  - EXTI31 -> LPUART1 (70)
///  - EXTI32 -> LPTIM1 (65)
///  - EXTI33 -> LPTIM2 (66)
///  - EXTI34 -> SWPMI1 (76)
///  - EXTI[35..38] -> PVM[1..4] (?)
///  - EXTI39 -> LCD (78)
///  - EXTI40 -> I2C4 (?) (only STM32L49x/L4Ax)
///
/// Warning: GPIO pin mapping to EXTI line is down to the GPIO pin number, so
/// all of PA0 and PB0 (etc. thru to PG0) all map to EXIT0. And PA1, etc map to EXTI1.
/// Ony one pin of any EXTI group (0..15) may be active an interrupt source.
///
/// EXTI groups (9..5) and (15..10) differ from hte one-to-one mapping, where
/// they each have only one NVIC IRQ, but may have multiple EXTI sources.
/// So EXTI9_5 is capable of having 5 GPIO pins trigger the respective IRQ,
/// but the IRQ handler would have to demultiplex them from the single IRQ to
/// separate pin activity.
///
/// Table of GPIO ports/pins to EXTI lines
/// PA0-PH0 - EXTI0
/// PA1-PH1 - EXTI1
/// PA2-PH2 - EXTI2
/// PA3-PH3 - EXTI3
/// PA4-PH4 - EXTI4
/// PA9-PH9 & PA8-PH8 & PA7-PH7 & PA6-PH6 & PA5-PH5 - EXTI9_5
/// PA15-PH15 & PA14-PH14 & PA13-PH13 & PA12-PH12 & PA11-PH11 & PA10-PH10 - EXTI15_10
///
/// Note: SYSCFG handles the mapping of GPIO PortX.pinY to EXTI line.
///
/// The EXTI_PR (pending) register when set, generates a level-triggered
/// interrupt on the NVIC. This means, that its the responsibility of the IRQ
/// handler to clear the interrupt source (pending bit), in order to prevent
/// multiple interrupts from occurring.
///
/// `EXTI_EVENTS` is modeled to capture information from `EXTI_PR` register. In
/// the top half IRQ handler, prior to clearing the pending bit, we set the
/// corresponding bit in `EXTI_EVENTS`. Once the bit is set, in `EXTI_EVENTS`,
/// we clear the pending bit and exit the ISR.

// #[no_mangle]
// #[used]
// pub static mut EXTI_EVENTS: u32 = 0;

enum_from_primitive! {
    #[repr(u8)]
    #[derive(Copy, Clone)]
    pub enum LineId {
        Exti0 = 0,
        Exti1 = 1,
        Exti2 = 2,
        Exti3 = 3,
        Exti4 = 4,
        Exti5 = 5,
        Exti6 = 6,
        Exti7 = 7,
        Exti8 = 8,
        Exti9 = 9,
        Exti10 = 10,
        Exti11 = 11,
        Exti12 = 12,
        Exti13 = 13,
        Exti14 = 14,
        Exti15 = 15,
    }
}

// `line_gpiopin_map` is used to call `handle_interrupt()` on the pin.
pub struct Exti {
    registers: StaticRef<ExtiRegisters>,
    // line_gpiopin_map: [OptionalCell<&'a gpio::Pin<'a>>; 16],
}

pub static mut EXTI: Exti = Exti::new();

impl Exti {
    const fn new() -> Exti {
        Exti {
            registers: EXTI_REGS,
        }
    }

    // pub fn associate_line_gpiopin(&self, lineid: LineId, client: &'a dyn hil::gpio::Client) {
    //     // Ensure that EXTI line is masked.
    //     self.mask_interrupt(lineid);
    //     // self.line_gpiopin_map[usize::from(lineid as u8)].set(pin);
    //     // unsafe {}
    //     // pin.set_exti_lineid(lineid);
    //     self.client[usize::from(lineid as u8)].set(client);
    // }

    pub fn mask_interrupt(&self, lineid: LineId) {
        match lineid {
            LineId::Exti0 => self.registers.imr1.modify(IMR1::IM0::CLEAR),
            LineId::Exti1 => self.registers.imr1.modify(IMR1::IM1::CLEAR),
            LineId::Exti2 => self.registers.imr1.modify(IMR1::IM2::CLEAR),
            LineId::Exti3 => self.registers.imr1.modify(IMR1::IM3::CLEAR),
            LineId::Exti4 => self.registers.imr1.modify(IMR1::IM4::CLEAR),
            LineId::Exti5 => self.registers.imr1.modify(IMR1::IM5::CLEAR),
            LineId::Exti6 => self.registers.imr1.modify(IMR1::IM6::CLEAR),
            LineId::Exti7 => self.registers.imr1.modify(IMR1::IM7::CLEAR),
            LineId::Exti8 => self.registers.imr1.modify(IMR1::IM8::CLEAR),
            LineId::Exti9 => self.registers.imr1.modify(IMR1::IM9::CLEAR),
            LineId::Exti10 => self.registers.imr1.modify(IMR1::IM10::CLEAR),
            LineId::Exti11 => self.registers.imr1.modify(IMR1::IM11::CLEAR),
            LineId::Exti12 => self.registers.imr1.modify(IMR1::IM12::CLEAR),
            LineId::Exti13 => self.registers.imr1.modify(IMR1::IM13::CLEAR),
            LineId::Exti14 => self.registers.imr1.modify(IMR1::IM14::CLEAR),
            LineId::Exti15 => self.registers.imr1.modify(IMR1::IM15::CLEAR),
            _ => {}
        }
    }

    pub fn unmask_interrupt(&self, lineid: LineId) {
        match lineid {
            LineId::Exti0 => self.registers.imr1.modify(IMR1::IM0::SET),
            LineId::Exti1 => self.registers.imr1.modify(IMR1::IM1::SET),
            LineId::Exti2 => self.registers.imr1.modify(IMR1::IM2::SET),
            LineId::Exti3 => self.registers.imr1.modify(IMR1::IM3::SET),
            LineId::Exti4 => self.registers.imr1.modify(IMR1::IM4::SET),
            LineId::Exti5 => self.registers.imr1.modify(IMR1::IM5::SET),
            LineId::Exti6 => self.registers.imr1.modify(IMR1::IM6::SET),
            LineId::Exti7 => self.registers.imr1.modify(IMR1::IM7::SET),
            LineId::Exti8 => self.registers.imr1.modify(IMR1::IM8::SET),
            LineId::Exti9 => self.registers.imr1.modify(IMR1::IM9::SET),
            LineId::Exti10 => self.registers.imr1.modify(IMR1::IM10::SET),
            LineId::Exti11 => self.registers.imr1.modify(IMR1::IM11::SET),
            LineId::Exti12 => self.registers.imr1.modify(IMR1::IM12::SET),
            LineId::Exti13 => self.registers.imr1.modify(IMR1::IM13::SET),
            LineId::Exti14 => self.registers.imr1.modify(IMR1::IM14::SET),
            LineId::Exti15 => self.registers.imr1.modify(IMR1::IM15::SET),
        }
    }

    // Pending clear happens by writing 1
    pub fn clear_pending(&self, lineid: LineId) {
        match lineid {
            LineId::Exti0 => self.registers.pr1.write(PR1::PIF0::SET),
            LineId::Exti1 => self.registers.pr1.write(PR1::PIF1::SET),
            LineId::Exti2 => self.registers.pr1.write(PR1::PIF2::SET),
            LineId::Exti3 => self.registers.pr1.write(PR1::PIF3::SET),
            LineId::Exti4 => self.registers.pr1.write(PR1::PIF4::SET),
            LineId::Exti5 => self.registers.pr1.write(PR1::PIF5::SET),
            LineId::Exti6 => self.registers.pr1.write(PR1::PIF6::SET),
            LineId::Exti7 => self.registers.pr1.write(PR1::PIF7::SET),
            LineId::Exti8 => self.registers.pr1.write(PR1::PIF8::SET),
            LineId::Exti9 => self.registers.pr1.write(PR1::PIF9::SET),
            LineId::Exti10 => self.registers.pr1.write(PR1::PIF10::SET),
            LineId::Exti11 => self.registers.pr1.write(PR1::PIF11::SET),
            LineId::Exti12 => self.registers.pr1.write(PR1::PIF12::SET),
            LineId::Exti13 => self.registers.pr1.write(PR1::PIF13::SET),
            LineId::Exti14 => self.registers.pr1.write(PR1::PIF14::SET),
            LineId::Exti15 => self.registers.pr1.write(PR1::PIF15::SET),
        }
    }

    pub fn is_pending(&self, lineid: LineId) -> bool {
        let val = match lineid {
            LineId::Exti0 => self.registers.pr1.read(PR1::PIF0),
            LineId::Exti1 => self.registers.pr1.read(PR1::PIF1),
            LineId::Exti2 => self.registers.pr1.read(PR1::PIF2),
            LineId::Exti3 => self.registers.pr1.read(PR1::PIF3),
            LineId::Exti4 => self.registers.pr1.read(PR1::PIF4),
            LineId::Exti5 => self.registers.pr1.read(PR1::PIF5),
            LineId::Exti6 => self.registers.pr1.read(PR1::PIF6),
            LineId::Exti7 => self.registers.pr1.read(PR1::PIF7),
            LineId::Exti8 => self.registers.pr1.read(PR1::PIF8),
            LineId::Exti9 => self.registers.pr1.read(PR1::PIF9),
            LineId::Exti10 => self.registers.pr1.read(PR1::PIF10),
            LineId::Exti11 => self.registers.pr1.read(PR1::PIF11),
            LineId::Exti12 => self.registers.pr1.read(PR1::PIF12),
            LineId::Exti13 => self.registers.pr1.read(PR1::PIF13),
            LineId::Exti14 => self.registers.pr1.read(PR1::PIF14),
            LineId::Exti15 => self.registers.pr1.read(PR1::PIF15),
        };
        val > 0
    }

    pub fn select_rising_trigger(&self, lineid: LineId) {
        match lineid {
            LineId::Exti0 => self.registers.rtsr1.modify(RTSR1::RT0::SET),
            LineId::Exti1 => self.registers.rtsr1.modify(RTSR1::RT1::SET),
            LineId::Exti2 => self.registers.rtsr1.modify(RTSR1::RT2::SET),
            LineId::Exti3 => self.registers.rtsr1.modify(RTSR1::RT3::SET),
            LineId::Exti4 => self.registers.rtsr1.modify(RTSR1::RT4::SET),
            LineId::Exti5 => self.registers.rtsr1.modify(RTSR1::RT5::SET),
            LineId::Exti6 => self.registers.rtsr1.modify(RTSR1::RT6::SET),
            LineId::Exti7 => self.registers.rtsr1.modify(RTSR1::RT7::SET),
            LineId::Exti8 => self.registers.rtsr1.modify(RTSR1::RT8::SET),
            LineId::Exti9 => self.registers.rtsr1.modify(RTSR1::RT9::SET),
            LineId::Exti10 => self.registers.rtsr1.modify(RTSR1::RT10::SET),
            LineId::Exti11 => self.registers.rtsr1.modify(RTSR1::RT11::SET),
            LineId::Exti12 => self.registers.rtsr1.modify(RTSR1::RT12::SET),
            LineId::Exti13 => self.registers.rtsr1.modify(RTSR1::RT13::SET),
            LineId::Exti14 => self.registers.rtsr1.modify(RTSR1::RT14::SET),
            LineId::Exti15 => self.registers.rtsr1.modify(RTSR1::RT15::SET),
        }
    }

    pub fn deselect_rising_trigger(&self, lineid: LineId) {
        match lineid {
            LineId::Exti0 => self.registers.rtsr1.modify(RTSR1::RT0::CLEAR),
            LineId::Exti1 => self.registers.rtsr1.modify(RTSR1::RT1::CLEAR),
            LineId::Exti2 => self.registers.rtsr1.modify(RTSR1::RT2::CLEAR),
            LineId::Exti3 => self.registers.rtsr1.modify(RTSR1::RT3::CLEAR),
            LineId::Exti4 => self.registers.rtsr1.modify(RTSR1::RT4::CLEAR),
            LineId::Exti5 => self.registers.rtsr1.modify(RTSR1::RT5::CLEAR),
            LineId::Exti6 => self.registers.rtsr1.modify(RTSR1::RT6::CLEAR),
            LineId::Exti7 => self.registers.rtsr1.modify(RTSR1::RT7::CLEAR),
            LineId::Exti8 => self.registers.rtsr1.modify(RTSR1::RT8::CLEAR),
            LineId::Exti9 => self.registers.rtsr1.modify(RTSR1::RT9::CLEAR),
            LineId::Exti10 => self.registers.rtsr1.modify(RTSR1::RT10::CLEAR),
            LineId::Exti11 => self.registers.rtsr1.modify(RTSR1::RT11::CLEAR),
            LineId::Exti12 => self.registers.rtsr1.modify(RTSR1::RT12::CLEAR),
            LineId::Exti13 => self.registers.rtsr1.modify(RTSR1::RT13::CLEAR),
            LineId::Exti14 => self.registers.rtsr1.modify(RTSR1::RT14::CLEAR),
            LineId::Exti15 => self.registers.rtsr1.modify(RTSR1::RT15::CLEAR),
        }
    }

    pub fn select_falling_trigger(&self, lineid: LineId) {
        match lineid {
            LineId::Exti0 => self.registers.ftsr1.modify(FTSR1::FT0::SET),
            LineId::Exti1 => self.registers.ftsr1.modify(FTSR1::FT1::SET),
            LineId::Exti2 => self.registers.ftsr1.modify(FTSR1::FT2::SET),
            LineId::Exti3 => self.registers.ftsr1.modify(FTSR1::FT3::SET),
            LineId::Exti4 => self.registers.ftsr1.modify(FTSR1::FT4::SET),
            LineId::Exti5 => self.registers.ftsr1.modify(FTSR1::FT5::SET),
            LineId::Exti6 => self.registers.ftsr1.modify(FTSR1::FT6::SET),
            LineId::Exti7 => self.registers.ftsr1.modify(FTSR1::FT7::SET),
            LineId::Exti8 => self.registers.ftsr1.modify(FTSR1::FT8::SET),
            LineId::Exti9 => self.registers.ftsr1.modify(FTSR1::FT9::SET),
            LineId::Exti10 => self.registers.ftsr1.modify(FTSR1::FT10::SET),
            LineId::Exti11 => self.registers.ftsr1.modify(FTSR1::FT11::SET),
            LineId::Exti12 => self.registers.ftsr1.modify(FTSR1::FT12::SET),
            LineId::Exti13 => self.registers.ftsr1.modify(FTSR1::FT13::SET),
            LineId::Exti14 => self.registers.ftsr1.modify(FTSR1::FT14::SET),
            LineId::Exti15 => self.registers.ftsr1.modify(FTSR1::FT15::SET),
        }
    }

    pub fn deselect_falling_trigger(&self, lineid: LineId) {
        match lineid {
            LineId::Exti0 => self.registers.ftsr1.modify(FTSR1::FT0::CLEAR),
            LineId::Exti1 => self.registers.ftsr1.modify(FTSR1::FT1::CLEAR),
            LineId::Exti2 => self.registers.ftsr1.modify(FTSR1::FT2::CLEAR),
            LineId::Exti3 => self.registers.ftsr1.modify(FTSR1::FT3::CLEAR),
            LineId::Exti4 => self.registers.ftsr1.modify(FTSR1::FT4::CLEAR),
            LineId::Exti5 => self.registers.ftsr1.modify(FTSR1::FT5::CLEAR),
            LineId::Exti6 => self.registers.ftsr1.modify(FTSR1::FT6::CLEAR),
            LineId::Exti7 => self.registers.ftsr1.modify(FTSR1::FT7::CLEAR),
            LineId::Exti8 => self.registers.ftsr1.modify(FTSR1::FT8::CLEAR),
            LineId::Exti9 => self.registers.ftsr1.modify(FTSR1::FT9::CLEAR),
            LineId::Exti10 => self.registers.ftsr1.modify(FTSR1::FT10::CLEAR),
            LineId::Exti11 => self.registers.ftsr1.modify(FTSR1::FT11::CLEAR),
            LineId::Exti12 => self.registers.ftsr1.modify(FTSR1::FT12::CLEAR),
            LineId::Exti13 => self.registers.ftsr1.modify(FTSR1::FT13::CLEAR),
            LineId::Exti14 => self.registers.ftsr1.modify(FTSR1::FT14::CLEAR),
            LineId::Exti15 => self.registers.ftsr1.modify(FTSR1::FT15::CLEAR),
        }
    }

    pub fn handle_exti_lines(&self, mask: u32) -> u32 {
        let mut active: u32 = 0;

        // `EXTI_PR1/2` is a read/clear write 1 register (`rc_w1`).
        // Read the `EXTI_PR` register and then write the value of `exti_pr` back.
        active = self.registers.pr1.get() & mask;
        self.registers.pr1.set(active);
        active
    }

    // pub fn handle_interrupt(&self) {
    //     let mut exti_pr: u32 = 0;

    //     // Read the `EXTI_PR` register and toggle the appropriate bits in
    //     // `exti_pr`. Once that is done, write the value of `exti_pr` back. We
    //     // can have a situation where memory value of `EXTI_PR` could have
    //     // changed due to an external interrupt. `EXTI_PR` is a read/clear write
    //     // 1 register (`rc_w1`). So, we only clear bits whose value has been
    //     // transferred to `exti_pr`.
    //     unsafe {
    //         atomic(|| {
    //             exti_pr = self.registers.pr1.get();
    //             self.registers.pr1.set(exti_pr);
    //         });
    //     }

    //     // ignore the "reserved" EXTI bits. Use bits [22..18, 16..0]. See `EXTI_PR1` for
    //     // details.
    //     exti_pr |= 0x007dffff;

    //     let mut flagged_bit = 0;

    //     // stay in loop until we have processed all the flagged event bits
    //     while exti_pr != 0 {
    //         if (exti_pr & 0b1) != 0 {
    //             if let Some(d) = LineId::from_u8(flagged_bit) {
    //                 self.line_gpiopin_map[usize::from(d as u8)].map(|pin| pin.handle_interrupt());
    //             }
    //         }
    //         // move to next bit
    //         flagged_bit += 1;
    //         exti_pr >>= 1;
    //     }
    // }
}
