//! Implementation of the STM32L4 Debug peripheral module
//!
//! Supports MCU-IDCODE field access, external debug behaviour during low-power
//! states and peripheral behaviour during breakpoints

use kernel::common::registers::{register_bitfields, ReadOnly, ReadWrite};
use kernel::common::StaticRef;

use crate::memory_map;

/// Debug support
#[repr(C)]
struct DbgRegisters {
    /// IDCODE
    dbgmcu_idcode: ReadOnly<u32, DBGMCU_IDCODE::Register>,
    /// Control Register
    dbgmcu_cr: ReadWrite<u32, DBGMCU_CR::Register>,
    /// Debug MCU APB1 Freeze register 1
    dbgmcu_apb1fzr1: ReadWrite<u32, DBGMCU_APB1FZR1::Register>,
    /// Debug MCU APB1 Freeze register 2
    dbgmcu_apb1fzr2: ReadWrite<u32, DBGMCU_APB1FZR2::Register>,
    /// Debug MCU APB2 Freeze register
    dbgmcu_apb2fzr: ReadWrite<u32, DBGMCU_APB2FZR::Register>,
}

register_bitfields![u32,
    DBGMCU_IDCODE [
        /// DEV_ID
        DEV_ID OFFSET(0) NUMBITS(12) [],
        /// REV_ID
        REV_ID OFFSET(16) NUMBITS(16) []
    ],
    DBGMCU_CR [
        /// Debug Sleep Mode
        DBG_SLEEP OFFSET(0) NUMBITS(1) [],
        /// Debug Stop Mode
        DBG_STOP OFFSET(1) NUMBITS(1) [],
        /// Debug Standby Mode
        DBG_STANDBY OFFSET(2) NUMBITS(1) [],
        /// Trace pin assignment control
        TRACE_IOEN OFFSET(5) NUMBITS(1) [],
        /// Trace pin assignment control
        TRACE_MODE OFFSET(6) NUMBITS(2) []
    ],
    DBGMCU_APB1FZR1 [
        /// TIM2 stopped when core is halted
        DBG_TIM2_STOP OFFSET(0) NUMBITS(1) [],
        /// TIM3  stopped when core is halted
        DBG_TIM3_STOP OFFSET(1) NUMBITS(1) [],
        /// TIM4 stopped when core is halted
        DBG_TIM4_STOP OFFSET(2) NUMBITS(1) [],
        /// TIM5 stopped when core is halted
        DBG_TIM5_STOP OFFSET(3) NUMBITS(1) [],
        /// TIM6 stopped when core is halted
        DBG_TIM6_STOP OFFSET(4) NUMBITS(1) [],
        /// TIM7 stopped when core is halted
        DBG_TIM7_STOP OFFSET(5) NUMBITS(1) [],
        /// RTC stopped when Core is halted
        DBG_RTC_STOP OFFSET(10) NUMBITS(1) [],
        /// WWDG stopped when core is halted
        DBG_WWDG_STOP OFFSET(11) NUMBITS(1) [],
        /// IWDG stopped when core is halted
        DBG_IWDG_STOP OFFSET(12) NUMBITS(1) [],
        /// I2C1 stopped when core is halted
        DBG_I2C1_STOP OFFSET(21) NUMBITS(1) [],
        /// I2C2 stopped when core is halted
        DBG_I2C2_STOP OFFSET(22) NUMBITS(1) [],
        /// I2C3 stopped when core is halted
        DBG_I2C3_STOP OFFSET(23) NUMBITS(1) [],
        /// CAN1 stopped when core is halted
        DBG_CAN1_STOP OFFSET(25) NUMBITS(1) [],
        /// CAN2 stopped when core is halted
        DBG_CAN2_STOP OFFSET(26) NUMBITS(1) [],
        /// LPTIM1 stopped when core is halted
        DBG_LPTIM1_STOP OFFSET(31) NUMBITS(1) []
    ],
    DBGMCU_APB1FZR2 [
        /// I2C4 stopped when core is halted
        DBG_I2C4_STOP OFFSET(1) NUMBITS(1) [],
        /// LPTIM2 stopped when core is halted
        DBG_LPTIM2_STOP OFFSET(5) NUMBITS(1) []
    ],
    DBGMCU_APB2FZR [
        /// TIM1 counter stopped when core is halted
        DBG_TIM1_STOP OFFSET(11) NUMBITS(1) [],
        /// TIM8 counter stopped when core is halted
        DBG_TIM8_STOP OFFSET(13) NUMBITS(1) [],
        /// TIM15 counter stopped when core is halted
        DBG_TIM15_STOP OFFSET(16) NUMBITS(1) [],
        /// TIM16 counter stopped when core is halted
        DBG_TIM16_STOP OFFSET(17) NUMBITS(1) [],
        /// TIM17 counter stopped when core is halted
        DBG_TIM17_STOP OFFSET(18) NUMBITS(1) []
    ]
];

const DBG_REGS: StaticRef<DbgRegisters> =
    unsafe { StaticRef::new(memory_map::DBGMCU_BASE as *const DbgRegisters) };

pub struct Dbg {
    registers: StaticRef<DbgRegisters>,
}

pub static mut DBG: Dbg = Dbg::new();

impl Dbg {
    const fn new() -> Dbg {
        Dbg {
            registers: DBG_REGS,
        }
    }

    /// Enable external debug connection during low-power states
    /// Sleep, Stop and Standby.
    /// This should be disabled/not enabled for release builds.
    pub fn enable_low_power_debug(&self) {
        // Enable external debugging during low-power modes
        self.registers.dbgmcu_cr.modify(DBGMCU_CR::DBG_SLEEP::SET);
        self.registers.dbgmcu_cr.modify(DBGMCU_CR::DBG_STOP::SET);
        self.registers.dbgmcu_cr.modify(DBGMCU_CR::DBG_STANDBY::SET);
    }

    /// Freeze TIM2 during breakpoint
    pub fn disable_tim2_counter(&self) {
        self.registers
            .dbgmcu_apb1fzr1
            .modify(DBGMCU_APB1FZR1::DBG_TIM2_STOP::SET);
    }
}
