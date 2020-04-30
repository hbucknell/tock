//! Implementation of the STM32L4 System Configuration controller.
//!
//! Handles remapping memory areas, managing external interrupt line
//! connection to the GPIOs, managing robustness features, settings for
//! SRAM2, configuring FPU interrupts, enabling the Firewall and
//! enabling/disabling I2C Fast-mode capability,some IO pin drive
//! capability, voltage booster and analog switches.

use enum_primitive::cast::FromPrimitive;
use enum_primitive::enum_from_primitive;
use kernel::common::registers::{register_bitfields, ReadWrite, WriteOnly};
use kernel::common::StaticRef;

use crate::memory_map;

/// System configuration controller
#[repr(C)]
struct SyscfgRegisters {
    /// memory remap register
    memrmp: ReadWrite<u32, MEMRMP::Register>,
    /// SYSCFG configuration register 1
    cfgr1: ReadWrite<u32, CFGR1::Register>,
    /// external interrupt configuration register 1
    exticr1: ReadWrite<u32, EXTICR1::Register>,
    /// external interrupt configuration register 2
    exticr2: ReadWrite<u32, EXTICR2::Register>,
    /// external interrupt configuration register 3
    exticr3: ReadWrite<u32, EXTICR3::Register>,
    /// external interrupt configuration register 4
    exticr4: ReadWrite<u32, EXTICR4::Register>,
    /// SRAM2 control and status register
    scsr: ReadWrite<u32, SCSR::Register>,
    /// SYSCFG configuration register 2
    cfgr2: ReadWrite<u32, CFGR2::Register>,
    /// SRAM2 write protection register
    swpr: ReadWrite<u32, SWPR::Register>,
    /// SRAM2 key register
    skr: WriteOnly<u32, SKR::Register>,
    /// SRAM2 write protection register 2, only applies to STM32L49x/L4Ax
    swpr2: ReadWrite<u32, SWPR2::Register>,
}

register_bitfields![u32,
    MEMRMP [
        /// Memory mapping selection
        MEM_MODE OFFSET(0) NUMBITS(3) [
        ],

        // MAIN_FLASH_MAPPED = 0,
        // SYSTEM_FLASH_MAPPED = 1,
        // FMC1_MAPPED = 2,
        // SRAM1_MAPPED = 3,
        // QUADSPI_MAPPED = 6,
        /// Flash bank mode selection
        FB_MODE OFFSET(8) NUMBITS(1) []
    ],
    CFGR1 [
        /// Floating Point Unit interrupts enable
        FPU_IE OFFSET(26) NUMBITS(6) [],
        /// I2C4 Fast-mode Plus enable (only available on STM32L49x/L4Ax)
        I2C4_FMP OFFSET(23) NUMBITS(1) [],
        /// I2C3 Fast-mode Plus enable
        I2C3_FMP OFFSET(22) NUMBITS(1) [],
        /// I2C2 Fast-mode Plus enable
        I2C2_FMP OFFSET(21) NUMBITS(1) [],
        /// I2C1 Fast-mode Plus enable
        I2C1_FMP OFFSET(20) NUMBITS(1) [],
        /// I2C-PB9 Fast-mode Plus enable
        I2C_BP9 OFFSET(19) NUMBITS(1) [],
        /// I2C-PB8 Fast-mode Plus enable
        I2C_BP8 OFFSET(18) NUMBITS(1) [],
        /// I2C-PB7 Fast-mode Plus enable
        I2C_BP7 OFFSET(19) NUMBITS(1) [],
        /// I2C-PB6 Fast-mode Plus enable
        I2C_BP6 OFFSET(16) NUMBITS(1) [],
        /// IO analog switch booster enable
        BOOSTEN OFFSET(8) NUMBITS(1) [],
        /// Firewall disable
        FWDIS OFFSET(0) NUMBITS(1) []
    ],
    EXTICR1 [
        /// EXTI x configuration (x = 0 to 3)
        EXTI3 OFFSET(12) NUMBITS(4) [],
        /// EXTI x configuration (x = 0 to 3)
        EXTI2 OFFSET(8) NUMBITS(4) [],
        /// EXTI x configuration (x = 0 to 3)
        EXTI1 OFFSET(4) NUMBITS(4) [],
        /// EXTI x configuration (x = 0 to 3)
        EXTI0 OFFSET(0) NUMBITS(4) []
    ],
    EXTICR2 [
        /// EXTI x configuration (x = 4 to 7)
        EXTI7 OFFSET(12) NUMBITS(4) [],
        /// EXTI x configuration (x = 4 to 7)
        EXTI6 OFFSET(8) NUMBITS(4) [],
        /// EXTI x configuration (x = 4 to 7)
        EXTI5 OFFSET(4) NUMBITS(4) [],
        /// EXTI x configuration (x = 4 to 7)
        EXTI4 OFFSET(0) NUMBITS(4) []
    ],
    EXTICR3 [
        /// EXTI x configuration (x = 8 to 11)
        EXTI11 OFFSET(12) NUMBITS(4) [],
        /// EXTI10
        EXTI10 OFFSET(8) NUMBITS(4) [],
        /// EXTI x configuration (x = 8 to 11)
        EXTI9 OFFSET(4) NUMBITS(4) [],
        /// EXTI x configuration (x = 8 to 11)
        EXTI8 OFFSET(0) NUMBITS(4) []
    ],
    EXTICR4 [
        /// EXTI x configuration (x = 12 to 15)
        EXTI15 OFFSET(12) NUMBITS(4) [],
        /// EXTI x configuration (x = 12 to 15)
        EXTI14 OFFSET(8) NUMBITS(4) [],
        /// EXTI x configuration (x = 12 to 15)
        EXTI13 OFFSET(4) NUMBITS(4) [],
        /// EXTI x configuration (x = 12 to 15)
        EXTI12 OFFSET(0) NUMBITS(4) []
    ],
    SCSR [
        /// SRAM2 busy by erase operation
        SRAM2_BSY OFFSET(1) NUMBITS(1) [],
        /// SRAM2 Erase, write-protected by key register
        SRAM2_ER OFFSET(0) NUMBITS(1) []
    ],
    CFGR2 [
        /// SRAM2 parity error flag
        SPF OFFSET(0) NUMBITS(32) [],
        /// ECC Lock
        ECCL OFFSET(0) NUMBITS(32) [],
        /// PVD Lock enable bit
        PVDL OFFSET(0) NUMBITS(32) [],
        /// SRAM2 parity lock bit
        SPL OFFSET(0) NUMBITS(32) [],
        /// Cortex-M4 LOCKUP (hard-fault) output enable bit
        CLL OFFSET(0) NUMBITS(32) []
    ],
    SWPR [
        /// SRAM2 page x write protection (x = 31 to 0)
        PxWP OFFSET(0) NUMBITS(32) []
    ],
    SKR [
        /// SRAM2 key register
        KEY OFFSET(0) NUMBITS(8) []
    ],
    SWPR2 [
        /// SRAM2 page x write protection (x = 63 to 32), only applies to STM32L49x/L4Ax
        PxWP OFFSET(0) NUMBITS(32) []
    ]
];

const SYSCFG_REGS: StaticRef<SyscfgRegisters> =
    unsafe { StaticRef::new(memory_map::SYSCFG_BASE as *const SyscfgRegisters) };

/// SYSCFG EXTI configuration
enum_from_primitive! {
    #[repr(u32)]
    pub enum ExtiPort {
        PA = 0,
        PB = 1,
        PC = 2,
        PD = 3,
        PE = 4,
        PF = 5,
        PG = 6,
        PH = 7,
        // PortI = 8,
    }
}

enum_from_primitive! {
    #[repr(u32)]
    pub enum ExtiPin {
        PIN0 = 0,
        PIN1 = 1,
        PIN2 = 2,
        PIN3 = 3,
        PIN4 = 4,
        PIN5 = 5,
        PIN6 = 6,
        PIN7 = 7,
        PIN8 = 8,
        PIN9 = 9,
        PIN10 = 10,
        PIN11 = 11,
        PIN12 = 12,
        PIN13 = 13,
        PIN14 = 14,
        PIN15 = 15,
    }
}

pub struct Syscfg {
    registers: StaticRef<SyscfgRegisters>,
}

pub static mut SYSCFG: Syscfg = Syscfg::new();

impl Syscfg {
    const fn new() -> Syscfg {
        Syscfg {
            registers: SYSCFG_REGS,
        }
    }

    /// Configures the SYSCFG_EXTICR{1, 2, 3, 4} registers
    pub fn select_exti_source(&self, pin: ExtiPin, port: ExtiPort) {
        match pin {
            // SYSCFG_EXTICR1
            ExtiPin::PIN0 => self
                .registers
                .exticr1
                .modify(EXTICR1::EXTI0.val(port as u32)),
            ExtiPin::PIN1 => self
                .registers
                .exticr1
                .modify(EXTICR1::EXTI1.val(port as u32)),
            ExtiPin::PIN2 => self
                .registers
                .exticr1
                .modify(EXTICR1::EXTI2.val(port as u32)),
            ExtiPin::PIN3 => self
                .registers
                .exticr1
                .modify(EXTICR1::EXTI3.val(port as u32)),
            // SYSCFG_EXTICR2
            ExtiPin::PIN4 => self
                .registers
                .exticr2
                .modify(EXTICR2::EXTI4.val(port as u32)),
            ExtiPin::PIN5 => self
                .registers
                .exticr2
                .modify(EXTICR2::EXTI5.val(port as u32)),
            ExtiPin::PIN6 => self
                .registers
                .exticr2
                .modify(EXTICR2::EXTI6.val(port as u32)),
            ExtiPin::PIN7 => self
                .registers
                .exticr2
                .modify(EXTICR2::EXTI7.val(port as u32)),
            // SYSCFG_EXTICR3
            ExtiPin::PIN8 => self
                .registers
                .exticr3
                .modify(EXTICR3::EXTI8.val(port as u32)),
            ExtiPin::PIN9 => self
                .registers
                .exticr3
                .modify(EXTICR3::EXTI9.val(port as u32)),
            ExtiPin::PIN10 => self
                .registers
                .exticr3
                .modify(EXTICR3::EXTI10.val(port as u32)),
            ExtiPin::PIN11 => self
                .registers
                .exticr3
                .modify(EXTICR3::EXTI11.val(port as u32)),
            // SYSCFG_EXTICR4
            ExtiPin::PIN12 => self
                .registers
                .exticr4
                .modify(EXTICR4::EXTI12.val(port as u32)),
            ExtiPin::PIN13 => self
                .registers
                .exticr4
                .modify(EXTICR4::EXTI13.val(port as u32)),
            ExtiPin::PIN14 => self
                .registers
                .exticr4
                .modify(EXTICR4::EXTI14.val(port as u32)),
            ExtiPin::PIN15 => self
                .registers
                .exticr4
                .modify(EXTICR4::EXTI15.val(port as u32)),
            _ => {}
        }
    }

    // fn get_exticrid_from_port_num(&self, port_num: u32) -> ExtiPort {
    //     ExtiPort::from_u32(port_num).unwrap()
    // }
}
