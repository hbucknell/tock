//! Peripheral implementations for the STM32L4xx MCU.
//!
//! STM32L476RG: <https://www.st.com/en/microcontrollers-microprocessors/stm32l476rg.html>

#![crate_name = "stm32l4xx"]
#![crate_type = "rlib"]
#![feature(asm, const_fn, in_band_lifetimes)]
#![no_std]
#![allow(unused_doc_comments)]

pub mod chip;
pub mod nvic;

// Peripherals
pub mod dbg;
// pub mod dma1;
// pub mod exti;
pub mod gpio;
pub mod rcc;
// pub mod spi;
// pub mod syscfg;
// pub mod tim2;
pub mod memory_map;
pub mod usart;

use cortexm4::{generic_isr, hard_fault_handler, svc_handler, systick_handler};

#[cfg(not(any(target_arch = "arm", target_os = "none")))]
unsafe extern "C" fn unhandled_interrupt() {
    unimplemented!()
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
unsafe extern "C" fn unhandled_interrupt() {
    let mut interrupt_number: u32;

    // IPSR[8:0] holds the currently active interrupt
    asm!(
    "mrs    r0, ipsr                    "
    : "={r0}"(interrupt_number)
    :
    : "r0"
    :
    );

    interrupt_number = interrupt_number & 0x1ff;

    panic!("Unhandled Interrupt. ISR {} is active.", interrupt_number);
}

extern "C" {
    // _estack is not really a function, but it makes the types work
    // You should never actually invoke it!!
    fn _estack();

    // Defined by platform
    fn reset_handler();
}

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static BASE_VECTORS: [unsafe extern "C" fn(); 16] = [
    _estack,
    reset_handler,
    unhandled_interrupt, // NMI
    hard_fault_handler,  // Hard Fault
    unhandled_interrupt, // MemManage
    unhandled_interrupt, // BusFault
    unhandled_interrupt, // UsageFault
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    unhandled_interrupt,
    svc_handler,         // SVC
    unhandled_interrupt, // DebugMon
    unhandled_interrupt,
    unhandled_interrupt, // PendSV
    systick_handler,     // SysTick
];

// STM32L476xx has total of 82 interrupts
// Extracted from `CMSIS/Device/ST/STM32L4xx/Include/stm32l476xx.h`
// NOTE: There are missing IRQn between 0 and 81
#[cfg(feature = "stm32l476rg")]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), link_section = ".irqs")]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static IRQS: [unsafe extern "C" fn(); 82] = [
    generic_isr,         // WWDG (0)
    generic_isr,         // PVD_PWM (1)
    generic_isr,         // TAMP_STAMP (2)
    generic_isr,         // RTC_WKUP (3)
    generic_isr,         // FLASH (4)
    generic_isr,         // RCC (5)
    generic_isr,         // EXTI0 (6)
    generic_isr,         // EXTI1 (7)
    generic_isr,         // EXTI2 (8)
    generic_isr,         // EXTI3 (9)
    generic_isr,         // EXTI4 (10)
    generic_isr,         // DMA1_Channel1 (11)
    generic_isr,         // DMA1_Channel2 (12)
    generic_isr,         // DMA1_Channel3 (13)
    generic_isr,         // DMA1_Channel4 (14)
    generic_isr,         // DMA1_Channel5 (15)
    generic_isr,         // DMA1_Channel6 (16)
    generic_isr,         // DMA1_Channel7 (17)
    generic_isr,         // ADC1_2 (18)
    generic_isr,         // CAN1_TX (19)
    generic_isr,         // CAN1_RX0 (20)
    generic_isr,         // CAN1_RX1 (21)
    generic_isr,         // CAN1_SCE (22)
    generic_isr,         // EXTI9_5 (23)
    generic_isr,         // TIM1_BRK_TIM9 (24)
    generic_isr,         // TIM1_UP_TIM16 (25)
    generic_isr,         // TIM1_TRG_COM_TIM17 (26)
    generic_isr,         // TIM1_CC (27)
    generic_isr,         // TIM2 (28)
    generic_isr,         // TIM3 (29)
    generic_isr,         // TIM4 (30)
    generic_isr,         // I2C1_EV (31)
    generic_isr,         // I2C1_ER (32)
    generic_isr,         // I2C2_EV (33)
    generic_isr,         // I2C2_ER (34)
    generic_isr,         // SPI1 (35)
    generic_isr,         // SPI2 (36)
    generic_isr,         // USART1 (37)
    generic_isr,         // USART2 (38)
    generic_isr,         // USART3 (39)
    generic_isr,         // EXTI15_10 (40)
    generic_isr,         // RTC_Alarm (41)
    generic_isr,         // DFSDM1_FLT3 (42)
    generic_isr,         // TIM8_BRK (43)
    generic_isr,         // TIM8_UP (44)
    generic_isr,         // TIM8_TRG_COM (45)
    generic_isr,         // TIM8_CC (46)
    generic_isr,         // ADC3 (47)
    generic_isr,         // FMC (48)
    generic_isr,         // SDMMC1 (49)
    generic_isr,         // TIM5 (50)
    generic_isr,         // SPI3 (51)
    generic_isr,         // UART4 (52)
    generic_isr,         // UART5 (53)
    generic_isr,         // TIM6_DAC (54)
    generic_isr,         // TIM7 (55)
    generic_isr,         // DMA2_Channel1 (56)
    generic_isr,         // DMA2_Channel2 (57)
    generic_isr,         // DMA2_Channel3 (58)
    generic_isr,         // DMA2_Channel4 (59)
    generic_isr,         // DMA2_Channel5 (60)
    generic_isr,         // DFSDM1_FLT0 (61)
    generic_isr,         // DFSDM1_FLT1 (62)
    generic_isr,         // DFSDM1_FLT2 (63)
    generic_isr,         // COMP (64)
    generic_isr,         // LPTIM1 (65)
    generic_isr,         // LPTIM2 (66)
    generic_isr,         // OTG_FS (67)
    generic_isr,         // DMA2_Channel6 (68)
    generic_isr,         // DMA2_Channel7 (69)
    generic_isr,         // LPUART1 (70)
    generic_isr,         // QUADSPI (71)
    generic_isr,         // I2C3_EV (72)
    generic_isr,         // I2C3_ER (73)
    generic_isr,         // SAI1 (74)
    generic_isr,         // SAI2 (75)
    generic_isr,         // SWPMI1 (76)
    generic_isr,         // TSC (77)
    generic_isr,         // LCD (78)
    unhandled_interrupt, // (79)
    generic_isr,         // RNG (80)
    generic_isr,         // FPU (81)
];

extern "C" {
    static mut _szero: u32;
    static mut _ezero: u32;
    static mut _etext: u32;
    static mut _srelocate: u32;
    static mut _erelocate: u32;
}

pub unsafe fn init() {
    tock_rt0::init_data(&mut _etext, &mut _srelocate, &mut _erelocate);
    tock_rt0::zero_bss(&mut _szero, &mut _ezero);

    cortexm4::nvic::disable_all();
    cortexm4::nvic::clear_all_pending();

    rcc::RCC.init();
    dbg::DBG.enable_low_power_debug();
}
