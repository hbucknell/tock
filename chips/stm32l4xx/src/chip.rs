//! Implementation of the Chip Trait for the STM32L4
//!
//! Handles interrupt mapping and basic chip features

use core::fmt::Write;
use cortexm4;
use kernel::Chip;

use crate::gpio;
use crate::nvic;

// use crate::dma1;
// use crate::spi;
// use crate::tim2;
use crate::usart;

pub struct Stm32L4xx {
    mpu: cortexm4::mpu::MPU,
    userspace_kernel_boundary: cortexm4::syscall::SysCall,
    systick: cortexm4::systick::SysTick,
}

impl Stm32L4xx {
    pub unsafe fn new() -> Stm32L4xx {
        Stm32L4xx {
            mpu: cortexm4::mpu::MPU::new(),
            userspace_kernel_boundary: cortexm4::syscall::SysCall::new(),
            systick: cortexm4::systick::SysTick::new(),
        }
    }
}

impl Chip for Stm32L4xx {
    type MPU = cortexm4::mpu::MPU;
    type UserspaceKernelBoundary = cortexm4::syscall::SysCall;
    type SysTick = cortexm4::systick::SysTick;

    fn service_pending_interrupts(&self) {
        unsafe {
            loop {
                if let Some(interrupt) = cortexm4::nvic::next_pending() {
                    match interrupt {
                        // nvic::DMA1_Stream1 => dma1::Dma1Peripheral::USART3_RX
                        //     .get_stream()
                        //     .handle_interrupt(),
                        // nvic::DMA1_Stream2 => dma1::Dma1Peripheral::SPI3_RX
                        //     .get_stream()
                        //     .handle_interrupt(),
                        // nvic::DMA1_Stream3 => dma1::Dma1Peripheral::USART3_TX
                        //     .get_stream()
                        //     .handle_interrupt(),
                        // nvic::DMA1_Stream5 => dma1::Dma1Peripheral::USART2_RX
                        //     .get_stream()
                        //     .handle_interrupt(),
                        // nvic::DMA1_Stream6 => dma1::Dma1Peripheral::USART2_TX
                        //     .get_stream()
                        //     .handle_interrupt(),
                        // nvic::DMA1_Stream7 => dma1::Dma1Peripheral::SPI3_TX
                        //     .get_stream()
                        //     .handle_interrupt(),

                        // USART ISR
                        nvic::USART1 => usart::USART1.handle_interrupt(),
                        nvic::USART2 => usart::USART2.handle_interrupt(),
                        nvic::USART3 => usart::USART3.handle_interrupt(),

                        // nvic::SPI3 => spi::SPI3.handle_interrupt(),

                        // EXTI Interrupts
                        nvic::EXTI0 => gpio::Pin::handle_exti0_interrupt(),
                        nvic::EXTI1 => gpio::Pin::handle_exti1_interrupt(),
                        nvic::EXTI2 => gpio::Pin::handle_exti2_interrupt(),
                        nvic::EXTI3 => gpio::Pin::handle_exti3_interrupt(),
                        nvic::EXTI4 => gpio::Pin::handle_exti4_interrupt(),
                        nvic::EXTI9_5 => gpio::Pin::handle_exti9_5_interrupt(),
                        nvic::EXTI15_10 => gpio::Pin::handle_exti15_10_interrupt(),

                        // nvic::TIM2 => tim2::TIM2.handle_interrupt(),
                        _ => {
                            panic!("unhandled interrupt {}", interrupt);
                        }
                    }

                    let n = cortexm4::nvic::Nvic::new(interrupt);
                    n.clear_pending();
                    n.enable();
                } else {
                    break;
                }
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { cortexm4::nvic::has_pending() }
    }

    fn mpu(&self) -> &cortexm4::mpu::MPU {
        &self.mpu
    }

    fn systick(&self) -> &Self::SysTick {
        &self.systick
    }

    fn userspace_kernel_boundary(&self) -> &cortexm4::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    fn sleep(&self) {
        unsafe {
            cortexm4::scb::unset_sleepdeep();
            cortexm4::support::wfi();
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        cortexm4::support::atomic(f)
    }

    unsafe fn print_state(&self, write: &mut dyn Write) {
        cortexm4::print_cortexm4_state(write);
    }
}
