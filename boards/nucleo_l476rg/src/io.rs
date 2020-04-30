//! Implementation of the Debug hooks and handlers for Nucleo_L476RG
//!
//! Generally this implements the IoWrite Trait and the panic_fmt function

use core::fmt::Write;
use core::panic::PanicInfo;

use cortexm4;

use kernel::debug;
use kernel::debug::IoWrite;
use kernel::hil::led;
use kernel::hil::uart::{self, Configure};

use stm32l4xx;
use stm32l4xx::gpio::Pin;

use crate::CHIP;
use crate::PROCESSES;

/// Writer is used by kernel::debug to panic message to the serial port.
pub struct Writer {
    initialized: bool,
}

/// Global static for debug writer
pub static mut WRITER: Writer = Writer { initialized: false };

// impl Writer {
//     /// Indicate that USART has already been initialized. Trying to double
//     /// initialize USART2 causes STM32F446RE to go into in in-deterministic state.
//     pub fn set_initialized(&mut self) {
//         self.initialized = true;
//     }
// }

impl Write for Writer {
    fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

impl IoWrite for Writer {
    fn write(&mut self, buf: &[u8]) {
        let uart = unsafe { &mut stm32l4xx::usart::USART2 };

        if !self.initialized {
            self.initialized = true;

            uart.configure(uart::Parameters {
                baud_rate: 115200,
                stop_bits: uart::StopBits::One,
                parity: uart::Parity::None,
                hw_flow_control: false,
                width: uart::Width::Eight,
            });
        }

        for &c in buf {
            uart.send_byte(c);
        }
    }
}

static mut DEBUG_LED: Pin = Pin::PB07;

/// Panic handler.
#[no_mangle]
#[panic_handler]
pub unsafe extern "C" fn panic_fmt(info: &PanicInfo) -> ! {
    // User LD2 is connected to PB07
    let led = &mut led::LedHigh::new(&mut DEBUG_LED);
    let writer = &mut WRITER;

    debug::panic(
        &mut [led],
        writer,
        info,
        &cortexm4::support::nop,
        &PROCESSES,
        &CHIP,
    )
}
