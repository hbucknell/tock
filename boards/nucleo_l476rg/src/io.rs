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
/// Default debug UART connected to ST-link
static mut DEBUG_UART: &stm32l4xx::usart::Usart = unsafe { &stm32l4xx::usart::USART2 };

/// Change console dbug UART, default is USART2
pub fn change_dedug_uart(uart: &'static stm32l4xx::usart::Usart) {
    unsafe { DEBUG_UART = uart };
}

impl Write for Writer {
    fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

impl IoWrite for Writer {
    fn write(&mut self, buf: &[u8]) {
        unsafe {
            if !self.initialized {
                self.initialized = true;

                DEBUG_UART.configure(uart::Parameters {
                    baud_rate: 115200,
                    stop_bits: uart::StopBits::One,
                    parity: uart::Parity::None,
                    hw_flow_control: false,
                    width: uart::Width::Eight,
                });
            }

            for &c in buf {
                DEBUG_UART.send_byte(c);
            }
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
