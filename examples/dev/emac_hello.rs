#![no_std]
#![no_main]

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{configure_clock_ext_in, regs, RefClockPin};

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

struct Uart0;

impl Uart0 {
    fn write_byte(&mut self, byte: u8) {
        while txfifo_count() >= UART_TXFIFO_CAPACITY {
            spin_loop();
        }

        // SAFETY: `UART_FIFO_REG` is the fixed UART0 TX FIFO MMIO register. Volatile write is
        // required so the compiler preserves the hardware side effect.
        unsafe {
            write_volatile(UART_FIFO_REG, byte as u32);
        }
    }
}

impl Write for Uart0 {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            if byte == b'\n' {
                self.write_byte(b'\r');
            }
            self.write_byte(byte);
        }

        Ok(())
    }
}

#[entry]
fn main() -> ! {
    configure_clock_ext_in(RefClockPin::Gpio32);

    // SAFETY: `VERSION_ADDR` is the documented EMAC version register. Volatile read is required
    // because MMIO values are not ordinary memory.
    let version = unsafe { read_volatile(regs::mac::VERSION_ADDR as *const u32) };
    let mut uart = Uart0;
    let _ = writeln!(uart, "EMAC_VERSION = 0x{version:08X}");

    loop {
        spin_loop();
    }
}

fn txfifo_count() -> u32 {
    // SAFETY: `UART_STATUS_REG` is the fixed UART0 status MMIO register. Volatile read is
    // required because the value changes independently from the CPU.
    let status = unsafe { read_volatile(UART_STATUS_REG) };
    (status & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
}
