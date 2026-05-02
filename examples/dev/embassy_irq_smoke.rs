#![no_std]
#![no_main]

//! Smoke test for the IRQ-driven embassy-time driver. NO polling task is
//! spawned; the executor must drop to `wfi` between alarms and only wake on
//! the SYSTIMER → CLIC interrupt.
//!
//! Issues five back-to-back `Timer::after(1.sec()).await`s and prints how
//! many SYSTIMER ticks elapsed across each. A working IRQ driver should
//! report ~16_000_000 ticks per second exactly (the polling driver was
//! ~16_003_xxx due to executor overhead — the IRQ path lands the alarm
//! to the tick).

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp_p4_eth::{systimer, time_driver_irq};
use panic_halt as _;
use riscv_rt::entry;
use static_cell::StaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

struct Uart0;

impl Uart0 {
    fn write_byte(&mut self, byte: u8) {
        while txfifo_count() >= UART_TXFIFO_CAPACITY {
            spin_loop();
        }
        unsafe { write_volatile(UART_FIFO_REG, byte as u32) };
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

fn txfifo_count() -> u32 {
    let status = unsafe { read_volatile(UART_STATUS_REG) };
    (status & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
}

#[embassy_executor::task]
async fn smoke_task() {
    let mut uart = Uart0;
    let _ = writeln!(uart, "smoke_task: 5x Timer::after(1s)");

    for i in 0..5 {
        let t0 = systimer::now_ticks();
        Timer::after(Duration::from_secs(1)).await;
        let t1 = systimer::now_ticks();
        let dt_ticks = t1 - t0;
        let _ = writeln!(
            uart,
            "  iter {}: dt = {} ticks  (expected ~16_000_000)",
            i, dt_ticks,
        );
    }

    let _ = writeln!(uart, "smoke_task: 100ms cadence x10");
    let mut prev = systimer::now_ticks();
    for i in 0..10 {
        Timer::after(Duration::from_millis(100)).await;
        let now = systimer::now_ticks();
        let _ = writeln!(uart, "  100ms[{}]: {} ticks", i, now - prev);
        prev = now;
    }

    let _ = writeln!(uart, "=== embassy_irq_smoke OK ===");
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== embassy_irq_smoke alive ===");

    // Configure SYSTIMER alarm 0, CLIC routing, mtvec, and global IRQ enable.
    time_driver_irq::init();
    let _ = writeln!(uart, "time_driver_irq::init() done");

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(smoke_task()).unwrap();
    });
}
