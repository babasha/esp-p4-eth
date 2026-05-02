#![no_std]
#![no_main]

//! Bring-up probe for ESP32-P4 SYSTIMER (base 0x500E_2000).
//!
//! Validates that we can read the 64-bit free-running counter UNIT0 via the
//! snapshot dance (WT bit 30 in UNIT0_OP, poll bit 29, then read VAL_HI/LO).
//! Prints DATE register, CONF, several timed snapshots, and an estimated tick
//! rate by comparing two reads spaced by a long busy-wait.
//!
//! Prerequisites assumed by HW reset state:
//! - SYSTIMER bus clock: HP_SYS_CLKRST.SOC_CLK_CTRL2 bit 23 default = 1.
//! - SYSTIMER reset:     HP_SYS_CLKRST.HP_RST_EN1   bit 5  default = 0 (not in reset).
//! - SYSTIMER UNIT0:     CONF.timer_unit0_work_en (bit 30) default = 1.
//! Expected source: XTAL 40 MHz / 2.5 = 16 MHz tick (62.5 ns/tick).

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const SYSTIMER_BASE: usize = 0x500E_2000;
const SYSTIMER_CONF: *mut u32 = SYSTIMER_BASE as *mut u32;
const SYSTIMER_UNIT0_OP: *mut u32 = (SYSTIMER_BASE + 0x04) as *mut u32;
const SYSTIMER_UNIT0_VAL_HI: *const u32 = (SYSTIMER_BASE + 0x40) as *const u32;
const SYSTIMER_UNIT0_VAL_LO: *const u32 = (SYSTIMER_BASE + 0x44) as *const u32;
const SYSTIMER_DATE: *const u32 = (SYSTIMER_BASE + 0xFC) as *const u32;

const HP_CLKRST_SOC_CLK_CTRL2: *const u32 = 0x500E_601C as *const u32;
const HP_CLKRST_PERI_CLK_CTRL21: *const u32 = 0x500E_6098 as *const u32;
const HP_CLKRST_HP_RST_EN1: *const u32 = 0x500E_60C4 as *const u32;

const UNIT_OP_UPDATE_BIT: u32 = 1 << 30;
const UNIT_OP_VALID_BIT: u32 = 1 << 29;

struct Uart0;

impl Uart0 {
    fn write_byte(&mut self, byte: u8) {
        while txfifo_count() >= UART_TXFIFO_CAPACITY {
            spin_loop();
        }
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

fn txfifo_count() -> u32 {
    let status = unsafe { read_volatile(UART_STATUS_REG) };
    (status & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
}

fn delay_cycles(spins: usize) {
    for _ in 0..spins {
        spin_loop();
    }
}

fn flush_uart() {
    while txfifo_count() != 0 {
        spin_loop();
    }
}

/// Atomic 64-bit read of UNIT0 via the snapshot dance.
fn read_unit0() -> u64 {
    unsafe {
        write_volatile(SYSTIMER_UNIT0_OP, UNIT_OP_UPDATE_BIT);
        // Poll value_valid; it is set once the snapshot is committed.
        while read_volatile(SYSTIMER_UNIT0_OP) & UNIT_OP_VALID_BIT == 0 {
            spin_loop();
        }
        let hi = read_volatile(SYSTIMER_UNIT0_VAL_HI) as u64;
        let lo = read_volatile(SYSTIMER_UNIT0_VAL_LO) as u64;
        (hi << 32) | lo
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== systimer_probe alive ===");
    flush_uart();

    let date = unsafe { read_volatile(SYSTIMER_DATE) };
    let conf = unsafe { read_volatile(SYSTIMER_CONF) };
    let soc_clk_ctrl2 = unsafe { read_volatile(HP_CLKRST_SOC_CLK_CTRL2) };
    let peri_clk_ctrl21 = unsafe { read_volatile(HP_CLKRST_PERI_CLK_CTRL21) };
    let hp_rst_en1 = unsafe { read_volatile(HP_CLKRST_HP_RST_EN1) };

    let _ = writeln!(uart, "SYSTIMER_DATE         = 0x{:08X}", date);
    let _ = writeln!(uart, "SYSTIMER_CONF         = 0x{:08X}", conf);
    let _ = writeln!(
        uart,
        "  decoded: clk_en={} unit0_work_en={} unit1_work_en={} target0_en={}",
        (conf >> 31) & 1,
        (conf >> 30) & 1,
        (conf >> 29) & 1,
        (conf >> 24) & 1,
    );
    let _ = writeln!(
        uart,
        "SOC_CLK_CTRL2          = 0x{:08X}  (bit23 SYSTIMER_APB_CLK_EN={})",
        soc_clk_ctrl2,
        (soc_clk_ctrl2 >> 23) & 1,
    );
    let _ = writeln!(
        uart,
        "PERI_CLK_CTRL21        = 0x{:08X}  (bit29 CLK_SRC_SEL={} 0=XTAL,1=RC_FAST)",
        peri_clk_ctrl21,
        (peri_clk_ctrl21 >> 29) & 1,
    );
    let _ = writeln!(
        uart,
        "HP_RST_EN1             = 0x{:08X}  (bit5  RST_EN_STIMER={})",
        hp_rst_en1,
        (hp_rst_en1 >> 5) & 1,
    );
    flush_uart();

    let _ = writeln!(uart, "--- 5 snapshots, ~5ms apart ---");
    flush_uart();
    let mut prev = 0u64;
    for i in 0..5 {
        let v = read_unit0();
        let _ = writeln!(uart, "  t[{}] = {:>14}  (delta = {})", i, v, v - prev);
        flush_uart();
        prev = v;
        delay_cycles(50_000);
    }

    // Linearity sweep — three different busy-wait lengths. If counter rate is
    // constant, ticks/spin should match across all three. The previous run
    // measured ~44 ticks per spin_loop iter at espflash --ram boot (CPU not
    // boosted past XTAL by IDF system_init).
    for spins in [50_000usize, 200_000, 1_000_000] {
        let t0 = read_unit0();
        delay_cycles(spins);
        let t1 = read_unit0();
        let dt = t1 - t0;
        let per_spin = dt / spins as u64;
        let _ = writeln!(
            uart,
            "  spins={:>8}  delta_ticks={:>10}  ticks/spin={}",
            spins, dt, per_spin
        );
        flush_uart();
    }
    let _ = writeln!(
        uart,
        "  expected systimer rate = XTAL 40 MHz / 2.5 = 16 MHz (62.5 ns/tick)"
    );
    flush_uart();

    let _ = writeln!(uart, "=== systimer_probe done ===");
    flush_uart();
    loop {
        spin_loop();
    }
}
