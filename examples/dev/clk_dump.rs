#![no_std]
#![no_main]

//! Bring-up diagnostic: clock-tree register dump.
//!
//! Bypasses `Ethernet::new_from_static_resources` (known to mis-behave due to
//! large stack frames) and instead drives the lib's public init helpers step by
//! step. Reads HP_SYS_CLKRST / LP_CLKRST / EMAC / HP_SYSTEM registers ONE AT A
//! TIME with explicit `fence(SeqCst)` and a short delay between accesses, both
//! BEFORE and AFTER `configure_clock_ext_in`. The label printed before each
//! read pinpoints exactly which address triggers a CPU crash if one occurs.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{fence, Ordering};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    configure_clock_ext_in, configure_rmii_pins, pins, release_waveshare_phy_reset, RefClockPin,
};

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

/// Reads one 32-bit MMIO register with strict ordering and a short settling
/// delay on either side, so a crash at this address is unambiguous.
#[inline(never)]
fn safe_read32(addr: usize) -> u32 {
    fence(Ordering::SeqCst);
    delay_cycles(64);
    let value = unsafe { read_volatile(addr as *const u32) };
    fence(Ordering::SeqCst);
    delay_cycles(64);
    value
}

/// Drains the UART TX FIFO so a print before a potential crash is observable.
fn flush_uart() {
    while txfifo_count() != 0 {
        spin_loop();
    }
    delay_cycles(50_000);
}

fn dump_named(uart: &mut Uart0, name: &str, addr: usize) {
    let _ = writeln!(uart, "  read {:18} @ 0x{:08X} ...", name, addr);
    flush_uart();
    let value = safe_read32(addr);
    let _ = writeln!(uart, "  {:18} = 0x{:08X}", name, value);
    flush_uart();
}

fn dump_clock_section(uart: &mut Uart0, label: &str) {
    let _ = writeln!(uart, "--- clock-tree dump ({}) ---", label);
    flush_uart();

    // HP_SYS_CLKRST (base 0x500E_6000)
    dump_named(uart, "SOC_CLK_CTRL1", 0x500E_6018);
    dump_named(uart, "REF_CLK_CTRL0", 0x500E_6024);
    dump_named(uart, "REF_CLK_CTRL1", 0x500E_6028);
    dump_named(uart, "PERI_CLK_CTRL00", 0x500E_6030);
    dump_named(uart, "PERI_CLK_CTRL01", 0x500E_6034);

    // LP_CLKRST (base 0x5011_1000)
    dump_named(uart, "LP_HP_CLK_CTRL", 0x5011_1040);
    dump_named(uart, "LP_EMAC_RST_CTRL", 0x5011_104C);

    // HP_SYSTEM (base 0x500E_5000)
    dump_named(uart, "SYS_GMAC_CTRL0", 0x500E_514C);
}

/// Dumps the EMAC core registers. CALLER MUST HAVE ENABLED THE EMAC CLOCK TREE
/// FIRST — reading any register in `0x5009_8000`–`0x5009_9FFF` while the EMAC
/// is unclocked stalls the AHB bus and the CPU never returns.
fn dump_emac_section(uart: &mut Uart0, label: &str) {
    let _ = writeln!(uart, "--- EMAC core dump ({}) ---", label);
    flush_uart();

    dump_named(uart, "MAC_CONFIG", 0x5009_8000);
    dump_named(uart, "MAC_FRAME_FILTER", 0x5009_8004);
    dump_named(uart, "MII_ADDR", 0x5009_8010);
    dump_named(uart, "MII_DATA", 0x5009_8014);
    dump_named(uart, "MAC_VERSION", 0x5009_8020);
    dump_named(uart, "DMA_BUS_MODE", 0x5009_9000);
    dump_named(uart, "DMA_OP_MODE", 0x5009_9018);
    dump_named(uart, "DMA_STATUS", 0x5009_9014);
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\r\n=== clk_dump alive ===");
    flush_uart();

    // PASS 1 — fresh boot. Skip EMAC reads here: the EMAC peripheral is not
    // clocked yet, so reading anything in 0x5009_8000+ would stall the AHB bus.
    dump_clock_section(&mut uart, "before init");

    // PASS 2 — after step-by-step lib init via public API. Mirrors what
    // `Ethernet::try_new` does up to (but not including) `mac_init`/`phy.init`.
    let _ = writeln!(uart, "release_waveshare_phy_reset()");
    flush_uart();
    release_waveshare_phy_reset();
    delay_cycles(3_000_000);

    let _ = writeln!(uart, "configure_rmii_pins()");
    flush_uart();
    configure_rmii_pins();

    let _ = writeln!(uart, "configure_mdio_pins()");
    flush_uart();
    pins::configure_mdio_pins();

    let _ = writeln!(uart, "configure_clock_ext_in(Gpio50)");
    flush_uart();
    configure_clock_ext_in(RefClockPin::Gpio50);

    // After `configure_clock_ext_in` the EMAC clock tree is up — EMAC reads
    // are now safe. Dump everything for side-by-side comparison with the IDF
    // baseline (see memory: reference_p4_emac_baseline_dump).
    dump_clock_section(&mut uart, "after configure_clock_ext_in");
    dump_emac_section(&mut uart, "after configure_clock_ext_in");

    let _ = writeln!(uart, "=== clk_dump done ===");
    flush_uart();
    loop {
        spin_loop();
    }
}
