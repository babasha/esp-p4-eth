#![no_std]
#![no_main]

//! Smoke test for the canonical `embassy-net-driver-channel` constructor.
//!
//! Validates that `esp_p4_eth::new_from_static_resources` (top-level
//! wrapper that allocates `ch::State` via the internal STATE cell, calls
//! `embassy_net_driver_channel::new`, and bundles the result with an
//! `Ethernet`) does not hang on startup.
//!
//! Earlier `STATE: StaticCell<ch::State<…>>` + `STATE.init_with(ch::State::new)`
//! built a ~12 KB value through a stack temporary, overflowing the riscv-rt
//! default stack and losing control around the tuple-return frame. After the
//! switch to `ConstStaticCell` the State storage lives in `.bss` and `take()`
//! is zero-cost.
//!
//! Success criterion: this prints `embassy_smoke OK` and reads PHY ID over
//! MDIO. Hardware data plane is already covered by `mdio_test.rs`.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    mdio_read, new_from_static_resources, release_waveshare_phy_reset, StaticDmaResources, BMSR,
    PHYIDR1, PHYIDR2,
};
use static_cell::ConstStaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0xE8, 0xF6, 0x0A, 0xE0, 0x93, 0xF5];

// Both StaticCell stores in this example use the const-init form to avoid
// the ~12 KB stack copy that StaticCell::init / init_with triggers on
// riscv-rt's default stack.
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());

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

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\r\n=== embassy_smoke alive ===");

    release_waveshare_phy_reset();
    delay_cycles(3_000_000);

    let resources = DMA_RESOURCES.take();
    let _ = writeln!(uart, "DMA resources taken");

    // The line that used to hang. With `STATE: ConstStaticCell<ch::State<…>>`
    // it should return promptly.
    let _ = writeln!(uart, "calling new_from_static_resources()...");
    let (_device, _runner, _eth) = new_from_static_resources(MAC_ADDR, resources);
    let _ = writeln!(uart, "  returned cleanly");

    // Confirm MDIO still works through this construction path.
    let bmsr = mdio_read(1, BMSR).unwrap_or(0xDEAD);
    let id1 = mdio_read(1, PHYIDR1).unwrap_or(0xDEAD);
    let id2 = mdio_read(1, PHYIDR2).unwrap_or(0xDEAD);
    let _ = writeln!(
        uart,
        "PHY@1: ID1=0x{:04X} ID2=0x{:04X} BMSR=0x{:04X}",
        id1, id2, bmsr
    );

    if id1 == 0x0243 {
        let _ = writeln!(uart, "embassy_smoke OK");
    } else {
        let _ = writeln!(uart, "embassy_smoke: PHY ID mismatch (expected 0x0243)");
    }

    loop {
        spin_loop();
    }
}
