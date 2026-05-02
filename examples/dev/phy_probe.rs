#![no_std]
#![no_main]

//! PHY diagnostic probe.
//!
//! Bootstraps the EMAC just enough to talk MDIO (no DMA traffic, no link wait),
//! then scans every PHY address and dumps the standard Clause 22 register block
//! for the first live address. Re-reads BMSR periodically and prints whenever
//! its value changes, so we can watch link/auto-neg state evolve in real time.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    mdio_read, release_waveshare_phy_reset, Ethernet, StaticDmaResources, ANAR, ANLPAR, BMCR,
    BMSR, PHYIDR1, PHYIDR2,
};
use static_cell::ConstStaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

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
    let _ = writeln!(uart, "\r\n=== phy_probe ===");

    release_waveshare_phy_reset();
    delay_cycles(3_000_000);
    let _ = writeln!(uart, "phy reset released");

    let dma_resources = DMA_RESOURCES.take();
    let _ = writeln!(uart, "dma resources allocated");

    // try_new performs configure_rmii_pins + configure_mdio_pins + configure_clock_ext_in +
    // mac_init + phy.init. After this, MDIO is fully usable at the right MDC rate.
    //
    // CRUCIAL: bind the result to a named local. `let _ = Ethernet::...` would
    // drop the value immediately, and `Ethernet::Drop::shutdown` disables the
    // EMAC clock tree. Subsequent `mdio_read` calls would then stall the AHB
    // bus because the peripheral is unclocked.
    let _eth = Ethernet::new_from_static_resources(MAC_ADDR, dma_resources);
    let _ = writeln!(uart, "ethernet bootstrap ok; starting MDIO scan...");

    // Scan every Clause 22 PHY address.
    let mut live_addr: Option<u8> = None;
    for addr in 0u8..32 {
        let v = mdio_read(addr, BMSR).unwrap_or(0xDEAD);
        let alive = v != 0x0000 && v != 0xFFFF && v != 0xDEAD;
        let _ = writeln!(
            uart,
            "  addr={:>2}: BMSR=0x{:04X}{}",
            addr,
            v,
            if alive { " <- live" } else { "" }
        );
        if alive && live_addr.is_none() {
            live_addr = Some(addr);
        }
    }

    let addr = live_addr.unwrap_or(0);
    let _ = writeln!(uart, "using PHY addr {}", addr);

    // Full register dump for the live PHY.
    let bmcr = mdio_read(addr, BMCR).unwrap_or(0xDEAD);
    let bmsr = mdio_read(addr, BMSR).unwrap_or(0xDEAD);
    let id1 = mdio_read(addr, PHYIDR1).unwrap_or(0xDEAD);
    let id2 = mdio_read(addr, PHYIDR2).unwrap_or(0xDEAD);
    let anar = mdio_read(addr, ANAR).unwrap_or(0xDEAD);
    let anlpar = mdio_read(addr, ANLPAR).unwrap_or(0xDEAD);
    let _ = writeln!(uart, "  BMCR    = 0x{:04X}", bmcr);
    let _ = writeln!(uart, "  BMSR    = 0x{:04X}  link={} an_done={}", bmsr, (bmsr >> 2) & 1, (bmsr >> 5) & 1);
    let _ = writeln!(uart, "  PHYIDR1 = 0x{:04X}", id1);
    let _ = writeln!(uart, "  PHYIDR2 = 0x{:04X}", id2);
    let _ = writeln!(uart, "  ANAR    = 0x{:04X}", anar);
    let _ = writeln!(uart, "  ANLPAR  = 0x{:04X}", anlpar);

    let _ = writeln!(uart, "polling BMSR (printing changes only)...");
    let mut last = bmsr;
    let mut tick = 0u32;
    loop {
        let v = mdio_read(addr, BMSR).unwrap_or(0xDEAD);
        if v != last {
            let _ = writeln!(
                uart,
                "[tick {}] BMSR 0x{:04X} -> 0x{:04X}  link={} an_done={}",
                tick, last, v, (v >> 2) & 1, (v >> 5) & 1
            );
            last = v;
        }
        delay_cycles(100_000);
        tick = tick.wrapping_add(1);
        if tick % 40 == 0 {
            let _ = writeln!(uart, "[tick {}] BMSR=0x{:04X} (still {})", tick, last, if (last >> 2) & 1 != 0 { "UP" } else { "DOWN" });
        }
    }
}
