#![no_std]
#![no_main]

//! Cold-boot hang diagnostic.
//!
//! Mirrors `embassy_tcp_echo_irq.rs`'s opening sequence verbatim
//! (`time_driver_irq::init()` + `release_waveshare_phy_reset()` + 5M cycle
//! wait), then calls `Ethernet::try_new` synchronously and prints the
//! outcome. No embassy executor, no rx/tx — only what's needed to reproduce
//! the suspected `phy.init` hang in isolation.
//!
//! Run two passes for the bisection:
//!   1. Cold: power-cycle the board, flash and monitor this directly.
//!   2. Warm: run `mdio_test.elf` first (espflash --after no-reset --monitor),
//!      kill monitor, then flash this within ~1 s.
//! Compare the `[phy_init]` markers between the two runs.
//!
//! Build:
//!   cargo build --no-default-features \
//!       --features p4-example,p4-time-driver-irq,phy-init-debug \
//!       --example phy_init_diag

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    release_waveshare_phy_reset, systimer, time_driver_irq, Ethernet, StaticDmaResources,
};
use static_cell::ConstStaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0xE8, 0xF6, 0x0A, 0xE0, 0x93, 0xF5];

#[link_section = ".dma_bss"]
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

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== phy_init_diag alive ===");

    // Match embassy_tcp_echo_irq exactly: configure SYSTIMER alarm + CLIC
    // routing + mtvec BEFORE the PHY reset deassert. Hypothesis: the IRQ
    // setup may matter for cold-boot phy.init behaviour vs. plain mdio_test.
    time_driver_irq::init();
    let _ = writeln!(uart, "diag: time_driver_irq::init() done, t={}us", systimer::now_us());

    release_waveshare_phy_reset();
    for _ in 0..5_000_000 {
        spin_loop();
    }
    let _ = writeln!(uart, "diag: PHY reset deasserted, waited 5M cycles, t={}us", systimer::now_us());

    let resources = DMA_RESOURCES.take();
    let (tx_desc, rx_desc, tx_buf, rx_buf) = resources.split();
    let _ = writeln!(uart, "diag: DMA resources allocated, t={}us", systimer::now_us());

    let _ = writeln!(uart, "diag: calling Ethernet::try_new ...");
    let t0 = systimer::now_us();
    let result = Ethernet::try_new(MAC_ADDR, tx_desc, rx_desc, tx_buf, rx_buf);
    let elapsed_us = systimer::now_us().wrapping_sub(t0);

    match result {
        Ok(_eth) => {
            let _ = writeln!(uart, "diag: Ethernet::try_new OK in {} us", elapsed_us);
        }
        Err(e) => {
            let _ = writeln!(uart, "diag: Ethernet::try_new ERR after {} us: {:?}", elapsed_us, e);
        }
    }

    // Post-state register dump regardless of outcome — same fields as
    // mdio_test for side-by-side comparison.
    let _ = writeln!(uart, "--- post-call register dump ---");
    let r = |a: usize| unsafe { read_volatile(a as *const u32) };
    let _ = writeln!(uart, "  PERI_CLK_CTRL00  = 0x{:08X}", r(0x500E_6030));
    let _ = writeln!(uart, "  PERI_CLK_CTRL01  = 0x{:08X}", r(0x500E_6034));
    let _ = writeln!(uart, "  LP_HP_CLK_CTRL   = 0x{:08X}", r(0x5011_1040));
    let _ = writeln!(uart, "  SYS_GMAC_CTRL0   = 0x{:08X}", r(0x500E_514C));
    let _ = writeln!(uart, "  MAC_CONFIG       = 0x{:08X}", r(0x5009_8000));
    let _ = writeln!(uart, "  MAC_FRAME_FILTER = 0x{:08X}", r(0x5009_8004));
    let mii_addr = r(0x5009_8010);
    let _ = writeln!(
        uart,
        "  MII_ADDR         = 0x{:08X}  (csr_clk_range bits[5:2] = 0b{:04b})",
        mii_addr,
        (mii_addr >> 2) & 0xF
    );
    let _ = writeln!(uart, "  MII_DATA         = 0x{:08X}", r(0x5009_8014));
    let _ = writeln!(uart, "  MAC_VERSION      = 0x{:08X}", r(0x5009_8020));
    let _ = writeln!(uart, "  DMA_BUS_MODE     = 0x{:08X}", r(0x5009_9000));
    let _ = writeln!(uart, "  DMA_OP_MODE      = 0x{:08X}", r(0x5009_9018));
    let _ = writeln!(uart, "  DMA_STATUS       = 0x{:08X}", r(0x5009_9014));

    let _ = writeln!(uart, "=== phy_init_diag done ===");
    loop {
        spin_loop();
    }
}
