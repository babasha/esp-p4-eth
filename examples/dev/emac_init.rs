#![no_std]
#![no_main]

//! Hardware sanity check for the esp-p4-eth driver init pipeline.
//!
//! Calls `Ethernet::try_new` directly (bypassing the channel wrapper) and prints checkpoints
//! over UART0. Successfully reaching `[init] try_new ok` means the entire EMAC bootstrap path
//! ran end-to-end on hardware: RMII pins routed, clock tree up, REF_CLK live, MAC/DMA reset
//! cleared, MDIO reaches the PHY, descriptor rings written back to memory through the cache.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{mdio_read, release_waveshare_phy_reset, Ethernet, StaticDmaResources, BMSR};
use static_cell::StaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

static DMA_RESOURCES: StaticCell<StaticDmaResources> = StaticCell::new();

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
    let _ = writeln!(uart, "\r\n[init] esp-p4-eth init sanity check");

    // Drive PHY RESET high then wait long enough for the IP101GRI internal oscillator and
    // its register file to come up (datasheet says ~100 ms after RESET deassert).
    release_waveshare_phy_reset();
    for _ in 0..3_000_000 {
        spin_loop();
    }
    let _ = writeln!(uart, "[init] PHY reset released, waited");

    let dma_resources = DMA_RESOURCES.init_with(StaticDmaResources::new);
    let _ = writeln!(uart, "[init] DMA resources allocated");

    let (tx, rx, txb, rxb) = dma_resources.split();
    let _ = writeln!(uart, "[init] calling Ethernet::try_new...");
    let mut eth = Ethernet::try_new(MAC_ADDR, tx, rx, txb, rxb)
        .expect("EMAC bootstrap failed");
    let _ = writeln!(uart, "[init] try_new ok");

    let _ = writeln!(uart, "[init] starting MAC + DMA datapath...");
    eth.start().expect("eth.start() failed");
    let _ = writeln!(uart, "[init] eth.start() ok");

    // Dump GPIO matrix routing state for MDC/MDIO so we can verify the ROM helpers actually
    // wrote the right signal indices to the right pins.
    let _ = writeln!(uart, "[init] dumping GPIO matrix routing for MDC/MDIO...");
    unsafe {
        const GPIO_FUNC_OUT_SEL_BASE: usize = 0x500E_0000 + 0x554;
        const GPIO_FUNC_IN_SEL_BASE: usize = 0x500E_0000 + 0x154;
        for pin in [31u32, 52] {
            let v = read_volatile((GPIO_FUNC_OUT_SEL_BASE + pin as usize * 4) as *const u32);
            let _ = writeln!(uart, "  GPIO_FUNC{}_OUT_SEL_CFG = 0x{:08X}", pin, v);
        }
        for sig in [107u32, 108, 109] {
            let v = read_volatile((GPIO_FUNC_IN_SEL_BASE + sig as usize * 4) as *const u32);
            let _ = writeln!(uart, "  GPIO_FUNC{}_IN_SEL_CFG  = 0x{:08X}", sig, v);
        }
        // Also dump IO_MUX for the two pins
        const IO_MUX_BASE: usize = 0x500E_1000;
        for pin in [31u32, 52] {
            let v = read_volatile((IO_MUX_BASE + 4 + pin as usize * 4) as *const u32);
            let _ = writeln!(uart, "  IO_MUX_GPIO{}        = 0x{:08X}", pin, v);
        }
    }

    let _ = writeln!(uart, "[init] scanning MDIO addresses 0..32 for live PHY...");
    let mut found_addr: Option<u8> = None;
    for phy_addr in 0u8..32 {
        let v = mdio_read(phy_addr, BMSR).unwrap_or(0xDEAD);
        let live = v != 0 && v != 0xFFFF && v != 0xDEAD;
        let _ = writeln!(
            uart,
            "  addr={:>2}: BMSR=0x{:04X}{}",
            phy_addr,
            v,
            if live { " <- LIVE" } else { "" }
        );
        if live && found_addr.is_none() {
            found_addr = Some(phy_addr);
        }
    }
    let phy_addr = found_addr.unwrap_or(0);
    let _ = writeln!(uart, "[init] using PHY address {}", phy_addr);

    let _ = writeln!(uart, "[init] entering link polling loop");
    let mut last_link = false;
    let mut tick = 0u32;
    loop {
        let link = eth.phy_link_status();
        if link != last_link {
            let _ = writeln!(
                uart,
                "[link] state changed: {}",
                if link { "UP" } else { "DOWN" }
            );
            last_link = link;
            if link {
                match eth.phy_negotiate() {
                    Ok((speed, duplex)) => {
                        let _ = writeln!(
                            uart,
                            "[link] negotiated: {:?} {:?}",
                            speed, duplex
                        );
                    }
                    Err(e) => {
                        let _ = writeln!(uart, "[link] negotiation error: {:?}", e);
                    }
                }
            }
        }
        for _ in 0..2_000_000 {
            spin_loop();
        }
        tick = tick.wrapping_add(1);
        if tick % 4 == 0 {
            let _ = writeln!(
                uart,
                "[tick {}] link={}",
                tick,
                if link { "UP" } else { "DOWN" }
            );
        }
    }
}
