#![no_std]
#![no_main]

//! End-to-end smoke: SYSTIMER-backed embassy-time + EMAC + embassy-net with
//! a static IPv4 address. embassy-net's `auto-icmp-echo-reply` feature
//! handles ICMP echo replies automatically — so once link is up the host
//! should be able to `ping 169.254.144.50` and get responses.
//!
//! Use case: in this lab the P4 is direct-attached to the host's Ethernet
//! NIC (no DHCP server). Host autoconf'd to 169.254.144.33/16 (Windows
//! Automatic Private IP). We pick 169.254.144.50/16 in the same link-local
//! subnet so the host can reach us without any additional config.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::Ordering;

use embassy_executor::{Executor, Spawner};
use embassy_net::{Config, Ipv4Address, Ipv4Cidr, Runner as NetRunner, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    diag::{
        RX_ARP, RX_FRAMES, RX_ICMP, RX_IPV4, RX_LAST_DST_MAC_HI, RX_LAST_ETHERTYPE, TX_FRAMES,
        TX_LAST_BUF_ADDR, TX_LAST_DESC_ADDR, TX_LAST_DST_MAC_HI, TX_LAST_ETHERTYPE, TX_LAST_LEN,
        TX_LAST_SRC_MAC_HI, TX_LAST_TDES0, TX_LAST_TDES1,
    },
    ethernet_task, new_from_static_resources as emac_new, release_waveshare_phy_reset, systimer,
    time_driver::time_polling_task, Device, Dma, StaticDmaResources,
};
use static_cell::{ConstStaticCell, StaticCell};

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0xE8, 0xF6, 0x0A, 0xE0, 0x93, 0xF5];
const NET_SEED: u64 = 0x5A17_2026_CAFE_BEEF;

// LAN topology: P4 plugged into router (192.168.0.0/24), host on Wi-Fi.
const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 0, 50);
const SELF_PREFIX: u8 = 24;
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 0, 1);

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
// DMA descriptors and packet buffers MUST live below 0x4FF80000 — the upper
// 256 KB of HP SRAM is the L2 cache backing region, not accessible to the
// EMAC DMA. See memory.x .dma_bss section.
#[link_section = ".dma_bss"]
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());
static NET_RESOURCES: ConstStaticCell<StackResources<4>> =
    ConstStaticCell::new(StackResources::<4>::new());

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

#[embassy_executor::task]
async fn net_task(mut runner: NetRunner<'static, Device<'static>>) -> ! {
    runner.run().await
}

/// Periodic DMA RX maintenance — drains DMA_STATUS so sticky RU/RBU bits get
/// acked and `handle_overflow` reissues the poll demand. Without this, RX
/// engine wedges in Suspended state after ~140 frames and silently drops all
/// further packets (including ARP replies and retransmitted ICMP echoes).
#[embassy_executor::task]
async fn dma_recovery_task() -> ! {
    loop {
        Timer::after(Duration::from_millis(1)).await;
        let dma_status = unsafe { read_volatile(0x5009_9014 as *const u32) };
        let rps = (dma_status >> 17) & 0b111;
        // Only kick when actually suspended/stopped — running state shouldn't
        // be touched. RPS=0 stopped, RPS=4 suspended (RBU). Sticky RU bit at
        // 7 also indicates we missed an unavailable event.
        if rps == 0 || rps == 4 || dma_status & (1 << 7) != 0 {
            // Clear sticky status bits (W1C) and re-issue demand. This is the
            // minimal safe recovery — we don't reset the descriptor list since
            // CPU writes to descriptors are working fine on this build.
            unsafe {
                write_volatile(0x5009_9014 as *mut u32, dma_status & 0x0001E7FF);
                write_volatile(0x5009_9008 as *mut u32, 1); // RX_POLL_DEMAND
            }
        }
    }
}

#[embassy_executor::task]
async fn app_task(stack: Stack<'static>) -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "app: started, self IP = 192.168.0.50/24 gw 192.168.0.1");

    // Wait for link state up.
    let t0 = systimer::now_us();
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(100)).await;
    }
    let _ = writeln!(uart, "app: link up after {} ms", (systimer::now_us() - t0) / 1000);

    // Force promiscuous so unicast ICMP requests addressed to our MAC pass —
    // the default PM-only filter requires the primary MAC address slot to be
    // matched, and ICMP from the host comes via ARP-resolved unicast.
    // 0x80000011 = RA | PR | PM. RA (Receive All, bit 31) bypasses every
    // remaining filter — pass even malformed frames so smoltcp sees them.
    unsafe {
        write_volatile(0x5009_8004 as *mut u32, 0x80000011);
    }
    let _ = writeln!(
        uart,
        "app: MAC_FRAME_FILTER = 0x{:08X} (RA+PR+PM)",
        unsafe { read_volatile(0x5009_8004 as *const u32) },
    );

    // Synopsys GMAC drops frames < 64 bytes (runt) by default. Windows ARP
    // replies are 42 bytes on the wire — they get filtered out, smoltcp never
    // sees them, and we keep re-ARP'ing forever. Set FUF (DMA_OP_MODE bit 6)
    // so the DMA forwards undersized frames to the host.
    unsafe {
        let dma_op_mode = 0x5009_9018 as *mut u32;
        let cur = read_volatile(dma_op_mode);
        write_volatile(dma_op_mode, cur | (1 << 6));
        let _ = writeln!(
            uart,
            "app: DMA_OP_MODE 0x{:08X} -> 0x{:08X} (FUF set)",
            cur,
            read_volatile(dma_op_mode),
        );
    }

    // Match IDF MAC_CONFIG (0x0000CC0C) more closely — clear ACS (Auto CRC
    // Strip, bit 25). Our default 0x0200C88C has ACS=1, IDF baseline ACS=0.
    // ACS=1 may interact badly with short frames after stripping.
    unsafe {
        let mac_cfg = 0x5009_8000 as *mut u32;
        let cur = read_volatile(mac_cfg);
        write_volatile(mac_cfg, cur & !(1 << 25));
        let _ = writeln!(
            uart,
            "app: MAC_CONFIG 0x{:08X} -> 0x{:08X} (ACS cleared)",
            cur,
            read_volatile(mac_cfg),
        );
    }

    if let Some(cfg) = stack.config_v4() {
        let _ = writeln!(uart, "app: stack live — {}", cfg.address);
    } else {
        let _ = writeln!(uart, "app: WARNING — stack.config_v4() is None");
    }

    let _ = writeln!(uart, "app: ready for ping 192.168.0.50");
    let _ = writeln!(uart, "    (run from host: `ping 192.168.0.50`)");

    let mut tick = 0u64;
    loop {
        Timer::after(Duration::from_secs(2)).await;
        tick += 1;
        let _ = writeln!(
            uart,
            "  t={}s rx={} tx={} arp={} icmp={}  rx_eth=0x{:04X}  tx_len={} tx_eth=0x{:04X} tx_dst=0x{:08X} tx_src=0x{:08X}",
            tick * 2,
            RX_FRAMES.load(Ordering::Relaxed),
            TX_FRAMES.load(Ordering::Relaxed),
            RX_ARP.load(Ordering::Relaxed),
            RX_ICMP.load(Ordering::Relaxed),
            RX_LAST_ETHERTYPE.load(Ordering::Relaxed),
            TX_LAST_LEN.load(Ordering::Relaxed),
            TX_LAST_ETHERTYPE.load(Ordering::Relaxed),
            TX_LAST_DST_MAC_HI.load(Ordering::Relaxed),
            TX_LAST_SRC_MAC_HI.load(Ordering::Relaxed),
        );
        // TX descriptor + buffer post-mortem. Compare CPU snapshot of buf
        // contents (cached) to memory view (after invalidate). If they differ,
        // CPU writes to TX buffer didn't make it to memory and DMA shipped
        // garbage on the wire.
        let buf_addr = TX_LAST_BUF_ADDR.load(Ordering::Relaxed);
        let desc_addr = TX_LAST_DESC_ADDR.load(Ordering::Relaxed);
        if buf_addr != 0 {
            let mem_buf = unsafe { Dma::peek_rdes(buf_addr) };
            let cached_buf = unsafe { Dma::peek_rdes_cached(buf_addr) };
            let _ = writeln!(
                uart,
                "    desc@0x{:08X} tdes0=0x{:08X} tdes1=0x{:08X} buf@0x{:08X}",
                desc_addr,
                TX_LAST_TDES0.load(Ordering::Relaxed),
                TX_LAST_TDES1.load(Ordering::Relaxed),
                buf_addr,
            );
            let _ = writeln!(
                uart,
                "    buf cached: {:08X} {:08X} {:08X} {:08X}",
                cached_buf[0], cached_buf[1], cached_buf[2], cached_buf[3],
            );
            let _ = writeln!(
                uart,
                "    buf mem   : {:08X} {:08X} {:08X} {:08X}",
                mem_buf[0], mem_buf[1], mem_buf[2], mem_buf[3],
            );
        }
        let _ = RX_LAST_DST_MAC_HI.load(Ordering::Relaxed);
        let _ = RX_IPV4.load(Ordering::Relaxed);
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== embassy_static_ping alive ===");

    release_waveshare_phy_reset();
    for _ in 0..5_000_000 {
        spin_loop();
    }
    let _ = writeln!(uart, "main: PHY reset deasserted, waited 5M cycles");

    let executor = EXECUTOR.init(Executor::new());
    executor.run(init);
}

fn init(spawner: Spawner) {
    let mut uart = Uart0;
    let _ = writeln!(uart, "init: emac_new...");
    let dma_resources = DMA_RESOURCES.take();
    let (device, emac_runner, eth) = emac_new(MAC_ADDR, dma_resources);
    let _ = writeln!(uart, "init: emac_new returned");

    let config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(SELF_IP, SELF_PREFIX),
        gateway: Some(GATEWAY),
        dns_servers: heapless::Vec::new(),
    });
    let resources = NET_RESOURCES.take();
    let (stack, net_runner) = embassy_net::new(device, config, resources, NET_SEED);
    let _ = writeln!(uart, "init: embassy_net::new returned");

    spawner.spawn(time_polling_task()).unwrap();
    spawner.spawn(ethernet_task(emac_runner, eth)).unwrap();
    spawner.spawn(net_task(net_runner)).unwrap();
    spawner.spawn(dma_recovery_task()).unwrap();
    spawner.spawn(app_task(stack)).unwrap();
    let _ = writeln!(uart, "init: all tasks spawned (incl. dma_recovery)");
}
