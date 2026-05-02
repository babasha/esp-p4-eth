#![no_std]
#![no_main]

//! End-to-end DHCP smoke: SYSTIMER-backed embassy-time + EMAC + embassy-net
//! with DHCPv4 config. Should obtain a lease from a real DHCP server (most
//! consumer routers).
//!
//! Includes the same lab fixes that made `embassy_static_ping` work:
//! - DMA_OP_MODE bit 6 (FUF) so undersized DHCP/ARP frames don't get dropped
//! - MAC_CONFIG ACS=0 to match IDF baseline
//! - MAC_FRAME_FILTER = RA|PR|PM (0x80000011) to bypass remaining filters
//! - 5M-cycle wait after PHY reset deassert
//! - dedicated dma_recovery_task at 1ms cadence
//! - 8 RX/TX descriptors + 8 channel buffers

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::Ordering;

use embassy_executor::{Executor, Spawner};
use embassy_net::{Config, Runner as NetRunner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    diag::{RX_ARP, RX_DHCP_FRAMES, RX_FRAMES, RX_ICMP, RX_LAST_DHCP_FRAME, TX_FRAMES},
    ethernet_task, new_from_static_resources as emac_new, release_waveshare_phy_reset, systimer,
    time_driver::time_polling_task, Device, StaticDmaResources,
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

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
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

#[embassy_executor::task]
async fn dma_recovery_task() -> ! {
    loop {
        Timer::after(Duration::from_millis(1)).await;
        let dma_status = unsafe { read_volatile(0x5009_9014 as *const u32) };
        let rps = (dma_status >> 17) & 0b111;
        if rps == 0 || rps == 4 || dma_status & (1 << 7) != 0 {
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
    let _ = writeln!(uart, "app: started, waiting for link");

    let t0 = systimer::now_us();
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(100)).await;
    }
    let _ = writeln!(uart, "app: link up after {} ms", (systimer::now_us() - t0) / 1000);

    // Apply the lab fixes that made static_ping work — these need to be done
    // AFTER ethernet_task::start() has run mac_init().
    unsafe {
        // FRAME_FILTER = RA | PR | PM
        write_volatile(0x5009_8004 as *mut u32, 0x80000011);
        // MAC_CONFIG: clear ACS (bit 25)
        let mac_cfg = 0x5009_8000 as *mut u32;
        write_volatile(mac_cfg, read_volatile(mac_cfg) & !(1 << 25));
        // DMA_OP_MODE: set FUF (bit 6) — accept undersized frames so 42-byte
        // ARP replies and small DHCP fragments aren't dropped as runts.
        let dma_op_mode = 0x5009_9018 as *mut u32;
        write_volatile(dma_op_mode, read_volatile(dma_op_mode) | (1 << 6));
    }
    let _ = writeln!(
        uart,
        "app: filter+config patched (FUF on, ACS off, RA+PR+PM)"
    );

    let _ = writeln!(uart, "app: requesting DHCP lease...");
    let t1 = systimer::now_us();
    let mut last_print = 0u64;
    loop {
        if stack.config_v4().is_some() {
            break;
        }
        Timer::after(Duration::from_millis(250)).await;
        let elapsed = (systimer::now_us() - t1) / 1_000_000;
        if elapsed != last_print {
            last_print = elapsed;
            let dhcp_n = RX_DHCP_FRAMES.load(Ordering::Relaxed);
            let _ = writeln!(
                uart,
                "  @{}s  rx_frames={} tx_frames={} arp={} icmp={} dhcp_rx={}",
                elapsed,
                RX_FRAMES.load(Ordering::Relaxed),
                TX_FRAMES.load(Ordering::Relaxed),
                RX_ARP.load(Ordering::Relaxed),
                RX_ICMP.load(Ordering::Relaxed),
                dhcp_n,
            );
            if dhcp_n > 0 && elapsed % 5 == 0 {
                let _ = writeln!(uart, "    last DHCP frame (first 64B):");
                for chunk in 0..4 {
                    let _ = writeln!(
                        uart,
                        "      {:02}: {:08X} {:08X} {:08X} {:08X}",
                        chunk * 16,
                        RX_LAST_DHCP_FRAME[chunk * 4].load(Ordering::Relaxed),
                        RX_LAST_DHCP_FRAME[chunk * 4 + 1].load(Ordering::Relaxed),
                        RX_LAST_DHCP_FRAME[chunk * 4 + 2].load(Ordering::Relaxed),
                        RX_LAST_DHCP_FRAME[chunk * 4 + 3].load(Ordering::Relaxed),
                    );
                }
            }
        }
    }
    let dt = (systimer::now_us() - t1) / 1_000;
    let _ = writeln!(uart, "🎉 app: DHCP lease obtained in {} ms!", dt);

    if let Some(cfg) = stack.config_v4() {
        let _ = writeln!(uart, "  address  : {}", cfg.address);
        if let Some(gw) = cfg.gateway {
            let _ = writeln!(uart, "  gateway  : {}", gw);
        }
        for (i, dns) in cfg.dns_servers.iter().enumerate() {
            let _ = writeln!(uart, "  dns[{}]   : {}", i, dns);
        }
    }

    let _ = writeln!(uart, "=== embassy_dhcp OK — staying alive ===");
    let mut tick = 0u64;
    loop {
        Timer::after(Duration::from_secs(5)).await;
        tick += 5;
        let _ = writeln!(uart, "  uptime: {}s  rx={} tx={}", tick,
            RX_FRAMES.load(Ordering::Relaxed),
            TX_FRAMES.load(Ordering::Relaxed));
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== embassy_dhcp alive ===");

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

    let config = Config::dhcpv4(Default::default());
    let resources = NET_RESOURCES.take();
    let (stack, net_runner) = embassy_net::new(device, config, resources, NET_SEED);
    let _ = writeln!(uart, "init: embassy_net::new returned");

    spawner.spawn(time_polling_task()).unwrap();
    spawner.spawn(ethernet_task(emac_runner, eth)).unwrap();
    spawner.spawn(net_task(net_runner)).unwrap();
    spawner.spawn(dma_recovery_task()).unwrap();
    spawner.spawn(app_task(stack)).unwrap();
    let _ = writeln!(uart, "init: all tasks spawned");
}
