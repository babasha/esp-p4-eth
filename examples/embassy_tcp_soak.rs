#![no_std]
#![no_main]

//! Long-duration TCP soak harness.
//!
//! Four parallel TCP echo listeners on ports 7780-7783, each backed by a
//! 4 KB RX + 4 KB TX socket buffer (2× the per-connection window of
//! `embassy_tcp_echo_irq`, 4× the parallelism). Per-port byte counters
//! plus full diagnostic snapshot every 60 seconds, hourly summary line.
//!
//! Designed for backbone reliability validation — leaks, ring-wedge,
//! cumulative cache-call drift, link-state churn — not for peak throughput.
//! The 192 KB on-chip SRAM does not fit four 32 KB sockets; if you need
//! BDP-saturating windows on a single connection, drop the listener count
//! to one and bump the buffer sizes locally.
//!
//! Run from host:
//!
//! ```bash
//! # Open four parallel iperf-style streams via socat (or use the soak
//! # driver script under `examples/dev/soak_driver.sh`):
//! for p in 7780 7781 7782 7783; do
//!     socat -u FILE:/dev/urandom TCP:192.168.0.50:$p &
//! done
//! ```
//!
//! Build:
//!   cargo build --no-default-features \
//!       --features p4-example,p4-time-driver-irq,embassy-net-tcp \
//!       --example embassy_tcp_soak

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{
    Config, IpListenEndpoint, Ipv4Address, Ipv4Cidr, Runner as NetRunner, Stack, StackResources,
    StaticConfigV4,
};
use embassy_time::{Duration, Timer};
use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    diag::{
        CACHE_INV_CALLS, CACHE_INV_TICKS, CACHE_WB_CALLS, CACHE_WB_TICKS, RX_ERROR_FRAMES_TOTAL,
        RX_FRAMES, RX_OVERSIZED_FRAMES_TOTAL, RX_RUNT_FRAMES_TOTAL, TX_FRAMES,
    },
    ethernet_task, new_from_static_resources as emac_new, release_waveshare_phy_reset, systimer,
    time_driver_irq, Device, StaticDmaResources,
};
use static_cell::{ConstStaticCell, StaticCell};

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0xE8, 0xF6, 0x0A, 0xE0, 0x93, 0xF5];
const NET_SEED: u64 = 0x504F_414B_5350_4F4B; // "POAKSPOK"

const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 0, 50);
const SELF_PREFIX: u8 = 24;
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 0, 1);

const SOAK_PORTS: [u16; 4] = [7780, 7781, 7782, 7783];
// 4 KB per direction × 4 listeners = 32 KB CPU-side buffers, the largest
// footprint that fits inside the 192 KB on-chip SRAM next to the existing
// driver code, ring storage, NetStackResources, and stack. Bump only after
// shrinking listener count or trimming the static_cell pools.
const TCP_BUF_SIZE: usize = 4 * 1024;
const STAT_PERIOD_S: u64 = 60;
const HOURLY_TICKS: u64 = 60;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
#[link_section = ".dma_bss"]
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());
static NET_RESOURCES: ConstStaticCell<StackResources<6>> =
    ConstStaticCell::new(StackResources::<6>::new());

// One TCP socket buffer pair per listener. Plain RAM (CPU-side), not DMA.
static TCP_RX_BUF_0: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_TX_BUF_0: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_RX_BUF_1: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_TX_BUF_1: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_RX_BUF_2: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_TX_BUF_2: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_RX_BUF_3: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_TX_BUF_3: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);

// Per-port observability. AtomicU32 (RISC-V 32-bit has no native AtomicU64)
// rolls over every ~4 GB; at saturating 100 Mbps line rate that is ~5.7 min,
// so the printed cumulative total is informational only. The per-tick delta
// (60 s × 12.5 MB/s = 750 MB max) always fits in u32 and the stat task uses
// `wrapping_sub` so the throughput line stays correct across rollovers.
static BYTES_ECHOED: [AtomicU32; 4] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];
static CONN_COUNT: [AtomicU32; 4] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

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
        Timer::after(Duration::from_millis(50)).await;
        let dma_status = unsafe { read_volatile(0x5009_9014 as *const u32) };
        let rps = (dma_status >> 17) & 0b111;
        if rps == 0 || rps == 4 || dma_status & (1 << 7) != 0 {
            unsafe {
                write_volatile(0x5009_9014 as *mut u32, dma_status & 0x0001E7FF);
                write_volatile(0x5009_9008 as *mut u32, 1);
            }
        }
    }
}

/// Waits for link-up and applies the post-bringup MAC pokes once, then
/// returns. Listener tasks call this in parallel; idempotent register
/// writes mean the four serialized races are safe.
async fn finish_emac_bringup(stack: Stack<'static>, idx: usize, uart: &mut Uart0) {
    if idx == 0 {
        let t0 = systimer::now_us();
        while !stack.is_link_up() {
            Timer::after(Duration::from_millis(100)).await;
        }
        let _ = writeln!(
            uart,
            "soak: link up after {} ms",
            (systimer::now_us() - t0) / 1000
        );

        unsafe {
            write_volatile(0x5009_8004 as *mut u32, 0x80000011);
            let dma_op_mode = 0x5009_9018 as *mut u32;
            write_volatile(dma_op_mode, read_volatile(dma_op_mode) | (1 << 6));
            let mac_cfg = 0x5009_8000 as *mut u32;
            write_volatile(mac_cfg, read_volatile(mac_cfg) & !(1 << 25));
        }
        let _ = writeln!(uart, "soak: filter+FUF+ACS-clear applied");
    } else {
        while !stack.is_link_up() {
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

#[embassy_executor::task(pool_size = 4)]
async fn tcp_echo_task(
    stack: Stack<'static>,
    idx: usize,
    port: u16,
    rx_buf: &'static mut [u8],
    tx_buf: &'static mut [u8],
) -> ! {
    let mut uart = Uart0;
    finish_emac_bringup(stack, idx, &mut uart).await;

    let mut socket = TcpSocket::new(stack, rx_buf, tx_buf);
    socket.set_timeout(Some(Duration::from_secs(120)));
    socket.set_keep_alive(Some(Duration::from_secs(30)));

    let _ = writeln!(uart, "soak[{}]: listening on :{}", idx, port);

    let mut buf = [0u8; 2048];
    loop {
        match socket
            .accept(IpListenEndpoint { addr: None, port })
            .await
        {
            Ok(()) => {
                CONN_COUNT[idx].fetch_add(1, Ordering::Relaxed);
            }
            Err(e) => {
                let _ = writeln!(uart, "soak[{}]: accept err {:?}", idx, e);
                socket.abort();
                let _ = socket.flush().await;
                continue;
            }
        }

        loop {
            match socket.read(&mut buf).await {
                Ok(0) => break,
                Ok(n) => {
                    // Drain the slice across potentially-partial writes so the
                    // soak run cannot lose bytes when the TX window is small.
                    let mut wrote = 0usize;
                    let mut write_err = false;
                    while wrote < n {
                        match socket.write(&buf[wrote..n]).await {
                            Ok(0) => {
                                write_err = true;
                                break;
                            }
                            Ok(w) => wrote += w,
                            Err(e) => {
                                let _ = writeln!(uart, "soak[{}]: write err {:?}", idx, e);
                                write_err = true;
                                break;
                            }
                        }
                    }
                    if write_err {
                        break;
                    }
                    BYTES_ECHOED[idx].fetch_add(n as u32, Ordering::Relaxed);
                }
                Err(e) => {
                    let _ = writeln!(uart, "soak[{}]: read err {:?}", idx, e);
                    break;
                }
            }
        }
        socket.close();
        let _ = socket.flush().await;
        socket.abort();
        let _ = socket.flush().await;
    }
}

#[embassy_executor::task]
async fn stat_task() -> ! {
    let mut uart = Uart0;
    let mut tick = 0u64;
    let mut prev_bytes = [0u32; 4];
    let mut prev_rx = 0u32;
    let mut prev_tx = 0u32;

    loop {
        Timer::after(Duration::from_secs(STAT_PERIOD_S)).await;
        tick += 1;
        let uptime_s = tick * STAT_PERIOD_S;

        let bytes: [u32; 4] = [
            BYTES_ECHOED[0].load(Ordering::Relaxed),
            BYTES_ECHOED[1].load(Ordering::Relaxed),
            BYTES_ECHOED[2].load(Ordering::Relaxed),
            BYTES_ECHOED[3].load(Ordering::Relaxed),
        ];
        let conns: [u32; 4] = [
            CONN_COUNT[0].load(Ordering::Relaxed),
            CONN_COUNT[1].load(Ordering::Relaxed),
            CONN_COUNT[2].load(Ordering::Relaxed),
            CONN_COUNT[3].load(Ordering::Relaxed),
        ];
        let rx_total = RX_FRAMES.load(Ordering::Relaxed);
        let tx_total = TX_FRAMES.load(Ordering::Relaxed);

        // wrapping_sub keeps deltas correct across u32 rollover (every ~4 GB
        // / ~6 min at line rate).
        let delta_bytes: u64 = bytes
            .iter()
            .zip(prev_bytes.iter())
            .map(|(b, p)| b.wrapping_sub(*p) as u64)
            .sum();
        let throughput_bps = delta_bytes * 8 / STAT_PERIOD_S;

        let _ = writeln!(
            uart,
            "[t={}m] thrpt={} kbps  rx_frm={} (+{})  tx_frm={} (+{})",
            uptime_s / 60,
            throughput_bps / 1000,
            rx_total,
            rx_total.wrapping_sub(prev_rx),
            tx_total,
            tx_total.wrapping_sub(prev_tx),
        );
        for i in 0..4 {
            let _ = writeln!(
                uart,
                "  port {}: bytes={} (+{}) conns={}",
                SOAK_PORTS[i],
                bytes[i],
                bytes[i].wrapping_sub(prev_bytes[i]),
                conns[i],
            );
        }
        let runt = RX_RUNT_FRAMES_TOTAL.load(Ordering::Relaxed);
        let oversized = RX_OVERSIZED_FRAMES_TOTAL.load(Ordering::Relaxed);
        let err = RX_ERROR_FRAMES_TOTAL.load(Ordering::Relaxed);
        if runt != 0 || oversized != 0 || err != 0 {
            let _ = writeln!(
                uart,
                "  ERRORS: runt={} oversized={} err={}",
                runt, oversized, err
            );
        }

        if tick % HOURLY_TICKS == 0 {
            let inv_calls = CACHE_INV_CALLS.load(Ordering::Relaxed);
            let inv_ticks = CACHE_INV_TICKS.load(Ordering::Relaxed);
            let wb_calls = CACHE_WB_CALLS.load(Ordering::Relaxed);
            let wb_ticks = CACHE_WB_TICKS.load(Ordering::Relaxed);
            let _ = writeln!(
                uart,
                "=== HOURLY @ t={}h: rx={} tx={} cache_inv={}/{} cache_wb={}/{} ===",
                tick / HOURLY_TICKS,
                rx_total,
                tx_total,
                inv_calls,
                inv_ticks,
                wb_calls,
                wb_ticks,
            );
        }

        prev_bytes = bytes;
        prev_rx = rx_total;
        prev_tx = tx_total;
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== embassy_tcp_soak alive ===");

    time_driver_irq::init();
    let _ = writeln!(uart, "main: time_driver_irq::init() done");

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
    let _ = writeln!(uart, "init: enter");
    let dma_resources = DMA_RESOURCES.take();
    let _ = writeln!(uart, "init: DMA_RESOURCES.take() ok");
    let (device, emac_runner, eth) = emac_new(MAC_ADDR, dma_resources);
    let _ = writeln!(uart, "init: emac_new returned");

    let config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(SELF_IP, SELF_PREFIX),
        gateway: Some(GATEWAY),
        dns_servers: heapless::Vec::new(),
    });
    let resources = NET_RESOURCES.take();
    let _ = writeln!(uart, "init: NET_RESOURCES.take() ok");
    let (stack, net_runner) = embassy_net::new(device, config, resources, NET_SEED);
    let _ = writeln!(uart, "init: embassy_net::new returned");

    spawner.spawn(ethernet_task(emac_runner, eth)).unwrap();
    spawner.spawn(net_task(net_runner)).unwrap();
    spawner.spawn(dma_recovery_task()).unwrap();

    spawner
        .spawn(tcp_echo_task(
            stack,
            0,
            SOAK_PORTS[0],
            TCP_RX_BUF_0.take(),
            TCP_TX_BUF_0.take(),
        ))
        .unwrap();
    spawner
        .spawn(tcp_echo_task(
            stack,
            1,
            SOAK_PORTS[1],
            TCP_RX_BUF_1.take(),
            TCP_TX_BUF_1.take(),
        ))
        .unwrap();
    spawner
        .spawn(tcp_echo_task(
            stack,
            2,
            SOAK_PORTS[2],
            TCP_RX_BUF_2.take(),
            TCP_TX_BUF_2.take(),
        ))
        .unwrap();
    spawner
        .spawn(tcp_echo_task(
            stack,
            3,
            SOAK_PORTS[3],
            TCP_RX_BUF_3.take(),
            TCP_TX_BUF_3.take(),
        ))
        .unwrap();

    spawner.spawn(stat_task()).unwrap();
    let _ = writeln!(uart, "init: 4 listeners + stat_task spawned");
}
