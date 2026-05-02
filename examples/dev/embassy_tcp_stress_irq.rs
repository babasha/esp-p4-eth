#![no_std]
#![no_main]

//! Sustained-throughput stress test on the IRQ-driven path.
//!
//! Two independent TCP listeners — one purely RX-bound, one purely TX-bound —
//! plus a 1-second `stat_task` that prints byte deltas, frame deltas, EMAC
//! IRQ deltas, and `dma_recovery_task` activity. The point is to push the
//! NIC hard in each direction in isolation and see whether `RU` / abnormal
//! recovery actually fires under sustained load (currently AIE is masked
//! and recovery lives in a polling task — we want data to decide if that's
//! still needed).
//!
//! ## Wiring
//!
//! Same Waveshare ESP32-P4-ETH topology as `embassy_tcp_echo_irq`. Static IP
//! `192.168.0.50/24`, gateway `192.168.0.1`.
//!
//! ## Driving load from the host
//!
//! RX-stress (host → P4):
//!
//!     pv -B 1M /dev/zero | nc 192.168.0.50 7780
//!
//! TX-stress (P4 → host):
//!
//!     nc 192.168.0.50 7781 | pv -B 1M > /dev/null
//!
//! Or with iperf3 (any side as server is fine — netcat is enough for
//! sustained-rate sanity, iperf3 adds RTT/jitter).

use core::arch::asm;
use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};

// ESP32-P4 HP CPU clock at boot is whatever the ROM bootloader left us at;
// no clock-tree init in the bare-metal Rust path. Empirically `mcycle` on
// the Andes core ticks at ~40 MHz (XTAL freq) regardless of CPU clock, and
// does not halt during wfi — useless for CPU% on its own. `minstret` does
// halt during wfi (no instructions retire), so we sample both: mcycle for
// wall-clock reference, minstret as the active-cycles signal.

#[inline(always)]
fn mcycle_low() -> u32 {
    let v: u32;
    unsafe { asm!("csrr {}, mcycle", out(reg) v, options(nostack, nomem)) };
    v
}

#[inline(always)]
fn minstret_low() -> u32 {
    let v: u32;
    unsafe { asm!("csrr {}, minstret", out(reg) v, options(nostack, nomem)) };
    v
}

use embassy_executor::{Executor, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{
    Config, IpListenEndpoint, Ipv4Address, Ipv4Cidr, Runner as NetRunner, Stack, StackResources,
    StaticConfigV4,
};
use embassy_time::{Duration, Timer};
use esp_p4_eth::{
    diag::{
        CACHE_INV_CALLS, CACHE_INV_TICKS, CACHE_WB_CALLS, CACHE_WB_TICKS, RX_ERROR_FRAMES_TOTAL,
        RX_FRAMES, RX_OVERSIZED_FRAMES_TOTAL, RX_RUNT_FRAMES_TOTAL, TX_FRAMES,
    },
    ethernet_task, new_from_static_resources as emac_new, release_waveshare_phy_reset, systimer,
    time_driver_irq, Device, StaticDmaResources,
};
use panic_halt as _;
use riscv_rt::entry;
use static_cell::{ConstStaticCell, StaticCell};

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0xE8, 0xF6, 0x0A, 0xE0, 0x93, 0xF5];
const NET_SEED: u64 = 0x5A17_2026_57_E5_5E_55;

const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 0, 50);
const SELF_PREFIX: u8 = 24;
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 0, 1);

const RX_PORT: u16 = 7780;
const TX_PORT: u16 = 7781;
// 2 KB matches the existing echo example. Bigger windows are tempting for
// higher steady-state throughput but the linker cuts it close to the RAM /
// L2-cache boundary at 0x4FF80000. Stay conservative.
const TCP_BUF_SIZE: usize = 2048;
const APP_RX_CHUNK: usize = 1460;
const APP_TX_CHUNK: usize = 1460;

/// App-level RX byte total (read off the RX socket). 32-bit because riscv32imafc
/// has no native 64-bit atomics. Wrap-around at ~4 GB is fine for the 1-second
/// deltas the stat task computes; a 30-second run at line rate is ~375 MB.
static RX_BYTES: AtomicU32 = AtomicU32::new(0);
/// App-level TX byte total (written to the TX socket). Same caveat as RX_BYTES.
static TX_BYTES: AtomicU32 = AtomicU32::new(0);
/// Number of times `dma_recovery_task` actually had to nudge `RX_POLL_DEMAND`.
/// If this stays 0 under multi-MB/s load, the polling recovery loop is
/// dead code under `p4-time-driver-irq` and AIE-recovery can be re-enabled
/// (or the task removed entirely under that feature).
static RBU_RECOVERIES: AtomicU32 = AtomicU32::new(0);

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
#[link_section = ".dma_bss"]
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());
static NET_RESOURCES: ConstStaticCell<StackResources<6>> =
    ConstStaticCell::new(StackResources::<6>::new());
static RX_SOCK_RX_BUF: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static RX_SOCK_TX_BUF: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TX_SOCK_RX_BUF: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TX_SOCK_TX_BUF: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);

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
    // Under `p4-time-driver-irq` the recovery loop is empirically dead code
    // (rbu = 0 over 50 s of sustained 4 Mbps RX + 3 Mbps TX), but we keep
    // it as belt-and-braces against an unmeasured edge case. 50 ms instead
    // of 1 ms drops the wake-up overhead from ~10 M instr/s to ~200 K instr/s
    // (~2.5 % CPU saved at 360 MHz HP CPU) without losing recovery for any
    // realistic stall: a 50 ms RBU window still wakes well before TCP's
    // retransmit timer (≥200 ms) would notice anything.
    loop {
        Timer::after(Duration::from_millis(50)).await;
        let dma_status = unsafe { read_volatile(0x5009_9014 as *const u32) };
        let rps = (dma_status >> 17) & 0b111;
        if rps == 0 || rps == 4 || dma_status & (1 << 7) != 0 {
            unsafe {
                write_volatile(0x5009_9014 as *mut u32, dma_status & 0x0001E7FF);
                write_volatile(0x5009_9008 as *mut u32, 1);
            }
            RBU_RECOVERIES.fetch_add(1, Ordering::Relaxed);
        }
    }
}

async fn finish_emac_bringup(stack: Stack<'static>, uart: &mut Uart0) {
    let t0 = systimer::now_us();
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(100)).await;
    }
    let _ = writeln!(uart, "app: link up after {} ms", (systimer::now_us() - t0) / 1000);

    unsafe {
        write_volatile(0x5009_8004 as *mut u32, 0x80000011);
        let dma_op_mode = 0x5009_9018 as *mut u32;
        write_volatile(dma_op_mode, read_volatile(dma_op_mode) | (1 << 6));
        let mac_cfg = 0x5009_8000 as *mut u32;
        write_volatile(mac_cfg, read_volatile(mac_cfg) & !(1 << 25));
    }
    let _ = writeln!(uart, "app: filter+FUF+ACS-clear applied");
}

#[embassy_executor::task]
async fn rx_sink_task(stack: Stack<'static>) -> ! {
    let mut uart = Uart0;
    finish_emac_bringup(stack, &mut uart).await;
    let _ = writeln!(uart, "app: RX-sink listening on :{}", RX_PORT);

    let rx_buf: &'static mut [u8] = RX_SOCK_RX_BUF.take();
    let tx_buf: &'static mut [u8] = RX_SOCK_TX_BUF.take();
    let mut socket = TcpSocket::new(stack, rx_buf, tx_buf);
    socket.set_timeout(Some(Duration::from_secs(60)));

    let mut buf = [0u8; APP_RX_CHUNK];
    let mut conn = 0u32;
    loop {
        match socket
            .accept(IpListenEndpoint { addr: None, port: RX_PORT })
            .await
        {
            Ok(()) => {
                conn += 1;
                let _ = writeln!(uart, "rx: conn #{} ACCEPTED from {:?}", conn, socket.remote_endpoint());
            }
            Err(e) => {
                let _ = writeln!(uart, "rx: accept error: {:?}", e);
                socket.abort();
                let _ = socket.flush().await;
                continue;
            }
        }

        let mut total = 0u64;
        loop {
            match socket.read(&mut buf).await {
                Ok(0) => {
                    let _ = writeln!(uart, "rx: conn #{} EOF, total = {} bytes", conn, total);
                    break;
                }
                Ok(n) => {
                    total += n as u64;
                    RX_BYTES.fetch_add(n as u32, Ordering::Relaxed);
                }
                Err(e) => {
                    let _ = writeln!(uart, "rx: read error: {:?}", e);
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
async fn tx_source_task(stack: Stack<'static>) -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "app: TX-source will listen on :{}", TX_PORT);

    let rx_buf: &'static mut [u8] = TX_SOCK_RX_BUF.take();
    let tx_buf: &'static mut [u8] = TX_SOCK_TX_BUF.take();
    let mut socket = TcpSocket::new(stack, rx_buf, tx_buf);
    socket.set_timeout(Some(Duration::from_secs(60)));

    // 1460-byte payload pattern (TCP MSS for 1500 MTU minus headers). Fill
    // with a recognisable byte so a Wireshark capture stays interpretable.
    let payload = [0xA5u8; APP_TX_CHUNK];

    let mut conn = 0u32;
    loop {
        match socket
            .accept(IpListenEndpoint { addr: None, port: TX_PORT })
            .await
        {
            Ok(()) => {
                conn += 1;
                let _ = writeln!(uart, "tx: conn #{} ACCEPTED from {:?}", conn, socket.remote_endpoint());
            }
            Err(e) => {
                let _ = writeln!(uart, "tx: accept error: {:?}", e);
                socket.abort();
                let _ = socket.flush().await;
                continue;
            }
        }

        let mut total = 0u64;
        loop {
            match socket.write(&payload).await {
                Ok(0) => {
                    let _ = writeln!(uart, "tx: write returned 0 — peer closed?");
                    break;
                }
                Ok(n) => {
                    total += n as u64;
                    TX_BYTES.fetch_add(n as u32, Ordering::Relaxed);
                }
                Err(e) => {
                    let _ = writeln!(uart, "tx: write error: {:?} (sent {} bytes)", e, total);
                    break;
                }
            }
        }
        let _ = writeln!(uart, "tx: conn #{} done, total = {} bytes", conn, total);
        socket.close();
        let _ = socket.flush().await;
        socket.abort();
        let _ = socket.flush().await;
    }
}

#[embassy_executor::task]
async fn stat_task() -> ! {
    let mut uart = Uart0;
    let mut last_rx_bytes = 0u32;
    let mut last_tx_bytes = 0u32;
    let mut last_rx_frames = 0u32;
    let mut last_tx_frames = 0u32;
    let mut last_irq_rx = 0u32;
    let mut last_irq_tx = 0u32;
    let mut last_rbu = 0u32;
    let mut last_mcycle = mcycle_low();
    let mut last_minstret = minstret_low();
    let mut last_inv_ticks = 0u32;
    let mut last_inv_calls = 0u32;
    let mut last_wb_ticks = 0u32;
    let mut last_wb_calls = 0u32;
    let mut tick = 0u64;

    loop {
        Timer::after(Duration::from_secs(1)).await;
        tick += 1;

        let rx_bytes = RX_BYTES.load(Ordering::Relaxed);
        let tx_bytes = TX_BYTES.load(Ordering::Relaxed);
        let rx_frames = RX_FRAMES.load(Ordering::Relaxed);
        let tx_frames = TX_FRAMES.load(Ordering::Relaxed);
        let irq_rx = time_driver_irq::EMAC_IRQ_RX.load(Ordering::Relaxed);
        let irq_tx = time_driver_irq::EMAC_IRQ_TX.load(Ordering::Relaxed);
        let rbu = RBU_RECOVERIES.load(Ordering::Relaxed);
        let mc = mcycle_low();
        let mi = minstret_low();
        let inv_ticks = CACHE_INV_TICKS.load(Ordering::Relaxed);
        let inv_calls = CACHE_INV_CALLS.load(Ordering::Relaxed);
        let wb_ticks = CACHE_WB_TICKS.load(Ordering::Relaxed);
        let wb_calls = CACHE_WB_CALLS.load(Ordering::Relaxed);

        let d_rx_bytes = rx_bytes.wrapping_sub(last_rx_bytes);
        let d_tx_bytes = tx_bytes.wrapping_sub(last_tx_bytes);
        let d_rx_frames = rx_frames.wrapping_sub(last_rx_frames);
        let d_tx_frames = tx_frames.wrapping_sub(last_tx_frames);
        let d_irq_rx = irq_rx.wrapping_sub(last_irq_rx);
        let d_irq_tx = irq_tx.wrapping_sub(last_irq_tx);
        let d_rbu = rbu.wrapping_sub(last_rbu);
        // mcycle wraps every ~11 s at 40 MHz; one second is fine.
        let d_mcycle = mc.wrapping_sub(last_mcycle);
        let d_minstret = mi.wrapping_sub(last_minstret);
        let d_inv_ticks = inv_ticks.wrapping_sub(last_inv_ticks);
        let d_inv_calls = inv_calls.wrapping_sub(last_inv_calls);
        let d_wb_ticks = wb_ticks.wrapping_sub(last_wb_ticks);
        let d_wb_calls = wb_calls.wrapping_sub(last_wb_calls);

        let rx_kbps = (d_rx_bytes as u64 * 8) / 1000;
        let tx_kbps = (d_tx_bytes as u64 * 8) / 1000;
        // SYSTIMER ticks at 16 MHz — divide by 16 for microseconds.
        let inv_us = d_inv_ticks / 16;
        let wb_us = d_wb_ticks / 16;

        let _ = writeln!(
            uart,
            "  t={}s mins={} inv={}us/{}c wb={}us/{}c rx={} kbps ({} fr) tx={} kbps ({} fr) rbu+={} err={}",
            tick,
            d_minstret,
            inv_us,
            d_inv_calls,
            wb_us,
            d_wb_calls,
            rx_kbps,
            d_rx_frames,
            tx_kbps,
            d_tx_frames,
            d_rbu,
            RX_ERROR_FRAMES_TOTAL.load(Ordering::Relaxed),
        );

        last_rx_bytes = rx_bytes;
        last_tx_bytes = tx_bytes;
        last_rx_frames = rx_frames;
        last_tx_frames = tx_frames;
        last_irq_rx = irq_rx;
        last_irq_tx = irq_tx;
        last_rbu = rbu;
        last_mcycle = mc;
        last_minstret = mi;
        last_inv_ticks = inv_ticks;
        last_inv_calls = inv_calls;
        last_wb_ticks = wb_ticks;
        last_wb_calls = wb_calls;
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== embassy_tcp_stress_irq alive ===");

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

    spawner.spawn(ethernet_task(emac_runner, eth)).unwrap();
    spawner.spawn(net_task(net_runner)).unwrap();
    spawner.spawn(dma_recovery_task()).unwrap();
    spawner.spawn(rx_sink_task(stack)).unwrap();
    spawner.spawn(tx_source_task(stack)).unwrap();
    spawner.spawn(stat_task()).unwrap();
    let _ = writeln!(uart, "init: all tasks spawned (rx:{} tx:{})", RX_PORT, TX_PORT);
}
