#![no_std]
#![no_main]

//! UDP echo smoke: same EMAC + embassy-net stack as embassy_tcp_echo, but with
//! a UDP listener on :7778 that echoes the datagram back to the sender.
//! Tests connectionless socket layer + ARP cache for first-packet path.
//!
//! Host test (WSL):
//!     python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); \
//!         s.sendto(b'hi', ('192.168.0.50', 7778)); print(s.recvfrom(64))"

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::Ordering;

use embassy_executor::{Executor, Spawner};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{
    Config, IpListenEndpoint, Ipv4Address, Ipv4Cidr, Runner as NetRunner, Stack, StackResources,
    StaticConfigV4,
};
use embassy_time::{Duration, Timer};
use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    diag::{
        RX_ERROR_FRAMES_TOTAL, RX_FRAMES, RX_LARGE_FRAMES, RX_LAST_FRAME_LEN, RX_LAST_LARGE_FRAME,
        RX_LAST_LARGE_FRAME_LEN, RX_LAST_RDES0, RX_OVERSIZED_FRAMES_TOTAL, RX_RUNT_FRAMES_TOTAL,
        TX_FRAMES,
    },
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

const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 0, 50);
const SELF_PREFIX: u8 = 24;
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 0, 1);

const ECHO_PORT: u16 = 7778;
const UDP_BUF_SIZE: usize = 4096;
const UDP_META_COUNT: usize = 4;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
#[link_section = ".dma_bss"]
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());
static NET_RESOURCES: ConstStaticCell<StackResources<4>> =
    ConstStaticCell::new(StackResources::<4>::new());
static UDP_RX_BUF: ConstStaticCell<[u8; UDP_BUF_SIZE]> = ConstStaticCell::new([0; UDP_BUF_SIZE]);
static UDP_TX_BUF: ConstStaticCell<[u8; UDP_BUF_SIZE]> = ConstStaticCell::new([0; UDP_BUF_SIZE]);
static UDP_RX_META: ConstStaticCell<[PacketMetadata; UDP_META_COUNT]> =
    ConstStaticCell::new([PacketMetadata::EMPTY; UDP_META_COUNT]);
static UDP_TX_META: ConstStaticCell<[PacketMetadata; UDP_META_COUNT]> =
    ConstStaticCell::new([PacketMetadata::EMPTY; UDP_META_COUNT]);

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

async fn finish_emac_bringup(stack: Stack<'static>, uart: &mut Uart0) {
    let t0 = systimer::now_us();
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(100)).await;
    }
    let _ = writeln!(uart, "app: link up after {} ms", (systimer::now_us() - t0) / 1000);

    unsafe {
        write_volatile(0x5009_8004 as *mut u32, 0x80000011);
        let mac_cfg = 0x5009_8000 as *mut u32;
        write_volatile(mac_cfg, read_volatile(mac_cfg) & !(1 << 25));
    }
    let _ = writeln!(uart, "app: filter+ACS-clear applied (FUF+!RSF set in driver)");

    unsafe {
        let _ = writeln!(
            uart,
            "regs: DMA_OP_MODE=0x{:08X} MAC_CONFIG=0x{:08X} MAC_FRAME_FILTER=0x{:08X}",
            read_volatile(0x5009_9018 as *const u32),
            read_volatile(0x5009_8000 as *const u32),
            read_volatile(0x5009_8004 as *const u32),
        );
    }
}

#[embassy_executor::task]
async fn udp_echo_task(stack: Stack<'static>) -> ! {
    let mut uart = Uart0;
    finish_emac_bringup(stack, &mut uart).await;

    if let Some(cfg) = stack.config_v4() {
        let _ = writeln!(uart, "app: stack live — {}", cfg.address);
    }

    let rx_buf: &'static mut [u8] = UDP_RX_BUF.take();
    let tx_buf: &'static mut [u8] = UDP_TX_BUF.take();
    let rx_meta: &'static mut [PacketMetadata] = UDP_RX_META.take();
    let tx_meta: &'static mut [PacketMetadata] = UDP_TX_META.take();
    let mut socket = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);

    if let Err(e) = socket.bind(IpListenEndpoint { addr: None, port: ECHO_PORT }) {
        let _ = writeln!(uart, "udp: BIND ERROR {:?}", e);
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
    let _ = writeln!(uart, "udp: bound on :{}", ECHO_PORT);

    let mut buf = [0u8; 2048];
    let mut pkt = 0u32;
    loop {
        match socket.recv_from(&mut buf).await {
            Ok((n, meta)) => {
                pkt += 1;
                let _ = writeln!(
                    uart,
                    "udp: pkt #{} got {} bytes from {:?}",
                    pkt, n, meta.endpoint,
                );
                if let Err(e) = socket.send_to(&buf[..n], meta.endpoint).await {
                    let _ = writeln!(uart, "udp: send_to error: {:?}", e);
                }
            }
            Err(e) => {
                let _ = writeln!(uart, "udp: recv error: {:?}", e);
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn stat_task() -> ! {
    let mut uart = Uart0;
    let mut tick = 0u64;
    loop {
        Timer::after(Duration::from_secs(5)).await;
        tick += 1;
        let _ = writeln!(
            uart,
            "  t={}s rx={} tx={} large={} last_len={} last_rdes0=0x{:08X} err={} runt={} oversized={}",
            tick * 5,
            RX_FRAMES.load(Ordering::Relaxed),
            TX_FRAMES.load(Ordering::Relaxed),
            RX_LARGE_FRAMES.load(Ordering::Relaxed),
            RX_LAST_FRAME_LEN.load(Ordering::Relaxed),
            RX_LAST_RDES0.load(Ordering::Relaxed),
            RX_ERROR_FRAMES_TOTAL.load(Ordering::Relaxed),
            RX_RUNT_FRAMES_TOTAL.load(Ordering::Relaxed),
            RX_OVERSIZED_FRAMES_TOTAL.load(Ordering::Relaxed),
        );
        let llen = RX_LAST_LARGE_FRAME_LEN.load(Ordering::Relaxed);
        if llen > 0 {
            let _ = writeln!(
                uart,
                "    last_large_len={} bytes 0..64: {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
                llen,
                RX_LAST_LARGE_FRAME[0].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[1].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[2].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[3].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[4].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[5].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[6].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[7].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[8].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[9].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[10].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[11].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[12].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[13].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[14].load(Ordering::Relaxed),
                RX_LAST_LARGE_FRAME[15].load(Ordering::Relaxed),
            );
        }
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== embassy_udp_echo alive ===");

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
    spawner.spawn(udp_echo_task(stack)).unwrap();
    spawner.spawn(stat_task()).unwrap();
    let _ = writeln!(uart, "init: all tasks spawned");
}
