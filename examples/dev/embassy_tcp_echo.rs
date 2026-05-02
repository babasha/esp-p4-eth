#![no_std]
#![no_main]

//! TCP echo smoke: same EMAC + embassy-net stack as embassy_static_ping, but
//! with a TCP listener on :7777 that echoes any bytes back. Run from host:
//!
//!     ncat 192.168.0.50 7777     (then type, see echo)
//!     # or PowerShell:
//!     $c = [System.Net.Sockets.TcpClient]::new("192.168.0.50", 7777)
//!     $s = $c.GetStream(); $b = [Text.Encoding]::ASCII.GetBytes("hello`n")
//!     $s.Write($b, 0, $b.Length); $s.Flush()
//!     $buf = New-Object byte[] 64; $n = $s.Read($buf, 0, 64)
//!     [Text.Encoding]::ASCII.GetString($buf, 0, $n)
//!
//! Confirms the TCP socket layer is alive on top of this driver.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::Ordering;

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
    diag::{RX_FRAMES, TX_FRAMES},
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

// LAN topology: P4 plugged into router (192.168.0.0/24), host on Wi-Fi.
const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 0, 50);
const SELF_PREFIX: u8 = 24;
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 0, 1);

const ECHO_PORT: u16 = 7777;
const TCP_BUF_SIZE: usize = 2048;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
#[link_section = ".dma_bss"]
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());
static NET_RESOURCES: ConstStaticCell<StackResources<4>> =
    ConstStaticCell::new(StackResources::<4>::new());
// TCP socket buffers — CPU-side, not DMA. Regular RAM is fine.
static TCP_RX_BUF: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);
static TCP_TX_BUF: ConstStaticCell<[u8; TCP_BUF_SIZE]> = ConstStaticCell::new([0; TCP_BUF_SIZE]);

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

/// Bring the EMAC into a state where TCP traffic can flow: link up wait,
/// promiscuous frame filter, FUF for runt frames, ACS clear. Same set of
/// post-init pokes that embassy_static_ping uses — needed for any traffic,
/// not just ICMP.
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
async fn tcp_echo_task(stack: Stack<'static>) -> ! {
    let mut uart = Uart0;
    finish_emac_bringup(stack, &mut uart).await;

    if let Some(cfg) = stack.config_v4() {
        let _ = writeln!(uart, "app: stack live — {}", cfg.address);
    }
    let _ = writeln!(uart, "app: TCP echo listening on :{}", ECHO_PORT);

    let rx_buf: &'static mut [u8] = TCP_RX_BUF.take();
    let tx_buf: &'static mut [u8] = TCP_TX_BUF.take();
    let mut socket = TcpSocket::new(stack, rx_buf, tx_buf);
    socket.set_timeout(Some(Duration::from_secs(30)));

    let mut conn = 0u32;
    loop {
        let _ = writeln!(uart, "tcp: waiting for client on :{} (conn #{})", ECHO_PORT, conn + 1);
        match socket
            .accept(IpListenEndpoint { addr: None, port: ECHO_PORT })
            .await
        {
            Ok(()) => {
                conn += 1;
                let remote = socket.remote_endpoint();
                let _ = writeln!(uart, "tcp: conn #{} ACCEPTED from {:?}", conn, remote);
            }
            Err(e) => {
                let _ = writeln!(uart, "tcp: accept error: {:?}", e);
                socket.abort();
                let _ = socket.flush().await;
                continue;
            }
        }

        let mut buf = [0u8; 256];
        let mut total = 0u32;
        loop {
            match socket.read(&mut buf).await {
                Ok(0) => {
                    let _ = writeln!(uart, "tcp: conn #{} EOF, total bytes echoed = {}", conn, total);
                    break;
                }
                Ok(n) => {
                    total += n as u32;
                    if let Err(e) = socket.write(&buf[..n]).await {
                        let _ = writeln!(uart, "tcp: write error: {:?}", e);
                        break;
                    }
                }
                Err(e) => {
                    let _ = writeln!(uart, "tcp: read error: {:?}", e);
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
    loop {
        Timer::after(Duration::from_secs(5)).await;
        tick += 1;
        let _ = writeln!(
            uart,
            "  t={}s rx={} tx={}",
            tick * 5,
            RX_FRAMES.load(Ordering::Relaxed),
            TX_FRAMES.load(Ordering::Relaxed),
        );
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== embassy_tcp_echo alive ===");

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
    spawner.spawn(tcp_echo_task(stack)).unwrap();
    spawner.spawn(stat_task()).unwrap();
    let _ = writeln!(uart, "init: all tasks spawned");
}
