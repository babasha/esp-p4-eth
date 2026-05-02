#![no_std]
#![no_main]

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use embassy_executor::{Executor, Spawner};
use embassy_futures::yield_now;
use embassy_net::{
    Config, Ipv4Address, Ipv4Cidr, Runner as NetRunner, Stack, StackResources, StaticConfigV4,
};
use embassy_time::{Duration, MockDriver};
use heapless::Vec;
use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    ethernet_task, new_from_static_resources as emac_new, release_waveshare_phy_reset, Device,
    StaticDmaResources,
};
use static_cell::{ConstStaticCell, StaticCell};

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x12];
const NET_SEED: u64 = 0x571A_7100_2026_0001;
const STATIC_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 77);
const STATIC_PREFIX_LEN: u8 = 24;
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 1, 1);
const DNS_SERVER: Ipv4Address = Ipv4Address::new(1, 1, 1, 1);

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
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

        // SAFETY: `UART_FIFO_REG` is the fixed UART0 TX FIFO MMIO register. Volatile write is
        // required so the compiler preserves the hardware side effect.
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

#[embassy_executor::task]
async fn net_task(mut runner: NetRunner<'static, Device<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn mock_time_task() -> ! {
    let driver = MockDriver::get();
    loop {
        driver.advance(Duration::from_millis(1));
        yield_now().await;
    }
}

#[embassy_executor::task]
async fn app_task(stack: Stack<'static>) -> ! {
    let mut uart = Uart0;

    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let _ = writeln!(uart, "Static IP: {}", config.address.address());
        if let Some(gateway) = config.gateway {
            let _ = writeln!(uart, "Gateway: {gateway}");
        }
    }

    let _ = writeln!(uart, "ICMP echo responder ready");

    loop {
        yield_now().await;
    }
}

#[entry]
fn main() -> ! {
    release_waveshare_phy_reset();
    for _ in 0..3_000_000 {
        spin_loop();
    }

    MockDriver::get().reset();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(init);
}

fn init(spawner: Spawner) {
    let dma_resources = DMA_RESOURCES.take();
    let (device, emac_runner, eth) = emac_new(MAC_ADDR, dma_resources);

    let mut dns_servers = Vec::new();
    dns_servers.push(DNS_SERVER).unwrap();

    let config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(STATIC_IP, STATIC_PREFIX_LEN),
        gateway: Some(GATEWAY),
        dns_servers,
    });
    let resources = NET_RESOURCES.take();
    let (stack, net_runner) = embassy_net::new(device, config, resources, NET_SEED);

    spawner.spawn(mock_time_task()).unwrap();
    spawner.spawn(ethernet_task(emac_runner, eth)).unwrap();
    spawner.spawn(net_task(net_runner)).unwrap();
    spawner.spawn(app_task(stack)).unwrap();
}

fn txfifo_count() -> u32 {
    // SAFETY: `UART_STATUS_REG` is the fixed UART0 status MMIO register. Volatile read is
    // required because the value changes independently from the CPU.
    let status = unsafe { read_volatile(UART_STATUS_REG) };
    (status & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
}
