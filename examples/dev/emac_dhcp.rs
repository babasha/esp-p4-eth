#![no_std]
#![no_main]

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use embassy_executor::{Executor, Spawner};
use embassy_futures::yield_now;
use embassy_net::{tcp::TcpSocket, Config, Runner as NetRunner, Stack, StackResources};
use embassy_time::{Duration, MockDriver};
use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    ethernet_task, new_from_static_resources as emac_new, release_waveshare_phy_reset, Device,
    StaticDmaResources, BUF_SIZE,
};
use static_cell::{ConstStaticCell, StaticCell};

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
const NET_SEED: u64 = 0x5A17_2026_CAFE_BEEF;
const TCP_ECHO_PORT: u16 = 7;

// `Executor::new()` is not `const`, so EXECUTOR has to stay on StaticCell.
// It is small (~hundreds of bytes), the stack copy is harmless.
static EXECUTOR: StaticCell<Executor> = StaticCell::new();
// All large resources use ConstStaticCell — `T::new()` is `const fn` on each
// of these, so the storage lives in `.bss` and `take()` returns
// `&'static mut T` without ever copying through the stack. The earlier
// `StaticCell::init(StaticDmaResources::new())` formulation built the value
// through a stack temp and overflowed riscv-rt's default stack.
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());
static NET_RESOURCES: ConstStaticCell<StackResources<4>> =
    ConstStaticCell::new(StackResources::<4>::new());
static TCP_RX_BUF: ConstStaticCell<[u8; BUF_SIZE]> = ConstStaticCell::new([0; BUF_SIZE]);
static TCP_TX_BUF: ConstStaticCell<[u8; BUF_SIZE]> = ConstStaticCell::new([0; BUF_SIZE]);

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

    let _ = writeln!(uart, "Waiting for DHCP lease...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        let _ = writeln!(uart, "Got IP: {}", config.address.address());
    }

    let rx_buf = TCP_RX_BUF.take();
    let tx_buf = TCP_TX_BUF.take();
    let mut socket = TcpSocket::new(stack, rx_buf, tx_buf);
    let mut buf = [0u8; 512];

    loop {
        socket
            .accept(TCP_ECHO_PORT)
            .await
            .expect("TCP accept failed");
        let _ = writeln!(uart, "TCP client connected");

        loop {
            let n = socket.read(&mut buf).await.expect("TCP read failed");
            if n == 0 {
                break;
            }

            let mut written = 0;
            while written < n {
                let m = socket
                    .write(&buf[written..n])
                    .await
                    .expect("TCP write failed");
                written += m;
            }

            socket.flush().await.expect("TCP flush failed");
        }

        socket.close();
        let _ = socket.flush().await;
        let _ = writeln!(uart, "TCP client disconnected");
    }
}

#[entry]
fn main() -> ! {
    release_waveshare_phy_reset();
    // PHY needs ~10 ms after reset deassert to start the 50 MHz oscillator.
    for _ in 0..2_000_000 {
        spin_loop();
    }

    MockDriver::get().reset();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(init);
}

fn init(spawner: Spawner) {
    let dma_resources = DMA_RESOURCES.take();
    let (device, emac_runner, eth) = emac_new(MAC_ADDR, dma_resources);

    let config = Config::dhcpv4(Default::default());
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
