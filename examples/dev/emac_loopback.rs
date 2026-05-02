#![no_std]
#![no_main]

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    regs, release_waveshare_phy_reset, Dma, DmaError, Ethernet, StaticDmaResources, BUF_SIZE,
};
use static_cell::ConstStaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
const LOOPBACK_POLL_SPINS: usize = 200_000;

static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());

struct Uart0;

#[derive(Clone, Copy)]
enum Pattern {
    Zeroes,
    Ones,
    Alternating,
    Sequential,
}

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

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "EMAC loopback smoke test");

    release_waveshare_phy_reset();
    for _ in 0..3_000_000 {
        spin_loop();
    }

    match Dma::dma_reset() {
        Ok(()) => {
            let _ = writeln!(uart, "DMA reset OK");
        }
        Err(DmaError::ResetTimeout) => {
            panic!("DMA reset timed out");
        }
    }

    Dma::dma_init();

    let dma_resources = DMA_RESOURCES.take();
    let ethernet = Ethernet::new_from_static_resources(MAC_ADDR, dma_resources);
    let mut ethernet = ethernet;

    ethernet.set_mac_addr(MAC_ADDR);
    enable_mac_loopback();
    start_dma_engines();

    let mut frame = [0u8; BUF_SIZE];
    let mut rx_frame = [0u8; BUF_SIZE];

    run_case(
        &mut uart,
        &mut ethernet,
        &mut frame,
        &mut rx_frame,
        64,
        Pattern::Zeroes,
        0,
    );
    run_case(
        &mut uart,
        &mut ethernet,
        &mut frame,
        &mut rx_frame,
        1514,
        Pattern::Ones,
        1,
    );

    for seq in 0..100u8 {
        let size = 64 + ((usize::from(seq) * 37) % (1514 - 64 + 1));
        let pattern = match seq & 0x03 {
            0 => Pattern::Zeroes,
            1 => Pattern::Ones,
            2 => Pattern::Alternating,
            _ => Pattern::Sequential,
        };

        run_case(
            &mut uart,
            &mut ethernet,
            &mut frame,
            &mut rx_frame,
            size,
            pattern,
            seq.wrapping_add(2),
        );
    }

    let _ = writeln!(uart, "Loopback OK");

    loop {
        spin_loop();
    }
}

fn run_case(
    uart: &mut Uart0,
    ethernet: &mut Ethernet<'_>,
    frame: &mut [u8; BUF_SIZE],
    rx_frame: &mut [u8; BUF_SIZE],
    len: usize,
    pattern: Pattern,
    sequence: u8,
) {
    fill_frame(frame, len, pattern, sequence);
    let tx = &frame[..len];

    ethernet.transmit(tx).expect("TX enqueue failed");
    Dma::demand_rx_poll();

    let rx_len = wait_for_frame(ethernet, rx_frame);
    assert_eq!(len, rx_len);
    assert_eq!(tx, &rx_frame[..rx_len]);

    let _ = writeln!(uart, "loopback ok: len={len} seq={sequence}");
}

fn wait_for_frame(ethernet: &mut Ethernet<'_>, rx_frame: &mut [u8; BUF_SIZE]) -> usize {
    for _ in 0..LOOPBACK_POLL_SPINS {
        if let Some((len, data)) = ethernet.receive() {
            rx_frame[..len].copy_from_slice(&data[..len]);
            ethernet.pop_rx();
            return len;
        }

        spin_loop();
    }

    panic!("loopback receive timeout");
}

fn fill_frame(frame: &mut [u8; BUF_SIZE], len: usize, pattern: Pattern, sequence: u8) {
    assert!(len <= 1514);
    assert!(len >= 64);

    for (i, byte) in frame[..len].iter_mut().enumerate() {
        *byte = match pattern {
            Pattern::Zeroes => 0x00,
            Pattern::Ones => 0xFF,
            Pattern::Alternating => {
                if i & 1 == 0 {
                    0x55
                } else {
                    0xAA
                }
            }
            Pattern::Sequential => i as u8,
        };
    }

    frame[0] = 0x02;
    frame[1] = 0x11;
    frame[2] = 0x22;
    frame[3] = 0x33;
    frame[4] = 0x44;
    frame[5] = 0x55;

    frame[6..12].copy_from_slice(&MAC_ADDR);
    frame[12] = 0x08;
    frame[13] = 0x00;
    frame[14] = sequence;
}

fn enable_mac_loopback() {
    let maccfg = regs::read(regs::mac::MACCFG);
    regs::write(
        regs::mac::MACCFG,
        maccfg | regs::bits::maccfg::LM | regs::bits::maccfg::TE | regs::bits::maccfg::RE,
    );
}

fn start_dma_engines() {
    let op_mode = regs::read(regs::dma::OP_MODE);
    regs::write(
        regs::dma::OP_MODE,
        op_mode | regs::bits::dmaopmode::ST | regs::bits::dmaopmode::SR,
    );
}

fn txfifo_count() -> u32 {
    // SAFETY: `UART_STATUS_REG` is the fixed UART0 status MMIO register. Volatile read is
    // required because the value changes independently from the CPU.
    let status = unsafe { read_volatile(UART_STATUS_REG) };
    (status & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
}
