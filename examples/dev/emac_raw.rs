#![no_std]
#![no_main]

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    mdio_read, release_waveshare_phy_reset, Dma, Ethernet, StaticDmaResources, ANAR, ANLPAR,
    BMCR, BMSR, BUF_SIZE, PHYIDR1, PHYIDR2,
};
use static_cell::ConstStaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
// Peer-to-peer link with PC. Windows has APIPA 169.254.144.33/16 on the Realtek port.
const SENDER_IP: [u8; 4] = [169, 254, 144, 50];
const TARGET_IP: [u8; 4] = [169, 254, 144, 33];
const BROADCAST_MAC: [u8; 6] = [0xFF; 6];

const LINK_POLL_SPINS: usize = 200_000;
const RX_POLL_SPINS: usize = 200_000;

static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());

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

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "EMAC raw PHY smoke test");

    release_waveshare_phy_reset();
    delay_cycles(3_000_000);

    let dma_resources = DMA_RESOURCES.take();
    let ethernet = Ethernet::new_from_static_resources(MAC_ADDR, dma_resources);
    let mut ethernet = ethernet;

    // RETRY hypothesis #2 with fixed pin routing: now OUT_SEL=256, peripheral-direct path is
    // active. start() may now have a real effect.
    let _ = writeln!(uart, "calling ethernet.start()...");
    match ethernet.start() {
        Ok(()) => { let _ = writeln!(uart, "  start() OK"); }
        Err(e) => { let _ = writeln!(uart, "  start() ERR: {:?}", e); }
    }
    delay_cycles(2_000_000);

    // GPIO matrix / IO_MUX / GPIO_ENABLE diagnostic dump for MDC/MDIO/REF_CLK.
    unsafe {
        const GPIO_BASE: usize = 0x500E_0000;
        const IO_MUX_BASE: usize = 0x500E_1000;
        let rd = |a: usize| read_volatile(a as *const u32);
        let _ = writeln!(uart, "--- routing dump ---");
        let _ = writeln!(uart, "IO_MUX_GPIO31  = 0x{:08X}", rd(IO_MUX_BASE + 4 + 31 * 4));
        let _ = writeln!(uart, "IO_MUX_GPIO52  = 0x{:08X}", rd(IO_MUX_BASE + 4 + 52 * 4));
        let _ = writeln!(uart, "IO_MUX_GPIO50  = 0x{:08X}", rd(IO_MUX_BASE + 4 + 50 * 4));
        let _ = writeln!(uart, "GPIO31_OUT_CFG = 0x{:08X}", rd(GPIO_BASE + 0x554 + 31 * 4));
        let _ = writeln!(uart, "GPIO52_OUT_CFG = 0x{:08X}", rd(GPIO_BASE + 0x554 + 52 * 4));
        let _ = writeln!(uart, "GPIO_FUNC107_IN= 0x{:08X}", rd(GPIO_BASE + 0x154 + 107 * 4));
        let _ = writeln!(uart, "GPIO_FUNC108_IN= 0x{:08X}", rd(GPIO_BASE + 0x154 + 108 * 4));
        let _ = writeln!(uart, "GPIO_FUNC109_IN= 0x{:08X}", rd(GPIO_BASE + 0x154 + 109 * 4));
        let en0 = rd(GPIO_BASE + 0x20);
        let en1 = rd(GPIO_BASE + 0x2C);
        let _ = writeln!(uart, "GPIO_ENABLE0   = 0x{:08X}  (bit31 MDC = {})", en0, (en0 >> 31) & 1);
        let _ = writeln!(uart, "GPIO_ENABLE1   = 0x{:08X}  (bit20 MDIO52 = {})", en1, (en1 >> (52 - 32)) & 1);
        // GPIO_OUT (level register) — bit per pin. Pin 51 RESET should be HIGH for IP101 to run.
        let out0 = rd(GPIO_BASE + 0x04);
        let out1 = rd(GPIO_BASE + 0x10);
        let _ = writeln!(uart, "GPIO_OUT0      = 0x{:08X}", out0);
        let _ = writeln!(uart, "GPIO_OUT1      = 0x{:08X}  (bit19 RESET51 level = {})", out1, (out1 >> (51 - 32)) & 1);
        // GPIO_IN — what the pin actually reads (after pad mux). For pin 50 (REF_CLK) high
        // bit means the clock is currently high; toggling = clock running.
        let in0 = rd(GPIO_BASE + 0x3C);
        let in1 = rd(GPIO_BASE + 0x44);
        let _ = writeln!(uart, "GPIO_IN0       = 0x{:08X}", in0);
        let _ = writeln!(uart, "GPIO_IN1       = 0x{:08X}  (bit18 REFCLK50 = {})", in1, (in1 >> (50 - 32)) & 1);
        // Sample REFCLK50 (GPIO50, bit18 in IN1) many times. If a 50 MHz clock is running,
        // we should see a mix of 0 and 1. If it's stuck high or low — clock is dead.
        let mut zeros = 0u32;
        let mut ones = 0u32;
        for _ in 0..200 {
            let v = rd(GPIO_BASE + 0x44);
            if (v >> 18) & 1 == 0 { zeros += 1; } else { ones += 1; }
        }
        let _ = writeln!(uart, "REFCLK50 sample: zeros={} ones={} (over 200 reads)", zeros, ones);
        // Same for MDC pin (GPIO31, bit31 in IN0). MDC only pulses during MDIO transactions
        // (and we're idle right now), so it's expected to be near-constant. But if we see
        // ones or transitions, then peripheral OE works and pin can drive.
        let mut mdc_zeros = 0u32;
        let mut mdc_ones = 0u32;
        for _ in 0..200 {
            let v = rd(GPIO_BASE + 0x3C);
            if (v >> 31) & 1 == 0 { mdc_zeros += 1; } else { mdc_ones += 1; }
        }
        let _ = writeln!(uart, "MDC31 sample: zeros={} ones={} (idle, expect mostly low)", mdc_zeros, mdc_ones);
        // EMAC MII_ADDR — CSR_CLOCK_RANGE field is bits[5:2]; show full reg.
        // EMAC_BASE on P4 = 0x5009_8000, MII_ADDR offset = 0x10.
        let mii_addr = rd(0x5009_8010);
        let _ = writeln!(uart, "EMAC MII_ADDR  = 0x{:08X}  (csr_clk_range = 0b{:04b})",
            mii_addr, (mii_addr >> 2) & 0xF);
    }

    // Drive ONE explicit mdio_read for addr 0 / BMSR and dump the post-state of MII_ADDR
    // and MII_DATA. This tells us whether the transaction actually completes (MII_BUSY
    // clears) and what the data register reads back. If BUSY stays set => MDC clock or
    // peripheral state issue. If BUSY=0 but data=0 => bus driven low (EMAC or floating).
    {
        let v = mdio_read(0, BMSR).unwrap_or(0xDEAD);
        let mii_addr_after = unsafe { read_volatile(0x5009_8010 as *const u32) };
        let mii_data_after = unsafe { read_volatile(0x5009_8014 as *const u32) };
        let _ = writeln!(uart, "single read addr=0 reg=BMSR -> 0x{:04X}", v);
        let _ = writeln!(uart, "  MII_ADDR after = 0x{:08X}  (BUSY={})", mii_addr_after, mii_addr_after & 1);
        let _ = writeln!(uart, "  MII_DATA after = 0x{:08X}", mii_data_after);
    }

    // MDIO scan to confirm PHY address + bus health.
    let mut found = 0xFFu8;
    for addr in 0u8..32 {
        let v = mdio_read(addr, BMSR).unwrap_or(0xDEAD);
        let live = v != 0x0000 && v != 0xFFFF && v != 0xDEAD;
        if live {
            let _ = writeln!(uart, "  scan addr={:>2} BMSR=0x{:04X} <- live", addr, v);
            if found == 0xFF { found = addr; }
        }
    }
    if found == 0xFF {
        let _ = writeln!(uart, "  scan: NO live PHY addresses found");
        found = 0;
    }
    let id1 = mdio_read(found, PHYIDR1).unwrap_or(0xDEAD);
    let id2 = mdio_read(found, PHYIDR2).unwrap_or(0xDEAD);
    let bmcr = mdio_read(found, BMCR).unwrap_or(0xDEAD);
    let bmsr = mdio_read(found, BMSR).unwrap_or(0xDEAD);
    let anar = mdio_read(found, ANAR).unwrap_or(0xDEAD);
    let anlpar = mdio_read(found, ANLPAR).unwrap_or(0xDEAD);
    let _ = writeln!(uart, "PHY@{} regs: ID1=0x{:04X} ID2=0x{:04X} BMCR=0x{:04X} BMSR=0x{:04X} ANAR=0x{:04X} ANLPAR=0x{:04X}",
        found, id1, id2, bmcr, bmsr, anar, anlpar);
    let _ = writeln!(uart, "BMSR decoded: link={} an_done={}", (bmsr >> 2) & 1, (bmsr >> 5) & 1);

    let _ = writeln!(uart, "Waiting for link up (poll BMSR)...");
    let mut link_up = false;
    let mut last_bmsr = bmsr;
    for iter in 0..60u32 {
        let v = mdio_read(found, BMSR).unwrap_or(0xDEAD);
        if v != last_bmsr {
            let _ = writeln!(uart, "[{iter}] BMSR 0x{:04X} -> 0x{:04X} link={} an_done={}",
                last_bmsr, v, (v >> 2) & 1, (v >> 5) & 1);
            last_bmsr = v;
        }
        if v & 0x0004 != 0 {
            link_up = true;
            let _ = writeln!(uart, "[{iter}] LINK UP", );
            break;
        }
        delay_cycles(LINK_POLL_SPINS);
    }
    if !link_up {
        let _ = writeln!(uart, "link still down after 60 polls (last BMSR=0x{:04X}); starting MAC anyway", last_bmsr);
    }

    let _ = writeln!(uart, "ethernet.start()...");
    ethernet.start().expect("failed to start EMAC");
    let _ = writeln!(uart, "phy_negotiate()...");
    let (speed, duplex) = ethernet.phy_negotiate().expect("PHY negotiation failed");
    log_link_mode(&mut uart, speed, duplex);

    let mut arp_request = [0u8; 64];
    let arp_len = build_arp_request(&mut arp_request, MAC_ADDR, SENDER_IP, TARGET_IP);
    ethernet
        .transmit(&arp_request[..arp_len])
        .expect("failed to send ARP request");
    let _ = writeln!(uart, "ARP request sent");

    let mut rx_frame = [0u8; BUF_SIZE];
    let mut echo_frame = [0u8; BUF_SIZE];

    loop {
        Dma::demand_rx_poll();

        if let Some(len) = wait_for_frame(&mut ethernet, &mut rx_frame) {
            log_frame(&mut uart, "rx", &rx_frame[..len]);

            if len >= 14 {
                let echo_len = build_echo_frame(&mut echo_frame, &rx_frame[..len], MAC_ADDR);
                ethernet
                    .transmit(&echo_frame[..echo_len])
                    .expect("failed to echo frame");
                let _ = writeln!(uart, "echo sent: len={echo_len}");
            }
        }
    }
}

fn wait_for_frame(ethernet: &mut Ethernet<'_>, rx_frame: &mut [u8; BUF_SIZE]) -> Option<usize> {
    for _ in 0..RX_POLL_SPINS {
        if let Some((len, data)) = ethernet.receive() {
            rx_frame[..len].copy_from_slice(&data[..len]);
            ethernet.pop_rx();
            return Some(len);
        }

        spin_loop();
    }

    None
}

fn build_arp_request(
    frame: &mut [u8; 64],
    source_mac: [u8; 6],
    sender_ip: [u8; 4],
    target_ip: [u8; 4],
) -> usize {
    frame.fill(0);

    frame[0..6].copy_from_slice(&BROADCAST_MAC);
    frame[6..12].copy_from_slice(&source_mac);
    frame[12..14].copy_from_slice(&0x0806u16.to_be_bytes());

    frame[14..16].copy_from_slice(&0x0001u16.to_be_bytes());
    frame[16..18].copy_from_slice(&0x0800u16.to_be_bytes());
    frame[18] = 6;
    frame[19] = 4;
    frame[20..22].copy_from_slice(&0x0001u16.to_be_bytes());
    frame[22..28].copy_from_slice(&source_mac);
    frame[28..32].copy_from_slice(&sender_ip);
    frame[32..38].fill(0);
    frame[38..42].copy_from_slice(&target_ip);

    64
}

fn build_echo_frame(out: &mut [u8; BUF_SIZE], input: &[u8], source_mac: [u8; 6]) -> usize {
    let len = input.len();
    out[..len].copy_from_slice(input);
    out[0..6].copy_from_slice(&input[6..12]);
    out[6..12].copy_from_slice(&source_mac);
    len
}

fn log_link_mode(uart: &mut Uart0, speed: esp_p4_eth::Speed, duplex: esp_p4_eth::Duplex) {
    let speed_str = match speed {
        esp_p4_eth::Speed::Mbps10 => "10M",
        esp_p4_eth::Speed::Mbps100 => "100M",
    };
    let duplex_str = match duplex {
        esp_p4_eth::Duplex::Half => "half",
        esp_p4_eth::Duplex::Full => "full",
    };

    let _ = writeln!(uart, "link up: speed={speed_str} duplex={duplex_str}");
}

fn log_frame(uart: &mut Uart0, prefix: &str, frame: &[u8]) {
    if frame.len() < 14 {
        let _ = writeln!(uart, "{prefix}: short frame len={}", frame.len());
        return;
    }

    let dst = &frame[0..6];
    let src = &frame[6..12];
    let ether_type = u16::from_be_bytes([frame[12], frame[13]]);

    let _ = writeln!(
        uart,
        "{prefix}: len={} dst={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} src={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ethertype=0x{:04x}",
        frame.len(),
        dst[0], dst[1], dst[2], dst[3], dst[4], dst[5],
        src[0], src[1], src[2], src[3], src[4], src[5],
        ether_type
    );
}

fn delay_cycles(spins: usize) {
    for _ in 0..spins {
        spin_loop();
    }
}

fn txfifo_count() -> u32 {
    // SAFETY: `UART_STATUS_REG` is the fixed UART0 status MMIO register. Volatile read is
    // required because the value changes independently from the CPU.
    let status = unsafe { read_volatile(UART_STATUS_REG) };
    (status & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
}
