#![no_std]
#![no_main]

//! Minimal MDIO bring-up test.
//!
//! Bypasses `Ethernet::new_from_static_resources` (known to mis-behave due to
//! large stack frames) by calling `Ethernet::try_new` directly with a static
//! DMA resource block split into the four `&mut` slices it expects. After
//! init, scans every Clause 22 PHY address and dumps the standard register
//! block for the first live PHY. If our clock-tree fixes (no
//! `PAD_EMAC_REF_CLK_EN`, no `HP_PAD_EMAC_TX/RX_CLK_EN`, no
//! `FORCE_NORST_EMAC`) actually work, the IP101GRI at addr=1 should answer
//! with non-zero BMSR/PHYIDR.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;
use esp_p4_eth::{
    diag::CPU_DESC0_SNAPSHOT, mdio_read, regs, release_waveshare_phy_reset, Dma, Ethernet,
    StaticDmaResources, ANAR, ANLPAR, BMCR, BMSR, PHYIDR1, PHYIDR2,
};
use static_cell::ConstStaticCell;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

const MAC_ADDR: [u8; 6] = [0xE8, 0xF6, 0x0A, 0xE0, 0x93, 0xF5];

// `ConstStaticCell` builds the value in `.bss` at compile time so we never copy
// the ~12 KB `StaticDmaResources` struct through a stack frame — that copy is
// what hangs `StaticCell::init(StaticDmaResources::new())` on this board.
static DMA_RESOURCES: ConstStaticCell<StaticDmaResources> =
    ConstStaticCell::new(StaticDmaResources::new());

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

fn delay_cycles(spins: usize) {
    for _ in 0..spins {
        spin_loop();
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\r\n=== mdio_test alive ===");

    release_waveshare_phy_reset();
    delay_cycles(3_000_000);
    let _ = writeln!(uart, "phy reset released");

    let resources = DMA_RESOURCES.take();
    let (tx_desc, rx_desc, tx_buf, rx_buf) = resources.split();
    let _ = writeln!(uart, "DMA resources allocated");

    let mut ethernet = match Ethernet::try_new(MAC_ADDR, tx_desc, rx_desc, tx_buf, rx_buf) {
        Ok(eth) => {
            let _ = writeln!(uart, "Ethernet::try_new OK");
            eth
        }
        Err(e) => {
            let _ = writeln!(uart, "Ethernet::try_new ERR: {:?}", e);
            loop {
                spin_loop();
            }
        }
    };

    // Some chips need MAC enabled for MDIO transactions to gate properly.
    match ethernet.start() {
        Ok(()) => {
            let _ = writeln!(uart, "ethernet.start() OK");
        }
        Err(e) => {
            let _ = writeln!(uart, "ethernet.start() ERR: {:?}", e);
        }
    }

    delay_cycles(2_000_000);

    // Control test: dump descriptor[0] state both via memory (after invalidate)
    // and via cached read, plus the snapshot of what CPU just wrote in reset().
    // If THIS run shows non-zero memory, then embassy_dhcp's setup is somehow
    // breaking writeback. If it ALSO shows zeros, the writeback path is broken
    // for both, and we have to look elsewhere for why mdio_test "works".
    {
        let base = regs::read(regs::dma::RX_DESC_LIST);
        let mem = unsafe { Dma::peek_rdes(base) };
        let cached = unsafe { Dma::peek_rdes_cached(base) };
        use core::sync::atomic::Ordering;
        let _ = writeln!(uart, "--- DESCRIPTOR[0] PEEK CONTROL ---");
        let _ = writeln!(uart, "  RX_DESC_LIST register = 0x{:08X}", base);
        let _ = writeln!(
            uart,
            "  CPU snapshot   : rdes0=0x{:08X} rdes1=0x{:08X} buf=0x{:08X} next=0x{:08X}",
            CPU_DESC0_SNAPSHOT[0].load(Ordering::Relaxed),
            CPU_DESC0_SNAPSHOT[1].load(Ordering::Relaxed),
            CPU_DESC0_SNAPSHOT[2].load(Ordering::Relaxed),
            CPU_DESC0_SNAPSHOT[3].load(Ordering::Relaxed),
        );
        let _ = writeln!(
            uart,
            "  cached (no inv): rdes0=0x{:08X} rdes1=0x{:08X} buf=0x{:08X} next=0x{:08X}",
            cached[0], cached[1], cached[2], cached[3],
        );
        let _ = writeln!(
            uart,
            "  mem (after inv): rdes0=0x{:08X} rdes1=0x{:08X} buf=0x{:08X} next=0x{:08X}",
            mem[0], mem[1], mem[2], mem[3],
        );
    }

    // Full post-init register dump before we touch MDIO. This is the side-by-
    // side reference for the IDF baseline state captured after Link Up:
    //   IDF: MAC_CONFIG=0xCC0C MAC_FRAME_FILTER=0x10 MII_ADDR=0x0940
    //        DMA_BUS_MODE=0x06022080 DMA_OP_MODE=0x2006 DMA_STATUS=0x00660000
    //        PERI_CLK_CTRL00=0x2800D05D PERI_CLK_CTRL01=0x00000601
    //        LP_HP_CLK_CTRL=0x1FFF9FFD SYS_GMAC_CTRL0=0x000000D0
    let _ = writeln!(uart, "--- post-init register dump ---");
    let r = |a: usize| unsafe { read_volatile(a as *const u32) };
    // GPIO routing for MDC/MDIO/REF_CLK — must match IDF baseline exactly.
    // IDF: IO_MUX_GPIO31=0x1802, IO_MUX_GPIO52=0x1A02, IO_MUX_GPIO50=0x3A02,
    //      GPIO31_OUT_CFG=0x100, GPIO52_OUT_CFG=0x100,
    //      GPIO_FUNC107_IN_SEL=0x3E (unrouted), GPIO_FUNC108_IN_SEL=0xB4
    //      (= matrix bit 0x80 | pin 52), GPIO_FUNC109_IN_SEL=0x3E (unrouted),
    //      GPIO_ENABLE0=0x80000000 (bit 31 = MDC OE),
    //      GPIO_ENABLE1=0x00180000 (bit 19 = RESET51 OE, bit 20 = MDIO52 OE).
    let _ = writeln!(uart, "  IO_MUX_GPIO31    = 0x{:08X}", r(0x500E_1004 + 31 * 4));
    let _ = writeln!(uart, "  IO_MUX_GPIO52    = 0x{:08X}", r(0x500E_1004 + 52 * 4));
    let _ = writeln!(uart, "  IO_MUX_GPIO50    = 0x{:08X}", r(0x500E_1004 + 50 * 4));
    // Real P4 GPIO matrix base: OUT_SEL = 0x558, IN_SEL = 0x158.
    let _ = writeln!(uart, "  GPIO31_OUT_CFG   = 0x{:08X}", r(0x500E_0558 + 31 * 4));
    let _ = writeln!(uart, "  GPIO52_OUT_CFG   = 0x{:08X}", r(0x500E_0558 + 52 * 4));
    let _ = writeln!(uart, "  FUNC107_IN_SEL   = 0x{:08X}", r(0x500E_0158 + 107 * 4));
    let _ = writeln!(uart, "  FUNC108_IN_SEL   = 0x{:08X}", r(0x500E_0158 + 108 * 4));
    let _ = writeln!(uart, "  FUNC109_IN_SEL   = 0x{:08X}", r(0x500E_0158 + 109 * 4));
    let _ = writeln!(uart, "  GPIO_ENABLE0     = 0x{:08X}", r(0x500E_0020));
    let _ = writeln!(uart, "  GPIO_ENABLE1     = 0x{:08X}", r(0x500E_002C));
    let _ = writeln!(uart, "  GPIO_OUT0        = 0x{:08X}", r(0x500E_0004));
    let _ = writeln!(uart, "  GPIO_OUT1        = 0x{:08X}", r(0x500E_0010));
    let _ = writeln!(uart, "  IO_MUX_GPIO51    = 0x{:08X}", r(0x500E_1004 + 51 * 4));
    let _ = writeln!(uart, "  PERI_CLK_CTRL00  = 0x{:08X}", r(0x500E_6030));
    let _ = writeln!(uart, "  PERI_CLK_CTRL01  = 0x{:08X}", r(0x500E_6034));
    let _ = writeln!(uart, "  LP_HP_CLK_CTRL   = 0x{:08X}", r(0x5011_1040));
    let _ = writeln!(uart, "  SYS_GMAC_CTRL0   = 0x{:08X}", r(0x500E_514C));
    let _ = writeln!(uart, "  MAC_CONFIG       = 0x{:08X}", r(0x5009_8000));
    let _ = writeln!(uart, "  MAC_FRAME_FILTER = 0x{:08X}", r(0x5009_8004));
    let mii_addr = r(0x5009_8010);
    let _ = writeln!(
        uart,
        "  MII_ADDR         = 0x{:08X}  (csr_clk_range bits[5:2] = 0b{:04b})",
        mii_addr,
        (mii_addr >> 2) & 0xF
    );
    let _ = writeln!(uart, "  MII_DATA         = 0x{:08X}", r(0x5009_8014));
    let _ = writeln!(uart, "  MAC_VERSION      = 0x{:08X}", r(0x5009_8020));
    let _ = writeln!(uart, "  DMA_BUS_MODE     = 0x{:08X}", r(0x5009_9000));
    let _ = writeln!(uart, "  DMA_OP_MODE      = 0x{:08X}", r(0x5009_9018));
    let _ = writeln!(uart, "  DMA_STATUS       = 0x{:08X}", r(0x5009_9014));

    // Force csr_clock_range to 0b0000 (= "AHB 60-100 MHz" = highest MDC
    // divider). This produces the slowest legal MDC regardless of actual AHB
    // frequency. If MDC was too fast (because actual AHB is ~80-90 MHz but
    // mac_init programmed the divider for AHB=40 MHz), MDIO transactions get
    // mistimed and PHY rejects them. Set this to match the IDF baseline value
    // (bits[5:2] = 0b0000) before scanning.
    unsafe {
        let mii_addr = 0x5009_8010 as *mut u32;
        let v = read_volatile(mii_addr);
        let v_new = v & !(0xF << 2);
        write_volatile(mii_addr, v_new);
        let _ = writeln!(
            uart,
            "  MII_ADDR forced csr_clk=0 -> 0x{:08X}",
            read_volatile(mii_addr as *const u32)
        );
    }

    // Probe a single MDIO read against the strap address (PHY=1, BMSR) and
    // dump MII_ADDR + MII_DATA before/after to catch a stuck-BUSY case.
    let _ = writeln!(uart, "--- single MDIO probe (PHY=1 BMSR) ---");
    let pre_addr = r(0x5009_8010);
    let pre_data = r(0x5009_8014);
    let _ = writeln!(
        uart,
        "  pre-read  MII_ADDR=0x{:08X} MII_DATA=0x{:08X}",
        pre_addr, pre_data
    );
    let v = mdio_read(1, BMSR).unwrap_or(0xDEAD);
    let post_addr = r(0x5009_8010);
    let post_data = r(0x5009_8014);
    let _ = writeln!(
        uart,
        "  post-read MII_ADDR=0x{:08X} MII_DATA=0x{:08X} return=0x{:04X}",
        post_addr, post_data, v
    );

    let _ = writeln!(uart, "--- MDIO scan (32 addresses) ---");
    let mut found: Option<u8> = None;
    for addr in 0u8..32 {
        let v = mdio_read(addr, BMSR).unwrap_or(0xDEAD);
        let alive = v != 0x0000 && v != 0xFFFF && v != 0xDEAD;
        if alive {
            let _ = writeln!(uart, "  addr={:>2} BMSR=0x{:04X} <- LIVE", addr, v);
            if found.is_none() {
                found = Some(addr);
            }
        }
    }

    let phy_addr = match found {
        Some(addr) => {
            let _ = writeln!(uart, "PHY found at addr {}", addr);
            addr
        }
        None => {
            let _ = writeln!(uart, "NO live PHY on the bus (all addresses returned 0x0000)");
            loop {
                spin_loop();
            }
        }
    };

    let id1 = mdio_read(phy_addr, PHYIDR1).unwrap_or(0xDEAD);
    let id2 = mdio_read(phy_addr, PHYIDR2).unwrap_or(0xDEAD);
    let bmcr = mdio_read(phy_addr, BMCR).unwrap_or(0xDEAD);
    let bmsr = mdio_read(phy_addr, BMSR).unwrap_or(0xDEAD);
    let anar = mdio_read(phy_addr, ANAR).unwrap_or(0xDEAD);
    let anlpar = mdio_read(phy_addr, ANLPAR).unwrap_or(0xDEAD);
    let _ = writeln!(uart, "PHY@{}: ID1=0x{:04X} ID2=0x{:04X}", phy_addr, id1, id2);
    let _ = writeln!(
        uart,
        "         BMCR=0x{:04X} BMSR=0x{:04X} ANAR=0x{:04X} ANLPAR=0x{:04X}",
        bmcr, bmsr, anar, anlpar
    );

    // For IP101GRI, PHYIDR1 = 0x0243, PHYIDR2 high bits = 0x0C50 family.
    if id1 == 0x0243 {
        let _ = writeln!(uart, "PHY ID matches IP101 family (0x0243). MDIO IS WORKING.");
    }

    // Poll BMSR for ~15 seconds, print every change. Auto-neg with a working
    // partner takes ~3 sec on 100M. Looking for link (bit 2) and an_done
    // (bit 5) to both go high, plus ANLPAR to populate.
    let _ = writeln!(uart, "--- waiting for auto-neg (poll BMSR up to 15s) ---");
    let mut last_bmsr = bmsr;
    let mut linked = false;
    for tick in 0..150u32 {
        let v = mdio_read(phy_addr, BMSR).unwrap_or(0xDEAD);
        if v != last_bmsr {
            let lp = mdio_read(phy_addr, ANLPAR).unwrap_or(0xDEAD);
            let _ = writeln!(
                uart,
                "[t={:>3}] BMSR 0x{:04X} -> 0x{:04X}  link={} an_done={} ANLPAR=0x{:04X}",
                tick * 100,
                last_bmsr,
                v,
                (v >> 2) & 1,
                (v >> 5) & 1,
                lp
            );
            last_bmsr = v;
        }
        if (v >> 2) & 1 != 0 && (v >> 5) & 1 != 0 {
            linked = true;
            let _ = writeln!(uart, "[t={:>3}ms] LINK UP", tick * 100);
            break;
        }
        // ~100ms delay; ROM runs CPU at low MHz so 200_000 spins ≈ 100ms.
        delay_cycles(200_000);
    }

    if !linked {
        let _ = writeln!(uart, "auto-neg did NOT complete in 15s. Final state:");
    } else {
        let _ = writeln!(uart, "Final post-link state:");
    }
    let bmcr2 = mdio_read(phy_addr, BMCR).unwrap_or(0xDEAD);
    let bmsr2 = mdio_read(phy_addr, BMSR).unwrap_or(0xDEAD);
    let anar2 = mdio_read(phy_addr, ANAR).unwrap_or(0xDEAD);
    let anlpar2 = mdio_read(phy_addr, ANLPAR).unwrap_or(0xDEAD);
    let _ = writeln!(
        uart,
        "  BMCR=0x{:04X} BMSR=0x{:04X} ANAR=0x{:04X} ANLPAR=0x{:04X}",
        bmcr2, bmsr2, anar2, anlpar2
    );
    let _ = writeln!(
        uart,
        "  decoded: link={} an_done={} 100M={} duplex_full={}",
        (bmsr2 >> 2) & 1,
        (bmsr2 >> 5) & 1,
        ((anar2 & anlpar2) >> 7) & 0x3, // bits 7,8 = 100M half/full ability
        ((anar2 & anlpar2) >> 6) & 0x3, // bits 6 (10full) | 8 (100full)
    );
    // IP101 specific: register 17 (0x11) = SPECIFIC_STATUS, bits[10:8] = link mode.
    let ip101_status = mdio_read(phy_addr, 0x11).unwrap_or(0xDEAD);
    let _ = writeln!(uart, "  IP101 SPECIFIC_STATUS (reg 0x11) = 0x{:04X}", ip101_status);

    if !linked {
        let _ = writeln!(uart, "=== no link, skipping TX test ===");
        loop {
            spin_loop();
        }
    }

    // Align MAC speed/duplex with what the PHY negotiated. Without this the
    // RMII reference clock divider stays at the default (10M) while the PHY
    // is at 100M → MAC TX bytes get clocked at the wrong rate.
    match ethernet.phy_negotiate() {
        Ok((speed, duplex)) => {
            let s = match speed {
                esp_p4_eth::Speed::Mbps10 => "10M",
                esp_p4_eth::Speed::Mbps100 => "100M",
            };
            let d = match duplex {
                esp_p4_eth::Duplex::Half => "half",
                esp_p4_eth::Duplex::Full => "full",
            };
            let _ = writeln!(uart, "phy_negotiate OK: speed={} duplex={}", s, d);
        }
        Err(e) => {
            let _ = writeln!(uart, "phy_negotiate ERR: {:?}", e);
        }
    }

    // --- TX test: send a gratuitous ARP request (broadcast) and watch RX ---
    // Source MAC = our MAC_ADDR (board strap). Sender IP = 169.254.144.50.
    // Target IP = 169.254.144.33 (Windows APIPA on Realtek port). If user is
    // on a real LAN, change these to match the local subnet. Either way the
    // frame is broadcast and triggers ARP-cache updates on the PC, which
    // should reply if it has the matching IP.
    const SENDER_IP: [u8; 4] = [169, 254, 144, 50];
    const TARGET_IP: [u8; 4] = [169, 254, 144, 33];
    let mut arp = [0u8; 64];
    build_arp_request(&mut arp, MAC_ADDR, SENDER_IP, TARGET_IP);
    let _ = writeln!(
        uart,
        "ARP who-has {}.{}.{}.{} tell {}.{}.{}.{}",
        TARGET_IP[0], TARGET_IP[1], TARGET_IP[2], TARGET_IP[3],
        SENDER_IP[0], SENDER_IP[1], SENDER_IP[2], SENDER_IP[3]
    );

    match ethernet.transmit(&arp) {
        Ok(()) => {
            let _ = writeln!(uart, "TX OK (60-byte ARP request sent)");
        }
        Err(e) => {
            let _ = writeln!(uart, "TX ERR: {:?}", e);
        }
    }

    // Poll RX for ~5 seconds. Print every frame received.
    let _ = writeln!(uart, "--- listening for RX frames (5s) ---");
    let mut rx_count = 0u32;
    for _ in 0..50u32 {
        esp_p4_eth::Dma::demand_rx_poll();
        if let Some((len, data)) = ethernet.receive() {
            rx_count += 1;
            let _ = writeln!(
                uart,
                "RX #{}: len={} dst={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} src={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ethertype=0x{:02x}{:02x}",
                rx_count,
                len,
                data[0], data[1], data[2], data[3], data[4], data[5],
                data[6], data[7], data[8], data[9], data[10], data[11],
                data[12], data[13]
            );
            ethernet.pop_rx();
        }
        delay_cycles(200_000); // ~100ms
    }
    let _ = writeln!(uart, "RX total: {} frames in 5s", rx_count);
    if rx_count == 0 {
        let _ = writeln!(uart, "(no frames seen — check cable, IP, or ARP cache)");
    }

    let _ = writeln!(uart, "=== mdio_test done ===");
    loop {
        spin_loop();
    }
}

const BROADCAST_MAC: [u8; 6] = [0xFF; 6];

fn build_arp_request(
    frame: &mut [u8; 64],
    source_mac: [u8; 6],
    sender_ip: [u8; 4],
    target_ip: [u8; 4],
) {
    frame.fill(0);
    frame[0..6].copy_from_slice(&BROADCAST_MAC);
    frame[6..12].copy_from_slice(&source_mac);
    frame[12..14].copy_from_slice(&0x0806u16.to_be_bytes()); // ARP ethertype
    frame[14..16].copy_from_slice(&0x0001u16.to_be_bytes()); // hw type = Ethernet
    frame[16..18].copy_from_slice(&0x0800u16.to_be_bytes()); // proto = IPv4
    frame[18] = 6; // hw len
    frame[19] = 4; // proto len
    frame[20..22].copy_from_slice(&0x0001u16.to_be_bytes()); // op = request
    frame[22..28].copy_from_slice(&source_mac);
    frame[28..32].copy_from_slice(&sender_ip);
    // target MAC zeros (unknown)
    frame[38..42].copy_from_slice(&target_ip);
}
