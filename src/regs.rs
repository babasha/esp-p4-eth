//! Raw EMAC register definitions for ESP32-P4.
//!
//! This module intentionally stays close to the vendor register layout so bring-up
//! work can be compared against the TRM and ESP-IDF headers. Higher-level code
//! should prefer the safe helpers in other modules instead of programming these
//! registers directly.

#[cfg(not(test))]
use core::ptr::{read_volatile, write_volatile};
#[cfg(test)]
use std::{cell::RefCell, thread_local, vec::Vec};

/// Base address of the EMAC peripheral.
pub const EMAC_BASE: usize = 0x5009_8000;

#[cfg(test)]
thread_local! {
    static TEST_REGISTERS: RefCell<Vec<(usize, u32)>> = const { RefCell::new(Vec::new()) };
}

const fn bit(n: u32) -> u32 {
    1u32 << n
}

pub mod mac {
    use super::EMAC_BASE;

    pub const MACCFG_ADDR: usize = EMAC_BASE;
    pub const MACFFILT_ADDR: usize = EMAC_BASE + 0x0004;
    pub const HASHTABLEHIGH_ADDR: usize = EMAC_BASE + 0x0008;
    pub const HASHTABLELOW_ADDR: usize = EMAC_BASE + 0x000c;
    pub const MIIADDR_ADDR: usize = EMAC_BASE + 0x0010;
    pub const MIIDATA_ADDR: usize = EMAC_BASE + 0x0014;
    pub const FLOWCTRL_ADDR: usize = EMAC_BASE + 0x0018;
    pub const VLANTAG_ADDR: usize = EMAC_BASE + 0x001c;
    pub const VERSION_ADDR: usize = EMAC_BASE + 0x0020;
    pub const DEBUG_ADDR: usize = EMAC_BASE + 0x0024;
    pub const INTSTATUS_ADDR: usize = EMAC_BASE + 0x0038;
    pub const INTMASK_ADDR: usize = EMAC_BASE + 0x003c;
    pub const MACADDR0HI_ADDR: usize = EMAC_BASE + 0x0040;
    pub const MACADDR0LO_ADDR: usize = EMAC_BASE + 0x0044;

    pub const MACCFG: usize = MACCFG_ADDR;
    pub const MACFFILT: usize = MACFFILT_ADDR;
    pub const HASHTABLEHIGH: usize = HASHTABLEHIGH_ADDR;
    pub const HASHTABLELOW: usize = HASHTABLELOW_ADDR;
    pub const MIIADDR: usize = MIIADDR_ADDR;
    pub const MIIDATA: usize = MIIDATA_ADDR;
    pub const FLOWCTRL: usize = FLOWCTRL_ADDR;
    pub const VLANTAG: usize = VLANTAG_ADDR;
    pub const VERSION: usize = VERSION_ADDR;
    pub const DEBUG: usize = DEBUG_ADDR;
    pub const INTSTATUS: usize = INTSTATUS_ADDR;
    pub const INTMASK: usize = INTMASK_ADDR;
    pub const MACADDR0HI: usize = MACADDR0HI_ADDR;
    pub const MACADDR0LO: usize = MACADDR0LO_ADDR;

    pub const CONFIG: usize = MACCFG;
    pub const FRAME_FILTER: usize = MACFFILT;
    pub const MII_ADDR: usize = MIIADDR;
    pub const MII_DATA: usize = MIIDATA;
    pub const ADDR0_HIGH: usize = MACADDR0HI;
    pub const ADDR0_LOW: usize = MACADDR0LO;
}

pub mod dma {
    use super::EMAC_BASE;

    pub const DMABUSMODE_ADDR: usize = EMAC_BASE + 0x1000;
    pub const TXPOLLDEMAND_ADDR: usize = EMAC_BASE + 0x1004;
    pub const RXPOLLDEMAND_ADDR: usize = EMAC_BASE + 0x1008;
    pub const DMARXDESCLIST_ADDR: usize = EMAC_BASE + 0x100c;
    pub const DMATXDESCLIST_ADDR: usize = EMAC_BASE + 0x1010;
    pub const DMASTATUS_ADDR: usize = EMAC_BASE + 0x1014;
    pub const DMAOPMODE_ADDR: usize = EMAC_BASE + 0x1018;
    pub const DMAINTEN_ADDR: usize = EMAC_BASE + 0x101c;
    pub const MISSEDFRAME_ADDR: usize = EMAC_BASE + 0x1020;
    pub const RXINTWATCHDOG_ADDR: usize = EMAC_BASE + 0x1024;
    pub const AXIBUSMODE_ADDR: usize = EMAC_BASE + 0x1028;
    pub const AHBAXISTATUS_ADDR: usize = EMAC_BASE + 0x102c;
    pub const CURHOSTTXDESC_ADDR: usize = EMAC_BASE + 0x1048;
    pub const CURHOSTRXDESC_ADDR: usize = EMAC_BASE + 0x104c;
    pub const CURHOSTTXBUF_ADDR: usize = EMAC_BASE + 0x1050;
    pub const CURHOSTRXBUF_ADDR: usize = EMAC_BASE + 0x1054;
    pub const HWFEATURE_ADDR: usize = EMAC_BASE + 0x1058;

    pub const DMABUSMODE: usize = DMABUSMODE_ADDR;
    pub const TXPOLLDEMAND: usize = TXPOLLDEMAND_ADDR;
    pub const RXPOLLDEMAND: usize = RXPOLLDEMAND_ADDR;
    pub const DMARXDESCLIST: usize = DMARXDESCLIST_ADDR;
    pub const DMATXDESCLIST: usize = DMATXDESCLIST_ADDR;
    pub const DMASTATUS: usize = DMASTATUS_ADDR;
    pub const DMAOPMODE: usize = DMAOPMODE_ADDR;
    pub const DMAINTEN: usize = DMAINTEN_ADDR;
    pub const MISSEDFRAME: usize = MISSEDFRAME_ADDR;
    pub const RXINTWATCHDOG: usize = RXINTWATCHDOG_ADDR;
    pub const AXIBUSMODE: usize = AXIBUSMODE_ADDR;
    pub const AHBAXISTATUS: usize = AHBAXISTATUS_ADDR;
    pub const CURHOSTTXDESC: usize = CURHOSTTXDESC_ADDR;
    pub const CURHOSTRXDESC: usize = CURHOSTRXDESC_ADDR;
    pub const CURHOSTTXBUF: usize = CURHOSTTXBUF_ADDR;
    pub const CURHOSTRXBUF: usize = CURHOSTRXBUF_ADDR;
    pub const HWFEATURE: usize = HWFEATURE_ADDR;

    pub const BUS_MODE: usize = DMABUSMODE;
    pub const TX_POLL_DEMAND: usize = TXPOLLDEMAND;
    pub const RX_POLL_DEMAND: usize = RXPOLLDEMAND;
    pub const RX_DESC_LIST: usize = DMARXDESCLIST;
    pub const TX_DESC_LIST: usize = DMATXDESCLIST;
    pub const STATUS: usize = DMASTATUS;
    pub const OP_MODE: usize = DMAOPMODE;
    pub const INT_EN: usize = DMAINTEN;
}

pub mod bits {
    use super::bit;

    pub mod maccfg {
        use super::bit;

        pub const RE: u32 = bit(2);
        pub const TE: u32 = bit(3);
        pub const ACS: u32 = bit(7);
        pub const DM: u32 = bit(11);
        pub const LM: u32 = bit(12);
        pub const FES: u32 = bit(14);
        pub const PS: u32 = bit(15);
        pub const IFG_SHIFT: u32 = 17;
        pub const IFG_MASK: u32 = 0x7 << IFG_SHIFT;
        pub const JD: u32 = bit(22);
        pub const WD: u32 = bit(23);
        pub const CST: u32 = bit(25);
    }

    pub mod macffilt {
        use super::bit;

        pub const PR: u32 = bit(0);
        pub const HUC: u32 = bit(1);
        pub const HMC: u32 = bit(2);
        pub const PM: u32 = bit(4);
        pub const HPF: u32 = bit(10);
        pub const RA: u32 = bit(31);
    }

    pub mod dmabusmode {
        use super::bit;

        pub const SWR: u32 = bit(0);
        pub const DSL_SHIFT: u32 = 2;
        pub const DSL_MASK: u32 = 0x1f << DSL_SHIFT;
        pub const ATDS: u32 = bit(7);
        pub const PBL_SHIFT: u32 = 8;
        pub const PBL_MASK: u32 = 0x3f << PBL_SHIFT;
        pub const FB: u32 = bit(16);
        pub const RPBL_SHIFT: u32 = 17;
        pub const RPBL_MASK: u32 = 0x3f << RPBL_SHIFT;
        pub const USP: u32 = bit(23);
        pub const PBLX8: u32 = bit(24);
        pub const AAL: u32 = bit(25);
    }

    pub mod dmastatus {
        use super::bit;

        pub const TI: u32 = bit(0);
        pub const TPS: u32 = bit(1);
        pub const TU: u32 = bit(2);
        pub const TJT: u32 = bit(3);
        pub const OVF: u32 = bit(4);
        pub const UNF: u32 = bit(5);
        pub const RI: u32 = bit(6);
        pub const RU: u32 = bit(7);
        pub const RPS: u32 = bit(8);
        pub const RWT: u32 = bit(9);
        pub const ETI: u32 = bit(10);
        pub const FBI: u32 = bit(13);
        pub const ERI: u32 = bit(14);
        pub const AIS: u32 = bit(15);
        pub const NIS: u32 = bit(16);
        pub const RS_SHIFT: u32 = 17;
        pub const RS_MASK: u32 = 0x7 << RS_SHIFT;
        pub const TS_SHIFT: u32 = 20;
        pub const TS_MASK: u32 = 0x7 << TS_SHIFT;
        pub const EB_SHIFT: u32 = 23;
        pub const EB_MASK: u32 = 0x7 << EB_SHIFT;
        pub const PROCESS_STOPPED: u32 = 0;
    }

    pub mod dmaopmode {
        use super::bit;

        pub const SR: u32 = bit(1);
        /// Forward Undersized Good Frames. Without this, GMAC drops < 64-byte
        /// frames (e.g. Windows ARP replies are 42 bytes on the wire).
        pub const FUF: u32 = bit(6);
        pub const ST: u32 = bit(13);
        pub const FTF: u32 = bit(20);
        pub const TSF: u32 = bit(21);
        /// Receive Store and Forward. **MUST stay 0 on ESP32-P4** — the EMAC
        /// RX FIFO is too small to hold a full Ethernet frame (empirically
        /// frames > ~256 bytes wire size are dropped before DMA writeback when
        /// RSF=1). With RSF=0 the GMAC operates in cut-through mode using the
        /// RTC threshold (default 64 bytes) and large frames flow through to
        /// the descriptor ring. See feedback memo `feedback_p4_emac_rsf_cutoff`.
        pub const RSF: u32 = bit(25);
    }

    pub mod dmainten {
        use super::bit;

        pub const TIE: u32 = bit(0);
        pub const TSE: u32 = bit(1);
        pub const TUE: u32 = bit(2);
        pub const TJE: u32 = bit(3);
        pub const OVE: u32 = bit(4);
        pub const UNE: u32 = bit(5);
        pub const RIE: u32 = bit(6);
        pub const RUE: u32 = bit(7);
        pub const RSE: u32 = bit(8);
        pub const RWE: u32 = bit(9);
        pub const ETE: u32 = bit(10);
        pub const FBE: u32 = bit(13);
        pub const ERE: u32 = bit(14);
        pub const AIE: u32 = bit(15);
        pub const NIE: u32 = bit(16);
    }

    pub mod flowctrl {
        use super::bit;

        pub const FCB_BPA: u32 = bit(0);
        pub const TFE: u32 = bit(1);
        pub const RFE: u32 = bit(2);
        pub const UP: u32 = bit(3);
        pub const PLT_SHIFT: u32 = 4;
        pub const PLT_MASK: u32 = 0x3 << PLT_SHIFT;
        pub const DZPQ: u32 = bit(7);
        pub const PT_SHIFT: u32 = 16;
        pub const PT_MASK: u32 = 0xffff << PT_SHIFT;
    }

    pub mod miiaddr {
        use super::bit;

        pub const MII_BUSY: u32 = bit(0);
        pub const MII_WRITE: u32 = bit(1);
        pub const CSR_CLOCK_RANGE_SHIFT: u32 = 2;
        pub const CSR_CLOCK_RANGE_MASK: u32 = 0x0f << CSR_CLOCK_RANGE_SHIFT;
        pub const REG_ADDR_SHIFT: u32 = 6;
        pub const REG_ADDR_MASK: u32 = 0x1f << REG_ADDR_SHIFT;
        pub const PHY_ADDR_SHIFT: u32 = 11;
        pub const PHY_ADDR_MASK: u32 = 0x1f << PHY_ADDR_SHIFT;
    }

    pub mod macaddr {
        use super::bit;

        pub const AE0: u32 = bit(31);
    }
}

/// Reads one 32-bit MMIO register.
#[inline]
pub fn read(addr: usize) -> u32 {
    #[cfg(test)]
    {
        TEST_REGISTERS.with(|registers| {
            registers
                .borrow()
                .iter()
                .find_map(|(stored_addr, value)| (*stored_addr == addr).then_some(*value))
                .unwrap_or(0)
        })
    }

    #[cfg(not(test))]
    // SAFETY: callers pass fixed MMIO register addresses defined in this module. `read_volatile`
    // is required here because these locations represent device registers, not normal memory.
    unsafe {
        read_volatile(addr as *const u32)
    }
}

/// Writes one 32-bit MMIO register.
#[inline]
pub fn write(addr: usize, value: u32) {
    #[cfg(test)]
    {
        TEST_REGISTERS.with(|registers| {
            let mut registers = registers.borrow_mut();
            if let Some((_, stored_value)) = registers
                .iter_mut()
                .find(|(stored_addr, _)| *stored_addr == addr)
            {
                *stored_value = value;
            } else {
                registers.push((addr, value));
            }
        });
    }

    #[cfg(not(test))]
    // SAFETY: callers pass fixed MMIO register addresses defined in this module. `write_volatile`
    // preserves side effects and ordering required for device register programming.
    unsafe {
        write_volatile(addr as *mut u32, value)
    }
}

/// Programs the primary station MAC address registers.
pub fn write_mac_address(mac: [u8; 6]) {
    let low = u32::from(mac[0])
        | (u32::from(mac[1]) << 8)
        | (u32::from(mac[2]) << 16)
        | (u32::from(mac[3]) << 24);
    let high = u32::from(mac[4]) | (u32::from(mac[5]) << 8) | bits::macaddr::AE0;

    write(mac::MACADDR0LO, low);
    write(mac::MACADDR0HI, high);
}

#[cfg(test)]
pub fn reset_test_registers() {
    TEST_REGISTERS.with(|registers| registers.borrow_mut().clear());
}

#[cfg(test)]
mod tests {
    use super::{bits, dma, mac, reset_test_registers, write_mac_address, EMAC_BASE};
    use super::{read, write};

    #[test]
    fn emac_base_matches_idf_hw_ver3() {
        assert_eq!(EMAC_BASE, 0x5009_8000);
    }

    #[test]
    fn mac_register_addresses_match_idf() {
        assert_eq!(mac::MACCFG_ADDR, 0x5009_8000);
        assert_eq!(mac::MACFFILT_ADDR, 0x5009_8004);
        assert_eq!(mac::MIIADDR_ADDR, 0x5009_8010);
        assert_eq!(mac::MIIDATA_ADDR, 0x5009_8014);
        assert_eq!(mac::VERSION_ADDR, 0x5009_8020);
        assert_eq!(mac::INTSTATUS_ADDR, 0x5009_8038);
        assert_eq!(mac::INTMASK_ADDR, 0x5009_803c);
        assert_eq!(mac::MACADDR0HI_ADDR, 0x5009_8040);
        assert_eq!(mac::MACADDR0LO_ADDR, 0x5009_8044);
    }

    #[test]
    fn dma_register_addresses_match_idf() {
        assert_eq!(dma::DMABUSMODE_ADDR, 0x5009_9000);
        assert_eq!(dma::TXPOLLDEMAND_ADDR, 0x5009_9004);
        assert_eq!(dma::RXPOLLDEMAND_ADDR, 0x5009_9008);
        assert_eq!(dma::DMARXDESCLIST_ADDR, 0x5009_900c);
        assert_eq!(dma::DMATXDESCLIST_ADDR, 0x5009_9010);
        assert_eq!(dma::DMASTATUS_ADDR, 0x5009_9014);
        assert_eq!(dma::DMAOPMODE_ADDR, 0x5009_9018);
        assert_eq!(dma::DMAINTEN_ADDR, 0x5009_901c);
    }

    #[test]
    fn maccfg_bit_masks_match_idf() {
        assert_eq!(bits::maccfg::RE, 1 << 2);
        assert_eq!(bits::maccfg::TE, 1 << 3);
        assert_eq!(bits::maccfg::ACS, 1 << 7);
        assert_eq!(bits::maccfg::DM, 1 << 11);
        assert_eq!(bits::maccfg::LM, 1 << 12);
        assert_eq!(bits::maccfg::FES, 1 << 14);
        assert_eq!(bits::maccfg::JD, 1 << 22);
        assert_eq!(bits::maccfg::WD, 1 << 23);
        assert_eq!(bits::maccfg::CST, 1 << 25);
    }

    #[test]
    fn macffilt_bit_masks_match_idf() {
        assert_eq!(bits::macffilt::PR, 1 << 0);
        assert_eq!(bits::macffilt::HUC, 1 << 1);
        assert_eq!(bits::macffilt::HMC, 1 << 2);
        assert_eq!(bits::macffilt::PM, 1 << 4);
        assert_eq!(bits::macffilt::HPF, 1 << 10);
        assert_eq!(bits::macffilt::RA, 1 << 31);
    }

    #[test]
    fn miiaddr_bit_fields_match_idf() {
        assert_eq!(bits::miiaddr::MII_BUSY, 1);
        assert_eq!(bits::miiaddr::MII_WRITE, 1 << 1);
        assert_eq!(bits::miiaddr::CSR_CLOCK_RANGE_SHIFT, 2);
        assert_eq!(bits::miiaddr::REG_ADDR_SHIFT, 6);
        assert_eq!(bits::miiaddr::PHY_ADDR_SHIFT, 11);
    }

    #[test]
    fn dma_status_and_mode_masks_match_idf() {
        assert_eq!(bits::dmabusmode::SWR, 1);
        assert_eq!(bits::dmabusmode::ATDS, 1 << 7);
        assert_eq!(bits::dmabusmode::FB, 1 << 16);
        assert_eq!(bits::dmabusmode::USP, 1 << 23);
        assert_eq!(bits::dmabusmode::PBLX8, 1 << 24);
        assert_eq!(bits::dmabusmode::AAL, 1 << 25);
        assert_eq!(bits::dmastatus::TI, 1 << 0);
        assert_eq!(bits::dmastatus::TPS, 1 << 1);
        assert_eq!(bits::dmastatus::OVF, 1 << 4);
        assert_eq!(bits::dmastatus::UNF, 1 << 5);
        assert_eq!(bits::dmastatus::RI, 1 << 6);
        assert_eq!(bits::dmastatus::RPS, 1 << 8);
        assert_eq!(bits::dmastatus::FBI, 1 << 13);
        assert_eq!(bits::dmastatus::ERI, 1 << 14);
        assert_eq!(bits::dmastatus::NIS, 1 << 16);
        assert_eq!(bits::dmastatus::RS_MASK, 0x7 << 17);
        assert_eq!(bits::dmastatus::TS_MASK, 0x7 << 20);
        assert_eq!(bits::dmastatus::EB_MASK, 0x7 << 23);
        assert_eq!(bits::dmainten::TSE, 1 << 1);
        assert_eq!(bits::dmainten::OVE, 1 << 4);
        assert_eq!(bits::dmainten::UNE, 1 << 5);
        assert_eq!(bits::dmainten::RUE, 1 << 7);
        assert_eq!(bits::dmainten::RSE, 1 << 8);
        assert_eq!(bits::dmainten::FBE, 1 << 13);
        assert_eq!(bits::dmainten::ERE, 1 << 14);
        assert_eq!(bits::dmaopmode::SR, 1 << 1);
        assert_eq!(bits::dmaopmode::ST, 1 << 13);
        assert_eq!(bits::flowctrl::FCB_BPA, 1 << 0);
        assert_eq!(bits::flowctrl::TFE, 1 << 1);
        assert_eq!(bits::flowctrl::RFE, 1 << 2);
        assert_eq!(bits::flowctrl::PT_MASK, 0xffff << 16);
    }

    #[test]
    fn write_mac_address_uses_little_endian_register_layout_and_enables_addr0() {
        reset_test_registers();

        write_mac_address([0x02, 0x13, 0x57, 0x9b, 0xdf, 0xf0]);

        assert_eq!(read(mac::MACADDR0LO), 0x9b57_1302);
        assert_eq!(read(mac::MACADDR0HI), bits::macaddr::AE0 | 0x0000_f0df);
    }

    #[test]
    fn test_register_bank_reads_unwritten_registers_as_zero() {
        reset_test_registers();

        assert_eq!(read(mac::MACCFG), 0);
        assert_eq!(read(dma::DMASTATUS), 0);
    }

    #[test]
    fn test_register_bank_overwrites_existing_register_value() {
        reset_test_registers();

        write(dma::DMASTATUS, 0x1111_2222);
        write(dma::DMASTATUS, 0x3333_4444);

        assert_eq!(read(dma::DMASTATUS), 0x3333_4444);
    }
}
