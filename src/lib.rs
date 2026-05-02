//! `#![no_std]` async Ethernet MAC driver for ESP32-P4 RMII designs,
//! plug-in compatible with [`embassy-net`](https://crates.io/crates/embassy-net).
//!
//! See the crate-level `README.md` for the project status, hardware support
//! matrix, quick-start instructions, and the mandatory build invariants
//! (linker `RAM_DMA` section below `0x4FF80000`, workspace `opt-level = 1`).
//!
//! # Architecture
//!
//! The crate is split into a few narrow layers:
//!
//! - [`regs`] exposes raw MMIO register addresses and bit fields.
//! - [`clock`] and [`pins`] program the P4 clock tree and IO_MUX for RMII.
//! - [`board`] groups pin maps, reference-clock pad, and PHY MDIO address
//!   into a single [`BoardConfig`]; [`BoardConfig::WAVESHARE_P4_ETH`] is the
//!   built-in default for the Waveshare ESP32-P4-ETH dev board.
//! - [`descriptors`] owns DMA descriptor/ring state and packet buffers.
//! - [`dma`] handles cache synchronization, DMA reset/init, and interrupt status.
//! - [`phy`] provides Clause 22 MDIO access plus an IP101-oriented PHY model.
//! - [`Ethernet`] composes all of the above into a device that can be driven
//!   directly or through [`ch`] / `embassy-net-driver-channel`.
//!
//! # Bring-up
//!
//! Two constructor families are provided:
//!
//! - [`new_from_static_resources`] / [`Ethernet::try_new`] — Waveshare
//!   ESP32-P4-ETH default pin map (back-compat, zero-config).
//! - [`eth::new_from_static_resources_with_board`] /
//!   [`Ethernet::try_new_with_board`] — accept an explicit [`BoardConfig`]
//!   for any other RMII layout.
//!
//! # ESP32-P4 specifics
//!
//! ESP32-P4 needs a few hardware-specific invariants that are easy to miss:
//!
//! - DMA descriptors and packet buffers must stay cache-line aligned. This crate
//!   uses 128-byte alignment to match the P4 L2 cache line.
//! - DMA-shared statics must live below `0x4FF80000` — the upper 256 KB of HP
//!   SRAM is the L2 cache backing region and is not accessible to bus masters.
//! - CPU writes must be synchronized to memory before DMA sees them, and DMA
//!   writes must be invalidated out of cache before the CPU reads them. The
//!   cache sync helpers in [`dma`] handle that requirement.
//! - `DMA_OP_MODE.RSF` (Receive Store and Forward) **must stay 0** — the P4
//!   RX FIFO is too small to hold a full Ethernet frame and `RSF = 1` silently
//!   drops everything larger than the FIFO. This crate uses cut-through
//!   receive by default; do not re-enable RSF.
//! - RMII pin routing is not implicit. The bring-up path applies the supplied
//!   [`BoardConfig`] through [`pins::configure_rmii_pin_set`],
//!   [`pins::configure_mdio_pin_set`], and [`clock::configure_clock_ext_in`].
//!
//! The examples use [`StaticDmaResources`] as the preferred safe way to own the
//! backing DMA memory in `'static` storage; `#[link_section = ".dma_bss"]`
//! places that storage inside the linker's `RAM_DMA` region.
#![no_std]

#[cfg(test)]
extern crate std;

pub use embassy_net_driver_channel as ch;

pub mod board;
pub mod eth;
pub use board::BoardConfig;
pub mod clic;
pub mod systimer;
#[cfg(all(target_arch = "riscv32", feature = "p4-time-driver"))]
pub mod time_driver;
#[cfg(all(target_arch = "riscv32", feature = "p4-time-driver-irq"))]
pub mod time_driver_irq;
pub mod time_driver_irq_logic;

pub use eth::{clock, descriptors, dma, phy, pins, regs};
#[cfg(target_arch = "riscv32")]
pub use eth::{ethernet_task, wake_rx_task, wake_tx_task};
pub use eth::{
    new, new_from_static_resources, Device, Duplex, Ethernet, MacError, Runner, CHANNEL_RX_COUNT,
    CHANNEL_TX_COUNT, MTU,
};

pub use clock::{
    configure_clock_ext_in, configure_clock_mpll_out, configure_speed_divider,
    disable_emac_clock_tree,
};
pub use clock::{MpllClockOutPin, RefClockPin};
pub use descriptors::{
    zeroed_rx_descriptors, zeroed_tx_descriptors, BufferTooLarge, DescriptorError, DmaBuffers,
    OwnedBy, RDes, RDesRing, RxRingStats, StaticDmaResources, TDes, TDesRing, TxRingStats,
    BUF_SIZE, MIN_RX_FRAME_SIZE, RX_DESC_COUNT, TX_DESC_COUNT,
};

/// Diagnostic counters and last-frame snapshots exposed for observability and
/// debugging. **The contents of this module are intentionally not part of
/// the crate's stable API** — atomics may be added, removed, or renamed in
/// any release. Build dashboards on top of them at your own risk and pin a
/// specific `esp-p4-eth` version if you do.
pub mod diag {
    #[cfg(target_arch = "riscv32")]
    pub use crate::descriptors::{
        CPU_DESC0_SNAPSHOT, TX_LAST_BUF_ADDR, TX_LAST_DESC_ADDR, TX_LAST_TDES0, TX_LAST_TDES1,
    };
    pub use crate::descriptors::{
        RX_ERROR_FRAMES_TOTAL, RX_LARGE_FRAMES, RX_LAST_FRAME_LEN, RX_LAST_RDES0,
        RX_OVERSIZED_FRAMES_TOTAL, RX_RUNT_FRAMES_TOTAL,
    };
    pub use crate::dma::{CACHE_INV_CALLS, CACHE_INV_TICKS, CACHE_WB_CALLS, CACHE_WB_TICKS};
    #[cfg(target_arch = "riscv32")]
    pub use crate::dma::{LAST_INVALIDATE_RC, LAST_WRITEBACK_RC};
    pub use crate::eth::RX_DESC_STRIDE;
    #[cfg(target_arch = "riscv32")]
    pub use crate::eth::{
        RX_ARP, RX_BUF_REQUESTED, RX_DESC_BASE, RX_DHCP_FRAMES, RX_FRAMES, RX_ICMP, RX_IPV4,
        RX_LAST_DHCP_FRAME, RX_LAST_DST_MAC_HI, RX_LAST_ETHERTYPE, RX_LAST_LARGE_FRAME,
        RX_LAST_LARGE_FRAME_LEN, TX_BUF_REQUESTED, TX_FRAMES, TX_LAST_DST_MAC_HI,
        TX_LAST_ETHERTYPE, TX_LAST_LEN, TX_LAST_SRC_MAC_HI,
    };
}
pub use dma::{Dma, DmaError, DmaInterruptStatus};
pub use phy::{
    anlpar, bmcr, bmsr, mdio_read, mdio_write, Ip101, LinkState, Phy, PhyError, Speed, ANAR,
    ANLPAR, BMCR, BMSR, PHYIDR1, PHYIDR2,
};
pub use pins::{
    configure_rmii_pins, release_phy_reset_pin, release_waveshare_phy_reset, PhyResetPinConfig,
};
