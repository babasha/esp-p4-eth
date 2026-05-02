//! DMA descriptor and ring management.
//!
//! Descriptors and backing buffers are aligned to 128 bytes to match the P4
//! L2 cache line size (`CONFIG_CACHE_L2_CACHE_LINE_128B=y` in the IDF default
//! sdkconfig). 128-byte alignment ensures each descriptor / buffer occupies
//! its own L2 line, so a `Cache_Invalidate_Addr` on one entry never drops
//! cached writes from a neighbour. P4 L1 D-cache line is 64 bytes, but
//! coherency must be maintained at the wider of the two. That alignment is
//! important because cache maintenance is
//! performed on whole cache lines, not individual descriptor words.

use core::mem::{align_of, offset_of, size_of};
use core::sync::atomic::{fence, AtomicU32, Ordering};

/// Diagnostic: total runt frames dropped (len < MIN_RX_FRAME_SIZE).
pub static RX_RUNT_FRAMES_TOTAL: AtomicU32 = AtomicU32::new(0);
/// Diagnostic: total oversized frames dropped (len > BUF_SIZE).
pub static RX_OVERSIZED_FRAMES_TOTAL: AtomicU32 = AtomicU32::new(0);
/// Diagnostic: total error frames dropped (RDES0_ES set or incomplete).
pub static RX_ERROR_FRAMES_TOTAL: AtomicU32 = AtomicU32::new(0);
/// Diagnostic: frame length of the last successfully-received RX frame.
pub static RX_LAST_FRAME_LEN: AtomicU32 = AtomicU32::new(0);
/// Diagnostic: RDES0 of the last RX descriptor we inspected (any path).
pub static RX_LAST_RDES0: AtomicU32 = AtomicU32::new(0);
/// Diagnostic: count of frames where len >= 200 (size-cutoff investigation).
pub static RX_LARGE_FRAMES: AtomicU32 = AtomicU32::new(0);

use vcell::VolatileCell;

use crate::dma::Dma;
#[cfg(test)]
use crate::regs;

/// Default number of TX descriptors in the ring.
pub const TX_DESC_COUNT: usize = 8;
/// Default number of RX descriptors in the ring.
pub const RX_DESC_COUNT: usize = 8;
/// Size of each packet buffer in bytes.
pub const BUF_SIZE: usize = 1536;
/// Minimum accepted RX frame length for non-empty packets.
pub const MIN_RX_FRAME_SIZE: usize = 64;

/// TX descriptor ownership bit.
pub const TDES0_OWN: u32 = 1 << 31;
/// TX interrupt-on-completion flag for a descriptor.
pub const TDES0_IC: u32 = 1 << 30;
/// TX last-segment flag.
pub const TDES0_LS: u32 = 1 << 29;
/// TX first-segment flag.
pub const TDES0_FS: u32 = 1 << 28;
/// TX chained-descriptor mode bit.
pub const TDES0_CHAINED: u32 = 1 << 20;

/// RX descriptor ownership bit.
pub const RDES0_OWN: u32 = 1 << 31;
/// Shift for the RX frame-length field inside `RDES0`.
pub const RDES0_FL_SHIFT: u32 = 16;
/// Mask for the RX frame-length field inside `RDES0`.
pub const RDES0_FL_MASK: u32 = 0x3fff << RDES0_FL_SHIFT;
/// RX error-summary bit.
pub const RDES0_ES: u32 = 1 << 15;
/// RX first-segment flag.
pub const RDES0_FS: u32 = 1 << 9;
/// RX last-segment flag.
pub const RDES0_LS: u32 = 1 << 8;

/// RX buffer-1 size field mask in `RDES1`.
pub const RDES1_BUF1_SIZE_MASK: u32 = 0x1fff;
/// RX chained-descriptor mode bit.
pub const RDES1_CHAINED: u32 = 1 << 14;

/// Current descriptor owner.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OwnedBy {
    /// The CPU owns the descriptor.
    Cpu,
    /// The DMA engine owns the descriptor.
    Dma,
}

/// Error returned when a packet does not fit into one DMA buffer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BufferTooLarge {
    /// Frame length exceeded the supported single-buffer size.
    Frame { len: usize, max: usize },
}

/// Ring-level transmit/receive submission errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DescriptorError {
    /// Frame did not fit in the backing DMA buffer.
    BufferTooLarge(BufferTooLarge),
    /// The ring has no CPU-owned descriptor available right now.
    RingFull,
    /// No frame is available in the current RX descriptor.
    NoFrame,
}

/// Cumulative TX ring counters useful during bring-up and soak testing.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TxRingStats {
    /// Number of frames successfully queued to DMA.
    pub transmitted_frames: u32,
    /// Number of times TX found no CPU-owned descriptor.
    pub ring_full_events: u32,
}

/// Cumulative RX ring counters useful during bring-up and soak testing.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RxRingStats {
    /// Number of frames returned to software successfully.
    pub received_frames: u32,
    /// Number of frames dropped because the descriptor reported errors.
    pub error_frames: u32,
    /// Number of frames dropped for being shorter than [`MIN_RX_FRAME_SIZE`].
    pub runt_frames: u32,
    /// Number of frames dropped for exceeding the backing DMA buffer size.
    pub oversized_frames: u32,
    /// Number of times the RX ring was rebuilt after overflow/recovery.
    pub overflow_resets: u32,
}

/// Raw DMA descriptor payload size before alignment padding.
pub const DMA_DESCRIPTOR_WORDS_SIZE: usize = 16;
/// Required cache-line alignment for descriptors and static buffers.
/// Matches `CONFIG_CACHE_L2_CACHE_LINE_SIZE = 128` on ESP32-P4.
pub const DMA_DESCRIPTOR_ALIGN: usize = 128;

/// Cache-line aligned packet buffer block.
#[repr(C, align(128))]
pub struct DmaBuffers<const N: usize> {
    /// Backing packet storage for one DMA ring.
    pub inner: [[u8; BUF_SIZE]; N],
}

impl<const N: usize> DmaBuffers<N> {
    /// Zero-initialized packet storage block.
    pub const ZERO: Self = Self {
        inner: [[0; BUF_SIZE]; N],
    };
}

/// Borrowed view into the descriptor and packet-buffer arrays owned by [`StaticDmaResources`].
pub type SplitDmaResources<'a> = (
    &'a mut [TDes; TX_DESC_COUNT],
    &'a mut [RDes; RX_DESC_COUNT],
    &'a mut [[u8; BUF_SIZE]; TX_DESC_COUNT],
    &'a mut [[u8; BUF_SIZE]; RX_DESC_COUNT],
);

/// Safe owner for all static DMA memory used by the driver.
pub struct StaticDmaResources {
    tx_desc: [TDes; TX_DESC_COUNT],
    rx_desc: [RDes; RX_DESC_COUNT],
    tx_buf: DmaBuffers<TX_DESC_COUNT>,
    rx_buf: DmaBuffers<RX_DESC_COUNT>,
}

impl StaticDmaResources {
    /// Creates a zero-initialized static DMA resource block.
    pub const fn new() -> Self {
        Self {
            tx_desc: zeroed_tx_desc_array(),
            rx_desc: zeroed_rx_desc_array(),
            tx_buf: DmaBuffers::ZERO,
            rx_buf: DmaBuffers::ZERO,
        }
    }

    /// Splits the block into the descriptor and packet-buffer slices expected by the driver.
    pub fn split(&mut self) -> SplitDmaResources<'_> {
        (
            &mut self.tx_desc,
            &mut self.rx_desc,
            &mut self.tx_buf.inner,
            &mut self.rx_buf.inner,
        )
    }
}

impl Default for StaticDmaResources {
    fn default() -> Self {
        Self::new()
    }
}

/// Raw transmit descriptor layout as seen by the DMA engine.
#[repr(C, align(128))]
pub struct TDes {
    tdes0: VolatileCell<u32>,
    tdes1: VolatileCell<u32>,
    buf_addr: VolatileCell<u32>,
    next_desc: VolatileCell<u32>,
}

const _: [(); DMA_DESCRIPTOR_ALIGN] = [(); size_of::<TDes>()];
const _: [(); DMA_DESCRIPTOR_ALIGN] = [(); align_of::<TDes>()];
const _: [(); 0] = [(); offset_of!(TDes, tdes0)];
const _: [(); 4] = [(); offset_of!(TDes, tdes1)];
const _: [(); 8] = [(); offset_of!(TDes, buf_addr)];
const _: [(); 12] = [(); offset_of!(TDes, next_desc)];

impl TDes {
    /// Creates a zero-initialized descriptor value suitable for static storage.
    pub const fn new_zeroed() -> Self {
        Self {
            tdes0: VolatileCell::new(0),
            tdes1: VolatileCell::new(0),
            buf_addr: VolatileCell::new(0),
            next_desc: VolatileCell::new(0),
        }
    }

    /// Sets the software-visible ownership bit.
    pub fn set_owned_by(&self, owner: OwnedBy) {
        let value = match owner {
            OwnedBy::Cpu => self.tdes0.get() & !TDES0_OWN,
            OwnedBy::Dma => self.tdes0.get() | TDES0_OWN,
        };
        self.tdes0.set(value);
    }

    /// Returns the current descriptor owner according to the ownership bit.
    pub fn owned_by(&self) -> OwnedBy {
        if self.tdes0.get() & TDES0_OWN != 0 {
            OwnedBy::Dma
        } else {
            OwnedBy::Cpu
        }
    }

    fn set_len_and_flags(&self, len: usize) {
        self.tdes1.set((len as u32) & RDES1_BUF1_SIZE_MASK);
        self.tdes0
            .set((self.tdes0.get() & TDES0_CHAINED) | TDES0_FS | TDES0_LS | TDES0_IC);
    }

    fn set_chained(&self) {
        self.tdes0.set(self.tdes0.get() | TDES0_CHAINED);
    }

    fn set_buffer_addr(&self, addr: *const u8) {
        self.buf_addr.set(addr as usize as u32);
    }

    fn set_next_desc(&self, addr: *const TDes) {
        self.next_desc.set(addr as usize as u32);
    }
}

/// Snapshot of what CPU last wrote into RX descriptor 0 — exposed for debug
/// to compare against memory state read via `Dma::peek_rdes` (after invalidate).
#[cfg(target_arch = "riscv32")]
pub static CPU_DESC0_SNAPSHOT: [core::sync::atomic::AtomicU32; 4] = [
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
];

/// Last TX descriptor address handed to DMA.
#[cfg(target_arch = "riscv32")]
pub static TX_LAST_DESC_ADDR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Last TX packet buffer address handed to DMA (DMA reads frame from here).
#[cfg(target_arch = "riscv32")]
pub static TX_LAST_BUF_ADDR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Last TX descriptor word 0 (status/flags) — should have OWN=1 (DMA), FS=1, LS=1, IC=1, CHAINED=1.
#[cfg(target_arch = "riscv32")]
pub static TX_LAST_TDES0: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Last TX descriptor word 1 (lengths). Low 13 bits = buffer 1 size = frame length.
#[cfg(target_arch = "riscv32")]
pub static TX_LAST_TDES1: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Raw receive descriptor layout as seen by the DMA engine.
#[repr(C, align(128))]
pub struct RDes {
    rdes0: VolatileCell<u32>,
    rdes1: VolatileCell<u32>,
    buf_addr: VolatileCell<u32>,
    next_desc: VolatileCell<u32>,
}

const _: [(); DMA_DESCRIPTOR_ALIGN] = [(); size_of::<RDes>()];
const _: [(); DMA_DESCRIPTOR_ALIGN] = [(); align_of::<RDes>()];
const _: [(); 0] = [(); offset_of!(RDes, rdes0)];
const _: [(); 4] = [(); offset_of!(RDes, rdes1)];
const _: [(); 8] = [(); offset_of!(RDes, buf_addr)];
const _: [(); 12] = [(); offset_of!(RDes, next_desc)];

const _: [(); DMA_DESCRIPTOR_ALIGN] = [(); align_of::<DmaBuffers<TX_DESC_COUNT>>()];
const _: [(); DMA_DESCRIPTOR_ALIGN] = [(); align_of::<DmaBuffers<RX_DESC_COUNT>>()];
const _: [(); DMA_DESCRIPTOR_ALIGN] = [(); align_of::<StaticDmaResources>()];

impl RDes {
    /// Creates a zero-initialized descriptor value suitable for static storage.
    pub const fn new_zeroed() -> Self {
        Self {
            rdes0: VolatileCell::new(0),
            rdes1: VolatileCell::new(0),
            buf_addr: VolatileCell::new(0),
            next_desc: VolatileCell::new(0),
        }
    }

    /// Sets the software-visible ownership bit.
    pub fn set_owned_by(&self, owner: OwnedBy) {
        let value = match owner {
            OwnedBy::Cpu => self.rdes0.get() & !RDES0_OWN,
            OwnedBy::Dma => self.rdes0.get() | RDES0_OWN,
        };
        self.rdes0.set(value);
    }

    /// Returns the current descriptor owner according to the ownership bit.
    pub fn owned_by(&self) -> OwnedBy {
        if self.rdes0.get() & RDES0_OWN != 0 {
            OwnedBy::Dma
        } else {
            OwnedBy::Cpu
        }
    }

    /// Extracts the hardware-reported frame length from `RDES0`.
    pub fn frame_len(&self) -> usize {
        ((self.rdes0.get() & RDES0_FL_MASK) >> RDES0_FL_SHIFT) as usize
    }

    /// Returns `true` when the descriptor marks both first and last segment.
    pub fn is_complete_frame(&self) -> bool {
        let status = self.rdes0.get();
        status & RDES0_FS != 0 && status & RDES0_LS != 0
    }

    fn set_buffer_addr(&self, addr: *const u8) {
        self.buf_addr.set(addr as usize as u32);
    }

    fn set_next_desc(&self, addr: *const RDes) {
        self.next_desc.set(addr as usize as u32);
    }

    fn configure_buffer(&self, len: usize) {
        self.rdes1
            .set(((len as u32) & RDES1_BUF1_SIZE_MASK) | RDES1_CHAINED);
    }
}

const fn zeroed_tx_desc_array() -> [TDes; TX_DESC_COUNT] {
    [const { TDes::new_zeroed() }; TX_DESC_COUNT]
}

const fn zeroed_rx_desc_array() -> [RDes; RX_DESC_COUNT] {
    [const { RDes::new_zeroed() }; RX_DESC_COUNT]
}

/// Builds a zeroed TX descriptor array for stack allocation and tests.
pub fn zeroed_tx_descriptors() -> [TDes; TX_DESC_COUNT] {
    zeroed_tx_desc_array()
}

/// Builds a zeroed RX descriptor array for stack allocation and tests.
pub fn zeroed_rx_descriptors() -> [RDes; RX_DESC_COUNT] {
    zeroed_rx_desc_array()
}

/// TX descriptor ring plus its backing packet buffers.
pub struct TDesRing<'a> {
    descriptors: &'a mut [TDes],
    buffers: &'a mut [[u8; BUF_SIZE]],
    index: usize,
    stats: TxRingStats,
}

impl<'a> TDesRing<'a> {
    /// Builds and initializes a TX descriptor ring.
    pub fn new(descriptors: &'a mut [TDes], buffers: &'a mut [[u8; BUF_SIZE]]) -> Self {
        assert!(
            !descriptors.is_empty(),
            "TX descriptor ring must not be empty"
        );
        assert_eq!(
            descriptors.len(),
            buffers.len(),
            "TX descriptors and buffers must have identical lengths"
        );

        let mut ring = Self {
            descriptors,
            buffers,
            index: 0,
            stats: TxRingStats::default(),
        };
        ring.reset();
        ring
    }

    /// Rebuilds the ring links and returns all descriptors to CPU ownership.
    ///
    /// **Does not** program the DMA `TX_DESC_LIST` peripheral register —
    /// that must happen separately via [`Dma::set_descriptor_lists`] AFTER
    /// [`Dma::dma_reset`]. Writing it from here at construction time hangs
    /// the AHB bus when the EMAC has not yet completed reset (intermittent,
    /// raced ~10 % of warm reboots before the split — see 2026-04-27 cold-
    /// boot stress notes).
    ///
    /// Per-descriptor cache writeback is intentionally moved out of this
    /// loop into `flush_all` so that the **first** ring construction (which
    /// runs before the L1/L2 cache writeback ROM helper is reachable —
    /// 2026-04-27 stress runs reproduced an AHB hang inside the cache ROM
    /// call here when invoked too early after warm reboot) does not race.
    /// Initial construction relies on `mac_init -> Dma::dma_reset` having
    /// already ensured DMA is idle, so the descriptors only need to become
    /// DMA-visible once `start()` flips the ring on; that is what the
    /// `flush_all` call inside `Ethernet::reset_dma` covers.
    pub fn reset(&mut self) {
        let desc_len = self.descriptors.len();
        for i in 0..desc_len {
            let desc = &self.descriptors[i];
            desc.tdes0.set(0);
            desc.tdes1.set(0);
            desc.set_chained();
            desc.set_buffer_addr(self.buffers[i].as_ptr());
            let next = if i + 1 == desc_len {
                &self.descriptors[0]
            } else {
                &self.descriptors[i + 1]
            };
            desc.set_next_desc(next as *const TDes);
            desc.set_owned_by(OwnedBy::Cpu);
        }
        self.index = 0;
    }

    /// Pushes every descriptor's CPU-side state out to DMA-visible memory.
    /// Call this **after** `Dma::dma_reset` has put the engine into a known
    /// state — invoking the cache ROM helper while the EMAC carries stale
    /// DMA state from a previous binary intermittently hangs the AHB bus.
    pub fn flush_all(&self) {
        for desc in self.descriptors.iter() {
            Dma::flush_descriptor(desc);
        }
    }

    /// Returns the descriptor base pointer (used by the DMA programming step).
    pub fn descriptors_ptr(&self) -> *const TDes {
        self.descriptors.as_ptr()
    }

    /// Copies `frame` into the current TX buffer and hands the descriptor to DMA.
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), DescriptorError> {
        if frame.len() > BUF_SIZE {
            return Err(DescriptorError::BufferTooLarge(BufferTooLarge::Frame {
                len: frame.len(),
                max: BUF_SIZE,
            }));
        }

        let desc = &self.descriptors[self.index];
        Dma::invalidate_descriptor(desc);
        if desc.owned_by() != OwnedBy::Cpu {
            self.stats.ring_full_events = self.stats.ring_full_events.saturating_add(1);
            return Err(DescriptorError::RingFull);
        }

        self.buffers[self.index][..frame.len()].copy_from_slice(frame);
        desc.set_len_and_flags(frame.len());
        Dma::flush_buffer(&self.buffers[self.index]);
        Dma::flush_descriptor(desc);
        fence(Ordering::Release);
        desc.set_owned_by(OwnedBy::Dma);
        Dma::flush_descriptor(desc);
        Dma::demand_tx_poll();

        // Debug snapshot: capture descriptor + buffer address + first 14 bytes.
        #[cfg(target_arch = "riscv32")]
        {
            use core::sync::atomic::Ordering::Relaxed;
            TX_LAST_DESC_ADDR
                .store(desc as *const _ as usize as u32, Relaxed);
            TX_LAST_BUF_ADDR
                .store(self.buffers[self.index].as_ptr() as usize as u32, Relaxed);
            TX_LAST_TDES0.store(desc.tdes0.get(), Relaxed);
            TX_LAST_TDES1.store(desc.tdes1.get(), Relaxed);
        }

        self.index = (self.index + 1) % self.descriptors.len();
        self.stats.transmitted_frames = self.stats.transmitted_frames.saturating_add(1);
        Ok(())
    }

    /// Returns `true` when the current TX descriptor is CPU-owned.
    pub fn has_capacity(&self) -> bool {
        let desc = &self.descriptors[self.index];
        Dma::invalidate_descriptor(desc);
        desc.owned_by() == OwnedBy::Cpu
    }

    /// Returns a snapshot of cumulative TX statistics.
    pub fn stats(&self) -> TxRingStats {
        self.stats
    }

    /// Test/fuzz helper that returns the current ring cursor.
    #[cfg(any(test, feature = "fuzzing"))]
    #[doc(hidden)]
    pub fn fuzz_current_index(&self) -> usize {
        self.index
    }
}

/// RX descriptor ring plus its backing packet buffers.
pub struct RDesRing<'a> {
    descriptors: &'a mut [RDes],
    buffers: &'a mut [[u8; BUF_SIZE]],
    index: usize,
    stats: RxRingStats,
}

impl<'a> RDesRing<'a> {
    /// Builds and initializes an RX descriptor ring.
    pub fn new(descriptors: &'a mut [RDes], buffers: &'a mut [[u8; BUF_SIZE]]) -> Self {
        assert!(
            !descriptors.is_empty(),
            "RX descriptor ring must not be empty"
        );
        assert_eq!(
            descriptors.len(),
            buffers.len(),
            "RX descriptors and buffers must have identical lengths"
        );

        let mut ring = Self {
            descriptors,
            buffers,
            index: 0,
            stats: RxRingStats::default(),
        };
        ring.reset();
        ring
    }

    /// Rebuilds the ring links and returns all descriptors to DMA ownership.
    ///
    /// **Does not** program the DMA `RX_DESC_LIST` peripheral register —
    /// that must happen separately via [`Dma::set_descriptor_lists`] AFTER
    /// [`Dma::dma_reset`]. See `TDesRing::reset` for the rationale, plus
    /// the explanation of why the cache writeback loop is split into a
    /// separate `flush_all`.
    pub fn reset(&mut self) {
        let desc_len = self.descriptors.len();
        for i in 0..desc_len {
            let desc = &self.descriptors[i];
            desc.rdes0.set(0);
            desc.configure_buffer(BUF_SIZE);
            desc.set_buffer_addr(self.buffers[i].as_ptr());
            let next = if i + 1 == desc_len {
                &self.descriptors[0]
            } else {
                &self.descriptors[i + 1]
            };
            desc.set_next_desc(next as *const RDes);
            desc.set_owned_by(OwnedBy::Dma);
        }
        self.index = 0;

        // Snapshot what CPU just wrote into descriptor[0] so external debug
        // code can compare CPU-view vs memory-view (peek_rdes after invalidate).
        // If snapshot != peek, the cache writeback path is broken.
        #[cfg(target_arch = "riscv32")]
        {
            let d = &self.descriptors[0];
            CPU_DESC0_SNAPSHOT[0].store(d.rdes0.get(), core::sync::atomic::Ordering::Relaxed);
            CPU_DESC0_SNAPSHOT[1].store(d.rdes1.get(), core::sync::atomic::Ordering::Relaxed);
            CPU_DESC0_SNAPSHOT[2].store(d.buf_addr.get(), core::sync::atomic::Ordering::Relaxed);
            CPU_DESC0_SNAPSHOT[3].store(d.next_desc.get(), core::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Returns the descriptor base pointer (used by the DMA programming step).
    pub fn descriptors_ptr(&self) -> *const RDes {
        self.descriptors.as_ptr()
    }

    /// Pushes every descriptor's CPU-side state out to DMA-visible memory.
    /// See `TDesRing::flush_all` for the cold-boot rationale.
    pub fn flush_all(&self) {
        for desc in self.descriptors.iter() {
            Dma::flush_descriptor(desc);
        }
    }

    /// Returns the current received frame when DMA has completed one.
    pub fn receive(&mut self) -> Option<(usize, &[u8])> {
        let desc = &self.descriptors[self.index];
        Dma::invalidate_descriptor(desc);
        if desc.owned_by() != OwnedBy::Cpu {
            return None;
        }

        let status = desc.rdes0.get();
        RX_LAST_RDES0.store(status, Ordering::Relaxed);
        if status & RDES0_ES != 0 || !desc.is_complete_frame() {
            self.stats.error_frames = self.stats.error_frames.saturating_add(1);
            RX_ERROR_FRAMES_TOTAL.fetch_add(1, Ordering::Relaxed);
            self.recycle_current();
            return None;
        }

        let len = desc.frame_len();
        if len != 0 && len < MIN_RX_FRAME_SIZE {
            self.stats.runt_frames = self.stats.runt_frames.saturating_add(1);
            RX_RUNT_FRAMES_TOTAL.fetch_add(1, Ordering::Relaxed);
            self.recycle_current();
            return None;
        }

        if len > BUF_SIZE {
            self.stats.oversized_frames = self.stats.oversized_frames.saturating_add(1);
            RX_OVERSIZED_FRAMES_TOTAL.fetch_add(1, Ordering::Relaxed);
            self.recycle_current();
            return None;
        }

        Dma::invalidate_buffer_prefix(&self.buffers[self.index], len);
        fence(Ordering::Acquire);
        self.stats.received_frames = self.stats.received_frames.saturating_add(1);
        RX_LAST_FRAME_LEN.store(len as u32, Ordering::Relaxed);
        if len >= 200 {
            RX_LARGE_FRAMES.fetch_add(1, Ordering::Relaxed);
        }
        Some((len, &self.buffers[self.index][..len]))
    }

    /// Releases the current RX descriptor back to DMA ownership.
    pub fn pop(&mut self) {
        self.recycle_current();
    }

    fn recycle_current(&mut self) {
        let desc = &self.descriptors[self.index];
        desc.rdes0.set(RDES0_OWN);
        Dma::flush_descriptor(desc);
        self.index = (self.index + 1) % self.descriptors.len();
    }

    /// Returns `true` when the current RX descriptor contains a CPU-owned frame.
    pub fn has_packet(&self) -> bool {
        let desc = &self.descriptors[self.index];
        Dma::invalidate_descriptor(desc);
        desc.owned_by() == OwnedBy::Cpu
    }

    /// Reinitializes the RX ring after DMA overflow or RX stopped conditions.
    pub fn handle_overflow(&mut self) {
        self.stats.overflow_resets = self.stats.overflow_resets.saturating_add(1);
        self.reset();
        Dma::demand_rx_poll();
    }

    /// Returns a snapshot of cumulative RX statistics.
    pub fn stats(&self) -> RxRingStats {
        self.stats
    }

    /// Test/fuzz helper that seeds the current RX descriptor with synthetic data.
    #[cfg(any(test, feature = "fuzzing"))]
    #[doc(hidden)]
    pub fn fuzz_seed_current(&mut self, status: u32, owner: OwnedBy, payload: &[u8]) {
        let index = self.index;
        let used = payload.len().min(BUF_SIZE);
        self.buffers[index][..used].copy_from_slice(&payload[..used]);
        self.descriptors[index].rdes0.set(status);
        self.descriptors[index].set_owned_by(owner);
    }

    /// Test/fuzz helper that returns the current ring cursor.
    #[cfg(any(test, feature = "fuzzing"))]
    #[doc(hidden)]
    pub fn fuzz_current_index(&self) -> usize {
        self.index
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STRESS_ITERATIONS: usize = 10_000;
    const FUZZ_ITERATIONS: usize = 4_096;

    fn tx_fixture() -> ([TDes; TX_DESC_COUNT], [[u8; BUF_SIZE]; TX_DESC_COUNT]) {
        (zeroed_tx_descriptors(), [[0; BUF_SIZE]; TX_DESC_COUNT])
    }

    fn rx_fixture() -> ([RDes; RX_DESC_COUNT], [[u8; BUF_SIZE]; RX_DESC_COUNT]) {
        (zeroed_rx_descriptors(), [[0; BUF_SIZE]; RX_DESC_COUNT])
    }

    fn advance_lcg(state: &mut u32) -> u32 {
        *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *state
    }

    #[test]
    fn tx_reset_builds_closed_ring() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let ring = TDesRing::new(&mut descriptors, &mut buffers);

        assert_eq!(ring.index, 0);
        assert_eq!(
            ring.descriptors[0].next_desc.get(),
            (&ring.descriptors[1] as *const TDes as usize) as u32
        );
        assert_eq!(
            ring.descriptors[TX_DESC_COUNT - 1].next_desc.get(),
            (&ring.descriptors[0] as *const TDes as usize) as u32
        );
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Cpu);
    }

    #[test]
    fn tx_transmit_copies_payload_and_advances() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);
        let payload = [0xAB; 64];

        ring.transmit(&payload).unwrap();

        assert_eq!(&ring.buffers[0][..payload.len()], &payload);
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Dma);
        assert_eq!(ring.index, 1);
        assert_eq!(
            ring.descriptors[0].tdes0.get(),
            TDES0_OWN | TDES0_FS | TDES0_LS | TDES0_IC | TDES0_CHAINED
        );
        assert_eq!(ring.descriptors[0].tdes1.get(), payload.len() as u32);
    }

    #[test]
    fn tx_transmit_keeps_descriptor_chained() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);

        ring.transmit(&[0xAB; 64]).unwrap();

        assert_ne!(ring.descriptors[0].tdes0.get() & TDES0_CHAINED, 0);
    }

    #[test]
    fn tx_transmit_fails_when_ring_is_full() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);

        for descriptor in ring.descriptors.iter() {
            descriptor.set_owned_by(OwnedBy::Dma);
        }

        let err = ring.transmit(&[1, 2, 3]).unwrap_err();
        assert_eq!(err, DescriptorError::RingFull);
        assert_eq!(ring.stats().ring_full_events, 1);
    }

    #[test]
    fn tx_transmit_rejects_oversized_frame_without_touching_ring() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);
        let payload = [0xA5; BUF_SIZE + 1];

        let err = ring.transmit(&payload).unwrap_err();

        assert_eq!(
            err,
            DescriptorError::BufferTooLarge(BufferTooLarge::Frame {
                len: BUF_SIZE + 1,
                max: BUF_SIZE,
            })
        );
        assert_eq!(ring.index, 0);
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Cpu);
        assert_eq!(ring.stats(), TxRingStats::default());
        assert!(ring.buffers[0].iter().all(|byte| *byte == 0));
    }

    #[test]
    fn tx_ring_full_does_not_advance_or_copy_payload() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);

        ring.descriptors[0].set_owned_by(OwnedBy::Dma);

        assert_eq!(ring.transmit(&[0x5A; 64]), Err(DescriptorError::RingFull));
        assert_eq!(ring.index, 0);
        assert!(ring.buffers[0][..64].iter().all(|byte| *byte == 0));
        assert_eq!(ring.stats().transmitted_frames, 0);
    }

    #[test]
    fn tx_has_capacity_tracks_current_descriptor_owner() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let ring = TDesRing::new(&mut descriptors, &mut buffers);

        assert!(ring.has_capacity());

        ring.descriptors[0].set_owned_by(OwnedBy::Dma);
        assert!(!ring.has_capacity());

        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);
        assert!(ring.has_capacity());
    }

    #[test]
    fn tx_transmit_wraps_index_after_last_descriptor() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);

        for expected_index in 0..TX_DESC_COUNT {
            ring.descriptors[expected_index].set_owned_by(OwnedBy::Cpu);
            ring.transmit(&[expected_index as u8]).unwrap();
        }

        assert_eq!(ring.index, 0);
    }

    #[test]
    fn tx_transmit_accepts_min_and_max_frame_sizes() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);

        ring.transmit(&[]).unwrap();
        ring.descriptors[1].set_owned_by(OwnedBy::Cpu);

        let payload = [0x5A; BUF_SIZE];
        ring.transmit(&payload).unwrap();

        assert_eq!(ring.descriptors[0].tdes1.get(), 0);
        assert_eq!(ring.descriptors[1].tdes1.get(), BUF_SIZE as u32);
        assert_eq!(&ring.buffers[1], &payload);
        assert_eq!(ring.stats().transmitted_frames, 2);
    }

    #[test]
    fn tx_transmit_recovers_after_descriptor_returns_to_cpu() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);

        for descriptor in ring.descriptors.iter() {
            descriptor.set_owned_by(OwnedBy::Dma);
        }

        assert_eq!(ring.transmit(&[0xA5; 64]), Err(DescriptorError::RingFull));

        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);
        ring.transmit(&[0x5A; 64]).unwrap();

        assert_eq!(ring.index, 1);
        assert_eq!(ring.stats().ring_full_events, 1);
        assert_eq!(ring.stats().transmitted_frames, 1);
    }

    #[test]
    fn tx_transmit_survives_ten_thousand_cycles() {
        let (mut descriptors, mut buffers) = tx_fixture();
        let mut ring = TDesRing::new(&mut descriptors, &mut buffers);
        let mut payload = [0u8; BUF_SIZE];

        for cycle in 0..STRESS_ITERATIONS {
            let slot = ring.index;
            let len = if cycle % 17 == 0 {
                0
            } else {
                MIN_RX_FRAME_SIZE + (cycle % 257)
            };

            for (offset, byte) in payload[..len].iter_mut().enumerate() {
                *byte = cycle.wrapping_add(offset) as u8;
            }

            ring.transmit(&payload[..len]).unwrap();

            assert_eq!(ring.descriptors[slot].tdes1.get(), len as u32);
            assert_eq!(&ring.buffers[slot][..len], &payload[..len]);

            ring.descriptors[slot].set_owned_by(OwnedBy::Cpu);
        }

        assert_eq!(ring.index, STRESS_ITERATIONS % TX_DESC_COUNT);
        assert_eq!(ring.stats().transmitted_frames, STRESS_ITERATIONS as u32);
        assert_eq!(ring.stats().ring_full_events, 0);
    }

    #[test]
    fn rx_receive_reads_frame_and_pop_returns_it_to_dma() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);
        let payload = [0x11; MIN_RX_FRAME_SIZE];

        ring.buffers[0][..payload.len()].copy_from_slice(&payload);
        ring.descriptors[0]
            .rdes0
            .set(RDES0_FS | RDES0_LS | ((payload.len() as u32) << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        {
            let (len, packet) = ring.receive().unwrap();
            assert_eq!(len, payload.len());
            assert_eq!(packet, &payload);
        }

        ring.pop();
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Dma);
        assert_eq!(ring.index, 1);
        assert_eq!(ring.stats().received_frames, 1);
    }

    #[test]
    fn rx_receive_valid_frame_waits_for_explicit_pop() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);
        let payload = [0x22; MIN_RX_FRAME_SIZE];

        ring.buffers[0][..payload.len()].copy_from_slice(&payload);
        ring.descriptors[0]
            .rdes0
            .set(RDES0_FS | RDES0_LS | ((payload.len() as u32) << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        let (len, frame) = ring.receive().unwrap();

        assert_eq!(len, payload.len());
        assert_eq!(frame, &payload);
        assert_eq!(ring.index, 0);
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Cpu);
    }

    #[test]
    fn rx_receive_returns_none_for_dma_owned_descriptor() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        assert!(ring.receive().is_none());
    }

    #[test]
    fn rx_has_packet_tracks_current_descriptor_owner() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let ring = RDesRing::new(&mut descriptors, &mut buffers);

        assert!(!ring.has_packet());

        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);
        assert!(ring.has_packet());

        ring.descriptors[0].set_owned_by(OwnedBy::Dma);
        assert!(!ring.has_packet());
    }

    #[test]
    fn rx_receive_wraps_index_after_pop() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.index = RX_DESC_COUNT - 1;
        ring.descriptors[RX_DESC_COUNT - 1]
            .rdes0
            .set(RDES0_FS | RDES0_LS);
        ring.descriptors[RX_DESC_COUNT - 1].set_owned_by(OwnedBy::Cpu);

        let _ = ring.receive().unwrap();
        ring.pop();

        assert_eq!(ring.index, 0);
    }

    #[test]
    fn rx_receive_skips_error_frame_and_recycles_descriptor() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.descriptors[0]
            .rdes0
            .set(RDES0_ES | RDES0_FS | RDES0_LS | (32 << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        assert!(ring.receive().is_none());
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Dma);
        assert_eq!(ring.index, 1);
        assert_eq!(ring.stats().error_frames, 1);
    }

    #[test]
    fn rx_receive_recycles_incomplete_frame() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.descriptors[0]
            .rdes0
            .set(RDES0_FS | ((MIN_RX_FRAME_SIZE as u32) << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        assert!(ring.receive().is_none());
        assert_eq!(ring.index, 1);
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Dma);
        assert_eq!(ring.stats().error_frames, 1);
    }

    #[test]
    fn rx_receive_handles_zero_length_frame() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.descriptors[0].rdes0.set(RDES0_FS | RDES0_LS);
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        let (len, packet) = ring.receive().unwrap();
        assert_eq!(len, 0);
        assert!(packet.is_empty());
    }

    #[test]
    fn rx_receive_accepts_max_buffer_sized_frame() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);
        let payload = [0x7E; BUF_SIZE];

        ring.buffers[0].copy_from_slice(&payload);
        ring.descriptors[0]
            .rdes0
            .set(RDES0_FS | RDES0_LS | ((BUF_SIZE as u32) << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        let (len, packet) = ring.receive().unwrap();

        assert_eq!(len, BUF_SIZE);
        assert_eq!(packet, &payload);
    }

    #[test]
    fn rx_error_flag_takes_precedence_over_runt_classification() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.descriptors[0].rdes0.set(
            RDES0_ES | RDES0_FS | RDES0_LS | (((MIN_RX_FRAME_SIZE - 1) as u32) << RDES0_FL_SHIFT),
        );
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        assert!(ring.receive().is_none());
        assert_eq!(ring.stats().error_frames, 1);
        assert_eq!(ring.stats().runt_frames, 0);
        assert_eq!(ring.index, 1);
    }

    #[test]
    fn rx_receive_handles_consecutive_frames() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);
        let payload0 = [0xAA; MIN_RX_FRAME_SIZE];
        let payload1 = [0xCC; 96];

        ring.buffers[0][..payload0.len()].copy_from_slice(&payload0);
        ring.buffers[1][..payload1.len()].copy_from_slice(&payload1);
        ring.descriptors[0]
            .rdes0
            .set(RDES0_FS | RDES0_LS | ((payload0.len() as u32) << RDES0_FL_SHIFT));
        ring.descriptors[1]
            .rdes0
            .set(RDES0_FS | RDES0_LS | ((payload1.len() as u32) << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);
        ring.descriptors[1].set_owned_by(OwnedBy::Cpu);

        let (len0, frame0) = ring.receive().unwrap();
        assert_eq!(len0, payload0.len());
        assert_eq!(frame0, &payload0);
        ring.pop();

        let (len1, frame1) = ring.receive().unwrap();
        assert_eq!(len1, payload1.len());
        assert_eq!(frame1, &payload1);
        ring.pop();

        assert_eq!(ring.index, 2);
        assert_eq!(ring.stats().received_frames, 2);
    }

    #[test]
    fn rx_receive_drops_runt_frame_and_counts_it() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.descriptors[0]
            .rdes0
            .set(RDES0_FS | RDES0_LS | (((MIN_RX_FRAME_SIZE - 1) as u32) << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        assert!(ring.receive().is_none());
        assert_eq!(ring.index, 1);
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Dma);
        assert_eq!(ring.stats().runt_frames, 1);
    }

    #[test]
    fn rx_receive_drops_oversized_frame_without_touching_buffer() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.descriptors[0]
            .rdes0
            .set(RDES0_FS | RDES0_LS | (((BUF_SIZE + 1) as u32) << RDES0_FL_SHIFT));
        ring.descriptors[0].set_owned_by(OwnedBy::Cpu);

        assert!(ring.receive().is_none());
        assert_eq!(ring.index, 1);
        assert_eq!(ring.descriptors[0].owned_by(), OwnedBy::Dma);
        assert_eq!(ring.stats().oversized_frames, 1);
    }

    #[test]
    fn rx_handle_overflow_rebuilds_ring_and_restarts_from_zero() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

        ring.index = 2;
        ring.descriptors[2].set_owned_by(OwnedBy::Cpu);
        ring.descriptors[2].rdes0.set(0);

        ring.handle_overflow();

        assert_eq!(ring.index, 0);
        assert_eq!(ring.stats().overflow_resets, 1);
        for descriptor in ring.descriptors.iter() {
            assert_eq!(descriptor.owned_by(), OwnedBy::Dma);
            assert_eq!(
                descriptor.rdes1.get() & RDES1_BUF1_SIZE_MASK,
                BUF_SIZE as u32
            );
            assert_ne!(descriptor.rdes1.get() & RDES1_CHAINED, 0);
        }
    }

    #[test]
    fn rx_handle_overflow_preserves_ring_invariants_under_repetition() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);
        let mut rng = 0x51F1_AA77u32;

        for expected_resets in 1..=FUZZ_ITERATIONS {
            let random = advance_lcg(&mut rng) as usize;
            ring.index = random % RX_DESC_COUNT;

            for (slot, descriptor) in ring.descriptors.iter().enumerate() {
                descriptor.set_owned_by(if (random + slot) & 1 == 0 {
                    OwnedBy::Cpu
                } else {
                    OwnedBy::Dma
                });
                descriptor
                    .rdes0
                    .set(((random + slot) as u32) << RDES0_FL_SHIFT);
            }

            ring.handle_overflow();

            assert_eq!(ring.index, 0);
            assert_eq!(ring.stats().overflow_resets, expected_resets as u32);
            assert_eq!(regs::read(regs::dma::RX_POLL_DEMAND), 1);

            for slot in 0..RX_DESC_COUNT {
                let descriptor = &ring.descriptors[slot];
                assert_eq!(descriptor.owned_by(), OwnedBy::Dma);
                assert_eq!(
                    descriptor.rdes1.get() & RDES1_BUF1_SIZE_MASK,
                    BUF_SIZE as u32
                );
                assert_ne!(descriptor.rdes1.get() & RDES1_CHAINED, 0);
                let expected_next = if slot + 1 == RX_DESC_COUNT {
                    &ring.descriptors[0] as *const RDes as usize as u32
                } else {
                    &ring.descriptors[slot + 1] as *const RDes as usize as u32
                };
                assert_eq!(descriptor.next_desc.get(), expected_next);
            }
        }
    }

    #[test]
    fn rx_receive_survives_ten_thousand_cycles() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);
        let mut payload = [0u8; BUF_SIZE];

        for cycle in 0..STRESS_ITERATIONS {
            let slot = ring.index;
            let len = MIN_RX_FRAME_SIZE + (cycle % 321);

            for (offset, byte) in payload[..len].iter_mut().enumerate() {
                *byte = cycle.wrapping_mul(3).wrapping_add(offset) as u8;
            }

            ring.buffers[slot][..len].copy_from_slice(&payload[..len]);
            ring.descriptors[slot]
                .rdes0
                .set(RDES0_FS | RDES0_LS | ((len as u32) << RDES0_FL_SHIFT));
            ring.descriptors[slot].set_owned_by(OwnedBy::Cpu);

            let (received_len, frame) = ring.receive().unwrap();
            assert_eq!(received_len, len);
            assert_eq!(frame, &payload[..len]);

            ring.pop();
            assert_eq!(ring.descriptors[slot].owned_by(), OwnedBy::Dma);
        }

        assert_eq!(ring.index, STRESS_ITERATIONS % RX_DESC_COUNT);
        assert_eq!(ring.stats().received_frames, STRESS_ITERATIONS as u32);
        assert_eq!(ring.stats().error_frames, 0);
        assert_eq!(ring.stats().runt_frames, 0);
        assert_eq!(ring.stats().oversized_frames, 0);
    }

    #[test]
    fn rx_receive_fuzz_preserves_ring_invariants() {
        let (mut descriptors, mut buffers) = rx_fixture();
        let mut ring = RDesRing::new(&mut descriptors, &mut buffers);
        let mut rng = 0xC0DE_1234u32;

        let mut expected_received = 0u32;
        let mut expected_errors = 0u32;
        let mut expected_runts = 0u32;
        let mut expected_oversized = 0u32;

        for _ in 0..FUZZ_ITERATIONS {
            let slot = ring.index;
            let random = advance_lcg(&mut rng);
            let len = (random as usize) % (BUF_SIZE + 256);
            let owner_dma = random & 0x1 != 0;
            let set_error = random & 0x2 != 0;
            let set_fs = random & 0x4 != 0;
            let set_ls = random & 0x8 != 0;

            if !owner_dma {
                let fill_len = len.min(BUF_SIZE);
                for (offset, byte) in ring.buffers[slot][..fill_len].iter_mut().enumerate() {
                    *byte = random.wrapping_add(offset as u32) as u8;
                }
            }

            let mut status = ((len as u32) << RDES0_FL_SHIFT) & RDES0_FL_MASK;
            if set_error {
                status |= RDES0_ES;
            }
            if set_fs {
                status |= RDES0_FS;
            }
            if set_ls {
                status |= RDES0_LS;
            }

            ring.descriptors[slot].rdes0.set(status);
            ring.descriptors[slot].set_owned_by(if owner_dma {
                OwnedBy::Dma
            } else {
                OwnedBy::Cpu
            });

            let expected = if owner_dma {
                None
            } else if set_error || !(set_fs && set_ls) {
                expected_errors = expected_errors.saturating_add(1);
                Some(("error", None))
            } else if len != 0 && len < MIN_RX_FRAME_SIZE {
                expected_runts = expected_runts.saturating_add(1);
                Some(("runt", None))
            } else if len > BUF_SIZE {
                expected_oversized = expected_oversized.saturating_add(1);
                Some(("oversized", None))
            } else {
                expected_received = expected_received.saturating_add(1);
                Some(("ok", Some(len)))
            };

            let start_index = ring.index;
            match (expected, ring.receive()) {
                (None, None) => {
                    assert_eq!(ring.index, start_index);
                    assert_eq!(ring.descriptors[slot].owned_by(), OwnedBy::Dma);
                }
                (Some((_kind, None)), None) => {
                    assert_eq!(ring.index, (start_index + 1) % RX_DESC_COUNT);
                    let recycled_slot = start_index;
                    assert_eq!(ring.descriptors[recycled_slot].owned_by(), OwnedBy::Dma);
                }
                (Some(("ok", Some(expected_len))), Some((len, frame))) => {
                    assert_eq!(len, expected_len);
                    assert_eq!(frame.len(), expected_len);
                    assert_eq!(ring.index, start_index);
                    ring.pop();
                    assert_eq!(ring.index, (start_index + 1) % RX_DESC_COUNT);
                }
                other => panic!("unexpected fuzz outcome: {other:?}"),
            }

            let stats = ring.stats();
            assert_eq!(stats.received_frames, expected_received);
            assert_eq!(stats.error_frames, expected_errors);
            assert_eq!(stats.runt_frames, expected_runts);
            assert_eq!(stats.oversized_frames, expected_oversized);
            assert!(ring.index < RX_DESC_COUNT);
        }
    }

    #[test]
    fn descriptor_layout_matches_checklist() {
        assert_eq!(size_of::<TDes>(), 128);
        assert_eq!(align_of::<TDes>(), 128);
        assert_eq!(size_of::<RDes>(), 128);
        assert_eq!(align_of::<RDes>(), 128);
        assert_eq!(offset_of!(TDes, tdes0), 0);
        assert_eq!(offset_of!(TDes, tdes1), 4);
        assert_eq!(offset_of!(TDes, buf_addr), 8);
        assert_eq!(offset_of!(TDes, next_desc), 12);
        assert_eq!(offset_of!(RDes, rdes0), 0);
        assert_eq!(offset_of!(RDes, rdes1), 4);
        assert_eq!(offset_of!(RDes, buf_addr), 8);
        assert_eq!(offset_of!(RDes, next_desc), 12);
    }

    #[test]
    fn descriptor_bit_constants_match_idf_layout() {
        assert_eq!(TDES0_OWN, 1 << 31);
        assert_eq!(TDES0_IC, 1 << 30);
        assert_eq!(TDES0_LS, 1 << 29);
        assert_eq!(TDES0_FS, 1 << 28);
        assert_eq!(RDES0_OWN, 1 << 31);
        assert_eq!(RDES0_ES, 1 << 15);
        assert_eq!(RDES0_FS, 1 << 9);
        assert_eq!(RDES0_LS, 1 << 8);
        assert_eq!(RDES0_FL_MASK, 0x3fff << 16);
        assert_eq!(RDES1_CHAINED, 1 << 14);
        assert_eq!(RDES1_BUF1_SIZE_MASK, 0x1fff);
    }

    #[test]
    fn dma_buffer_wrapper_is_cacheline_aligned() {
        let buffers = DmaBuffers::<TX_DESC_COUNT>::ZERO;

        assert_eq!(align_of::<DmaBuffers<TX_DESC_COUNT>>(), 128);
        assert_eq!((buffers.inner.as_ptr() as usize) % 128, 0);
    }

    #[test]
    fn static_dma_resources_split_exposes_all_zeroed_dma_storage() {
        let mut resources = StaticDmaResources::new();
        let (tx_desc, rx_desc, tx_buf, rx_buf) = resources.split();

        assert_eq!(tx_desc.len(), TX_DESC_COUNT);
        assert_eq!(rx_desc.len(), RX_DESC_COUNT);
        assert_eq!(tx_buf.len(), TX_DESC_COUNT);
        assert_eq!(rx_buf.len(), RX_DESC_COUNT);
        assert!(tx_desc.iter().all(|desc| desc.owned_by() == OwnedBy::Cpu));
        assert!(rx_desc.iter().all(|desc| desc.owned_by() == OwnedBy::Cpu));
        assert!(tx_buf.iter().flatten().all(|byte| *byte == 0));
        assert!(rx_buf.iter().flatten().all(|byte| *byte == 0));
    }
}
