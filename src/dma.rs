//! DMA helpers for descriptor ownership, cache synchronization and interrupt status.
//!
//! ESP32-P4 uses a write-back data cache, so DMA-visible descriptor and packet
//! memory must be explicitly flushed before ownership is handed to hardware and
//! invalidated before the CPU reads data produced by DMA.

#[cfg(target_arch = "riscv32")]
use core::sync::atomic::{compiler_fence, fence, Ordering};

use crate::{descriptors::BUF_SIZE, regs, RDes, TDes};

/// Cache line granularity used to compute aligned ranges for ROM cache ops.
/// Set to the wider of the two P4 cache lines (L2 = 128 byte), so a single
/// `Cache_Invalidate_Addr` never operates on a partial L2 line that happens
/// to share storage with an unrelated descriptor / buffer.
const CACHE_LINE_SIZE: usize = 128;
const CACHE_LINE_MASK: usize = CACHE_LINE_SIZE - 1;

// Cache map bits per ESP32-P4 ROM cache.h. The ROM helpers must be invoked
// ONCE PER LEVEL — passing a combined `L1|L2` bitmask returns success (0) but
// silently no-ops in practice, leaving descriptors invisible to DMA. IDF's
// `cache_ll_l2_writeback_cache_addr` calls L2 only for HP SRAM; we tried
// L2-only here on 2026-04-28 and it broke the data plane (DMA read garbage,
// `regs::read(MAC.CONFIG)` after `start()` hung the AHB), so on this boot
// path (espflash --ram --no-stub) HP SRAM also has live L1 D-cache state
// that needs flushing. Both maps are kept.
#[cfg(target_arch = "riscv32")]
const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
#[cfg(target_arch = "riscv32")]
const CACHE_MAP_L2_CACHE: u32 = 1 << 5;

// ROM-resident cache helpers exported by the ESP32-P4 boot ROM. Addresses come from
// `components/esp_rom/esp32p4/ld/esp32p4.rom.ld` in ESP-IDF. Signature is
// `int fn(uint32_t map, uint32_t addr, uint32_t size)` returning 0 on success and 1 if the
// address is outside the cacheable address room (in which case the ROM silently does nothing).
//
// We prefer the ROM helpers over driving the cache MMIO directly because they take care of
// autoload suspend, line alignment and atomic L1/L2 ordering on our behalf.
#[cfg(target_arch = "riscv32")]
type RomCacheOp = unsafe extern "C" fn(map: u32, addr: u32, size: u32) -> i32;
#[cfg(target_arch = "riscv32")]
const ROM_CACHE_INVALIDATE_ADDR: usize = 0x4FC0_03E4;
#[cfg(target_arch = "riscv32")]
const ROM_CACHE_INVALIDATE_ALL: usize = 0x4FC0_0404;
#[cfg(target_arch = "riscv32")]
const ROM_CACHE_WRITEBACK_ALL: usize = 0x4FC0_0414;
#[cfg(target_arch = "riscv32")]
type RomCacheAll = unsafe extern "C" fn(map: u32) -> i32;

// `Cache_Set_L2_Cache_Mode(size, ways, line_size)` — must be called once
// before any other L2 cache ROM helper after a CPU reset, because the L2
// controller's internal mode/ways/line-size shadow registers are NOT
// re-initialised by a warm reset. IDF runs this from `cache_hal_init`
// in the second-stage bootloader; our `--ram --no-stub` boot path skips
// it, which is the strongest theory for the intermittent
// `Cache_WriteBack_All` hang observed 2026-04-28.
#[cfg(target_arch = "riscv32")]
const ROM_CACHE_SET_L2_CACHE_MODE: usize = 0x4FC0_03D4;
#[cfg(target_arch = "riscv32")]
type RomCacheSetL2Mode = unsafe extern "C" fn(size: u32, ways: u32, line_size: u32);

// Enum values from `components/esp_rom/esp32p4/include/esp32p4/rom/cache.h`.
// IDF default sdkconfig for ESP32-P4: 256 KB / 8-way / 64-byte line.
#[cfg(target_arch = "riscv32")]
const CACHE_SIZE_256K: u32 = 10;
#[cfg(target_arch = "riscv32")]
const CACHE_8WAYS_ASSOC: u32 = 2;
#[cfg(target_arch = "riscv32")]
const CACHE_LINE_SIZE_64B: u32 = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct CacheAlignedRange {
    start: usize,
    size: usize,
}

const DMA_RESET_TIMEOUT_POLLS: usize = 1_000;
const DMA_BURST_LENGTH: u32 = 8;

/// DMA-level errors surfaced during initialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DmaError {
    /// Software reset bit did not clear before the polling timeout expired.
    ResetTimeout,
}

/// Decoded snapshot of the DMA status register.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DmaInterruptStatus {
    raw: u32,
}

impl DmaInterruptStatus {
    /// Creates a status view from the raw DMA status register value.
    pub const fn from_raw(raw: u32) -> Self {
        Self { raw }
    }

    /// Returns the original register value.
    pub const fn raw(self) -> u32 {
        self.raw
    }

    /// Returns `true` when a transmit completion interrupt is pending.
    pub const fn has_tx_interrupt(self) -> bool {
        self.raw & regs::bits::dmastatus::TI != 0
    }

    /// Returns `true` when TX ran out of descriptors or buffers.
    pub const fn has_tx_buffer_unavailable(self) -> bool {
        self.raw & regs::bits::dmastatus::TU != 0
    }

    /// Returns `true` when the TX DMA state machine entered the stopped state.
    pub const fn has_tx_process_stopped(self) -> bool {
        self.raw & regs::bits::dmastatus::TPS != 0
    }

    /// Returns `true` when the MAC reported a transmit jabber timeout.
    pub const fn has_tx_jabber_timeout(self) -> bool {
        self.raw & regs::bits::dmastatus::TJT != 0
    }

    /// Returns `true` when RX FIFO overflow was reported.
    pub const fn has_rx_overflow(self) -> bool {
        self.raw & regs::bits::dmastatus::OVF != 0
    }

    /// Returns `true` when TX underflow was reported.
    pub const fn has_tx_underflow(self) -> bool {
        self.raw & regs::bits::dmastatus::UNF != 0
    }

    /// Returns `true` when a receive completion interrupt is pending.
    pub const fn has_rx_interrupt(self) -> bool {
        self.raw & regs::bits::dmastatus::RI != 0
    }

    /// Returns `true` when RX ran out of available descriptors or buffers.
    pub const fn has_rx_buffer_unavailable(self) -> bool {
        self.raw & regs::bits::dmastatus::RU != 0
    }

    /// Returns `true` when the RX DMA state machine entered the stopped state.
    pub const fn has_rx_process_stopped(self) -> bool {
        self.raw & regs::bits::dmastatus::RPS != 0
    }

    /// Returns `true` when RX watchdog timeout was reported.
    pub const fn has_rx_watchdog_timeout(self) -> bool {
        self.raw & regs::bits::dmastatus::RWT != 0
    }

    /// Returns `true` when an early-transmit interrupt is pending.
    pub const fn has_early_transmit_interrupt(self) -> bool {
        self.raw & regs::bits::dmastatus::ETI != 0
    }

    /// Returns `true` when a fatal bus error was reported.
    pub const fn has_fatal_bus_error(self) -> bool {
        self.raw & regs::bits::dmastatus::FBI != 0
    }

    /// Returns `true` when an early-receive interrupt is pending.
    pub const fn has_early_receive_interrupt(self) -> bool {
        self.raw & regs::bits::dmastatus::ERI != 0
    }

    /// Returns `true` when the normal interrupt summary bit is set.
    pub const fn is_normal_summary(self) -> bool {
        self.raw & regs::bits::dmastatus::NIS != 0
    }

    /// Returns `true` when the abnormal interrupt summary bit is set.
    pub const fn is_abnormal_summary(self) -> bool {
        self.raw & regs::bits::dmastatus::AIS != 0
    }

    /// Returns the encoded DMA bus error subtype.
    pub const fn error_bits(self) -> u32 {
        (self.raw & regs::bits::dmastatus::EB_MASK) >> regs::bits::dmastatus::EB_SHIFT
    }

    /// Returns `true` when the RX path should be nudged after handling this status.
    pub const fn should_kick_rx(self) -> bool {
        self.has_rx_interrupt()
            || self.has_early_receive_interrupt()
            || self.has_rx_overflow()
            || self.has_rx_buffer_unavailable()
            || self.has_rx_process_stopped()
            || self.has_rx_watchdog_timeout()
    }

    /// Returns `true` when the TX path should be nudged after handling this status.
    pub const fn should_kick_tx(self) -> bool {
        self.has_tx_interrupt()
            || self.has_tx_buffer_unavailable()
            || self.has_tx_underflow()
            || self.has_tx_process_stopped()
            || self.has_tx_jabber_timeout()
            || self.has_early_transmit_interrupt()
    }

    /// Returns the sticky status bits that must be written back to clear this snapshot.
    pub const fn clear_mask(self) -> u32 {
        self.raw
            & (regs::bits::dmastatus::TI
                | regs::bits::dmastatus::TPS
                | regs::bits::dmastatus::TU
                | regs::bits::dmastatus::TJT
                | regs::bits::dmastatus::OVF
                | regs::bits::dmastatus::UNF
                | regs::bits::dmastatus::RI
                | regs::bits::dmastatus::RU
                | regs::bits::dmastatus::RPS
                | regs::bits::dmastatus::RWT
                | regs::bits::dmastatus::ETI
                | regs::bits::dmastatus::FBI
                | regs::bits::dmastatus::ERI
                | regs::bits::dmastatus::AIS
                | regs::bits::dmastatus::NIS)
    }
}

/// Safe entry points for cache sync, DMA setup and interrupt bookkeeping.
pub struct Dma;

impl Dma {
    /// Resets the DMA engine and waits until the reset bit self-clears.
    pub fn dma_reset() -> Result<(), DmaError> {
        regs::write(regs::dma::BUS_MODE, regs::bits::dmabusmode::SWR);
        #[cfg(test)]
        regs::write(regs::dma::BUS_MODE, 0);
        wait_for_dma_reset_clear(|| regs::read(regs::dma::BUS_MODE))
    }

    /// Applies baseline DMA bus mode, op mode and interrupt mask configuration.
    pub fn dma_init() {
        regs::write(regs::dma::BUS_MODE, dma_bus_mode_value(DMA_BURST_LENGTH));
        regs::write(regs::dma::OP_MODE, dma_op_mode_value());
        regs::write(regs::dma::INT_EN, dma_interrupt_enable_value());
    }

    /// Programs the TX and RX descriptor base addresses.
    pub fn set_descriptor_lists(tx_descriptors: &[TDes], rx_descriptors: &[RDes]) {
        regs::write(
            regs::dma::TX_DESC_LIST,
            tx_descriptors.as_ptr() as usize as u32,
        );
        regs::write(
            regs::dma::RX_DESC_LIST,
            rx_descriptors.as_ptr() as usize as u32,
        );
    }

    /// Nudges the DMA engine to resume transmit descriptor processing.
    pub fn demand_tx_poll() {
        regs::write(regs::dma::TX_POLL_DEMAND, 1);
    }

    /// Nudges the DMA engine to resume receive descriptor processing.
    pub fn demand_rx_poll() {
        regs::write(regs::dma::RX_POLL_DEMAND, 1);
    }

    /// Reads the current DMA interrupt/status register.
    pub fn read_interrupt_status() -> DmaInterruptStatus {
        DmaInterruptStatus::from_raw(regs::read(regs::dma::STATUS))
    }

    /// Clears all sticky interrupt bits present in `status`.
    pub fn clear_interrupt_status(status: DmaInterruptStatus) {
        let clear_mask = status.clear_mask();
        if clear_mask != 0 {
            regs::write(regs::dma::STATUS, clear_mask);
        }
    }

    /// Disables all DMA interrupt sources.
    pub fn disable_interrupts() {
        regs::write(regs::dma::INT_EN, 0);
    }

    /// Invalidates cache lines covering one descriptor object.
    pub fn invalidate_descriptor<T>(descriptor: &T) {
        // SAFETY: `descriptor` is a live shared reference, so the pointed-to memory is valid
        // for reads for `size_of::<T>()` bytes. The cache operation does not create aliases or
        // outlive the reference; it only synchronizes cache lines covering this object.
        unsafe {
            Self::cache_invalidate(
                descriptor as *const T as *const u8,
                core::mem::size_of::<T>(),
            );
        }
    }

    /// Invalidates the used prefix of a DMA packet buffer.
    pub fn invalidate_buffer_prefix(buffer: &[u8; BUF_SIZE], used: usize) {
        let used = used.min(BUF_SIZE);
        // SAFETY: `buffer` is a live reference to a fixed-size DMA buffer. Invalidating at most
        // `BUF_SIZE` bytes starting from `buffer.as_ptr()` stays within that allocation.
        unsafe {
            Self::cache_invalidate(buffer.as_ptr(), used);
        }
    }

    /// Debug helper: invalidate cache and read the 16 bytes (rdes0..rdes3)
    /// of an RX descriptor at the given absolute address. Use to peek at DMA
    /// state from diagnostic code that lives outside the EMAC Mutex.
    ///
    /// # Safety
    /// `addr` must point to live RX descriptor memory.
    pub unsafe fn peek_rdes(addr: u32) -> [u32; 4] {
        Self::cache_invalidate(addr as *const u8, 16);
        let p = addr as *const u32;
        [
            core::ptr::read_volatile(p),
            core::ptr::read_volatile(p.offset(1)),
            core::ptr::read_volatile(p.offset(2)),
            core::ptr::read_volatile(p.offset(3)),
        ]
    }

    /// Debug helper: read raw 16 bytes WITHOUT invalidating cache. Returns the
    /// CPU-side cache view (so this should match the snapshot the writer just
    /// stored). Compare against [`Dma::peek_rdes`] to tell whether
    /// `cache_invalidate` is dropping CPU dirty lines (cache holds writes,
    /// memory does not).
    ///
    /// # Safety
    /// `addr` must point to live RX descriptor memory.
    pub unsafe fn peek_rdes_cached(addr: u32) -> [u32; 4] {
        let p = addr as *const u32;
        [
            core::ptr::read_volatile(p),
            core::ptr::read_volatile(p.offset(1)),
            core::ptr::read_volatile(p.offset(2)),
            core::ptr::read_volatile(p.offset(3)),
        ]
    }

    /// # Safety
    ///
    /// `addr..addr+size` must describe a valid memory range owned by the caller for the full
    /// duration of the cache operation. The pointed region must be DMA-visible memory.
    pub unsafe fn cache_writeback(addr: *const u8, size: usize) {
        let Some(range) = aligned_range(addr, size) else {
            return;
        };

        sync_cache_region(CacheOp::Writeback, range);
    }

    /// # Safety
    ///
    /// `addr..addr+size` must describe a valid memory range owned by the caller for the full
    /// duration of the cache operation. Any Rust references into that region must not be used to
    /// observe stale cached contents across the invalidation boundary.
    pub unsafe fn cache_invalidate(addr: *const u8, size: usize) {
        let Some(range) = aligned_range(addr, size) else {
            return;
        };

        sync_cache_region(CacheOp::Invalidate, range);
    }

    /// Flushes one descriptor object out of cache into DMA-visible memory.
    pub fn flush_descriptor<T>(descriptor: &T) {
        // SAFETY: `descriptor` is a valid reference, and flushing its exact byte range does not
        // mutate Rust-visible state; it only makes CPU writes observable to DMA.
        unsafe {
            Self::cache_writeback(
                descriptor as *const T as *const u8,
                core::mem::size_of::<T>(),
            );
        }
    }

    /// Flushes a full packet buffer out of cache into DMA-visible memory.
    pub fn flush_buffer(buffer: &[u8; BUF_SIZE]) {
        // SAFETY: `buffer` is a valid reference to a DMA backing allocation. Flushing `BUF_SIZE`
        // bytes from its base pointer remains within the buffer and only synchronizes caches.
        unsafe {
            Self::cache_writeback(buffer.as_ptr(), BUF_SIZE);
        }
    }

    /// Initialises the ESP32-P4 L2 cache controller mode shadow registers
    /// the way IDF's `cache_hal_init` does at second-stage bootloader.
    /// Must be called once at driver init **before** any
    /// `Cache_WriteBack_*` / `Cache_Invalidate_*` ROM helper, otherwise
    /// the helpers walk the L2 with stale `mode`/`ways`/`line_size`
    /// state inherited from a previous binary and intermittently hang
    /// the AHB bus. After the mode is set, a `Cache_Invalidate_All`
    /// drops any stale dirty lines so subsequent writebacks have nothing
    /// stale to chase.
    ///
    /// No-op on host targets.
    pub fn init_l2_cache_mode() {
        #[cfg(target_arch = "riscv32")]
        unsafe {
            let set_mode: RomCacheSetL2Mode =
                core::mem::transmute(ROM_CACHE_SET_L2_CACHE_MODE);
            set_mode(CACHE_SIZE_256K, CACHE_8WAYS_ASSOC, CACHE_LINE_SIZE_64B);
            let inv_all: RomCacheAll = core::mem::transmute(ROM_CACHE_INVALIDATE_ALL);
            let _ = inv_all(CACHE_MAP_L2_CACHE);
        }
    }
}

fn aligned_range(addr: *const u8, size: usize) -> Option<CacheAlignedRange> {
    if size == 0 {
        return None;
    }

    let start = (addr as usize) & !CACHE_LINE_MASK;
    let end = (addr as usize)
        .checked_add(size)
        .expect("cache sync range overflow")
        .checked_add(CACHE_LINE_MASK)
        .expect("cache sync range overflow")
        & !CACHE_LINE_MASK;

    Some(CacheAlignedRange {
        start,
        size: end - start,
    })
}

fn wait_for_dma_reset_clear<F>(mut read_bus_mode: F) -> Result<(), DmaError>
where
    F: FnMut() -> u32,
{
    for _ in 0..DMA_RESET_TIMEOUT_POLLS {
        if read_bus_mode() & regs::bits::dmabusmode::SWR == 0 {
            return Ok(());
        }
    }

    Err(DmaError::ResetTimeout)
}

fn dma_bus_mode_value(pbl: u32) -> u32 {
    let pbl = pbl & 0x3f;

    regs::bits::dmabusmode::AAL
        | regs::bits::dmabusmode::FB
        | regs::bits::dmabusmode::USP
        | (pbl << regs::bits::dmabusmode::PBL_SHIFT)
        | (pbl << regs::bits::dmabusmode::RPBL_SHIFT)
}

fn dma_op_mode_value() -> u32 {
    // RSF intentionally NOT set: P4 EMAC RX FIFO can't hold full frames, so
    // store-and-forward silently drops anything > ~256 bytes wire size. We use
    // cut-through receive (RTC threshold 64 bytes by default).
    regs::bits::dmaopmode::TSF | regs::bits::dmaopmode::FUF
}

fn dma_interrupt_enable_value() -> u32 {
    regs::bits::dmainten::TIE
        | regs::bits::dmainten::TSE
        | regs::bits::dmainten::TUE
        | regs::bits::dmainten::TJE
        | regs::bits::dmainten::OVE
        | regs::bits::dmainten::UNE
        | regs::bits::dmainten::RIE
        | regs::bits::dmainten::RUE
        | regs::bits::dmainten::RSE
        | regs::bits::dmainten::RWE
        | regs::bits::dmainten::ETE
        | regs::bits::dmainten::FBE
        | regs::bits::dmainten::ERE
        | regs::bits::dmainten::AIE
        | regs::bits::dmainten::NIE
}

/// Cache operations that DMA helpers can request. Maps onto the two ROM helpers
/// `Cache_WriteBack_Addr` and `Cache_Invalidate_Addr`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CacheOp {
    Writeback,
    Invalidate,
}

/// Last return code from a Cache_WriteBack_Addr ROM call. 0 means success per
/// IDF rom/cache.h. Anything else is an error path we'd otherwise miss.
#[cfg(target_arch = "riscv32")]
pub static LAST_WRITEBACK_RC: core::sync::atomic::AtomicI32 =
    core::sync::atomic::AtomicI32::new(-999);
/// Last return code from a Cache_Invalidate_Addr ROM call.
#[cfg(target_arch = "riscv32")]
pub static LAST_INVALIDATE_RC: core::sync::atomic::AtomicI32 =
    core::sync::atomic::AtomicI32::new(-999);

// --- Wall-clock instrumentation for cache ROM calls --------------------------
//
// SYSTIMER ticks at 16 MHz (62.5 ns per tick). Counters wrap at 2^32 ticks
// ≈ 268 s, so per-second deltas via `wrapping_sub` are safe. Calls counters
// are u32 incremented per ROM call. Atomics are unconditional (16 bytes
// total) so host tests can exercise the recording path; on host
// `sync_cache_region` is a no-op so they stay at zero in production.
pub static CACHE_INV_TICKS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static CACHE_INV_CALLS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static CACHE_WB_TICKS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static CACHE_WB_CALLS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Adds a per-call (ticks, calls += 1) sample for one ROM cache op. Extracted
/// from `sync_cache_region` so the recording path is testable without a real
/// SYSTIMER or ROM table on the host target.
#[allow(dead_code)]
#[inline]
fn record_cache_call(op: CacheOp, dt_ticks: u32) {
    use core::sync::atomic::Ordering::Relaxed;
    match op {
        CacheOp::Writeback => {
            CACHE_WB_TICKS.fetch_add(dt_ticks, Relaxed);
            CACHE_WB_CALLS.fetch_add(1, Relaxed);
        }
        CacheOp::Invalidate => {
            CACHE_INV_TICKS.fetch_add(dt_ticks, Relaxed);
            CACHE_INV_CALLS.fetch_add(1, Relaxed);
        }
    }
}

#[inline]
fn sync_cache_region(_op: CacheOp, _range: CacheAlignedRange) {
    #[cfg(target_arch = "riscv32")]
    {
        // SYSTIMER tick timestamps before/after the ROM call. We sample the
        // 32-bit low word only — at 16 MHz it wraps every ~268 s, but each
        // cache call is microseconds so wrapping_sub gives the correct delta.
        let t0 = crate::systimer::now_ticks() as u32;
        // Hold a critical_section across the ROM call so a SYSTIMER (or
        // EMAC) interrupt cannot trap the CPU while the cache controller
        // is mid-transaction. 2026-04-28 stress confirmed CS suppression
        // buys ~5 percentage points of reliability on warm reboots.
        let rc = critical_section::with(|_| unsafe {
            fence(Ordering::SeqCst);
            compiler_fence(Ordering::SeqCst);
            let rc = match _op {
                CacheOp::Writeback => {
                    // _Addr writeback variant returns success but does NOT
                    // actually flush data to RAM on `--ram --no-stub` boots
                    // (verified 2026-04-26 + 2026-04-28: with _Addr-only
                    // path the data plane breaks — `regs::read(MAC_CONFIG)`
                    // after `start()` hangs the AHB because the DMA read
                    // garbage). Use `_All` for both L1 and L2 so the
                    // descriptors actually become DMA-visible.
                    let f: RomCacheAll = core::mem::transmute(ROM_CACHE_WRITEBACK_ALL);
                    let rc1 = f(CACHE_MAP_L1_DCACHE);
                    let rc2 = f(CACHE_MAP_L2_CACHE);
                    if rc1 != 0 {
                        rc1
                    } else {
                        rc2
                    }
                }
                CacheOp::Invalidate => {
                    let f: RomCacheOp = core::mem::transmute(ROM_CACHE_INVALIDATE_ADDR);
                    let rc1 = f(CACHE_MAP_L1_DCACHE, _range.start as u32, _range.size as u32);
                    let rc2 = f(CACHE_MAP_L2_CACHE, _range.start as u32, _range.size as u32);
                    if rc1 != 0 {
                        rc1
                    } else {
                        rc2
                    }
                }
            };
            compiler_fence(Ordering::SeqCst);
            fence(Ordering::SeqCst);
            rc
        });
        let t1 = crate::systimer::now_ticks() as u32;
        let dt = t1.wrapping_sub(t0);
        match _op {
            CacheOp::Writeback => {
                LAST_WRITEBACK_RC.store(rc, core::sync::atomic::Ordering::Relaxed);
            }
            CacheOp::Invalidate => {
                LAST_INVALIDATE_RC.store(rc, core::sync::atomic::Ordering::Relaxed);
            }
        }
        record_cache_call(_op, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::{
        aligned_range, dma_bus_mode_value, dma_interrupt_enable_value, dma_op_mode_value,
        record_cache_call, wait_for_dma_reset_clear, CacheAlignedRange, CacheOp, Dma, DmaError,
        DmaInterruptStatus, CACHE_INV_CALLS, CACHE_INV_TICKS, CACHE_LINE_SIZE, CACHE_WB_CALLS,
        CACHE_WB_TICKS, DMA_BURST_LENGTH, DMA_RESET_TIMEOUT_POLLS,
    };
    use crate::{regs, zeroed_rx_descriptors, zeroed_tx_descriptors};
    use core::sync::atomic::Ordering;
    use std::sync::Mutex;

    /// Cache counters live in `static`s so parallel tests would race. Tests
    /// that touch them must take this lock, then call [`reset_cache_counters`]
    /// before observing.
    static CACHE_COUNTER_LOCK: Mutex<()> = Mutex::new(());

    fn reset_cache_counters() {
        CACHE_INV_TICKS.store(0, Ordering::Relaxed);
        CACHE_INV_CALLS.store(0, Ordering::Relaxed);
        CACHE_WB_TICKS.store(0, Ordering::Relaxed);
        CACHE_WB_CALLS.store(0, Ordering::Relaxed);
    }

    #[test]
    fn zero_size_range_is_skipped() {
        assert_eq!(aligned_range(0x2000 as *const u8, 0), None);
    }

    #[test]
    fn aligned_range_is_left_unchanged() {
        assert_eq!(
            aligned_range(0x2000 as *const u8, CACHE_LINE_SIZE),
            Some(CacheAlignedRange {
                start: 0x2000,
                size: CACHE_LINE_SIZE,
            })
        );
    }

    #[test]
    fn unaligned_range_is_expanded_to_cache_lines() {
        // 0x2033..0x2083 expanded to 128-byte alignment → 0x2000..0x2100.
        assert_eq!(
            aligned_range(0x2033 as *const u8, 80),
            Some(CacheAlignedRange {
                start: 0x2000,
                size: 256,
            })
        );
    }

    #[test]
    fn dma_reset_succeeds_when_swr_clears() {
        let mut polls = 0usize;
        let result = wait_for_dma_reset_clear(|| {
            polls += 1;
            if polls < 3 {
                regs::bits::dmabusmode::SWR
            } else {
                0
            }
        });

        assert_eq!(result, Ok(()));
        assert_eq!(polls, 3);
    }

    #[test]
    fn dma_reset_times_out_when_swr_stays_set() {
        let mut polls = 0usize;
        let result = wait_for_dma_reset_clear(|| {
            polls += 1;
            regs::bits::dmabusmode::SWR
        });

        assert_eq!(result, Err(DmaError::ResetTimeout));
        assert_eq!(polls, DMA_RESET_TIMEOUT_POLLS);
    }

    #[test]
    fn dma_reset_programs_software_reset_and_observes_clear_on_host() {
        regs::reset_test_registers();

        assert_eq!(Dma::dma_reset(), Ok(()));

        assert_eq!(regs::read(regs::dma::BUS_MODE), 0);
    }

    #[test]
    fn dma_bus_mode_enables_burst_and_alignment() {
        let bus_mode = dma_bus_mode_value(DMA_BURST_LENGTH);

        assert_ne!(bus_mode & regs::bits::dmabusmode::AAL, 0);
        assert_ne!(bus_mode & regs::bits::dmabusmode::FB, 0);
        assert_ne!(bus_mode & regs::bits::dmabusmode::USP, 0);
        assert_eq!(
            bus_mode & regs::bits::dmabusmode::PBL_MASK,
            DMA_BURST_LENGTH << regs::bits::dmabusmode::PBL_SHIFT
        );
        assert_eq!(
            bus_mode & regs::bits::dmabusmode::RPBL_MASK,
            DMA_BURST_LENGTH << regs::bits::dmabusmode::RPBL_SHIFT
        );
        assert_eq!(bus_mode & regs::bits::dmabusmode::DSL_MASK, 0);
    }

    #[test]
    fn dma_op_mode_uses_tx_store_forward_and_cut_through_rx() {
        let op_mode = dma_op_mode_value();

        assert_eq!(
            op_mode,
            regs::bits::dmaopmode::TSF | regs::bits::dmaopmode::FUF
        );
        assert_eq!(op_mode & regs::bits::dmaopmode::RSF, 0,
            "RSF must stay 0 on P4 — RX FIFO too small for full frames");
    }

    #[test]
    fn dma_interrupt_enable_value_covers_normal_and_abnormal_paths() {
        let inten = dma_interrupt_enable_value();

        assert_ne!(inten & regs::bits::dmainten::TIE, 0);
        assert_ne!(inten & regs::bits::dmainten::RIE, 0);
        assert_ne!(inten & regs::bits::dmainten::OVE, 0);
        assert_ne!(inten & regs::bits::dmainten::UNE, 0);
        assert_ne!(inten & regs::bits::dmainten::FBE, 0);
        assert_ne!(inten & regs::bits::dmainten::AIE, 0);
        assert_ne!(inten & regs::bits::dmainten::NIE, 0);
    }

    #[test]
    fn dma_init_programs_bus_op_mode_and_interrupt_mask() {
        regs::reset_test_registers();

        Dma::dma_init();

        assert_eq!(
            regs::read(regs::dma::BUS_MODE),
            dma_bus_mode_value(DMA_BURST_LENGTH)
        );
        assert_eq!(regs::read(regs::dma::OP_MODE), dma_op_mode_value());
        assert_eq!(regs::read(regs::dma::INT_EN), dma_interrupt_enable_value());
    }

    #[test]
    fn dma_descriptor_lists_are_programmed_from_slice_addresses() {
        regs::reset_test_registers();
        let tx_descriptors = zeroed_tx_descriptors();
        let rx_descriptors = zeroed_rx_descriptors();

        Dma::set_descriptor_lists(&tx_descriptors, &rx_descriptors);

        assert_eq!(
            regs::read(regs::dma::TX_DESC_LIST),
            tx_descriptors.as_ptr() as usize as u32
        );
        assert_eq!(
            regs::read(regs::dma::RX_DESC_LIST),
            rx_descriptors.as_ptr() as usize as u32
        );
    }

    #[test]
    fn dma_poll_demands_and_interrupt_disable_update_registers() {
        regs::reset_test_registers();
        regs::write(regs::dma::INT_EN, u32::MAX);

        Dma::demand_tx_poll();
        Dma::demand_rx_poll();
        Dma::disable_interrupts();

        assert_eq!(regs::read(regs::dma::TX_POLL_DEMAND), 1);
        assert_eq!(regs::read(regs::dma::RX_POLL_DEMAND), 1);
        assert_eq!(regs::read(regs::dma::INT_EN), 0);
    }

    #[test]
    fn dma_interrupt_status_extracts_recovery_flags() {
        let status = DmaInterruptStatus::from_raw(
            regs::bits::dmastatus::OVF
                | regs::bits::dmastatus::RU
                | regs::bits::dmastatus::UNF
                | regs::bits::dmastatus::FBI
                | regs::bits::dmastatus::AIS
                | regs::bits::dmastatus::NIS
                | (0b111 << regs::bits::dmastatus::EB_SHIFT),
        );

        assert!(status.has_rx_overflow());
        assert!(status.has_rx_buffer_unavailable());
        assert!(status.has_tx_underflow());
        assert!(status.has_fatal_bus_error());
        assert!(status.should_kick_rx());
        assert!(status.should_kick_tx());
        assert!(status.is_abnormal_summary());
        assert!(status.is_normal_summary());
        assert_eq!(status.error_bits(), 0b111);
    }

    #[test]
    fn dma_interrupt_status_clear_mask_does_not_touch_state_fields() {
        let status = DmaInterruptStatus::from_raw(
            regs::bits::dmastatus::TI
                | regs::bits::dmastatus::OVF
                | regs::bits::dmastatus::AIS
                | regs::bits::dmastatus::NIS
                | (0b101 << regs::bits::dmastatus::RS_SHIFT)
                | (0b110 << regs::bits::dmastatus::TS_SHIFT)
                | (0b011 << regs::bits::dmastatus::EB_SHIFT),
        );

        assert_eq!(
            status.clear_mask(),
            regs::bits::dmastatus::TI
                | regs::bits::dmastatus::OVF
                | regs::bits::dmastatus::AIS
                | regs::bits::dmastatus::NIS
        );
    }

    #[test]
    fn dma_clear_interrupt_status_writes_only_clearable_bits() {
        regs::reset_test_registers();
        let raw = regs::bits::dmastatus::RI
            | regs::bits::dmastatus::AIS
            | (0b101 << regs::bits::dmastatus::RS_SHIFT);
        let status = DmaInterruptStatus::from_raw(raw);

        Dma::clear_interrupt_status(status);

        assert_eq!(
            regs::read(regs::dma::STATUS),
            regs::bits::dmastatus::RI | regs::bits::dmastatus::AIS
        );
    }

    #[test]
    fn dma_clear_interrupt_status_skips_zero_clear_mask() {
        regs::reset_test_registers();
        regs::write(regs::dma::STATUS, 0xDEAD_BEEF);
        let status = DmaInterruptStatus::from_raw(0b101 << regs::bits::dmastatus::RS_SHIFT);

        Dma::clear_interrupt_status(status);

        assert_eq!(regs::read(regs::dma::STATUS), 0xDEAD_BEEF);
    }

    #[test]
    fn dma_interrupt_status_clear_mask_covers_all_sticky_bits() {
        let clearable = regs::bits::dmastatus::TI
            | regs::bits::dmastatus::TPS
            | regs::bits::dmastatus::TU
            | regs::bits::dmastatus::TJT
            | regs::bits::dmastatus::OVF
            | regs::bits::dmastatus::UNF
            | regs::bits::dmastatus::RI
            | regs::bits::dmastatus::RU
            | regs::bits::dmastatus::RPS
            | regs::bits::dmastatus::RWT
            | regs::bits::dmastatus::ETI
            | regs::bits::dmastatus::FBI
            | regs::bits::dmastatus::ERI
            | regs::bits::dmastatus::AIS
            | regs::bits::dmastatus::NIS;
        let state_fields = (0b101 << regs::bits::dmastatus::RS_SHIFT)
            | (0b110 << regs::bits::dmastatus::TS_SHIFT)
            | (0b011 << regs::bits::dmastatus::EB_SHIFT);

        let status = DmaInterruptStatus::from_raw(clearable | state_fields);

        assert_eq!(status.clear_mask(), clearable);
    }

    #[test]
    fn record_cache_call_invalidate_increments_only_invalidate_counters() {
        let _g = CACHE_COUNTER_LOCK.lock().unwrap();
        reset_cache_counters();

        record_cache_call(CacheOp::Invalidate, 100);

        assert_eq!(CACHE_INV_TICKS.load(Ordering::Relaxed), 100);
        assert_eq!(CACHE_INV_CALLS.load(Ordering::Relaxed), 1);
        assert_eq!(CACHE_WB_TICKS.load(Ordering::Relaxed), 0);
        assert_eq!(CACHE_WB_CALLS.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn record_cache_call_writeback_increments_only_writeback_counters() {
        let _g = CACHE_COUNTER_LOCK.lock().unwrap();
        reset_cache_counters();

        record_cache_call(CacheOp::Writeback, 200);

        assert_eq!(CACHE_WB_TICKS.load(Ordering::Relaxed), 200);
        assert_eq!(CACHE_WB_CALLS.load(Ordering::Relaxed), 1);
        assert_eq!(CACHE_INV_TICKS.load(Ordering::Relaxed), 0);
        assert_eq!(CACHE_INV_CALLS.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn record_cache_call_accumulates_across_multiple_calls() {
        let _g = CACHE_COUNTER_LOCK.lock().unwrap();
        reset_cache_counters();

        record_cache_call(CacheOp::Invalidate, 50);
        record_cache_call(CacheOp::Invalidate, 75);
        record_cache_call(CacheOp::Writeback, 1000);
        record_cache_call(CacheOp::Invalidate, 25);
        record_cache_call(CacheOp::Writeback, 500);

        assert_eq!(CACHE_INV_TICKS.load(Ordering::Relaxed), 150);
        assert_eq!(CACHE_INV_CALLS.load(Ordering::Relaxed), 3);
        assert_eq!(CACHE_WB_TICKS.load(Ordering::Relaxed), 1500);
        assert_eq!(CACHE_WB_CALLS.load(Ordering::Relaxed), 2);
    }

    /// `wrapping_sub` against the prior SYSTIMER snapshot must give a correct
    /// delta even when the counter rolls past 2^32 inside the ROM call.
    /// Simulates t0 just below wrap and t1 just above.
    #[test]
    fn record_cache_call_handles_systimer_wraparound() {
        let _g = CACHE_COUNTER_LOCK.lock().unwrap();
        reset_cache_counters();

        // Real call site does `t1.wrapping_sub(t0)` and passes the delta in.
        let t0: u32 = u32::MAX - 10;
        let t1: u32 = 50; // ROM call took ~60 ticks across the wrap.
        let dt = t1.wrapping_sub(t0);
        assert_eq!(dt, 61, "wrapping_sub across u32 boundary must give 61");

        record_cache_call(CacheOp::Invalidate, dt);
        assert_eq!(CACHE_INV_TICKS.load(Ordering::Relaxed), 61);
    }

    /// Counters survive their own internal u32 wrap. Catches a defensive
    /// regression where someone might add a saturating_add or panic-on-overflow
    /// branch and break the wrapping convention.
    #[test]
    fn record_cache_call_counters_wrap_cleanly_on_overflow() {
        let _g = CACHE_COUNTER_LOCK.lock().unwrap();
        reset_cache_counters();

        CACHE_INV_TICKS.store(u32::MAX - 100, Ordering::Relaxed);
        record_cache_call(CacheOp::Invalidate, 200);

        // Wrapping_add on AtomicU32::fetch_add: result is (u32::MAX - 100) + 200 mod 2^32 = 99.
        assert_eq!(CACHE_INV_TICKS.load(Ordering::Relaxed), 99);
    }
}
