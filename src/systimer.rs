//! ESP32-P4 SYSTIMER — low-level access to the 64-bit free-running counter.
//!
//! After power-on the SYSTIMER bus clock is enabled, the peripheral is out of
//! reset, and UNIT0 is already counting at XTAL/2.5 = 16 MHz (verified on
//! Waveshare ESP32-P4-ETH via `examples/systimer_probe.rs`). No init is needed
//! to read the counter via [`now_ticks`].
//!
//! Atomic 64-bit reads use the snapshot dance: write UPDATE bit in UNIT0_OP,
//! poll VALUE_VALID, then read VAL_HI/VAL_LO. The pair is held until the next
//! snapshot is requested.
//!
//! All MMIO is gated to riscv32 — pure constants and the
//! `target_ticks_split` helper used by `arm_alarm0` are host-testable.

#[cfg(target_arch = "riscv32")]
use core::hint::spin_loop;
#[cfg(target_arch = "riscv32")]
use core::ptr::{read_volatile, write_volatile};

#[cfg(target_arch = "riscv32")]
const SYSTIMER_BASE: usize = 0x500E_2000;

#[cfg(target_arch = "riscv32")]
const SYSTIMER_UNIT0_OP: *mut u32 = (SYSTIMER_BASE + 0x04) as *mut u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_UNIT0_VAL_HI: *const u32 = (SYSTIMER_BASE + 0x40) as *const u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_UNIT0_VAL_LO: *const u32 = (SYSTIMER_BASE + 0x44) as *const u32;

#[cfg(target_arch = "riscv32")]
const UNIT_OP_UPDATE_BIT: u32 = 1 << 30;
#[cfg(target_arch = "riscv32")]
const UNIT_OP_VALID_BIT: u32 = 1 << 29;

/// SYSTIMER tick rate (Hz). XTAL 40 MHz divided by the fixed 2.5 prescaler.
pub const TICK_RATE_HZ: u64 = 16_000_000;

/// Ticks per microsecond at the SYSTIMER rate.
pub const TICKS_PER_US: u64 = TICK_RATE_HZ / 1_000_000;

/// TARGET0 high half occupies the low 20 bits of `SYSTIMER_TARGET0_HI`. The
/// upper 12 bits of a 64-bit timer counter are hardware-truncated.
pub const TARGET0_HI_MASK: u32 = 0x000F_FFFF;

/// Read the 64-bit SYSTIMER UNIT0 counter atomically.
#[inline]
#[cfg(target_arch = "riscv32")]
pub fn now_ticks() -> u64 {
    unsafe {
        write_volatile(SYSTIMER_UNIT0_OP, UNIT_OP_UPDATE_BIT);
        while read_volatile(SYSTIMER_UNIT0_OP) & UNIT_OP_VALID_BIT == 0 {
            spin_loop();
        }
        let hi = read_volatile(SYSTIMER_UNIT0_VAL_HI) as u64;
        let lo = read_volatile(SYSTIMER_UNIT0_VAL_LO) as u64;
        (hi << 32) | lo
    }
}

/// Host stub: returns 0. The driver only calls `now_ticks` from riscv32
/// hot paths; non-riscv32 callers (host tests) should not depend on real
/// time, so the stub avoids pulling in `std::time` and keeps the lib
/// `no_std`-pure on host builds too.
#[inline]
#[cfg(not(target_arch = "riscv32"))]
pub fn now_ticks() -> u64 {
    0
}

#[inline]
pub const fn ticks_to_us(ticks: u64) -> u64 {
    ticks / TICKS_PER_US
}

#[inline]
pub const fn us_to_ticks(us: u64) -> u64 {
    us * TICKS_PER_US
}

/// Convenience: current time in microseconds since SYSTIMER power-on.
#[inline]
pub fn now_us() -> u64 {
    ticks_to_us(now_ticks())
}

// SYSTIMER alarm 0 (TARGET0 / COMP0) — used by the IRQ-driven embassy time
// driver to schedule a single one-shot wake-up at an absolute counter value.

#[cfg(target_arch = "riscv32")]
const SYSTIMER_CONF: *mut u32 = SYSTIMER_BASE as *mut u32;
#[cfg(target_arch = "riscv32")]
const CONF_TARGET0_WORK_EN: u32 = 1 << 24;

#[cfg(target_arch = "riscv32")]
const SYSTIMER_TARGET0_HI: *mut u32 = (SYSTIMER_BASE + 0x1C) as *mut u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_TARGET0_LO: *mut u32 = (SYSTIMER_BASE + 0x20) as *mut u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_TARGET0_CONF: *mut u32 = (SYSTIMER_BASE + 0x34) as *mut u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_COMP0_LOAD: *mut u32 = (SYSTIMER_BASE + 0x50) as *mut u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_INT_ENA: *mut u32 = (SYSTIMER_BASE + 0x64) as *mut u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_INT_RAW: *const u32 = (SYSTIMER_BASE + 0x68) as *const u32;
#[cfg(target_arch = "riscv32")]
const SYSTIMER_INT_CLR: *mut u32 = (SYSTIMER_BASE + 0x6C) as *mut u32;

pub const TARGET0_INT_BIT: u32 = 1 << 0;

/// Split a 64-bit absolute target value into `(hi, lo)` for
/// `SYSTIMER_TARGET0_HI` / `SYSTIMER_TARGET0_LO`. The HI half is truncated
/// to 20 bits — anything beyond the 52-bit window the SYSTIMER comparator
/// can address is silently dropped, which matches the hardware behavior.
#[inline]
pub const fn target_ticks_split(target_ticks: u64) -> (u32, u32) {
    let hi = ((target_ticks >> 32) as u32) & TARGET0_HI_MASK;
    let lo = target_ticks as u32;
    (hi, lo)
}

/// Enable the TARGET0 comparator in `SYSTIMER_CONF` (bit 24). Idempotent.
/// Must be called once before any [`arm_alarm0`] — without this bit the
/// comparator stays gated off and `INT_RAW.target0` never asserts.
pub fn init_alarm0() {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        let conf = read_volatile(SYSTIMER_CONF);
        write_volatile(SYSTIMER_CONF, conf | CONF_TARGET0_WORK_EN);
    }
}

/// Arm SYSTIMER alarm 0 to fire when UNIT0 reaches `target_ticks`. One-shot,
/// compares against UNIT0. Caller is responsible for calling [`init_alarm0`]
/// once at startup, routing the interrupt (`crate::clic`), and globally
/// enabling traps before this can fire.
pub fn arm_alarm0(_target_ticks: u64) {
    #[cfg(target_arch = "riscv32")]
    {
        let (hi, lo) = target_ticks_split(_target_ticks);
        unsafe {
            // Disarm + clear before reprogramming so a fire mid-update can't be
            // observed with stale TARGET0 values.
            write_volatile(SYSTIMER_INT_ENA, 0);
            write_volatile(SYSTIMER_INT_CLR, TARGET0_INT_BIT);
            write_volatile(SYSTIMER_TARGET0_HI, hi);
            write_volatile(SYSTIMER_TARGET0_LO, lo);
            // CONF=0 → one-shot, compare against UNIT0.
            write_volatile(SYSTIMER_TARGET0_CONF, 0);
            // Commit TARGET0_HI/LO/CONF to the internal compare register.
            write_volatile(SYSTIMER_COMP0_LOAD, 1);
            write_volatile(SYSTIMER_INT_ENA, TARGET0_INT_BIT);
        }
    }
}

/// Disarm SYSTIMER alarm 0 (clears INT_ENA only — leaves TARGET0 as-is).
#[inline]
pub fn disarm_alarm0() {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        write_volatile(SYSTIMER_INT_ENA, 0)
    };
}

/// Clear the latched INT_ST.target0 bit (write-1-to-clear semantics).
#[inline]
pub fn clear_alarm0() {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        write_volatile(SYSTIMER_INT_CLR, TARGET0_INT_BIT)
    };
}

/// Read the raw INT_RAW.target0 bit. Useful for polling the alarm without
/// an IRQ during bring-up.
#[inline]
pub fn alarm0_pending() -> bool {
    #[cfg(target_arch = "riscv32")]
    {
        return unsafe { read_volatile(SYSTIMER_INT_RAW) & TARGET0_INT_BIT != 0 };
    }
    #[cfg(not(target_arch = "riscv32"))]
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SYSTIMER UNIT0 ticks at XTAL / 2.5 = 40 MHz / 2.5 = 16 MHz. The whole
    /// time-driver math hinges on this constant; if the prescaler changes
    /// across a P4 silicon revision, every Timer::after duration drifts.
    #[test]
    fn tick_rate_hz_is_sixteen_mhz() {
        assert_eq!(TICK_RATE_HZ, 16_000_000);
    }

    /// embassy-time runs at 1 MHz (1 µs ticks); SYSTIMER at 16 MHz. So one
    /// embassy-tick = 16 SYSTIMER ticks.
    #[test]
    fn ticks_per_us_is_sixteen() {
        assert_eq!(TICKS_PER_US, 16);
    }

    /// 1 second = 16,000,000 ticks → 1,000,000 µs.
    #[test]
    fn ticks_to_us_round_trips_one_second() {
        let one_sec_ticks: u64 = TICK_RATE_HZ;
        assert_eq!(ticks_to_us(one_sec_ticks), 1_000_000);
        assert_eq!(us_to_ticks(1_000_000), one_sec_ticks);
    }

    #[test]
    fn ticks_to_us_truncates_sub_microsecond() {
        // 15 ticks (< 1 µs at 16 MHz) → 0 µs.
        assert_eq!(ticks_to_us(15), 0);
        // 16 ticks = 1 µs.
        assert_eq!(ticks_to_us(16), 1);
        // 17 ticks → 1 µs (integer division floors).
        assert_eq!(ticks_to_us(17), 1);
    }

    /// `us_to_ticks` is `us * 16`. For inputs that won't overflow u64
    /// (< 2^60 µs ≈ 36 500 years), the round-trip is exact.
    #[test]
    fn us_to_ticks_round_trips_arbitrary_microsecond_values() {
        for us in [
            0u64,
            1,
            500,
            1_000_000,           // 1 s
            60 * 1_000_000,      // 1 minute
            3_600 * 1_000_000,   // 1 hour
        ] {
            let ticks = us_to_ticks(us);
            assert_eq!(ticks, us * 16, "us_to_ticks({}) wrong", us);
            assert_eq!(ticks_to_us(ticks), us);
        }
    }

    /// The HI half of TARGET0 is 20 bits; the comparator only sees a
    /// 52-bit window of the 64-bit counter.
    #[test]
    fn target0_hi_mask_is_twenty_bits() {
        assert_eq!(TARGET0_HI_MASK, (1u32 << 20) - 1);
    }

    /// Alarm just after midnight: hi=0, lo=full embassy-time-ish value.
    #[test]
    fn target_ticks_split_zero_is_zero_zero() {
        assert_eq!(target_ticks_split(0), (0, 0));
    }

    #[test]
    fn target_ticks_split_lo_carries_low_thirty_two_bits() {
        let (hi, lo) = target_ticks_split(0x0000_0000_DEAD_BEEF);
        assert_eq!(hi, 0);
        assert_eq!(lo, 0xDEAD_BEEF);

        let (hi, lo) = target_ticks_split(0x0000_0001_DEAD_BEEF);
        assert_eq!(hi, 1);
        assert_eq!(lo, 0xDEAD_BEEF);
    }

    /// HI half clamps to 20 bits — anything above 2^52 is hardware-truncated.
    /// Catches a bug where someone widens the mask and the wrong upper bits
    /// land in the register.
    #[test]
    fn target_ticks_split_hi_truncates_above_twenty_bits() {
        // Upper bits of HI must be clipped: write 0xFFFF_FFFF in HI half →
        // only low 20 bits survive.
        let target: u64 = 0xFFFF_FFFF_0000_0000;
        let (hi, lo) = target_ticks_split(target);
        assert_eq!(hi, 0x000F_FFFF);
        assert_eq!(lo, 0);
    }

    #[test]
    fn target0_int_bit_is_lowest_bit() {
        assert_eq!(TARGET0_INT_BIT, 1);
    }
}
