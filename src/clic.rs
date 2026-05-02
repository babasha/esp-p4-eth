//! ESP32-P4 HP-CPU interrupt routing — INTERRUPT_CORE0 matrix + CLIC.
//!
//! See `reference_p4_clic_irq_routing.md` for the full hardware map. Routing a
//! peripheral interrupt to the CPU is a two-step process:
//!
//! 1. Pick a CPU INT line `N` ∈ 1..31 and program the matrix register for the
//!    peripheral source: `INTERRUPT_CORE0_<peripheral>_INT_MAP_REG = N`.
//! 2. Configure CLIC entry `N + CLIC_EXT_INTR_NUM_OFFSET` (= `N + 16`):
//!    ATTR (level/edge, mode), CTL (priority), IE (enable). Clear pending via
//!    `CLIC_INT_IP` byte. Threshold register gates global priority.
//!
//! Constants and bit positions cross-checked against IDF v5.3.5
//! `components/soc/esp32p4/include/soc/clic_reg.h` and
//! `components/soc/esp32p4/include/soc/interrupt_core0_reg.h`.
//!
//! All MMIO writes are gated to riscv32 — the file is host-compilable so the
//! pure address-arithmetic and byte-packing helpers can be unit-tested on the
//! development host. On non-riscv32 targets every public function that would
//! touch MMIO is a no-op.

#[cfg(target_arch = "riscv32")]
use core::ptr::{read_volatile, write_volatile};

const INTERRUPT_CORE0_BASE: usize = 0x500D_6000;
/// `INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_REG` — bits[5:0] = CLIC index.
pub const INT_MAP_SYSTIMER_TARGET0: *mut u32 = (INTERRUPT_CORE0_BASE + 0xD4) as *mut u32;
/// `INTERRUPT_CORE0_SBD_INT_MAP_REG` — main EMAC/GMAC DMA interrupt
/// (Synopsys DesignWare bus driver). Source ID 92 = offset 0x170.
pub const INT_MAP_EMAC_SBD: *mut u32 = (INTERRUPT_CORE0_BASE + 0x170) as *mut u32;

#[cfg(target_arch = "riscv32")]
const CLIC_BASE: usize = 0x2080_0000;
#[allow(dead_code)]
const CLIC_CTRL_BASE: usize = 0x2080_1000;
#[cfg(target_arch = "riscv32")]
const CLIC_INT_THRESH_REG: *mut u32 = (CLIC_BASE + 0x08) as *mut u32;

/// External interrupts in the CLIC index space start at 16 — peripheral CPU
/// INT line `N` (1..31) lives at CLIC entry `N + 16`.
pub const CLIC_EXT_INTR_NUM_OFFSET: u8 = 16;

/// Trigger types for CLIC ATTR.TRIG bits[2:1].
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Trigger {
    LevelPositive = 0b00,
    EdgePositive = 0b01,
    LevelNegative = 0b10,
    EdgeNegative = 0b11,
}

// ---------------------------------------------------------------------------
// Pure-data helpers (host-testable). All MMIO wrappers below build on these.
// ---------------------------------------------------------------------------

/// CLIC entry index for a given peripheral CPU INT line. External interrupts
/// start at offset 16, so CPU line `N` maps to CLIC entry `N + 16`.
#[inline]
pub const fn clic_idx_for_cpu_line(cpu_int_n: u8) -> u8 {
    cpu_int_n + CLIC_EXT_INTR_NUM_OFFSET
}

/// Byte offset of CLIC control word for entry `idx`, relative to
/// `CLIC_CTRL_BASE`. Each entry is 4 bytes: `[IP, IE, ATTR, CTL]`.
#[inline]
pub const fn clic_ctrl_offset(idx: u8) -> usize {
    (idx as usize) * 4
}

/// Pack the CLIC ATTR byte: MODE = 11 (machine), TRIG = `trigger`, SHV = 0.
/// MODE in the top 2 bits, TRIG in bits[2:1], SHV in bit 0.
#[inline]
pub const fn clic_attr_byte(trigger: Trigger) -> u8 {
    (3u8 << 6) | ((trigger as u8) << 1)
}

/// Pack the CLIC CTL byte: priority occupies the top NLBITS = 3 bits. Higher
/// is more important; the bottom 5 bits are reserved/zero.
#[inline]
pub const fn clic_ctl_byte(priority: u8) -> u8 {
    (priority & 0x07) << 5
}

/// Value to write to `INTERRUPT_CORE0_<peripheral>_INT_MAP_REG`. The register
/// accepts a 6-bit CLIC index — clamps the input to its valid range.
#[inline]
pub const fn int_map_value(clic_idx: u8) -> u32 {
    (clic_idx & 0x3F) as u32
}

/// Value to write to the CLIC global threshold register: `thresh` byte
/// occupies bits[31:24]. `0x00` = pass everything; `0xFF` = block all.
#[inline]
pub const fn clic_threshold_value(thresh: u8) -> u32 {
    (thresh as u32) << 24
}

// ---------------------------------------------------------------------------
// Raw pointer helpers — only meaningful on riscv32. On host the addresses
// are still computed correctly but never dereferenced.
// ---------------------------------------------------------------------------

#[allow(dead_code)]
#[inline]
fn clic_word_ptr(idx: u8) -> *mut u32 {
    (CLIC_CTRL_BASE + clic_ctrl_offset(idx)) as *mut u32
}

#[inline]
fn clic_byte_ip(idx: u8) -> *mut u8 {
    (CLIC_CTRL_BASE + clic_ctrl_offset(idx)) as *mut u8
}

#[inline]
fn clic_byte_ie(idx: u8) -> *mut u8 {
    (CLIC_CTRL_BASE + clic_ctrl_offset(idx) + 1) as *mut u8
}

#[inline]
fn clic_byte_attr(idx: u8) -> *mut u8 {
    (CLIC_CTRL_BASE + clic_ctrl_offset(idx) + 2) as *mut u8
}

#[inline]
fn clic_byte_ctl(idx: u8) -> *mut u8 {
    (CLIC_CTRL_BASE + clic_ctrl_offset(idx) + 3) as *mut u8
}

// ---------------------------------------------------------------------------
// MMIO wrappers — riscv32-only.
// ---------------------------------------------------------------------------

/// Route a peripheral interrupt source to CLIC entry `clic_idx` (16..47 for
/// external peripherals — that's `cpu_int_line + 16`). `int_map_reg` is the
/// absolute address of the matrix register for the source (e.g.
/// [`INT_MAP_SYSTIMER_TARGET0`], [`INT_MAP_EMAC_SBD`]). Writing 0 unmaps
/// the source.
///
/// Note: the value written to INTERRUPT_CORE0_*_INT_MAP_REG is the CLIC
/// index directly, NOT just the CPU INT line number. IDF's
/// `intr_matrix_route(src, n)` does `interrupt_clic_ll_route(core, src,
/// n + RV_EXTERNAL_INT_OFFSET)` and that final value is what lands in the
/// matrix register.
#[inline]
pub fn route_to_clic(_int_map_reg: *mut u32, _clic_idx: u8) {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        write_volatile(_int_map_reg, int_map_value(_clic_idx));
    }
}

/// Convenience wrapper for the SYSTIMER TARGET0 source.
#[inline]
pub fn route_systimer_target0(clic_idx: u8) {
    route_to_clic(INT_MAP_SYSTIMER_TARGET0, clic_idx);
}

/// Convenience wrapper for the EMAC main DMA / SBD source.
#[inline]
pub fn route_emac_sbd(clic_idx: u8) {
    route_to_clic(INT_MAP_EMAC_SBD, clic_idx);
}

/// Configure a CLIC entry for one peripheral CPU INT line. `priority` is the
/// CTL byte top-3-bit value (NLBITS=3) — use `0..=7`. Sets level-positive
/// trigger, machine mode, IE = 1, no selective vectoring (SHV = 0).
pub fn enable_cpu_int(_cpu_int_n: u8, _priority: u8) {
    #[cfg(target_arch = "riscv32")]
    {
        let idx = clic_idx_for_cpu_line(_cpu_int_n);
        let prio_byte = clic_ctl_byte(_priority);
        let attr_byte = clic_attr_byte(Trigger::LevelPositive);
        unsafe {
            // Disable while reconfiguring to avoid spurious fires.
            write_volatile(clic_byte_ie(idx), 0);
            // Clear any latched IP.
            write_volatile(clic_byte_ip(idx), 0);
            write_volatile(clic_byte_attr(idx), attr_byte);
            write_volatile(clic_byte_ctl(idx), prio_byte);
            write_volatile(clic_byte_ie(idx), 1);
        }
    }
}

/// Disable a CLIC entry (clears IE byte). Source mapping in INTERRUPT_CORE0
/// is left in place.
#[inline]
pub fn disable_cpu_int(_cpu_int_n: u8) {
    #[cfg(target_arch = "riscv32")]
    {
        let idx = clic_idx_for_cpu_line(_cpu_int_n);
        unsafe { write_volatile(clic_byte_ie(idx), 0) };
    }
}

/// Clear pending bit on a CLIC entry. Most level-triggered sources auto-clear
/// when the upstream peripheral deasserts, but this is useful as a fence /
/// for edge sources.
#[inline]
pub fn clear_pending(_cpu_int_n: u8) {
    #[cfg(target_arch = "riscv32")]
    {
        let idx = clic_idx_for_cpu_line(_cpu_int_n);
        unsafe { write_volatile(clic_byte_ip(idx), 0) };
    }
}

/// Read the raw CLIC control word for diagnostics.
#[inline]
pub fn read_ctrl_word(_cpu_int_n: u8) -> u32 {
    #[cfg(target_arch = "riscv32")]
    {
        let idx = clic_idx_for_cpu_line(_cpu_int_n);
        return unsafe { read_volatile(clic_word_ptr(idx)) };
    }
    #[cfg(not(target_arch = "riscv32"))]
    0
}

/// Set the CLIC global threshold (bits[31:24]). CPU responds when an int's
/// CTL ≥ threshold. `0x00` = pass everything; `0xFF` = block all.
#[inline]
pub fn set_threshold(_thresh: u8) {
    #[cfg(target_arch = "riscv32")]
    unsafe {
        write_volatile(CLIC_INT_THRESH_REG, clic_threshold_value(_thresh));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// IDF baseline `interrupt_core0_reg.h`: external interrupts start at
    /// CLIC entry 16. If this drifts, every IRQ routing computation is off.
    #[test]
    fn clic_ext_intr_num_offset_is_sixteen() {
        assert_eq!(CLIC_EXT_INTR_NUM_OFFSET, 16);
    }

    /// Verified against IDF `interrupt_core0_reg.h`:
    /// `DR_REG_INTERRUPT_CORE0_BASE = 0x500D6000` and
    /// `INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_REG = base + 0xD4`.
    #[test]
    fn int_map_systimer_target0_address_matches_idf() {
        assert_eq!(INT_MAP_SYSTIMER_TARGET0 as usize, 0x500D_60D4);
    }

    /// Verified against IDF: `INTERRUPT_CORE0_SBD_INT_MAP_REG = base + 0x170`.
    /// Source ID 92 in the matrix corresponds to the GMAC SBD aggregate.
    #[test]
    fn int_map_emac_sbd_address_matches_idf() {
        assert_eq!(INT_MAP_EMAC_SBD as usize, 0x500D_6170);
    }

    #[test]
    fn clic_idx_for_cpu_line_is_offset_by_sixteen() {
        assert_eq!(clic_idx_for_cpu_line(0), 16);
        assert_eq!(clic_idx_for_cpu_line(1), 17);
        assert_eq!(clic_idx_for_cpu_line(2), 18);
        // 31 is the highest legal CPU INT line — yields CLIC index 47.
        assert_eq!(clic_idx_for_cpu_line(31), 47);
    }

    #[test]
    fn clic_ctrl_offset_is_four_bytes_per_entry() {
        assert_eq!(clic_ctrl_offset(0), 0);
        assert_eq!(clic_ctrl_offset(1), 4);
        assert_eq!(clic_ctrl_offset(17), 17 * 4);
        assert_eq!(clic_ctrl_offset(255), 255 * 4);
    }

    /// Each CLIC entry is `[IP, IE, ATTR, CTL]` in increasing-address order.
    /// A swap here silently breaks every interrupt setup.
    #[test]
    fn clic_byte_offsets_are_ip_ie_attr_ctl_in_order() {
        let idx = 17;
        let ip = clic_byte_ip(idx) as usize;
        let ie = clic_byte_ie(idx) as usize;
        let attr = clic_byte_attr(idx) as usize;
        let ctl = clic_byte_ctl(idx) as usize;
        let word = clic_word_ptr(idx) as usize;
        assert_eq!(ip, word);
        assert_eq!(ie, word + 1);
        assert_eq!(attr, word + 2);
        assert_eq!(ctl, word + 3);
    }

    /// CLIC entry 17 (SYSTIMER alarm) lives at `0x2080_1000 + 17*4 = 0x2080_1044`.
    #[test]
    fn clic_word_address_for_systimer_entry_matches_layout() {
        assert_eq!(clic_word_ptr(17) as usize, 0x2080_1044);
        // Entry 18 (EMAC SBD) is the next one.
        assert_eq!(clic_word_ptr(18) as usize, 0x2080_1048);
    }

    #[test]
    fn clic_attr_byte_machine_mode_level_positive_is_0xc0() {
        // MODE = 11 → top 2 bits = 11000000.
        // TRIG = 00 (level positive) → bits[2:1] = 00.
        // SHV = 0 → bit 0 = 0.
        assert_eq!(clic_attr_byte(Trigger::LevelPositive), 0b1100_0000);
    }

    #[test]
    fn clic_attr_byte_encodes_each_trigger_distinctly() {
        let lp = clic_attr_byte(Trigger::LevelPositive);
        let ep = clic_attr_byte(Trigger::EdgePositive);
        let ln = clic_attr_byte(Trigger::LevelNegative);
        let en = clic_attr_byte(Trigger::EdgeNegative);
        // All four must differ — no aliasing.
        assert_ne!(lp, ep);
        assert_ne!(lp, ln);
        assert_ne!(lp, en);
        assert_ne!(ep, ln);
        assert_ne!(ep, en);
        assert_ne!(ln, en);
        // MODE bits and SHV bit must be the same across triggers.
        let mode_shv_mask = !0b0000_0110;
        assert_eq!(lp & mode_shv_mask, ep & mode_shv_mask);
        assert_eq!(lp & mode_shv_mask, ln & mode_shv_mask);
        assert_eq!(lp & mode_shv_mask, en & mode_shv_mask);
    }

    #[test]
    fn clic_ctl_byte_packs_priority_into_top_three_bits() {
        // Priority 0 → 0b000_00000.
        assert_eq!(clic_ctl_byte(0), 0b0000_0000);
        // Priority 1 → top 3 bits = 001 → 0b001_00000 = 0x20.
        assert_eq!(clic_ctl_byte(1), 0b0010_0000);
        // Priority 7 → top 3 bits = 111 → 0xE0.
        assert_eq!(clic_ctl_byte(7), 0b1110_0000);
    }

    #[test]
    fn clic_ctl_byte_clamps_priority_to_three_bits() {
        // 8 has bit 3 set, which falls outside NLBITS=3 — must wrap to 0.
        assert_eq!(clic_ctl_byte(8), 0);
        // 0xFF clamps to 0b111 → 0xE0.
        assert_eq!(clic_ctl_byte(0xFF), 0b1110_0000);
        // The bottom 5 bits of CTL must remain zero.
        for p in 0..=255u8 {
            assert_eq!(
                clic_ctl_byte(p) & 0b0001_1111,
                0,
                "priority {} leaked into reserved bottom bits",
                p
            );
        }
    }

    #[test]
    fn int_map_value_writes_clic_index_in_low_six_bits() {
        // For CPU line 1 → CLIC index 17, the matrix register receives 17.
        // This catches the prior 2026-04-26 bug where we wrote `cpu_int_n`
        // (= 1) instead of `clic_idx_for_cpu_line(1)` (= 17) and the
        // peripheral never reached the CPU.
        assert_eq!(int_map_value(17), 17);
        assert_eq!(int_map_value(18), 18);
        // Top bits ignored: even if upstream packs flags, only bits[5:0]
        // hit the register.
        assert_eq!(int_map_value(0xC0 | 17), 17);
        // 6-bit max stays inside the field.
        assert_eq!(int_map_value(0x3F), 0x3F);
    }

    #[test]
    fn clic_threshold_value_packs_threshold_byte_into_high_byte() {
        // 0x00 → pass everything → low 24 bits stay zero.
        assert_eq!(clic_threshold_value(0x00), 0x0000_0000);
        // 0xFF → block all → high byte = 0xFF.
        assert_eq!(clic_threshold_value(0xFF), 0xFF00_0000);
        // Any byte goes only into bits[31:24].
        for t in 0..=255u8 {
            let v = clic_threshold_value(t);
            assert_eq!(v & 0x00FF_FFFF, 0, "threshold {:#x} leaked", t);
            assert_eq!(v >> 24, t as u32);
        }
    }
}
