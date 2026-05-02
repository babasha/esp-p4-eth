//! Pure-data logic shared with `time_driver_irq.rs`. Lives in a separate
//! module so it is host-compilable regardless of the `p4-time-driver-irq`
//! feature flag — the parent module is gated on both the feature and the
//! `riscv32` target, which would otherwise hide every helper from host
//! tests.
//!
//! Nothing in here touches MMIO or executor state. The actual MMIO-driving
//! `time_driver_irq.rs` re-exports the constants from this module so a
//! mismatch between the two files would fail to compile.

use crate::clic;

/// CPU INT line for SYSTIMER_TARGET0. CLIC supports lines 1..31.
pub const CPU_INT_LINE_SYSTIMER: u8 = 1;
/// CPU INT line for the EMAC main DMA / SBD interrupt.
pub const CPU_INT_LINE_EMAC: u8 = 2;

/// CLIC entry index for SYSTIMER alarm 0 — must match the value programmed
/// into `INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_REG`.
pub const SYSTIMER_TARGET0_CLIC_INDEX: u8 = clic::clic_idx_for_cpu_line(CPU_INT_LINE_SYSTIMER);
/// CLIC entry index for the EMAC SBD interrupt.
pub const EMAC_SBD_CLIC_INDEX: u8 = clic::clic_idx_for_cpu_line(CPU_INT_LINE_EMAC);

/// `mcause` bit 31 distinguishes asynchronous interrupts from synchronous
/// exceptions per the RISC-V privileged spec.
pub const MCAUSE_INTERRUPT_BIT: u32 = 1 << 31;
/// CLIC variant uses bits[11:0] of `mcause` for the exception/interrupt
/// code. `0xFFF` covers the largest CLIC index (255 entries) plus a
/// generous head-room.
pub const MCAUSE_CODE_MASK: u32 = 0x0FFF;

/// `true` if `mcause` describes an asynchronous interrupt (CLIC entry fired)
/// rather than a synchronous exception (illegal instruction, ecall, etc.).
#[inline]
pub const fn is_async_irq(mcause: u32) -> bool {
    mcause & MCAUSE_INTERRUPT_BIT != 0
}

/// Extract the low 12-bit interrupt / exception code from `mcause` and cast
/// to the byte representation our dispatcher matches against. Caller is
/// responsible for ensuring `mcause` describes an interrupt — see
/// [`is_async_irq`].
#[inline]
pub const fn mcause_code(mcause: u32) -> u8 {
    (mcause & MCAUSE_CODE_MASK) as u8
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clic::CLIC_EXT_INTR_NUM_OFFSET;

    #[test]
    fn cpu_int_line_systimer_is_one() {
        assert_eq!(CPU_INT_LINE_SYSTIMER, 1);
    }

    #[test]
    fn cpu_int_line_emac_is_two_distinct_from_systimer() {
        assert_eq!(CPU_INT_LINE_EMAC, 2);
        assert_ne!(CPU_INT_LINE_SYSTIMER, CPU_INT_LINE_EMAC);
    }

    /// CLIC index = CPU line + 16. With CPU line 1, the index lands at 17 —
    /// this is what `INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_REG` must be
    /// programmed with. Catches a regression where someone writes the CPU
    /// line directly (1) and the source never reaches the CPU.
    #[test]
    fn systimer_target0_clic_index_is_seventeen() {
        assert_eq!(SYSTIMER_TARGET0_CLIC_INDEX, 17);
        assert_eq!(
            SYSTIMER_TARGET0_CLIC_INDEX,
            CPU_INT_LINE_SYSTIMER + CLIC_EXT_INTR_NUM_OFFSET,
        );
    }

    #[test]
    fn emac_sbd_clic_index_is_eighteen() {
        assert_eq!(EMAC_SBD_CLIC_INDEX, 18);
        assert_eq!(
            EMAC_SBD_CLIC_INDEX,
            CPU_INT_LINE_EMAC + CLIC_EXT_INTR_NUM_OFFSET,
        );
    }

    /// Two indices must differ — sharing one would make the dispatcher
    /// jump to the wrong branch and silently swallow events.
    #[test]
    fn clic_indices_for_systimer_and_emac_are_distinct() {
        assert_ne!(SYSTIMER_TARGET0_CLIC_INDEX, EMAC_SBD_CLIC_INDEX);
    }

    #[test]
    fn mcause_interrupt_bit_is_msb() {
        assert_eq!(MCAUSE_INTERRUPT_BIT, 0x8000_0000);
    }

    #[test]
    fn mcause_code_mask_keeps_low_twelve_bits_only() {
        assert_eq!(MCAUSE_CODE_MASK, 0x0000_0FFF);
    }

    /// CLIC delivery: bit 31 set → asynchronous IRQ, low 12 bits hold the
    /// CLIC entry index. The dispatcher's first branch checks this bit.
    #[test]
    fn is_async_irq_recognises_clic_delivery() {
        // SYSTIMER alarm fires: mcause = MEI bit | 17.
        let m = MCAUSE_INTERRUPT_BIT | 17;
        assert!(is_async_irq(m));
        // EMAC: mcause = MEI bit | 18.
        let m = MCAUSE_INTERRUPT_BIT | 18;
        assert!(is_async_irq(m));
    }

    #[test]
    fn is_async_irq_returns_false_for_synchronous_exception() {
        // Illegal instruction = code 2, no interrupt bit.
        assert!(!is_async_irq(2));
        // Ecall from M-mode = code 11.
        assert!(!is_async_irq(11));
        // Even with all low bits set, no MSB → not an IRQ.
        assert!(!is_async_irq(0x7FFF_FFFF));
    }

    #[test]
    fn mcause_code_extracts_clic_index() {
        assert_eq!(mcause_code(MCAUSE_INTERRUPT_BIT | 17), 17);
        assert_eq!(mcause_code(MCAUSE_INTERRUPT_BIT | 18), 18);
        // Synchronous codes pass through unchanged when MSB is clear.
        assert_eq!(mcause_code(2), 2);
    }

    /// Cast to `u8` mustn't lose information for valid CLIC indices
    /// (max 255). Catches a refactor that drops the mask and lets the
    /// upper bits leak in.
    #[test]
    fn mcause_code_masks_above_byte_range() {
        // 0x123 would saturate `as u8` to 0x23 — the mask must do that
        // explicitly so the result is consistent regardless of compiler
        // truncation rules.
        assert_eq!(mcause_code(MCAUSE_INTERRUPT_BIT | 0x123), 0x23);
        // Anything beyond MCAUSE_CODE_MASK (0xFFF) is dropped.
        assert_eq!(mcause_code(MCAUSE_INTERRUPT_BIT | 0xF000), 0);
    }
}
