//! IRQ-driven `embassy-time-driver` backed by SYSTIMER + CLIC on ESP32-P4.
//!
//! Replaces the polling driver in `time_driver.rs`. The executor drops to
//! `wfi` whenever no task is ready; SYSTIMER alarm 0 fires the soonest pending
//! deadline through CLIC entry 17 → `_p4_eth_trap_entry` → [`rust_trap_dispatch`].
//! The dispatcher drains expired wakers (which signal the executor via the
//! standard `__pender` path) and re-arms for the next outstanding deadline.
//!
//! Hardware quirk worth knowing: P4 forces `mtvec.MODE = 11` (CLIC mode)
//! regardless of writes, and clamps the lower 8 bits — the trap base address
//! is `mtvec & ~0xFF`. The asm trap entry is `.balign 256` so the symbol's
//! address has all-zero low byte. ATTR.SHV = 0 means CLIC delivers all
//! interrupts to that single entry.
//!
//! Tick rate is `1_000_000` (1 µs) to match `embassy-time/tick-hz-1_000_000`.
//! Conversion to/from SYSTIMER ticks lives in [`crate::systimer`].

#![cfg(all(target_arch = "riscv32", feature = "p4-time-driver-irq"))]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

/// Total EMAC IRQs serviced (RX + TX + summary-only). Useful for spotting
/// IRQ storms via diagnostic UART output.
pub static EMAC_IRQS: AtomicU32 = AtomicU32::new(0);
/// EMAC IRQs that signalled an RX completion (DMASTATUS.RI).
pub static EMAC_IRQ_RX: AtomicU32 = AtomicU32::new(0);
/// EMAC IRQs that signalled a TX completion (DMASTATUS.TI).
pub static EMAC_IRQ_TX: AtomicU32 = AtomicU32::new(0);

use crate::time_driver_irq_logic::{
    is_async_irq, mcause_code, CPU_INT_LINE_EMAC, CPU_INT_LINE_SYSTIMER, EMAC_SBD_CLIC_INDEX,
    SYSTIMER_TARGET0_CLIC_INDEX,
};
use crate::{clic, dma::Dma, regs, systimer};

struct P4TimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl Driver for P4TimeDriver {
    fn now(&self) -> u64 {
        systimer::now_us()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow(cs).borrow_mut();
            q.schedule_wake(at, waker);
            arm_next_locked(&mut q);
        });
    }
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: P4TimeDriver = P4TimeDriver {
        queue: Mutex::new(RefCell::new(Queue::new())),
    }
);

/// Drain expired wakers, then arm SYSTIMER alarm 0 for the next deadline,
/// or disarm if no timers are queued. Must be called under the queue
/// `critical_section`.
fn arm_next_locked(q: &mut Queue) {
    let now_us = systimer::now_us();
    let next_us = q.next_expiration(now_us);
    if next_us == u64::MAX {
        systimer::disarm_alarm0();
        return;
    }
    // SYSTIMER ticks at 16 MHz, embassy-time runs at 1 MHz — convert.
    let target_ticks = systimer::us_to_ticks(next_us);
    systimer::arm_alarm0(target_ticks);
}

/// Initialize the IRQ-driven time driver. Must be called once during boot,
/// before spawning any task that uses `embassy_time::Timer`.
///
/// Side effects:
/// - Enables SYSTIMER alarm 0 comparator (`CONF.target0_work_en`).
/// - Routes SYSTIMER_TARGET0 → CPU INT line 1 → CLIC entry 17.
/// - Installs `_p4_eth_trap_entry` into `mtvec`.
/// - Enables `mie.MEIE` and `mstatus.MIE` (global M-mode IRQs).
pub fn init() {
    systimer::init_alarm0();
    clic::route_systimer_target0(SYSTIMER_TARGET0_CLIC_INDEX);
    clic::set_threshold(0);
    clic::enable_cpu_int(CPU_INT_LINE_SYSTIMER, 1);

    extern "C" {
        fn _p4_eth_trap_entry();
    }
    let addr = _p4_eth_trap_entry as *const () as usize;
    unsafe {
        core::arch::asm!("csrw mtvec, {}", in(reg) addr);
        let mie_meie: usize = 1 << 11;
        core::arch::asm!("csrs mie, {}", in(reg) mie_meie);
        let mstatus_mie: usize = 1 << 3;
        core::arch::asm!("csrs mstatus, {}", in(reg) mstatus_mie);
    }
}

/// Route the EMAC main interrupt (Synopsys SBD source) through CLIC. Call
/// AFTER `Ethernet::start()` (which programs DMA_INTEN with TIE/RIE/etc.)
/// but BEFORE any RX/TX work depends on wakers being signalled by IRQ.
///
/// On RX completion the dispatcher calls `crate::wake_rx_task`; on TX
/// completion `crate::wake_tx_task`. The full DMA status word is also
/// drained and cleared, matching what `dma_recovery_task` would otherwise
/// do via polling.
pub fn enable_emac_irq() {
    // Narrow DMA_INTEN to NORMAL summary only (TI/RI/TU/ERI) — drop AIE so
    // abnormal-status bits like RU (RX Buffer Unavailable, sticky) cannot
    // fire IRQs in a tight loop right after start when the RX ring is still
    // empty. The polling `dma_recovery_task` continues to handle abnormal
    // recovery via direct register reads.
    let inten = regs::bits::dmainten::TIE
        | regs::bits::dmainten::RIE
        | regs::bits::dmainten::NIE;
    regs::write(regs::dma::INT_EN, inten);
    // Clear any sticky DMA status latched during start before unmasking the
    // CLIC entry, so the very first interrupt actually corresponds to a
    // real RX/TX completion and not to setup noise.
    Dma::clear_interrupt_status(Dma::read_interrupt_status());

    clic::route_emac_sbd(EMAC_SBD_CLIC_INDEX);
    clic::enable_cpu_int(CPU_INT_LINE_EMAC, 1);
}

// Trap entry: saves integer caller-saved + mepc/mcause, calls
// `rust_trap_dispatch(mcause)`, restores, mret. F-registers are NOT saved —
// the dispatcher must avoid floating-point. `.balign 256` is mandatory: P4
// hardware clamps mtvec's low 8 bits, so the trap base address is
// `mtvec & ~0xFF`. If this symbol isn't 256-byte aligned the CPU jumps to
// junk on the first interrupt and hangs.
core::arch::global_asm!(
    r#"
.section .trap.start, "ax"
.global _p4_eth_trap_entry
.balign 256
_p4_eth_trap_entry:
    addi sp, sp, -80
    sw ra,   0(sp)
    sw t0,   4(sp)
    sw t1,   8(sp)
    sw t2,  12(sp)
    sw t3,  16(sp)
    sw t4,  20(sp)
    sw t5,  24(sp)
    sw t6,  28(sp)
    sw a0,  32(sp)
    sw a1,  36(sp)
    sw a2,  40(sp)
    sw a3,  44(sp)
    sw a4,  48(sp)
    sw a5,  52(sp)
    sw a6,  56(sp)
    sw a7,  60(sp)
    csrr t0, mepc
    sw t0, 64(sp)
    csrr t0, mcause
    sw t0, 68(sp)

    csrr a0, mcause
    call rust_trap_dispatch

    lw t0, 64(sp)
    csrw mepc, t0

    lw ra,   0(sp)
    lw t0,   4(sp)
    lw t1,   8(sp)
    lw t2,  12(sp)
    lw t3,  16(sp)
    lw t4,  20(sp)
    lw t5,  24(sp)
    lw t6,  28(sp)
    lw a0,  32(sp)
    lw a1,  36(sp)
    lw a2,  40(sp)
    lw a3,  44(sp)
    lw a4,  48(sp)
    lw a5,  52(sp)
    lw a6,  56(sp)
    lw a7,  60(sp)
    addi sp, sp, 80
    mret
"#
);

/// Trap dispatcher called from `_p4_eth_trap_entry`. Identifies the source
/// from `mcause`, clears it, drains expired wakers (which signal the
/// executor via the standard waker path), and re-arms SYSTIMER for the
/// next outstanding deadline.
#[no_mangle]
extern "C" fn rust_trap_dispatch(mcause: u32) {
    if !is_async_irq(mcause) {
        // Synchronous exception. We have no recovery path here — leave
        // mepc/registers alone and return; the executor loop will spin
        // and the operator will notice the wedge via UART.
        return;
    }

    let code = mcause_code(mcause);
    match code {
        x if x == SYSTIMER_TARGET0_CLIC_INDEX => {
            // Drop SYSTIMER's INT_ST first → CLIC level signal deasserts.
            // Disarm via INT_ENA last so main can arm a fresh deadline
            // without re-firing.
            systimer::clear_alarm0();
            clic::clear_pending(CPU_INT_LINE_SYSTIMER);
            systimer::disarm_alarm0();

            critical_section::with(|cs| {
                let mut q = DRIVER.queue.borrow(cs).borrow_mut();
                arm_next_locked(&mut q);
            });
        }
        x if x == EMAC_SBD_CLIC_INDEX => {
            // Read + clear DMA status, then signal the embassy-net rx/tx
            // futures via the existing wakers. We do NOT touch the eth
            // Mutex here — the awoken futures will lock it on the executor
            // thread and call eth.receive() / eth.transmit() normally.
            //
            // DMA_INTEN is configured in `enable_emac_irq` to mask AIE so
            // sticky abnormal bits like RU (RX Buffer Unavailable, set
            // immediately when the ring is empty after start) cannot fire
            // a trap storm. `dma_recovery_task` continues to nudge poll
            // demand on RBU via direct register polling.
            let status = Dma::read_interrupt_status();
            Dma::clear_interrupt_status(status);
            clic::clear_pending(CPU_INT_LINE_EMAC);

            EMAC_IRQS.fetch_add(1, Ordering::Relaxed);
            if status.has_rx_interrupt() {
                EMAC_IRQ_RX.fetch_add(1, Ordering::Relaxed);
                crate::wake_rx_task();
            }
            if status.has_tx_interrupt() {
                EMAC_IRQ_TX.fetch_add(1, Ordering::Relaxed);
                crate::wake_tx_task();
            }
        }
        _ => {
            // Unknown source — disable to avoid storms.
            clic::disable_cpu_int(CPU_INT_LINE_SYSTIMER);
            clic::disable_cpu_int(CPU_INT_LINE_EMAC);
        }
    }
}
