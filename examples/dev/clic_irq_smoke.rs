#![no_std]
#![no_main]

//! Bring-up smoke for CLIC IRQ routing on ESP32-P4. This example is the first
//! step toward an IRQ-driven embassy-time driver — see
//! `reference_p4_clic_irq_routing.md` for the full plan.
//!
//! What it does:
//! - Programs SYSTIMER alarm 0 to fire 1 second in the future.
//! - Routes SYSTIMER_TARGET0 → CPU INT line 1, configures CLIC entry 17 for
//!   level-positive trigger at priority 1.
//! - Installs a naked-asm trap entry, sets `mtvec` to it (Direct mode), and
//!   enables `mstatus.MIE`.
//! - Drops to `wfi` in `main`. On each IRQ the dispatcher clears the alarm,
//!   sets a flag, and the main loop prints elapsed ticks + re-arms for the
//!   next second.
//!
//! Success criterion: UART prints `GOT IRQ #N at ...` once per second with
//! deltas ~16_000_000 ticks. Failure modes:
//! - No "armed" message → SYSTIMER programming wrong (check `arm_alarm0`).
//! - Hangs at `wfi` and never wakes → CLIC routing is wrong; the alarm fires
//!   in HW but never reaches the CPU. Check INTERRUPT_CORE0 mapping value
//!   matches the CLIC entry being enabled.
//! - "GOT TRAP cause=..." with code != expected → cause encoding differs from
//!   plain RISC-V; CLIC index lives in `mcause[11:0]`.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};

use esp_p4_eth::{clic, systimer};
use panic_halt as _;
use riscv_rt::entry;

// ---- UART0 (debug printf) -----------------------------------------------

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

struct Uart0;

impl Uart0 {
    fn write_byte(&mut self, byte: u8) {
        while txfifo_count() >= UART_TXFIFO_CAPACITY {
            spin_loop();
        }
        unsafe { write_volatile(UART_FIFO_REG, byte as u32) };
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

// ---- IRQ state shared with the trap dispatcher --------------------------

/// Number of SYSTIMER_TARGET0 IRQs serviced. Bumped from the ISR.
static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
/// SYSTIMER tick value at the most recent IRQ — low 32 bits only. The 16 MHz
/// counter wraps in u32 every ~268 s, longer than this smoke test runs.
static IRQ_LAST_TICKS_LO: AtomicU32 = AtomicU32::new(0);
/// `mcause` of the most recent trap, for diagnostics.
static LAST_MCAUSE: AtomicU32 = AtomicU32::new(0);
/// Bumped on EVERY trap entry, IRQ or exception. If this exceeds IRQ_COUNT
/// significantly the dispatcher is being re-entered for an unexpected cause.
static TRAP_COUNT: AtomicU32 = AtomicU32::new(0);

// ---- Naked-asm trap entry -----------------------------------------------
//
// Saves the integer caller-saved set + mepc/mcause/mstatus, then calls
// `rust_trap_dispatch(mcause)`. F-registers are NOT saved — the dispatcher
// must avoid floating-point operations. RV32IMAFC has F-ext available, but
// our code paths don't use it.

// Trap entry: saves the integer caller-saved set + mepc/mcause/mstatus,
// then calls `rust_trap_dispatch(mcause)`. F-registers are NOT saved — the
// dispatcher must avoid floating-point operations.
//
// IMPORTANT P4 CLIC quirk: hardware forces `mtvec.MODE = 11` (CLIC mode)
// regardless of what we write. The lower 8 bits of mtvec are reserved /
// MODE bits and get clamped, so the trap base address the CPU jumps to is
// `mtvec & ~0xFF`. We MUST `.balign 256` so the symbol's address has all
// low 8 bits zero — otherwise the CPU jumps to whatever code lives at the
// 256-byte-rounded-down address.
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

extern "C" {
    fn _p4_eth_trap_entry();
}

const CPU_INT_LINE: u8 = 1;
const SYSTIMER_TARGET0_CLIC_INDEX: u32 = (CPU_INT_LINE as u32) + (clic::CLIC_EXT_INTR_NUM_OFFSET as u32);
const MCAUSE_INTERRUPT_BIT: u32 = 1 << 31;
const MCAUSE_CODE_MASK: u32 = 0x0FFF;

#[no_mangle]
extern "C" fn rust_trap_dispatch(mcause: u32) {
    let n = TRAP_COUNT.fetch_add(1, Ordering::Relaxed);
    LAST_MCAUSE.store(mcause, Ordering::Relaxed);

    // Hard cap to prevent infinite trap storm if our source-clear sequence
    // is wrong. Disables everything and globally masks future IRQs.
    if n >= 50 {
        systimer::disarm_alarm0();
        systimer::clear_alarm0();
        clic::disable_cpu_int(CPU_INT_LINE);
        clic::clear_pending(CPU_INT_LINE);
        unsafe {
            // Clear mstatus.MPIE so mret leaves MIE off after this trap.
            let mpie: usize = 1 << 7;
            core::arch::asm!("csrc mstatus, {}", in(reg) mpie);
        }
        return;
    }

    if mcause & MCAUSE_INTERRUPT_BIT == 0 {
        // Synchronous exception — main reports via diag.
        return;
    }

    let code = mcause & MCAUSE_CODE_MASK;
    if code == SYSTIMER_TARGET0_CLIC_INDEX {
        // Order matters: drop SYSTIMER's INT_ST first so the level-positive
        // line into the CLIC entry deasserts, then explicitly clear the
        // CLIC IP bit, then disarm via INT_ENA so a fresh deadline can be
        // armed by main without an immediate re-fire.
        systimer::clear_alarm0();
        clic::clear_pending(CPU_INT_LINE);
        systimer::disarm_alarm0();

        IRQ_LAST_TICKS_LO.store(systimer::now_ticks() as u32, Ordering::Relaxed);
        IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    } else {
        clic::disable_cpu_int(CPU_INT_LINE);
    }
}

// ---- main -------------------------------------------------------------

fn install_trap_vector() {
    let addr = _p4_eth_trap_entry as usize;
    // Direct mode (mtvec[1:0] = 00). All traps land in our single entry.
    let mtvec = addr & !0b11;
    unsafe { core::arch::asm!("csrw mtvec, {}", in(reg) mtvec) };
}

fn enable_machine_interrupts() {
    unsafe {
        // mie.MEIE = bit 11 → enable Machine External Interrupts.
        let mie_meie: usize = 1 << 11;
        core::arch::asm!("csrs mie, {}", in(reg) mie_meie);
        // mstatus.MIE = bit 3 → global interrupt enable.
        let mstatus_mie: usize = 1 << 3;
        core::arch::asm!("csrs mstatus, {}", in(reg) mstatus_mie);
    }
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== clic_irq_smoke alive ===");

    install_trap_vector();
    let _ = writeln!(
        uart,
        "mtvec installed -> {:#X}",
        _p4_eth_trap_entry as usize
    );

    // 1. Route the peripheral source. IDF writes (cpu_line + 16) — the CLIC
    //    index — into the matrix register, not just the CPU line number.
    clic::route_systimer_target0(SYSTIMER_TARGET0_CLIC_INDEX as u8);
    let _ = writeln!(
        uart,
        "INT_MAP_SYSTIMER_TARGET0 = {} (CLIC idx)",
        SYSTIMER_TARGET0_CLIC_INDEX,
    );

    // 2. CLIC: enable line 1 at priority 1, threshold 0.
    clic::set_threshold(0);
    clic::enable_cpu_int(CPU_INT_LINE, 1);
    let _ = writeln!(
        uart,
        "CLIC ctrl word = {:#010X}",
        clic::read_ctrl_word(CPU_INT_LINE),
    );

    enable_machine_interrupts();
    let mtvec_now: usize;
    let mstatus_now: usize;
    let mie_now: usize;
    unsafe {
        core::arch::asm!("csrr {}, mtvec", out(reg) mtvec_now);
        core::arch::asm!("csrr {}, mstatus", out(reg) mstatus_now);
        core::arch::asm!("csrr {}, mie", out(reg) mie_now);
    }
    let _ = writeln!(
        uart,
        "post-enable: mtvec={:#X} mstatus={:#X} mie={:#X}",
        mtvec_now, mstatus_now, mie_now
    );

    // 3. Arm first alarm 1 second from now (enable comparator first!).
    systimer::init_alarm0();
    let mut next_target = systimer::now_ticks() + systimer::TICK_RATE_HZ;
    systimer::arm_alarm0(next_target);
    let _ = writeln!(uart, "armed alarm at {} ticks (1s out)", next_target);

    let mut last_seen = 0u32;
    let mut last_traps = 0u32;
    let mut diag_deadline = systimer::now_ticks() + systimer::TICK_RATE_HZ * 2;
    loop {
        // Don't wfi yet — poll SYSTIMER + CLIC state too so we can diagnose
        // why the alarm isn't reaching the CPU. Re-enable wfi once this
        // smoke proves IRQs land.
        for _ in 0..1000 {
            spin_loop();
        }

        let irq_count = IRQ_COUNT.load(Ordering::Relaxed);
        let trap_count = TRAP_COUNT.load(Ordering::Relaxed);

        if trap_count != last_traps {
            last_traps = trap_count;
            if trap_count > irq_count {
                let _ = writeln!(
                    uart,
                    "!! UNEXPECTED TRAP: mcause={:#010X} (trap_count={}, irq_count={})",
                    LAST_MCAUSE.load(Ordering::Relaxed),
                    trap_count,
                    irq_count,
                );
            }
        }

        let now = systimer::now_ticks();
        if now >= diag_deadline {
            let int_raw_addr: *const u32 = (0x500E_2000 + 0x68) as *const u32;
            let int_raw = unsafe { read_volatile(int_raw_addr) };
            let int_st_addr: *const u32 = (0x500E_2000 + 0x70) as *const u32;
            let int_st = unsafe { read_volatile(int_st_addr) };
            let int_map = unsafe { read_volatile(clic::INT_MAP_SYSTIMER_TARGET0) };
            let clic_word = clic::read_ctrl_word(CPU_INT_LINE);
            let mip: usize;
            unsafe { core::arch::asm!("csrr {}, mip", out(reg) mip) };
            let _ = writeln!(
                uart,
                "DIAG t={} target={} INT_RAW={:#X} INT_ST={:#X} INT_MAP={} CLIC={:#010X} mip={:#X} traps={} irqs={}",
                now,
                next_target,
                int_raw,
                int_st,
                int_map,
                clic_word,
                mip,
                trap_count,
                irq_count,
            );
            diag_deadline = now + systimer::TICK_RATE_HZ * 2;
        }

        if irq_count != last_seen {
            let last_ticks = IRQ_LAST_TICKS_LO.load(Ordering::Relaxed);
            let _ = writeln!(
                uart,
                "GOT IRQ #{} at lo32={} ticks (mcause={:#010X})",
                irq_count,
                last_ticks,
                LAST_MCAUSE.load(Ordering::Relaxed),
            );
            last_seen = irq_count;

            // Re-arm 1 second past the previous deadline (drift-free cadence).
            next_target = next_target + systimer::TICK_RATE_HZ;
            systimer::arm_alarm0(next_target);

            if irq_count >= 5 {
                let _ = writeln!(uart, "=== clic_irq_smoke OK ===");
                loop {
                    spin_loop();
                }
            }
        }
    }
}
