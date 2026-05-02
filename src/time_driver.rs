//! `embassy-time-driver` implementation backed by the ESP32-P4 SYSTIMER.
//!
//! Polling variant — no IRQ. Pending wakeups sit in a `Queue` that
//! [`time_polling_task`] drains on every executor poll. The polling task must
//! be spawned once on the executor, and another always-ready task (or the
//! ethernet loop) must keep the executor from going idle, otherwise wakeups
//! never fire. This is a stop-gap until the IRQ-driven variant lands.
//!
//! Tick rate is `1_000_000` (1 µs) to match `embassy-time/tick-hz-1_000_000`.
//! Conversion to/from SYSTIMER ticks lives in [`crate::systimer`].

#![cfg(all(target_arch = "riscv32", feature = "p4-time-driver"))]

use core::cell::RefCell;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::systimer;

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
        });
    }
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: P4TimeDriver = P4TimeDriver {
        queue: Mutex::new(RefCell::new(Queue::new())),
    }
);

/// Polling task that wakes any expired timers. Spawn once on the executor.
#[embassy_executor::task]
pub async fn time_polling_task() {
    loop {
        embassy_futures::yield_now().await;
        let now = systimer::now_us();
        critical_section::with(|cs| {
            let mut q = DRIVER.queue.borrow(cs).borrow_mut();
            let _ = q.next_expiration(now);
        });
    }
}
