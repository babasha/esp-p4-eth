//! Ethernet implementation modules staged under a future `esp-hal/src/eth/` layout.
//!
//! The standalone crate still re-exports these modules from the crate root so
//! existing examples keep working, but grouping them here makes the eventual
//! upstream move much more mechanical.

#[cfg(target_arch = "riscv32")]
use core::cell::RefCell;
#[cfg(target_arch = "riscv32")]
use core::future::poll_fn;
#[cfg(target_arch = "riscv32")]
use core::task::Poll;

use static_cell::ConstStaticCell;

#[cfg(target_arch = "riscv32")]
use embassy_futures::{join::join3, yield_now};
// AtomicWaker is needed unconditionally so the RX/TX wakers and their
// host-side wake_rx_task / wake_tx_task tests can exercise the plumbing.
// `Mutex` is only used inside the riscv32-only `ethernet_task`.
use embassy_sync::waitqueue::AtomicWaker;
#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};

use crate::ch;

#[path = "../clock.rs"]
pub mod clock;
#[path = "../descriptors.rs"]
pub mod descriptors;
#[path = "../dma.rs"]
pub mod dma;
#[path = "../phy.rs"]
pub mod phy;
#[path = "../pins.rs"]
pub mod pins;
#[path = "../regs.rs"]
pub mod regs;

use self::{
    clock::{configure_clock_ext_in, configure_speed_divider, disable_emac_clock_tree},
    descriptors::{
        DescriptorError, RDes, RDesRing, RxRingStats, StaticDmaResources, TDes, TDesRing,
        TxRingStats, BUF_SIZE, RX_DESC_COUNT, TX_DESC_COUNT,
    },
    dma::{Dma, DmaInterruptStatus},
    phy::{Ip101, LinkState, Phy, PhyError, Speed},
    pins::{configure_mdio_pin_set, configure_rmii_pin_set},
};

// After `--ram --no-stub` boot the ESP32-P4 ROM leaves the HP CPU/APB at the default 40 MHz that
// the boot ROM itself runs on (no second-stage bootloader has bumped the PLLs yet). With APB at
// 40 MHz the IDF table picks div=3 → MDC = APB / 26 ≈ 1.54 MHz, well within the 2.5 MHz MDIO
// limit. If/when the example installs a custom clock setup that raises APB, this constant must
// be updated to match — too-fast MDC will make the PHY ignore us.
const DEFAULT_AHB_CLOCK_HZ: u32 = 40_000_000;
const MAC_STOP_TIMEOUT_POLLS: usize = 1_000;
const MAC_DEFAULT_IFG_96_BIT_TIMES: u32 = 0;
// IDF v5.3 baseline post-link reads MAC_FRAME_FILTER = 0x10 (PM = pass all
// multicast). HPF (1<<10) only changes hash-vs-perfect filtering semantics
// for the address filter; it does NOT make multicast pass. Without PM the
// MAC drops mDNS, IGMP, IPv6 NS, etc. We follow IDF and set PM by default.
const MACFFILT_DEFAULT: u32 = regs::bits::macffilt::PM;

/// Lowest address of the ESP32-P4 L2 cache backing region. Any descriptor or
/// packet buffer at or above this address is invisible to bus masters: the
/// EMAC DMA cannot read or write the cache backing SRAM.
const DMA_CACHE_BOUNDARY: u32 = 0x4FF8_0000;

// Linker region `RAM_DMA` (memory.x) starts at 0x4FF40000 with LENGTH 0x10000.
// If the boundary is ever bumped or the region is widened, the linker check
// here will fail at compile time before the runtime guard ever runs.
const _: () = assert!(0x4FF4_0000 + 0x1_0000 <= DMA_CACHE_BOUNDARY);
/// PHY link-state poll period. With this set to a real `Timer::after` delay
/// (rather than a `yield_now` loop) the executor parks in `wfi` between
/// link checks, freeing the cycles that would otherwise be burned re-polling
/// `rx_fut.has_packet()` / `tx_fut.has_capacity()` — each of which does an
/// L1+L2 cache invalidate ROM call. 100 ms is responsive enough to catch
/// cable insert/remove without measurable user latency.
const LINK_POLL_INTERVAL_MS: u64 = 100;

// Compile-time guard: keep `link_poll_delay` in the sane range. Catches a
// future contributor reverting to `yield_now × N` (which would either
// remove this const entirely, breaking the compile, or set the interval
// to 0/1 ms which we lower-bound here) or picking an interval so long
// that link-state changes go unnoticed for seconds.
// See `feedback_p4_yield_now_antipattern.md`.
const _: () = {
    assert!(
        LINK_POLL_INTERVAL_MS >= 50 && LINK_POLL_INTERVAL_MS <= 500,
        "LINK_POLL_INTERVAL_MS must stay in [50, 500] ms",
    );
};

/// Duplex mode configured in the MAC and negotiated with the PHY.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Duplex {
    /// Half duplex operation.
    Half,
    /// Full duplex operation.
    Full,
}

/// Default MTU exposed through the embassy channel device.
pub const MTU: usize = BUF_SIZE;
/// Number of receive slots in the embassy driver channel.
pub const CHANNEL_RX_COUNT: usize = 8;
/// Number of transmit slots in the embassy driver channel.
pub const CHANNEL_TX_COUNT: usize = 8;

// `ch::State<MTU, RX, TX>` is ~12 KB on this configuration. The earlier
// `StaticCell::init_with(ch::State::new)` formulation built the value through
// a stack-resident temporary which overflowed the riscv-rt default stack and
// hung the chain `new_from_static_resources → new → init_with → tuple return`
// somewhere around the tuple build. `ch::State::new()` is `const fn`, so we
// can place the storage directly in `.bss` via `ConstStaticCell` and never
// allocate it on the stack. `take()` then hands out `&'static mut` on the
// in-place value, so the construction is zero-cost at startup.
static STATE: ConstStaticCell<ch::State<MTU, CHANNEL_RX_COUNT, CHANNEL_TX_COUNT>> =
    ConstStaticCell::new(ch::State::new());
static RX_WAKER: AtomicWaker = AtomicWaker::new();
static TX_WAKER: AtomicWaker = AtomicWaker::new();

/// Base address of the RX descriptor ring, captured by `ethernet_task::start()`
/// for live debug peeking from outside the eth `Mutex`. Reads as 0 before init.
pub static RX_DESC_BASE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Stride (bytes) between consecutive RDes objects. Matches the descriptor
/// type's `align(128)` (one L2 cache line each).
pub const RX_DESC_STRIDE: usize = 128;

/// Bring-up counters incremented inside [`ethernet_task`]. Reset to 0 at boot.
pub static RX_FRAMES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static TX_FRAMES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static RX_BUF_REQUESTED: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static TX_BUF_REQUESTED: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
/// Per-ethertype counters (RX classification by EtherType field).
pub static RX_ARP: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static RX_IPV4: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static RX_ICMP: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Pending counters set by the rx_fut just before delivering a frame to the
/// embassy channel. Use for RX content snooping from outside ethernet_task.
pub static RX_LAST_ETHERTYPE: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static RX_LAST_DST_MAC_HI: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
/// First 16 u32 words (= 64 bytes) of the most recent IPv4-with-UDP RX frame
/// where source or destination port falls in the DHCP range (67/68). Used
/// to confirm whether the router's DHCP OFFER actually reaches our DMA.
pub static RX_LAST_DHCP_FRAME: [core::sync::atomic::AtomicU32; 16] = [
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
];
pub static RX_DHCP_FRAMES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Diagnostic: first 64 bytes (16 u32 words) of the most recent RX frame whose
/// length is >= 200 bytes. Used to investigate the 200/240 byte cutoff bug.
pub static RX_LAST_LARGE_FRAME: [core::sync::atomic::AtomicU32; 16] = [
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
];
/// Length of the most recent RX frame stored in [`RX_LAST_LARGE_FRAME`].
pub static RX_LAST_LARGE_FRAME_LEN: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
/// TX frame content snooping — set by tx_fut just before handing the frame
/// to the EMAC. Diagnoses smoltcp/embassy-net producing malformed frames.
pub static TX_LAST_LEN: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub static TX_LAST_ETHERTYPE: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static TX_LAST_DST_MAC_HI: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static TX_LAST_SRC_MAC_HI: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// `embassy-net-driver-channel` device handle returned by [`new`].
pub type Device<'d> = ch::Device<'d, MTU>;
/// `embassy-net-driver-channel` runner handle returned by [`new`].
pub type Runner<'d> = ch::Runner<'d, MTU>;

/// Wakes the async RX path when new RX work becomes available.
pub fn wake_rx_task() {
    RX_WAKER.wake();
}

/// Wakes the async TX path when new TX capacity becomes available.
pub fn wake_tx_task() {
    TX_WAKER.wake();
}

/// Errors surfaced by the top-level MAC/device lifecycle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MacError {
    /// PHY initialization or negotiation failed.
    Phy(PhyError),
    /// DMA reset did not complete in time.
    ResetTimeout,
    /// DMA stop sequence did not quiesce in time.
    StopTimeout,
    /// One of the descriptor or buffer arrays handed to the constructor lives
    /// at or above the L2-cache backing region (`0x4FF80000`). Bus masters
    /// cannot read or write that range, so any DMA transfer would silently
    /// fail. Place [`StaticDmaResources`](crate::StaticDmaResources) under
    /// `#[link_section = ".dma_bss"]` (the linker script's `RAM_DMA` region
    /// keeps it below the cache boundary).
    DmaBufferAboveCacheBoundary {
        /// Address that fell outside the DMA-safe range.
        addr: u32,
    },
}

/// Verifies that a descriptor or packet-buffer address lies below the L2
/// cache backing region. Folds to a no-op on host targets where pointers
/// are 64-bit and the chip-specific boundary is irrelevant.
#[inline]
fn check_dma_buffer_address(addr: usize) -> Result<(), MacError> {
    #[cfg(target_arch = "riscv32")]
    {
        if addr >= DMA_CACHE_BOUNDARY as usize {
            return Err(MacError::DmaBufferAboveCacheBoundary { addr: addr as u32 });
        }
    }
    #[cfg(not(target_arch = "riscv32"))]
    {
        let _ = addr;
    }
    Ok(())
}

/// Builds the embassy driver device/runner pair plus an initialized [`Ethernet`].
///
/// This entry point expects the caller to provide `'static` DMA descriptor and
/// packet buffer storage. The constructor configures the default RMII pin bank,
/// selects the default external `REF_CLK` input pin, initializes the MAC, and
/// resets both DMA rings before returning.
pub fn new(
    mac_addr: [u8; 6],
    tx_descriptors: &'static mut [TDes; TX_DESC_COUNT],
    rx_descriptors: &'static mut [RDes; RX_DESC_COUNT],
    tx_buffers: &'static mut [[u8; BUF_SIZE]; TX_DESC_COUNT],
    rx_buffers: &'static mut [[u8; BUF_SIZE]; RX_DESC_COUNT],
) -> (Device<'static>, Runner<'static>, Ethernet<'static>) {
    new_with_board(
        mac_addr,
        tx_descriptors,
        rx_descriptors,
        tx_buffers,
        rx_buffers,
        &crate::board::BoardConfig::WAVESHARE_P4_ETH,
    )
}

/// Same as [`new`] but accepts an explicit [`BoardConfig`].
pub fn new_with_board(
    mac_addr: [u8; 6],
    tx_descriptors: &'static mut [TDes; TX_DESC_COUNT],
    rx_descriptors: &'static mut [RDes; RX_DESC_COUNT],
    tx_buffers: &'static mut [[u8; BUF_SIZE]; TX_DESC_COUNT],
    rx_buffers: &'static mut [[u8; BUF_SIZE]; RX_DESC_COUNT],
    board: &crate::board::BoardConfig,
) -> (Device<'static>, Runner<'static>, Ethernet<'static>) {
    let state = STATE.take();
    let (runner, device) = ch::new(state, ch::driver::HardwareAddress::Ethernet(mac_addr));
    let eth = Ethernet::try_new_with_board(
        mac_addr,
        tx_descriptors,
        rx_descriptors,
        tx_buffers,
        rx_buffers,
        board,
    )
    .expect("EMAC bootstrap failed");

    (device, runner, eth)
}

/// Builds the embassy driver device/runner pair from a single static resource block.
///
/// This is the preferred top-level constructor for examples and board support
/// code because it keeps all DMA-capable memory in one dedicated owner type.
pub fn new_from_static_resources(
    mac_addr: [u8; 6],
    resources: &'static mut StaticDmaResources,
) -> (Device<'static>, Runner<'static>, Ethernet<'static>) {
    new_from_static_resources_with_board(
        mac_addr,
        resources,
        &crate::board::BoardConfig::WAVESHARE_P4_ETH,
    )
}

/// Same as [`new_from_static_resources`] but accepts an explicit [`BoardConfig`].
pub fn new_from_static_resources_with_board(
    mac_addr: [u8; 6],
    resources: &'static mut StaticDmaResources,
    board: &crate::board::BoardConfig,
) -> (Device<'static>, Runner<'static>, Ethernet<'static>) {
    let (tx_descriptors, rx_descriptors, tx_buffers, rx_buffers) = resources.split();
    new_with_board(
        mac_addr,
        tx_descriptors,
        rx_descriptors,
        tx_buffers,
        rx_buffers,
        board,
    )
}

/// Background task that moves packets between the DMA rings and the embassy channel.
///
/// The task starts the MAC/DMA datapath, forwards RX frames into the embassy
/// channel, drains TX frames from the channel into the descriptor ring, and
/// continuously reflects PHY link state changes back into the network stack.
#[embassy_executor::task]
#[cfg(target_arch = "riscv32")]
pub async fn ethernet_task(runner: Runner<'static>, mut eth: Ethernet<'static>) {
    eth.start().expect("EMAC start failed");
    // With the IRQ-driven time driver wired up, route the EMAC main IRQ
    // through CLIC now that DMA_INTEN is programmed. Done here (not in main)
    // so the routing happens AFTER `start()` regardless of how the task
    // is launched.
    #[cfg(feature = "p4-time-driver-irq")]
    crate::time_driver_irq::enable_emac_irq();
    // Capture the RX descriptor list base after start() so external debug code
    // can peek at descriptor state without reaching into the eth Mutex.
    RX_DESC_BASE.store(
        regs::read(regs::dma::RX_DESC_LIST),
        core::sync::atomic::Ordering::Relaxed,
    );

    let (state_runner, mut rx_runner, mut tx_runner) = runner.split();
    let eth = Mutex::<NoopRawMutex, RefCell<Ethernet<'static>>>::new(RefCell::new(eth));

    state_runner.set_link_state(ch::driver::LinkState::Down);

    let rx_fut = async {
        loop {
            let buf = rx_runner.rx_buf().await;
            RX_BUF_REQUESTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            // Kick the RX DMA engine. After it drains the ring and the OWN bits
            // flip to CPU it parks in RX_BUFFER_UNAVAILABLE and stops scanning
            // until a poll-demand write occurs. mdio_test does this every
            // ~100ms; here we do it on every buffer hand-off.
            Dma::demand_rx_poll();

            poll_fn(|cx| {
                RX_WAKER.register(cx.waker());

                let ready = eth.lock(|eth| eth.borrow().rx_ring.has_packet());
                if ready {
                    Poll::Ready(())
                } else {
                    // Without the EMAC IRQ wired up, the only thing that can
                    // wake us is `dma_recovery_task` polling the ring. Force
                    // a re-poll so we don't deadlock waiting on a wake that
                    // never comes. Under `p4-time-driver-irq` the SBD ISR
                    // calls `wake_rx_task()` directly, so this self-wake is
                    // skipped and the executor sleeps on `wfi`.
                    #[cfg(not(feature = "p4-time-driver-irq"))]
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            })
            .await;

            let len = eth.lock(|eth| {
                let mut eth = eth.borrow_mut();
                match eth.receive() {
                    Some((len, data)) => {
                        buf[..len].copy_from_slice(&data[..len]);
                        eth.pop_rx();
                        len
                    }
                    None => 0,
                }
            });

            if len != 0 {
                classify_rx_frame(buf, len);
                rx_runner.rx_done(len);
                RX_FRAMES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                wake_rx_task();
            } else {
                yield_now().await;
            }
        }
    };

    let tx_fut = async {
        loop {
            let buf = tx_runner.tx_buf().await;
            TX_BUF_REQUESTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

            loop {
                poll_fn(|cx| {
                    TX_WAKER.register(cx.waker());

                    let ready = eth.lock(|eth| eth.borrow().tx_ring.has_capacity());
                    if ready {
                        Poll::Ready(())
                    } else {
                        // See rx_fut comment — same rationale.
                        #[cfg(not(feature = "p4-time-driver-irq"))]
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await;

                record_tx_frame_metadata(buf);
                let result = eth.lock(|eth| eth.borrow_mut().transmit(buf));

                match result {
                    Ok(()) => {
                        tx_runner.tx_done();
                        TX_FRAMES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                        wake_tx_task();
                        break;
                    }
                    Err(DescriptorError::RingFull) => {
                        yield_now().await;
                    }
                    Err(_) => {
                        tx_runner.tx_done();
                        break;
                    }
                }
            }
        }
    };

    let link_fut = async {
        let mut link_up = false;

        loop {
            let is_up = eth.lock(|eth| eth.borrow_mut().phy_link_status());

            match link_state_action(link_up, is_up) {
                LinkAction::NoChange => {}
                LinkAction::AttemptNegotiate => {
                    if eth.lock(|eth| eth.borrow_mut().phy_negotiate()).is_ok() {
                        state_runner.set_link_state(ch::driver::LinkState::Up);
                        link_up = true;
                    }
                }
                LinkAction::GoDown => {
                    state_runner.set_link_state(ch::driver::LinkState::Down);
                    link_up = false;
                }
            }

            link_poll_delay().await;
        }
    };

    let _ = join3(rx_fut, tx_fut, link_fut).await;
}

/// Fully-initialized Ethernet MAC + DMA + PHY instance.
pub struct Ethernet<'a> {
    tx_ring: TDesRing<'a>,
    rx_ring: RDesRing<'a>,
    phy: Ip101,
    mac_addr: [u8; 6],
}

impl<'a> Ethernet<'a> {
    /// Creates and initializes the driver from caller-provided DMA memory.
    ///
    /// This infallible convenience constructor uses [`Ethernet::try_new`] and
    /// panics if the low-level EMAC bootstrap fails.
    pub fn new(
        mac_addr: [u8; 6],
        tx_descriptors: &'a mut [TDes; TX_DESC_COUNT],
        rx_descriptors: &'a mut [RDes; RX_DESC_COUNT],
        tx_buffers: &'a mut [[u8; BUF_SIZE]; TX_DESC_COUNT],
        rx_buffers: &'a mut [[u8; BUF_SIZE]; RX_DESC_COUNT],
    ) -> Self {
        Self::try_new(
            mac_addr,
            tx_descriptors,
            rx_descriptors,
            tx_buffers,
            rx_buffers,
        )
        .expect("EMAC bootstrap failed")
    }

    /// Creates and initializes the driver from a [`StaticDmaResources`] block.
    ///
    /// This is the preferred constructor when the DMA backing memory is owned
    /// by a single static resource block.
    pub fn new_from_static_resources(
        mac_addr: [u8; 6],
        resources: &'a mut StaticDmaResources,
    ) -> Self {
        let (tx_descriptors, rx_descriptors, tx_buffers, rx_buffers) = resources.split();
        Self::new(
            mac_addr,
            tx_descriptors,
            rx_descriptors,
            tx_buffers,
            rx_buffers,
        )
    }

    /// Fallible constructor variant for the **Waveshare ESP32-P4-ETH** default
    /// pin map. Equivalent to [`Ethernet::try_new_with_board`] called with
    /// [`BoardConfig::WAVESHARE_P4_ETH`].
    pub fn try_new(
        mac_addr: [u8; 6],
        tx_descriptors: &'a mut [TDes; TX_DESC_COUNT],
        rx_descriptors: &'a mut [RDes; RX_DESC_COUNT],
        tx_buffers: &'a mut [[u8; BUF_SIZE]; TX_DESC_COUNT],
        rx_buffers: &'a mut [[u8; BUF_SIZE]; RX_DESC_COUNT],
    ) -> Result<Self, MacError> {
        Self::try_new_with_board(
            mac_addr,
            tx_descriptors,
            rx_descriptors,
            tx_buffers,
            rx_buffers,
            &crate::board::BoardConfig::WAVESHARE_P4_ETH,
        )
    }

    /// Fallible constructor that takes an explicit [`BoardConfig`].
    ///
    /// Use this for any hardware that isn't the Waveshare ESP32-P4-ETH dev
    /// board: provide your own `BoardConfig` literal with the matching pin
    /// map, reference-clock pin, and PHY MDIO address.
    pub fn try_new_with_board(
        mac_addr: [u8; 6],
        tx_descriptors: &'a mut [TDes; TX_DESC_COUNT],
        rx_descriptors: &'a mut [RDes; RX_DESC_COUNT],
        tx_buffers: &'a mut [[u8; BUF_SIZE]; TX_DESC_COUNT],
        rx_buffers: &'a mut [[u8; BUF_SIZE]; RX_DESC_COUNT],
        board: &crate::board::BoardConfig,
    ) -> Result<Self, MacError> {
        check_dma_buffer_address(tx_descriptors.as_ptr() as usize)?;
        check_dma_buffer_address(rx_descriptors.as_ptr() as usize)?;
        check_dma_buffer_address(tx_buffers.as_ptr() as usize)?;
        check_dma_buffer_address(rx_buffers.as_ptr() as usize)?;

        crate::phy::diag_log("[try_new] enter, init_l2_cache_mode ...");
        // ESP32-P4 L2 cache controller carries `mode/ways/line_size` shadow
        // registers across CPU resets. IDF's `cache_hal_init` re-programs
        // them in the second-stage bootloader; we bypass that via
        // `--ram --no-stub` and the ROM `Cache_WriteBack_*` helpers
        // intermittently hang on warm reboot if the shadow state is stale
        // (root-caused 2026-04-28). Run the same one-shot init here so any
        // subsequent cache writeback / invalidate has a valid configuration
        // to walk.
        Dma::init_l2_cache_mode();
        crate::phy::diag_log("[try_new] L2 cache mode set, configure_rmii_pin_set ...");
        configure_rmii_pin_set(board.rmii_pins);
        crate::phy::diag_log("[try_new] rmii ok, configure_mdio_pin_set ...");
        configure_mdio_pin_set(board.mdio_pins);
        crate::phy::diag_log("[try_new] mdio pins ok, configure_clock_ext_in ...");
        configure_clock_ext_in(board.ref_clock);
        crate::phy::diag_log("[try_new] clock ext in ok, Dma::dma_reset ...");
        // Reset the DMA engine BEFORE any descriptor cache flush. The
        // EMAC may carry over an active DMA state from a previous binary
        // (or from a partial init that the previous CPU reset interrupted),
        // and in that state the L1/L2 cache writeback ROM helpers race
        // with the DMA engine on the same descriptor lines and intermittently
        // hang the AHB bus. Calling `dma_reset` here puts the DMA into a
        // known idle state and removes the ~10 % cold-boot hang rate that
        // showed up at `Dma::flush_descriptor` inside `TDesRing::new`.
        Dma::dma_reset().map_err(|_| MacError::ResetTimeout)?;
        crate::phy::diag_log("[try_new] dma_reset ok, TDesRing::new ...");
        let tx_ring = TDesRing::new(tx_descriptors, tx_buffers);
        crate::phy::diag_log("[try_new] tx_ring ok, RDesRing::new ...");
        let rx_ring = RDesRing::new(rx_descriptors, rx_buffers);
        crate::phy::diag_log("[try_new] rx_ring ok, assembling struct ...");
        let phy = Ip101::new(board.phy_addr);
        let mut ethernet = Self { tx_ring, rx_ring, phy, mac_addr };
        crate::phy::diag_log("[try_new] struct built, calling mac_init ...");
        ethernet.mac_init()?;
        crate::phy::diag_log("[try_new] mac_init ok, set_mac_address ...");
        ethernet.set_mac_address(mac_addr);
        crate::phy::diag_log("[try_new] set_mac_address ok, calling phy.init ...");
        ethernet
            .with_phy(|phy, eth| phy.init(eth))
            .map_err(MacError::Phy)?;
        crate::phy::diag_log("[try_new] phy.init ok, reset_dma ...");
        ethernet.reset_dma();
        crate::phy::diag_log("[try_new] DONE");

        Ok(ethernet)
    }

    /// Returns the currently programmed MAC address.
    pub fn mac_addr(&self) -> [u8; 6] {
        self.mac_addr
    }

    /// Programs the primary station MAC address into hardware.
    pub fn set_mac_address(&mut self, mac_addr: [u8; 6]) {
        self.mac_addr = mac_addr;
        regs::write_mac_address(mac_addr);
    }

    /// Compatibility alias for [`Ethernet::set_mac_address`].
    pub fn set_mac_addr(&mut self, mac_addr: [u8; 6]) {
        self.set_mac_address(mac_addr);
    }

    /// Applies baseline MAC defaults and programs the MDIO CSR clock range.
    ///
    /// This does not start frame traffic yet; it only prepares the MAC core for
    /// later DMA/ring activation.
    pub fn mac_init(&mut self) -> Result<(), MacError> {
        crate::phy::diag_log("[mac_init] enter, Dma::dma_reset() ...");
        Dma::dma_reset().map_err(|_| MacError::ResetTimeout)?;
        crate::phy::diag_log("[mac_init] dma_reset ok, program_mdio_clock_range ...");
        self.program_mdio_clock_range(DEFAULT_AHB_CLOCK_HZ);
        crate::phy::diag_log("[mac_init] mdio clock ok, write MAC CONFIG ...");
        regs::write(regs::mac::CONFIG, mac_default_config());
        crate::phy::diag_log("[mac_init] MAC CONFIG ok, write FRAME_FILTER ...");
        regs::write(regs::mac::FRAME_FILTER, MACFFILT_DEFAULT);
        crate::phy::diag_log("[mac_init] FRAME_FILTER ok, set_mac_address ...");
        self.set_mac_address(self.mac_addr);
        crate::phy::diag_log("[mac_init] DONE");
        Ok(())
    }

    /// Reinitializes the TX and RX descriptor rings, flushes their CPU-side
    /// state out to DMA-visible memory, and programs the DMA peripheral with
    /// their base addresses. Must be called only after [`Dma::dma_reset`]
    /// (e.g. via [`Self::mac_init`]) — writing the descriptor-list registers
    /// or running the L1/L2 cache writeback ROM helper while the EMAC has
    /// not finished reset hangs the AHB bus (intermittent ~10 % rate on
    /// warm reboot, characterised 2026-04-27).
    pub fn reset_dma(&mut self) {
        self.reset_descriptor_state();
        self.flush_dma_visibility();
        self.program_dma_descriptor_lists();
    }

    /// Resets descriptor ring software state (links, ownership, indices) and
    /// programs the DMA descriptor-list peripheral registers — but does
    /// **not** flush caches. Safe to call from `try_new` before the DMA
    /// engine has been started; the cache flush is deferred to `start()`
    /// where the chip has had more time to settle out of warm-reboot state.
    pub fn reset_descriptor_state(&mut self) {
        crate::phy::diag_log("[reset_dma] tx_ring.reset ...");
        self.tx_ring.reset();
        crate::phy::diag_log("[reset_dma] rx_ring.reset ...");
        self.rx_ring.reset();
    }

    /// Pushes ring CPU-side state out to DMA-visible memory using
    /// per-descriptor `Cache_WriteBack_Addr(CACHE_MAP_L2_CACHE, ...)` —
    /// this matches IDF's `cache_ll_l2_writeback_cache_addr` path and
    /// requires `Dma::init_l2_cache_mode` to have been called once at
    /// driver init so the ROM helper has a valid mode/ways/line-size to
    /// work with.
    pub fn flush_dma_visibility(&self) {
        crate::phy::diag_log("[reset_dma] flush descriptors (per-desc L2) ...");
        self.tx_ring.flush_all();
        self.rx_ring.flush_all();
    }

    /// Programs the DMA `TX_DESC_LIST` / `RX_DESC_LIST` peripheral registers
    /// with the current ring base addresses. Must be called only after
    /// `Dma::dma_reset` (e.g. via `mac_init`).
    pub fn program_dma_descriptor_lists(&self) {
        crate::phy::diag_log("[reset_dma] write TX_DESC_LIST ...");
        regs::write(
            regs::dma::TX_DESC_LIST,
            self.tx_ring.descriptors_ptr() as usize as u32,
        );
        crate::phy::diag_log("[reset_dma] write RX_DESC_LIST ...");
        regs::write(
            regs::dma::RX_DESC_LIST,
            self.rx_ring.descriptors_ptr() as usize as u32,
        );
        crate::phy::diag_log("[reset_dma] DONE");
    }

    /// Returns `true` when the PHY currently reports an active link.
    pub fn phy_link_status(&mut self) -> bool {
        matches!(
            self.with_phy(|phy, eth| phy.poll_link(eth)),
            LinkState::Up { .. }
        )
    }

    /// Runs PHY negotiation and applies the resulting speed/duplex to the MAC.
    pub fn phy_negotiate(&mut self) -> Result<(Speed, Duplex), PhyError> {
        let (speed, duplex) = self.with_phy(|phy, eth| phy.negotiate(eth))?;
        self.set_speed(speed);
        self.set_duplex(duplex);
        Ok((speed, duplex))
    }

    /// Programs the MAC and RMII divider for the selected line speed.
    pub fn set_speed(&mut self, speed: Speed) {
        let mut config = regs::read(regs::mac::CONFIG);
        match speed {
            Speed::Mbps100 => {
                config |= regs::bits::maccfg::FES | regs::bits::maccfg::PS;
            }
            Speed::Mbps10 => {
                config = (config & !regs::bits::maccfg::FES) | regs::bits::maccfg::PS;
            }
        }

        regs::write(regs::mac::CONFIG, config);
        configure_speed_divider(speed);
    }

    /// Programs the MAC duplex mode bit.
    pub fn set_duplex(&mut self, duplex: Duplex) {
        let mut config = regs::read(regs::mac::CONFIG);
        match duplex {
            Duplex::Full => config |= regs::bits::maccfg::DM,
            Duplex::Half => config &= !regs::bits::maccfg::DM,
        }

        regs::write(regs::mac::CONFIG, config);
    }

    /// Starts MAC TX/RX and the DMA engines.
    ///
    /// The startup sequence reapplies MAC defaults, initializes DMA operating
    /// modes, rebuilds both descriptor rings, then enables TX/RX in both MAC
    /// and DMA.
    pub fn start(&mut self) -> Result<(), MacError> {
        self.mac_init()?;
        Dma::dma_init();
        self.reset_dma();

        let config =
            regs::read(regs::mac::CONFIG) | regs::bits::maccfg::TE | regs::bits::maccfg::RE;
        regs::write(regs::mac::CONFIG, config);

        let op_mode =
            regs::read(regs::dma::OP_MODE) | regs::bits::dmaopmode::ST | regs::bits::dmaopmode::SR;
        regs::write(regs::dma::OP_MODE, op_mode);
        Ok(())
    }

    /// Stops DMA first and then disables MAC TX/RX.
    ///
    /// The call waits until the DMA state machine reports stopped TX/RX paths
    /// or returns [`MacError::StopTimeout`] if the hardware does not quiesce.
    pub fn stop(&mut self) -> Result<(), MacError> {
        let op_mode = regs::read(regs::dma::OP_MODE)
            & !(regs::bits::dmaopmode::ST | regs::bits::dmaopmode::SR);
        regs::write(regs::dma::OP_MODE, op_mode);

        let config =
            regs::read(regs::mac::CONFIG) & !(regs::bits::maccfg::TE | regs::bits::maccfg::RE);
        regs::write(regs::mac::CONFIG, config);

        wait_for_dma_stop(|| regs::read(regs::dma::STATUS))
    }

    /// Enqueues a frame for transmission through the TX descriptor ring.
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), DescriptorError> {
        self.tx_ring.transmit(frame)
    }

    /// Returns the next received frame, if one is ready in the RX ring.
    pub fn receive(&mut self) -> Option<(usize, &[u8])> {
        self.rx_ring.receive()
    }

    /// Returns the current RX descriptor back to DMA ownership.
    pub fn pop_rx(&mut self) {
        self.rx_ring.pop();
    }

    /// Stops the device and powers down the EMAC clock tree.
    ///
    /// This also disables DMA and MAC interrupt sources so the peripheral can be
    /// cleanly reinitialized later.
    pub fn shutdown(&mut self) {
        let _ = self.stop();
        Dma::disable_interrupts();
        regs::write(regs::mac::INTMASK, 0);
        Dma::clear_interrupt_status(Dma::read_interrupt_status());
        disable_emac_clock_tree();
    }

    /// Returns cumulative TX ring statistics.
    pub fn tx_stats(&self) -> TxRingStats {
        self.tx_ring.stats()
    }

    /// Returns cumulative RX ring statistics.
    pub fn rx_stats(&self) -> RxRingStats {
        self.rx_ring.stats()
    }

    /// Handles one DMA interrupt snapshot and performs lightweight recovery.
    ///
    /// The returned status value is the decoded snapshot that was processed, so
    /// callers can log or inspect the reason for the recovery path.
    pub fn handle_dma_interrupt(&mut self) -> DmaInterruptStatus {
        let status = Dma::read_interrupt_status();
        self.apply_dma_interrupt_status(status);
        Dma::clear_interrupt_status(status);
        status
    }

    fn program_mdio_clock_range(&mut self, ahb_clock_hz: u32) {
        let mut miiaddr = regs::read(regs::mac::MII_ADDR);
        miiaddr &= !regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK;
        miiaddr |= csr_clock_range_bits(ahb_clock_hz) << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT;
        regs::write(regs::mac::MII_ADDR, miiaddr);
    }

    fn with_phy<R>(&mut self, f: impl FnOnce(&mut Ip101, &mut Self) -> R) -> R {
        let mut phy = self.phy;
        let result = f(&mut phy, self);
        self.phy = phy;
        result
    }

    fn apply_dma_interrupt_status(&mut self, status: DmaInterruptStatus) {
        if status.has_fatal_bus_error() {
            Dma::dma_init();
            self.reset_dma();
            let op_mode = regs::read(regs::dma::OP_MODE)
                | regs::bits::dmaopmode::ST
                | regs::bits::dmaopmode::SR;
            regs::write(regs::dma::OP_MODE, op_mode);
        } else if status.has_rx_overflow()
            || status.has_rx_buffer_unavailable()
            || status.has_rx_process_stopped()
        {
            self.rx_ring.handle_overflow();
        }

        if status.has_tx_buffer_unavailable() || status.has_tx_underflow() {
            Dma::demand_tx_poll();
        }

        if status.has_rx_interrupt() || status.has_early_receive_interrupt() {
            Dma::demand_rx_poll();
        }

        #[cfg(target_arch = "riscv32")]
        {
            if status.should_kick_rx() {
                wake_rx_task();
            }

            if status.should_kick_tx() {
                wake_tx_task();
            }
        }
    }
}

impl Drop for Ethernet<'_> {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn mac_default_config() -> u32 {
    regs::bits::maccfg::ACS
        | regs::bits::maccfg::DM
        | regs::bits::maccfg::PS
        | regs::bits::maccfg::FES
        | regs::bits::maccfg::CST
        | (MAC_DEFAULT_IFG_96_BIT_TIMES << regs::bits::maccfg::IFG_SHIFT)
}

fn csr_clock_range_bits(ahb_clock_hz: u32) -> u32 {
    match ahb_clock_hz {
        20_000_000..=34_999_999 => 0b0010,
        35_000_000..=59_999_999 => 0b0011,
        60_000_000..=99_999_999 => 0b0000,
        100_000_000..=149_999_999 => 0b0001,
        150_000_000..=249_999_999 => 0b0100,
        250_000_000..=300_000_000 => 0b0101,
        _ => 0b0001,
    }
}

fn wait_for_dma_stop<F>(mut read_status: F) -> Result<(), MacError>
where
    F: FnMut() -> u32,
{
    for _ in 0..MAC_STOP_TIMEOUT_POLLS {
        let status = read_status();
        let rx_state = (status & regs::bits::dmastatus::RS_MASK) >> regs::bits::dmastatus::RS_SHIFT;
        let tx_state = (status & regs::bits::dmastatus::TS_MASK) >> regs::bits::dmastatus::TS_SHIFT;

        if rx_state == regs::bits::dmastatus::PROCESS_STOPPED
            && tx_state == regs::bits::dmastatus::PROCESS_STOPPED
        {
            return Ok(());
        }
    }

    Err(MacError::StopTimeout)
}

#[cfg(target_arch = "riscv32")]
async fn link_poll_delay() {
    embassy_time::Timer::after(embassy_time::Duration::from_millis(LINK_POLL_INTERVAL_MS)).await;
}

#[cfg(test)]
fn link_poll_interval_ms() -> u64 {
    LINK_POLL_INTERVAL_MS
}

// ---------------------------------------------------------------------------
// Pure-data helpers extracted from `ethernet_task` futures so they can be
// host-tested without standing up an embassy executor + Runner. Each function
// is side-effecting on the global diagnostic atomics defined above.
// ---------------------------------------------------------------------------

/// Read a 4-byte big-endian word at `buf[off..off+4]`. Caller must guarantee
/// `off + 4 <= buf.len()`.
#[inline]
fn read_be_u32(buf: &[u8], off: usize) -> u32 {
    (u32::from(buf[off]) << 24)
        | (u32::from(buf[off + 1]) << 16)
        | (u32::from(buf[off + 2]) << 8)
        | u32::from(buf[off + 3])
}

/// Snapshot up to 16 × 4 bytes of `buf` into `slot`. Used for hex-dumping
/// large or DHCP frames into static storage that diagnostic readers can
/// poll from outside the rx_fut.
#[inline]
fn hex_dump_into(slot: &[core::sync::atomic::AtomicU32; 16], buf: &[u8], len: usize) {
    use core::sync::atomic::Ordering::Relaxed;
    let n = (len / 4).min(16);
    for (i, cell) in slot.iter().take(n).enumerate() {
        cell.store(read_be_u32(buf, i * 4), Relaxed);
    }
}

/// Bumps the per-ethertype RX classifiers and last-* diagnostic atomics
/// based on the contents of a received frame. Pure side-effecting helper —
/// no MMIO, no waker. The 4-byte big-endian destination MAC prefix and the
/// 2-byte ethertype are extracted only when `len >= 14`. ICMP / DHCP
/// detection layers on top of IPv4.
///
/// The caller guarantees `len > 0` (an empty descriptor short-circuits in
/// `rx_fut` before reaching here) and `len <= buf.len()` (the descriptor
/// payload is always the full frame).
pub(crate) fn classify_rx_frame(buf: &[u8], len: usize) {
    use core::sync::atomic::Ordering::Relaxed;

    if len >= 200 {
        RX_LAST_LARGE_FRAME_LEN.store(len as u32, Relaxed);
        hex_dump_into(&RX_LAST_LARGE_FRAME, buf, len);
    }

    if len < 14 {
        return;
    }

    let ethertype = (u16::from(buf[12]) << 8) | u16::from(buf[13]);
    RX_LAST_ETHERTYPE.store(u32::from(ethertype), Relaxed);
    RX_LAST_DST_MAC_HI.store(read_be_u32(buf, 0), Relaxed);

    match ethertype {
        0x0806 => {
            RX_ARP.fetch_add(1, Relaxed);
        }
        0x0800 => {
            RX_IPV4.fetch_add(1, Relaxed);
            // IPv4 header starts at byte 14. Protocol byte at offset 14+9 = 23.
            if len >= 24 && buf[23] == 0x01 {
                RX_ICMP.fetch_add(1, Relaxed);
            }
            // UDP src/dst port at IP+20 + 0/2. Capture DHCP (67/68) frames
            // to a static for diagnosis.
            if len >= 14 + 20 + 4 && buf[23] == 0x11 {
                let src_port = (u16::from(buf[14 + 20]) << 8) | u16::from(buf[14 + 20 + 1]);
                let dst_port = (u16::from(buf[14 + 20 + 2]) << 8) | u16::from(buf[14 + 20 + 3]);
                if src_port == 67 || src_port == 68 || dst_port == 67 || dst_port == 68 {
                    RX_DHCP_FRAMES.fetch_add(1, Relaxed);
                    hex_dump_into(&RX_LAST_DHCP_FRAME, buf, len);
                }
            }
        }
        _ => {}
    }
}

/// Records the headers of an outbound TX frame into the diagnostic atomics
/// (length, dst MAC prefix, src MAC prefix, ethertype). Mirrors the
/// `record_tx_frame_metadata` block from `tx_fut`.
pub(crate) fn record_tx_frame_metadata(buf: &[u8]) {
    use core::sync::atomic::Ordering::Relaxed;

    TX_LAST_LEN.store(buf.len() as u32, Relaxed);
    if buf.len() >= 14 {
        let dst_hi = read_be_u32(buf, 0);
        let src_hi = read_be_u32(buf, 6);
        let etype = (u32::from(buf[12]) << 8) | u32::from(buf[13]);
        TX_LAST_DST_MAC_HI.store(dst_hi, Relaxed);
        TX_LAST_SRC_MAC_HI.store(src_hi, Relaxed);
        TX_LAST_ETHERTYPE.store(etype, Relaxed);
    }
}

/// Action returned by `link_state_action` — captures the three observable
/// transitions of the link state machine in `link_fut`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinkAction {
    /// Link state unchanged — no observable side-effect.
    NoChange,
    /// PHY just came up. The caller must run `phy_negotiate()` and only
    /// flip the channel state on success.
    AttemptNegotiate,
    /// PHY just went down. The caller must report `LinkState::Down` to
    /// embassy-net.
    GoDown,
}

/// Pure state machine for `link_fut`: given the previous tracked link state
/// (`prev_link_up`) and the current PHY readout (`is_up_now`), returns the
/// action to take. Side-effects on the channel runner happen in the caller.
#[inline]
pub(crate) const fn link_state_action(prev_link_up: bool, is_up_now: bool) -> LinkAction {
    match (prev_link_up, is_up_now) {
        (false, true) => LinkAction::AttemptNegotiate,
        (true, false) => LinkAction::GoDown,
        _ => LinkAction::NoChange,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        classify_rx_frame,
        clock::{emac_clock_tree_enabled, emac_reset_asserted},
        csr_clock_range_bits,
        descriptors::{OwnedBy, RDES0_FL_SHIFT, RDES0_FS, RDES0_LS},
        link_poll_interval_ms, link_state_action, mac_default_config, record_tx_frame_metadata,
        wait_for_dma_stop, DmaInterruptStatus, Duplex, Ethernet, LinkAction, MacError, RX_ARP,
        RX_DHCP_FRAMES, RX_ICMP, RX_IPV4, RX_LAST_DHCP_FRAME, RX_LAST_ETHERTYPE,
        RX_LAST_LARGE_FRAME, RX_LAST_LARGE_FRAME_LEN, TX_LAST_DST_MAC_HI, TX_LAST_ETHERTYPE,
        TX_LAST_LEN, TX_LAST_SRC_MAC_HI,
    };
    use crate::{
        regs, zeroed_rx_descriptors, zeroed_tx_descriptors, Speed, StaticDmaResources, BUF_SIZE,
        RX_DESC_COUNT, TX_DESC_COUNT,
    };

    const IRQ_FUZZ_ITERATIONS: usize = 2_048;
    const DOMAIN_MAC: [u8; 6] = [0x02, 0x44, 0x52, 0x49, 0x56, 0x01];

    fn advance_lcg(state: &mut u32) -> u32 {
        *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *state
    }

    fn with_bootstrapped_ethernet<R>(scenario: impl FnOnce(&mut Ethernet<'_>) -> R) -> R {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mut ethernet = Ethernet::new(
            DOMAIN_MAC,
            &mut tx_desc,
            &mut rx_desc,
            &mut tx_buf,
            &mut rx_buf,
        );

        scenario(&mut ethernet)
    }

    #[test]
    fn domain_bootstrap_declares_station_identity_and_empty_rings() {
        with_bootstrapped_ethernet(|ethernet| {
            assert_eq!(ethernet.mac_addr(), DOMAIN_MAC);
            assert_eq!(regs::read(regs::mac::ADDR0_LOW), 0x4952_4402);
            assert_eq!(
                regs::read(regs::mac::ADDR0_HIGH),
                regs::bits::macaddr::AE0 | 0x0000_0156
            );
            assert_eq!(ethernet.tx_stats(), Default::default());
            assert_eq!(ethernet.rx_stats(), Default::default());
            assert_eq!(ethernet.tx_ring.fuzz_current_index(), 0);
            assert_eq!(ethernet.rx_ring.fuzz_current_index(), 0);
        });
    }

    #[test]
    fn domain_transmit_frame_queues_dma_work_and_advances_tx_cursor() {
        with_bootstrapped_ethernet(|ethernet| {
            let frame = [0xA5; 64];

            ethernet.transmit(&frame).unwrap();

            assert_eq!(ethernet.tx_stats().transmitted_frames, 1);
            assert_eq!(ethernet.tx_stats().ring_full_events, 0);
            assert_eq!(ethernet.tx_ring.fuzz_current_index(), 1);
            assert_eq!(regs::read(regs::dma::TX_POLL_DEMAND), 1);
        });
    }

    #[test]
    fn domain_receive_frame_delivers_payload_until_driver_acknowledges_it() {
        with_bootstrapped_ethernet(|ethernet| {
            let payload = [0xC3; 96];
            let status = RDES0_FS | RDES0_LS | ((payload.len() as u32) << RDES0_FL_SHIFT);
            ethernet
                .rx_ring
                .fuzz_seed_current(status, OwnedBy::Cpu, &payload);

            {
                let (len, frame) = ethernet.receive().unwrap();
                assert_eq!(len, payload.len());
                assert_eq!(frame, &payload);
            }

            assert_eq!(ethernet.rx_stats().received_frames, 1);
            assert_eq!(ethernet.rx_ring.fuzz_current_index(), 0);

            ethernet.pop_rx();

            assert_eq!(ethernet.rx_ring.fuzz_current_index(), 1);
        });
    }

    #[test]
    fn domain_rx_overflow_interrupt_rebuilds_receive_path() {
        with_bootstrapped_ethernet(|ethernet| {
            ethernet.apply_dma_interrupt_status(DmaInterruptStatus::from_raw(
                regs::bits::dmastatus::OVF | regs::bits::dmastatus::AIS,
            ));

            assert_eq!(ethernet.rx_stats().overflow_resets, 1);
            assert_eq!(ethernet.rx_ring.fuzz_current_index(), 0);
            assert_eq!(regs::read(regs::dma::RX_POLL_DEMAND), 1);
        });
    }

    #[test]
    fn domain_fatal_dma_error_restores_transmit_path() {
        with_bootstrapped_ethernet(|ethernet| {
            ethernet.apply_dma_interrupt_status(DmaInterruptStatus::from_raw(
                regs::bits::dmastatus::FBI | regs::bits::dmastatus::AIS,
            ));

            assert_ne!(
                regs::read(regs::dma::OP_MODE) & regs::bits::dmaopmode::ST,
                0
            );
            assert_ne!(
                regs::read(regs::dma::OP_MODE) & regs::bits::dmaopmode::SR,
                0
            );
            assert!(ethernet.transmit(&[0x5A; 64]).is_ok());
            assert_eq!(ethernet.tx_stats().transmitted_frames, 1);
        });
    }

    #[test]
    fn bdd_given_bootstrapped_driver_when_station_mac_changes_then_hardware_identity_changes() {
        with_bootstrapped_ethernet(|ethernet| {
            let new_mac = [0x02, 0x10, 0x20, 0x30, 0x40, 0x50];

            ethernet.set_mac_address(new_mac);

            assert_eq!(ethernet.mac_addr(), new_mac);
            assert_eq!(regs::read(regs::mac::ADDR0_LOW), 0x3020_1002);
            assert_eq!(
                regs::read(regs::mac::ADDR0_HIGH),
                regs::bits::macaddr::AE0 | 0x0000_5040
            );
        });
    }

    #[test]
    fn bdd_given_tx_ring_is_full_when_stack_transmits_then_backpressure_is_reported() {
        with_bootstrapped_ethernet(|ethernet| {
            for slot in 0..TX_DESC_COUNT {
                assert_eq!(ethernet.tx_ring.fuzz_current_index(), slot);
                ethernet.transmit(&[slot as u8; 64]).unwrap();
            }

            let result = ethernet.transmit(&[0xEE; 64]);

            assert_eq!(result, Err(super::DescriptorError::RingFull));
            assert_eq!(ethernet.tx_stats().transmitted_frames, TX_DESC_COUNT as u32);
            assert_eq!(ethernet.tx_stats().ring_full_events, 1);
            assert_eq!(ethernet.tx_ring.fuzz_current_index(), 0);
        });
    }

    #[test]
    fn bdd_given_runt_frame_when_driver_receives_then_frame_is_dropped_and_dma_rearmed() {
        with_bootstrapped_ethernet(|ethernet| {
            let payload = [0x11; 32];
            let status = RDES0_FS | RDES0_LS | ((payload.len() as u32) << RDES0_FL_SHIFT);
            ethernet
                .rx_ring
                .fuzz_seed_current(status, OwnedBy::Cpu, &payload);

            let received = ethernet.receive();

            assert!(received.is_none());
            assert_eq!(ethernet.rx_stats().received_frames, 0);
            assert_eq!(ethernet.rx_stats().runt_frames, 1);
            assert_eq!(ethernet.rx_ring.fuzz_current_index(), 1);
        });
    }

    #[test]
    fn bdd_given_rx_and_tx_recovery_bits_when_interrupt_is_handled_then_dma_paths_are_polled() {
        with_bootstrapped_ethernet(|ethernet| {
            regs::write(
                regs::dma::STATUS,
                regs::bits::dmastatus::RI
                    | regs::bits::dmastatus::ERI
                    | regs::bits::dmastatus::TU
                    | regs::bits::dmastatus::UNF
                    | regs::bits::dmastatus::AIS
                    | regs::bits::dmastatus::NIS,
            );

            let status = ethernet.handle_dma_interrupt();

            assert!(status.should_kick_rx());
            assert!(status.should_kick_tx());
            assert_eq!(regs::read(regs::dma::RX_POLL_DEMAND), 1);
            assert_eq!(regs::read(regs::dma::TX_POLL_DEMAND), 1);
            assert_eq!(regs::read(regs::dma::STATUS), status.clear_mask());
        });
    }

    #[test]
    fn bdd_given_running_driver_when_shutdown_is_requested_then_peripheral_is_quiesced() {
        with_bootstrapped_ethernet(|ethernet| {
            ethernet.start().unwrap();

            ethernet.shutdown();

            assert_eq!(regs::read(regs::dma::INT_EN), 0);
            assert_eq!(regs::read(regs::mac::INTMASK), 0);
            assert!(!emac_clock_tree_enabled());
            assert!(emac_reset_asserted());
        });
    }

    #[test]
    fn mac_default_config_matches_expected_defaults() {
        let config = mac_default_config();

        assert_ne!(config & regs::bits::maccfg::ACS, 0);
        assert_ne!(config & regs::bits::maccfg::DM, 0);
        assert_ne!(config & regs::bits::maccfg::FES, 0);
        assert_ne!(config & regs::bits::maccfg::PS, 0);
        assert_ne!(config & regs::bits::maccfg::CST, 0);
        assert_eq!(
            (config & regs::bits::maccfg::IFG_MASK) >> regs::bits::maccfg::IFG_SHIFT,
            0
        );
    }

    #[test]
    fn csr_clock_range_matches_expected_bucket() {
        assert_eq!(csr_clock_range_bits(30_000_000), 0b0010);
        assert_eq!(csr_clock_range_bits(50_000_000), 0b0011);
        assert_eq!(csr_clock_range_bits(80_000_000), 0b0000);
        assert_eq!(csr_clock_range_bits(120_000_000), 0b0001);
        assert_eq!(csr_clock_range_bits(200_000_000), 0b0100);
        assert_eq!(csr_clock_range_bits(300_000_000), 0b0101);
        assert_eq!(csr_clock_range_bits(10_000_000), 0b0001);
    }

    /// Regression guard for the `link_poll_delay` performance bug
    /// (`yield_now × 500` re-polled every join3 sibling and burned ~50 % of
    /// wall time on cache ROM calls). Mirrors the const-level assert next
    /// to the constant itself so the failure mode is visible from the
    /// test report, not just the compile error.
    #[test]
    fn link_poll_interval_stays_in_sane_range() {
        let ms = link_poll_interval_ms();
        assert!(
            ms >= 50,
            "LINK_POLL_INTERVAL_MS = {} ms is too aggressive — \
             see feedback_p4_yield_now_antipattern in project memory",
            ms,
        );
        assert!(
            ms <= 500,
            "LINK_POLL_INTERVAL_MS = {} ms is too long — \
             link state changes will go unnoticed for over half a second",
            ms,
        );
    }

    #[test]
    fn wait_for_dma_stop_succeeds_when_states_reach_stopped() {
        let mut polls = 0;
        let result = wait_for_dma_stop(|| {
            polls += 1;
            if polls < 4 {
                (0b011 << regs::bits::dmastatus::RS_SHIFT)
                    | (0b011 << regs::bits::dmastatus::TS_SHIFT)
            } else {
                0
            }
        });

        assert_eq!(result, Ok(()));
        assert_eq!(polls, 4);
    }

    #[test]
    fn wait_for_dma_stop_times_out_when_dma_keeps_running() {
        let result = wait_for_dma_stop(|| {
            (0b111 << regs::bits::dmastatus::RS_SHIFT) | (0b111 << regs::bits::dmastatus::TS_SHIFT)
        });

        assert_eq!(result, Err(MacError::StopTimeout));
    }

    #[test]
    fn enums_stay_stable() {
        assert_eq!(Speed::Mbps10 as u8, 0);
        assert_eq!(Speed::Mbps100 as u8, 1);
        assert_eq!(Duplex::Half as u8, 0);
        assert_eq!(Duplex::Full as u8, 1);
    }

    #[test]
    fn ethernet_new_from_static_resources_uses_single_dma_resource_owner() {
        regs::reset_test_registers();
        let mut resources = StaticDmaResources::new();

        let ethernet = Ethernet::new_from_static_resources(DOMAIN_MAC, &mut resources);

        assert_eq!(ethernet.mac_addr(), DOMAIN_MAC);
        assert_eq!(ethernet.tx_stats(), Default::default());
        assert_eq!(ethernet.rx_stats(), Default::default());
    }

    #[test]
    fn mac_init_programs_mdio_filter_config_and_station_address() {
        with_bootstrapped_ethernet(|ethernet| {
            regs::write(regs::mac::CONFIG, 0);
            regs::write(regs::mac::FRAME_FILTER, 0);
            regs::write(regs::mac::MII_ADDR, 0);

            ethernet.mac_init().unwrap();

            assert_eq!(regs::read(regs::mac::CONFIG), mac_default_config());
            assert_eq!(regs::read(regs::mac::FRAME_FILTER), super::MACFFILT_DEFAULT);
            assert_eq!(
                (regs::read(regs::mac::MII_ADDR) & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK)
                    >> regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
                csr_clock_range_bits(super::DEFAULT_AHB_CLOCK_HZ)
            );
            assert_eq!(ethernet.mac_addr(), DOMAIN_MAC);
        });
    }

    #[test]
    fn start_enables_mac_and_dma_datapaths() {
        with_bootstrapped_ethernet(|ethernet| {
            ethernet.start().unwrap();

            let mac_config = regs::read(regs::mac::CONFIG);
            let op_mode = regs::read(regs::dma::OP_MODE);
            assert_ne!(mac_config & regs::bits::maccfg::TE, 0);
            assert_ne!(mac_config & regs::bits::maccfg::RE, 0);
            assert_ne!(op_mode & regs::bits::dmaopmode::ST, 0);
            assert_ne!(op_mode & regs::bits::dmaopmode::SR, 0);
            let interrupt_mask = regs::read(regs::dma::INT_EN);
            assert_ne!(interrupt_mask & regs::bits::dmainten::TIE, 0);
            assert_ne!(interrupt_mask & regs::bits::dmainten::RIE, 0);
            assert_ne!(interrupt_mask & regs::bits::dmainten::AIE, 0);
            assert_ne!(interrupt_mask & regs::bits::dmainten::NIE, 0);
        });
    }

    #[test]
    fn stop_disables_mac_and_dma_datapaths_when_hardware_reports_stopped() {
        with_bootstrapped_ethernet(|ethernet| {
            regs::write(
                regs::dma::OP_MODE,
                regs::bits::dmaopmode::ST | regs::bits::dmaopmode::SR,
            );
            regs::write(
                regs::mac::CONFIG,
                regs::bits::maccfg::TE | regs::bits::maccfg::RE | regs::bits::maccfg::DM,
            );
            regs::write(regs::dma::STATUS, 0);

            assert_eq!(ethernet.stop(), Ok(()));

            assert_eq!(
                regs::read(regs::dma::OP_MODE)
                    & (regs::bits::dmaopmode::ST | regs::bits::dmaopmode::SR),
                0
            );
            assert_eq!(
                regs::read(regs::mac::CONFIG) & (regs::bits::maccfg::TE | regs::bits::maccfg::RE),
                0
            );
            assert_ne!(regs::read(regs::mac::CONFIG) & regs::bits::maccfg::DM, 0);
        });
    }

    #[test]
    fn set_speed_updates_speed_bits_and_preserves_unrelated_config() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);

        regs::write(
            regs::mac::CONFIG,
            regs::bits::maccfg::JD | regs::bits::maccfg::DM | regs::bits::maccfg::FES,
        );

        ethernet.set_speed(Speed::Mbps10);
        let config_10 = regs::read(regs::mac::CONFIG);
        assert_eq!(config_10 & regs::bits::maccfg::FES, 0);
        assert_ne!(config_10 & regs::bits::maccfg::PS, 0);
        assert_ne!(config_10 & regs::bits::maccfg::JD, 0);
        assert_ne!(config_10 & regs::bits::maccfg::DM, 0);

        ethernet.set_speed(Speed::Mbps100);
        let config_100 = regs::read(regs::mac::CONFIG);
        assert_ne!(config_100 & regs::bits::maccfg::FES, 0);
        assert_ne!(config_100 & regs::bits::maccfg::PS, 0);
        assert_ne!(config_100 & regs::bits::maccfg::JD, 0);
        assert_ne!(config_100 & regs::bits::maccfg::DM, 0);
    }

    #[test]
    fn set_duplex_only_toggles_duplex_bit() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);

        let base = regs::bits::maccfg::JD | regs::bits::maccfg::PS | regs::bits::maccfg::FES;
        regs::write(regs::mac::CONFIG, base | regs::bits::maccfg::DM);

        ethernet.set_duplex(Duplex::Half);
        assert_eq!(regs::read(regs::mac::CONFIG), base);

        ethernet.set_duplex(Duplex::Full);
        assert_eq!(regs::read(regs::mac::CONFIG), base | regs::bits::maccfg::DM);
    }

    #[test]
    fn ethernet_new_keeps_mac_and_resets_rings() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);

        assert_eq!(ethernet.mac_addr(), mac);
    }

    #[test]
    fn phy_link_status_is_down_on_host_stub() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);

        assert!(!ethernet.phy_link_status());
    }

    #[test]
    fn phy_negotiate_returns_fallback_mode_on_host_stub() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);

        assert_eq!(ethernet.phy_negotiate(), Ok((Speed::Mbps10, Duplex::Half)));
    }

    #[test]
    fn dma_interrupt_status_resets_rx_ring_on_overflow() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);

        ethernet.apply_dma_interrupt_status(DmaInterruptStatus::from_raw(
            regs::bits::dmastatus::OVF | regs::bits::dmastatus::AIS,
        ));

        assert_eq!(ethernet.rx_stats().overflow_resets, 1);
        assert_eq!(ethernet.receive(), None);
    }

    #[test]
    fn dma_interrupt_status_restarts_dma_on_fatal_bus_error() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);
        ethernet.apply_dma_interrupt_status(DmaInterruptStatus::from_raw(
            regs::bits::dmastatus::FBI | regs::bits::dmastatus::AIS,
        ));

        assert!(ethernet.transmit(&[0xAB; 64]).is_ok());
        assert_eq!(ethernet.tx_stats().transmitted_frames, 1);
    }

    #[test]
    fn handle_dma_interrupt_reads_and_clears_status_register() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);
        let raw =
            regs::bits::dmastatus::OVF | regs::bits::dmastatus::RI | regs::bits::dmastatus::AIS;
        regs::write(regs::dma::STATUS, raw);

        let status = ethernet.handle_dma_interrupt();

        assert_eq!(status.raw(), raw);
        assert_eq!(regs::read(regs::dma::STATUS), status.clear_mask());
        assert_eq!(regs::read(regs::dma::RX_POLL_DEMAND), 1);
        assert_eq!(ethernet.rx_stats().overflow_resets, 1);
    }

    #[test]
    fn handle_dma_interrupt_kicks_tx_poll_on_tx_recovery_bits() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);
        regs::write(
            regs::dma::STATUS,
            regs::bits::dmastatus::TU | regs::bits::dmastatus::UNF | regs::bits::dmastatus::AIS,
        );

        ethernet.handle_dma_interrupt();

        assert_eq!(regs::read(regs::dma::TX_POLL_DEMAND), 1);
    }

    #[test]
    fn handle_dma_interrupt_property_fuzz_preserves_driver_invariants() {
        regs::reset_test_registers();
        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);
        let mut rng = 0xA17C_E55Du32;

        for _ in 0..IRQ_FUZZ_ITERATIONS {
            let random = advance_lcg(&mut rng);
            let mut raw = 0u32;

            if random & 0x01 != 0 {
                raw |= regs::bits::dmastatus::OVF;
            }
            if random & 0x02 != 0 {
                raw |= regs::bits::dmastatus::RU;
            }
            if random & 0x04 != 0 {
                raw |= regs::bits::dmastatus::RPS;
            }
            if random & 0x08 != 0 {
                raw |= regs::bits::dmastatus::TU;
            }
            if random & 0x10 != 0 {
                raw |= regs::bits::dmastatus::UNF;
            }
            if random & 0x20 != 0 {
                raw |= regs::bits::dmastatus::RI;
            }
            if random & 0x40 != 0 {
                raw |= regs::bits::dmastatus::ERI;
            }
            if random & 0x80 != 0 {
                raw |= regs::bits::dmastatus::FBI;
            }
            if raw
                & (regs::bits::dmastatus::OVF
                    | regs::bits::dmastatus::RU
                    | regs::bits::dmastatus::RPS
                    | regs::bits::dmastatus::TU
                    | regs::bits::dmastatus::UNF
                    | regs::bits::dmastatus::FBI)
                != 0
            {
                raw |= regs::bits::dmastatus::AIS;
            }
            if raw & (regs::bits::dmastatus::RI | regs::bits::dmastatus::TI) != 0 {
                raw |= regs::bits::dmastatus::NIS;
            }

            regs::write(regs::dma::STATUS, raw);
            let previous_overflows = ethernet.rx_stats().overflow_resets;
            let status = ethernet.handle_dma_interrupt();

            assert_eq!(status.raw(), raw);
            assert_eq!(regs::read(regs::dma::STATUS), status.clear_mask());
            assert_eq!(ethernet.mac_addr(), mac);
            assert!(ethernet.tx_ring.fuzz_current_index() < TX_DESC_COUNT);
            assert!(ethernet.rx_ring.fuzz_current_index() < RX_DESC_COUNT);

            if status.has_rx_overflow()
                || status.has_rx_buffer_unavailable()
                || status.has_rx_process_stopped()
                || status.has_fatal_bus_error()
            {
                assert!(ethernet.rx_stats().overflow_resets >= previous_overflows);
            }
        }
    }

    #[test]
    fn drop_disables_interrupts_and_powers_down_emac() {
        regs::reset_test_registers();
        {
            let mut tx_desc = zeroed_tx_descriptors();
            let mut rx_desc = zeroed_rx_descriptors();
            let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
            let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
            let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

            let mut ethernet =
                Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);
            ethernet.start().unwrap();
        }

        assert_eq!(regs::read(regs::dma::INT_EN), 0);
        assert_eq!(regs::read(regs::mac::INTMASK), 0);
        assert!(!emac_clock_tree_enabled());
        assert!(emac_reset_asserted());
    }

    #[test]
    fn ethernet_can_be_reinitialized_after_drop() {
        regs::reset_test_registers();
        let mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

        {
            let mut tx_desc = zeroed_tx_descriptors();
            let mut rx_desc = zeroed_rx_descriptors();
            let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
            let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];

            let mut ethernet =
                Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);
            ethernet.start().unwrap();
        }

        let mut tx_desc = zeroed_tx_descriptors();
        let mut rx_desc = zeroed_rx_descriptors();
        let mut tx_buf = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
        let mut rx_buf = [[0u8; BUF_SIZE]; RX_DESC_COUNT];

        let mut ethernet = Ethernet::new(mac, &mut tx_desc, &mut rx_desc, &mut tx_buf, &mut rx_buf);
        assert_eq!(ethernet.mac_addr(), mac);
        assert!(ethernet.start().is_ok());
        assert!(emac_clock_tree_enabled());
        assert!(!emac_reset_asserted());
    }

    // -------------------------------------------------------------------
    // Phase 3: rx_fut / tx_fut / link_fut pure-logic helpers.
    //
    // The atomics touched by these tests are global, so a `Mutex` keeps
    // parallel test runs from racing. Each test snapshots the relevant
    // counters before the call and asserts on the delta, so absolute
    // values from earlier (or concurrent) tests are not observed.
    // -------------------------------------------------------------------

    use core::sync::atomic::Ordering::Relaxed;
    use std::sync::Mutex;
    use std::vec;
    use std::vec::Vec;

    static CLASSIFIER_LOCK: Mutex<()> = Mutex::new(());

    /// Build a minimal Ethernet header: 6-byte dst MAC, 6-byte src MAC, 2-byte ethertype.
    fn eth_header(dst: [u8; 6], src: [u8; 6], ethertype: u16) -> Vec<u8> {
        let mut v = Vec::with_capacity(14);
        v.extend_from_slice(&dst);
        v.extend_from_slice(&src);
        v.push((ethertype >> 8) as u8);
        v.push(ethertype as u8);
        v
    }

    /// Build an IPv4 header positioned right after a 14-byte Ethernet header.
    /// Only the protocol byte and total-length are realistic; the rest is
    /// zero-padded — enough to drive `classify_rx_frame`'s 23-byte protocol
    /// check.
    fn ipv4_payload(proto: u8, ip_payload_len: usize) -> Vec<u8> {
        let mut v = vec![0u8; 20 + ip_payload_len];
        v[0] = 0x45; // version=4, IHL=5 (20 bytes)
        v[9] = proto;
        v
    }

    fn snapshot_arp() -> u32 {
        RX_ARP.load(Relaxed)
    }
    fn snapshot_ipv4() -> u32 {
        RX_IPV4.load(Relaxed)
    }
    fn snapshot_icmp() -> u32 {
        RX_ICMP.load(Relaxed)
    }
    fn snapshot_dhcp() -> u32 {
        RX_DHCP_FRAMES.load(Relaxed)
    }

    #[test]
    fn classify_rx_frame_runt_below_14_bytes_skips_classification() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let arp0 = snapshot_arp();
        let ipv4_0 = snapshot_ipv4();

        // 13 bytes — one byte short of an Ethernet header.
        let buf = [0u8; 13];
        classify_rx_frame(&buf, 13);

        assert_eq!(snapshot_arp(), arp0, "runt must not bump RX_ARP");
        assert_eq!(snapshot_ipv4(), ipv4_0, "runt must not bump RX_IPV4");
    }

    #[test]
    fn classify_rx_frame_arp_bumps_arp_counter() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let before = snapshot_arp();

        // ARP request, ethertype 0x0806, padded to 60 bytes (Ethernet min frame).
        let mut buf = eth_header(
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            [0x02, 0x44, 0x52, 0x49, 0x56, 0x01],
            0x0806,
        );
        buf.resize(60, 0);
        classify_rx_frame(&buf, buf.len());

        assert_eq!(snapshot_arp(), before + 1);
        assert_eq!(RX_LAST_ETHERTYPE.load(Relaxed), 0x0806);
    }

    #[test]
    fn classify_rx_frame_ipv4_with_icmp_proto_bumps_both_counters() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let ipv4_before = snapshot_ipv4();
        let icmp_before = snapshot_icmp();
        let dhcp_before = snapshot_dhcp();

        let mut buf = eth_header([0; 6], [0; 6], 0x0800);
        buf.extend_from_slice(&ipv4_payload(0x01, 8)); // ICMP proto
        classify_rx_frame(&buf, buf.len());

        assert_eq!(snapshot_ipv4(), ipv4_before + 1);
        assert_eq!(snapshot_icmp(), icmp_before + 1);
        assert_eq!(
            snapshot_dhcp(),
            dhcp_before,
            "ICMP must not be misclassified as DHCP"
        );
    }

    #[test]
    fn classify_rx_frame_ipv4_with_unknown_proto_bumps_only_ipv4_counter() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let ipv4_before = snapshot_ipv4();
        let icmp_before = snapshot_icmp();

        let mut buf = eth_header([0; 6], [0; 6], 0x0800);
        buf.extend_from_slice(&ipv4_payload(0x06, 16)); // TCP proto, not ICMP
        classify_rx_frame(&buf, buf.len());

        assert_eq!(snapshot_ipv4(), ipv4_before + 1);
        assert_eq!(snapshot_icmp(), icmp_before, "TCP must not bump ICMP");
    }

    #[test]
    fn classify_rx_frame_dhcp_bootp_server_port_67_is_detected() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let before = snapshot_dhcp();

        // Eth (14) + IPv4 (20) + UDP header (8): src=68, dst=67 (DHCP client → server).
        let mut buf = eth_header([0; 6], [0; 6], 0x0800);
        let mut ip = ipv4_payload(0x11, 240); // UDP proto, 8 UDP header + ≥232 payload
        // UDP src/dst ports at IP+20 (= eth_header_len + IP_header_len + 20):
        // Actually offsets are relative to the IP payload's start at index 20.
        ip[20] = 0; // src port hi
        ip[21] = 68; // src port lo (DHCP client)
        ip[22] = 0; // dst port hi
        ip[23] = 67; // dst port lo (DHCP server)
        buf.extend_from_slice(&ip);
        classify_rx_frame(&buf, buf.len());

        assert_eq!(snapshot_dhcp(), before + 1);
    }

    #[test]
    fn classify_rx_frame_dhcp_offer_dst_port_68_is_detected() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let before = snapshot_dhcp();

        let mut buf = eth_header([0; 6], [0; 6], 0x0800);
        let mut ip = ipv4_payload(0x11, 240);
        // DHCP server → client direction: src=67, dst=68.
        ip[21] = 67;
        ip[23] = 68;
        buf.extend_from_slice(&ip);
        classify_rx_frame(&buf, buf.len());

        assert_eq!(snapshot_dhcp(), before + 1);
    }

    #[test]
    fn classify_rx_frame_non_dhcp_udp_is_not_dhcp_classified() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let before = snapshot_dhcp();

        let mut buf = eth_header([0; 6], [0; 6], 0x0800);
        let mut ip = ipv4_payload(0x11, 240);
        // Random UDP ports — neither end is 67/68.
        ip[20] = 0x12;
        ip[21] = 0x34;
        ip[22] = 0x12;
        ip[23] = 0x35;
        buf.extend_from_slice(&ip);
        classify_rx_frame(&buf, buf.len());

        assert_eq!(snapshot_dhcp(), before);
    }

    #[test]
    fn classify_rx_frame_ipv6_skips_classification() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        let arp_before = snapshot_arp();
        let ipv4_before = snapshot_ipv4();

        let mut buf = eth_header([0x33, 0x33, 0, 0, 0, 1], [0; 6], 0x86DD);
        buf.resize(80, 0);
        classify_rx_frame(&buf, buf.len());

        assert_eq!(snapshot_arp(), arp_before);
        assert_eq!(snapshot_ipv4(), ipv4_before);
        assert_eq!(RX_LAST_ETHERTYPE.load(Relaxed), 0x86DD);
    }

    #[test]
    fn classify_rx_frame_large_frame_writes_hex_dump_into_static_slot() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        // Wipe the slot so we can detect what was written.
        for cell in RX_LAST_LARGE_FRAME.iter() {
            cell.store(0, Relaxed);
        }
        RX_LAST_LARGE_FRAME_LEN.store(0, Relaxed);

        // Construct a 250-byte frame; first 4 bytes form a recognisable u32.
        let mut buf = vec![0u8; 250];
        buf[0] = 0xDE;
        buf[1] = 0xAD;
        buf[2] = 0xBE;
        buf[3] = 0xEF;
        // Use ethertype 0x0800 to avoid touching the DHCP path.
        buf[12] = 0x08;
        classify_rx_frame(&buf, buf.len());

        assert_eq!(RX_LAST_LARGE_FRAME_LEN.load(Relaxed), 250);
        assert_eq!(RX_LAST_LARGE_FRAME[0].load(Relaxed), 0xDEAD_BEEF);
    }

    #[test]
    fn classify_rx_frame_dhcp_writes_dump_into_static_slot() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        for cell in RX_LAST_DHCP_FRAME.iter() {
            cell.store(0, Relaxed);
        }

        let mut buf = eth_header([0; 6], [0xCA, 0xFE, 0xBA, 0xBE, 0x00, 0x01], 0x0800);
        buf[0] = 0xAA; // recognisable dst MAC prefix
        buf[1] = 0xBB;
        buf[2] = 0xCC;
        buf[3] = 0xDD;
        let mut ip = ipv4_payload(0x11, 240);
        ip[21] = 67; // DHCP server src
        ip[23] = 68; // DHCP client dst
        buf.extend_from_slice(&ip);
        classify_rx_frame(&buf, buf.len());

        // First 4 bytes of the frame == dst MAC prefix.
        assert_eq!(RX_LAST_DHCP_FRAME[0].load(Relaxed), 0xAABB_CCDD);
    }

    #[test]
    fn record_tx_frame_metadata_short_buf_records_only_length() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        TX_LAST_LEN.store(0, Relaxed);
        TX_LAST_DST_MAC_HI.store(0xDEAD_BEEF, Relaxed);
        TX_LAST_SRC_MAC_HI.store(0xDEAD_BEEF, Relaxed);
        TX_LAST_ETHERTYPE.store(0xDEAD_BEEF, Relaxed);

        let buf = [0u8; 10];
        record_tx_frame_metadata(&buf);

        assert_eq!(TX_LAST_LEN.load(Relaxed), 10);
        // Header fields must remain untouched on short frames.
        assert_eq!(TX_LAST_DST_MAC_HI.load(Relaxed), 0xDEAD_BEEF);
        assert_eq!(TX_LAST_SRC_MAC_HI.load(Relaxed), 0xDEAD_BEEF);
        assert_eq!(TX_LAST_ETHERTYPE.load(Relaxed), 0xDEAD_BEEF);
    }

    #[test]
    fn record_tx_frame_metadata_full_frame_captures_all_header_fields() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();
        TX_LAST_LEN.store(0, Relaxed);

        let buf: [u8; 14] = [
            0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, // dst MAC
            0x11, 0x22, 0x33, 0x44, 0x55, 0x66, // src MAC
            0x08, 0x06, // ethertype = ARP
        ];
        record_tx_frame_metadata(&buf);

        assert_eq!(TX_LAST_LEN.load(Relaxed), 14);
        // dst_hi captures the first 4 bytes of dst MAC big-endian.
        assert_eq!(TX_LAST_DST_MAC_HI.load(Relaxed), 0xAABB_CCDD);
        // src_hi captures the first 4 bytes of src MAC big-endian.
        assert_eq!(TX_LAST_SRC_MAC_HI.load(Relaxed), 0x1122_3344);
        // Ethertype is the 2-byte big-endian field at offsets 12..14.
        assert_eq!(TX_LAST_ETHERTYPE.load(Relaxed), 0x0806);
    }

    #[test]
    fn record_tx_frame_metadata_handles_oversize_frame() {
        let _g = CLASSIFIER_LOCK.lock().unwrap();

        let mut buf = vec![0u8; 1500];
        buf[0] = 0x12;
        buf[12] = 0x86;
        buf[13] = 0xDD;
        record_tx_frame_metadata(&buf);

        assert_eq!(TX_LAST_LEN.load(Relaxed), 1500);
        assert_eq!(TX_LAST_ETHERTYPE.load(Relaxed), 0x86DD);
    }

    #[test]
    fn link_state_action_down_to_down_is_no_change() {
        assert_eq!(link_state_action(false, false), LinkAction::NoChange);
    }

    #[test]
    fn link_state_action_down_to_up_demands_negotiate() {
        // When the PHY first reports up, the caller must negotiate before
        // committing the channel state. Catches the regression where someone
        // skips negotiate and the MAC ends up in the wrong speed/duplex.
        assert_eq!(
            link_state_action(false, true),
            LinkAction::AttemptNegotiate
        );
    }

    #[test]
    fn link_state_action_up_to_up_is_no_change() {
        assert_eq!(link_state_action(true, true), LinkAction::NoChange);
    }

    #[test]
    fn link_state_action_up_to_down_signals_go_down() {
        assert_eq!(link_state_action(true, false), LinkAction::GoDown);
    }

    // NOTE: tests for `wake_rx_task` / `wake_tx_task` are intentionally
    // omitted. Both functions are one-liners (`RX_WAKER.wake()` /
    // `TX_WAKER.wake()`) over the embassy-sync `AtomicWaker`, whose
    // semantics are owned upstream and already tested there. A direct
    // host test would need to coordinate with the existing
    // `handle_dma_interrupt_*` tests which also fire those wakers
    // through the global statics — the synchronisation cost outweighs
    // the value of the assertion.
}
