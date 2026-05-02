# Changelog

All notable changes to `esp-p4-eth` will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project will adhere to [Semantic Versioning](https://semver.org/) once
the first 0.x tag ships to crates.io.

## [Unreleased]

### Release readiness — 0.1.0 pre-flight (2026-04-29)
- **`cargo package` clean.** 45 files, 120 KB compressed; `cargo package
  --no-verify` succeeds.
- **180 host-side tests pass** under `cargo test --target
  x86_64-unknown-linux-gnu --lib --features p4-example`.
- **Canonical examples** (`embassy_static_ping`, `embassy_dhcp`,
  `embassy_tcp_echo_irq`, `embassy_tcp_soak`) build clean (release + debug)
  with **zero warnings**.
- **Hardware sanity:** the production-flavour `embassy_tcp_echo_irq.elf`
  (no `phy-init-debug` feature) boots cleanly across a 5/5 warm-reboot
  stress.
- **API audit:** ~130 `pub` items; every documented; `pub mod diag`
  carries an explicit "intentionally not part of the stable API"
  disclaimer.
- **Static footprint vs IDF (release, embassy_tcp_echo_irq):**
  `.text` 71.5 KB, `.rodata` 12.6 KB, `.bss` 31.3 KB, `.dma_bss` 26.1 KB,
  stack budget 76.0 KB, total runtime RAM ~134 KB. Code is roughly
  half the size of the equivalent IDF + LWIP build; static RAM is
  comparable, slightly heavier on DMA buffers (1536 B per descriptor vs
  IDF's 512 B + chaining). See README "Performance & footprint" for the
  full table and known follow-ups.
- **`critical-section` is now an unconditional dependency.** The
  cache-controller wrap in `dma::sync_cache_region` requires it
  regardless of which time-driver feature is enabled.
- **Doc-link housekeeping:** broken intra-doc links to
  `new_from_static_resources_with_board` and `peek_rdes` fixed.

### Reliability
- **Cold-boot / warm-reboot hang root-caused and FULLY RESOLVED**
  (2026-04-28). The intermittent deadlock during `Ethernet::try_new`
  was traced to the chip ROM `Cache_WriteBack_All` helper at
  `0x4FC00414`. Root cause: ESP32-P4's L2 cache controller carries
  `mode / ways / line_size` shadow registers across CPU reset; the ROM
  helpers re-read them and walk the L2 by line index. Without an
  explicit re-program after warm reboot the helpers walk with stale
  parameters and intermittently hang the AHB bus. IDF runs
  `Cache_Set_L2_Cache_Mode` from `cache_hal_init` in the second-stage
  bootloader; we bypass the bootloader via `--ram --no-stub` and skipped
  this step. Fix: new `Dma::init_l2_cache_mode()` helper that calls
  `Cache_Set_L2_Cache_Mode(CACHE_SIZE_256K=10, CACHE_8WAYS_ASSOC=2,
  CACHE_LINE_SIZE_64B=3)` at ROM `0x4FC003D4` followed by
  `Cache_Invalidate_All(CACHE_MAP_L2_CACHE)` at ROM `0x4FC00404`,
  invoked as the very first step of `Ethernet::try_new_with_board`.
  Stress on `embassy_tcp_soak`: 30/30 warm reboots OK = **100 %
  reliability**, up from 5/15 (~33 %) baseline and 23/30 (~77 %) with
  the earlier mitigation set. Cold boot (true power-cycle) was already
  100 % and remains so.

  Earlier mitigation steps shipped in the same release that contributed
  reliability improvements before the L2 mode init landed:
    - `CLOCK_CONFIG_DELAY_SPINS` 1024 → 100 000 cycles in `clock.rs` so
      the EMAC peripheral has time to leave reset before the first MMIO.
    - DMA `TX_DESC_LIST` / `RX_DESC_LIST` register writes moved out of
      `TDesRing::reset` / `RDesRing::reset` into
      `Ethernet::reset_dma::program_dma_descriptor_lists` so the rings no
      longer poke the DMA engine before `Dma::dma_reset` has run.
    - Early `Dma::dma_reset()` inside `try_new_with_board` right after
      `configure_clock_ext_in` so the engine reaches a known-idle state
      before any descriptor work.
    - `critical_section::with` wrapping the ROM cache helper in
      `dma::sync_cache_region` so SYSTIMER / CLIC interrupts cannot land
      while the cache controller is mid-transaction (kept; empirically
      gives a small additional reliability margin).

### Public API
- New `Dma::init_l2_cache_mode()` helper. Call once at driver init
  before any descriptor work — replicates the `cache_hal_init` step IDF
  runs from the second-stage bootloader. `Ethernet::try_new_with_board`
  now invokes it as its very first step.
- New `MacError::DmaBufferAboveCacheBoundary { addr: u32 }` variant
  returned by `Ethernet::try_new_with_board` if any descriptor or buffer
  pointer lands at or above `0x4FF80000` (the L2 cache backing region,
  invisible to bus masters). Compile-time `const _: () = assert!(...)`
  in `eth/mod.rs` keeps the linker `RAM_DMA` region in sync with the
  runtime guard.
- New `pub mod diag` re-exports diagnostic atomics (RX_ARP, RX_IPV4,
  TX_LAST_*, CACHE_*_CALLS / TICKS, etc.) with explicit unstable-API
  doc disclaimer. Crate-root re-exports for those items removed; the
  bundled examples now `use esp_p4_eth::diag::{...}` instead.
- New `phy-init-debug` cargo feature that enables UART markers in
  `Ethernet::try_new_with_board`, `Ethernet::mac_init`,
  `Ethernet::reset_dma`, and `Ip101::init`. Compiles to no-ops when off;
  exists primarily to localise the cold-boot hang point during future
  bring-up but is left in tree for any future ISR / clock-tree
  regression debugging.
- New helpers on `TDesRing` / `RDesRing`: `flush_all` (single bulk
  cache writeback) and `descriptors_ptr` (raw pointer for the DMA-list
  programming step). Existing `reset` no longer touches DMA peripheral
  registers.
- New helpers on `Ethernet`: `reset_descriptor_state` (RAM-only ring
  reinit), `flush_dma_visibility` (single bulk writeback), and
  `program_dma_descriptor_lists` (DMA-list register write). The
  composite `reset_dma` still exists and calls all three; the split is
  available for advanced callers that need to reorder these phases.
- New helper on `Dma`: `flush_caches_all` — one-shot L1+L2 writeback
  via the ROM `_All` variant.

### Build / hygiene
- Removed dead-code constants (`DEFAULT_PHY_ADDR`, `DEFAULT_REF_CLOCK_PIN`
  in `eth/mod.rs`; `ROM_CACHE_WRITEBACK_ADDR` in `dma.rs`). Canonical
  builds now produce zero warnings.
- 19 bring-up scratch examples moved to `examples/dev/` and gated behind
  the new `dev-examples` cargo feature. Three canonical examples
  (`embassy_dhcp`, `embassy_static_ping`, `embassy_tcp_echo_irq`) stay
  in `examples/` and build by default with their required features.
- Added `*.log` to `.gitignore` so UART/test capture archives stay
  local.

### Tooling
- New `examples/embassy_tcp_soak.rs` — 4-listener TCP echo on
  ports 7780–7783 (4 KB sockets each), per-port byte counters,
  60-second `stat_task` with hourly summary line, manual short-write
  drain loop. Built for long-duration reliability validation.
- New `examples/dev/soak_driver.py` — Python 3.12 host driver, 4
  concurrent TCP streams, xorshift32 deterministic stream, byte-exact
  echo verification, MB/s + mismatch reporting per minute. Run:
  `python3 soak_driver.py --host 192.168.0.50 --duration 3600`.

### Tests
- **Host-side test count: 115 → 180** (+65 tests across three refactor
  passes to expose previously target-only modules and async-task pure-logic
  to the `x86_64-unknown-linux-gnu` test harness). Phase 3 (+17) extracts
  pure helpers from `ethernet_task` futures so RX classification, TX header
  snooping, and link-state transitions become host-testable without
  standing up the embassy executor:
  - `classify_rx_frame(buf, len)` — bumps RX_ARP / RX_IPV4 / RX_ICMP /
    RX_DHCP_FRAMES atomics and writes hex-dumps based on frame contents.
    10 tests cover runt frames (< 14 bytes), ARP, IPv4 with TCP, IPv4
    with ICMP, IPv4 with UDP DHCP-67 / DHCP-68 client / server, IPv4 with
    non-DHCP UDP ports, IPv6 (ethertype 0x86DD passes through without
    misclassification), large-frame hex dump, DHCP-frame hex dump.
  - `record_tx_frame_metadata(buf)` — populates TX_LAST_LEN /
    TX_LAST_DST_MAC_HI / TX_LAST_SRC_MAC_HI / TX_LAST_ETHERTYPE. 3 tests
    cover short buf (only length recorded), exact 14-byte minimum
    Ethernet header, oversize frames.
  - `link_state_action(prev, is_up_now) -> LinkAction` — pure state
    machine for `link_fut`. 4 tests cover all four (prev, is_up_now)
    inputs: down→down (NoChange), down→up (AttemptNegotiate — caller
    must run `phy_negotiate` before committing), up→up (NoChange),
    up→down (GoDown).
  - Helper utilities: `read_be_u32`, `hex_dump_into` (factored out of
    the inline rx_fut body — shared by large-frame and DHCP-frame
    hex dumps).
  - The diagnostic atomics (RX_ARP, RX_IPV4, RX_LAST_LARGE_FRAME, etc.)
    and the wake_rx_task / wake_tx_task functions had to be ungated
    from `#[cfg(target_arch = "riscv32")]` so the host-test classifier
    can reach them. They cost ~80 bytes of `.bss` on host builds and
    are no-op on host beyond storing values.
  - `critical-section = { features = ["std"] }` added to
    `[target.x86_64-unknown-linux-gnu.dev-dependencies]` (was
    Windows-only). Required because `embassy_sync::AtomicWaker` uses
    `critical_section::with` internally and now compiles on host too.

  Phase 1 + 2 (+48 tests) was a pair of refactor passes that exposed
  previously target-only modules to the host test harness. Coverage
  broken down:
  - `board.rs`: 0 → 8 tests. WAVESHARE_P4_ETH constant validation
    (PHY addr legal range, RMII / MDIO pin map, reset polarity, ref-clock
    pad), GPIO uniqueness, `Copy + Clone` API guarantee.
  - `dma.rs`: 17 → 22 tests. Cache counter instrumentation: per-op routing,
    SYSTIMER tick wrap-around, multi-call accumulation, u32 overflow wrap.
    `record_cache_call` extracted as a host-testable helper from the
    riscv32-only ROM-call path.
  - `eth/mod.rs`: 31 → 32 tests. `link_poll_interval_stays_in_sane_range`
    runtime guard plus a const-level `assert!` next to `LINK_POLL_INTERVAL_MS`
    that fires at compile time if the constant ever drifts outside [50, 500] ms.
  - `clic.rs`: 0 → 13 tests. Pure-data helpers extracted: `clic_attr_byte`,
    `clic_ctl_byte`, `clic_idx_for_cpu_line`, `int_map_value`,
    `clic_threshold_value`, `clic_ctrl_offset`. INT_MAP register addresses
    cross-checked against IDF v5.3.5 baseline; CLIC entry layout (IP/IE/ATTR/CTL
    byte order) verified.
  - `systimer.rs`: 0 → 10 tests. `target_ticks_split` extracted for HI/LO
    packing. TICK_RATE_HZ = 16 MHz, ticks↔µs round-trip, 20-bit HI mask
    truncation, integer-floor sub-microsecond cases.
  - `time_driver_irq_logic.rs` (new module): 0 → 11 tests. `is_async_irq`,
    `mcause_code`, CPU INT line constants and CLIC index derivations. The
    pure-data parts of the IRQ-driven time driver are now host-testable
    independently of the `p4-time-driver-irq` feature flag.
- All four previously riscv32-only modules (`clic`, `systimer`,
  `time_driver_irq_logic`) now compile on host. MMIO `read_volatile` /
  `write_volatile` calls remain `#[cfg(target_arch = "riscv32")]` so the
  host build is a pure no-op on those code paths; address arithmetic and
  byte-packing logic execute on both targets.

### Fixed
- **`link_poll_delay` was burning ~50 % of wall-clock time on cache ROM
  calls.** The function used a 500-iteration `yield_now` loop to space
  out PHY link-state checks. Each `yield_now` re-polls all `join3`
  children in `ethernet_task`, so every yield triggered an
  `rx_ring.has_packet()` and a `tx_ring.has_capacity()` call — and each
  of those does an L1 + L2 cache invalidate ROM call (~20 µs per pair).
  Result: ~24 400 invalidate calls per second in idle, consuming ~500 ms
  of wall time per second. Replaced the yield loop with a single
  `embassy_time::Timer::after(100 ms)`, which lets the executor park in
  `wfi` between link checks. Measured impact on the Waveshare board
  (espflash `--ram` boot, IRQ time driver):
  | metric | before | after |
  |---|---|---|
  | idle `minstret` | 22.3 M instr/s | 0.062 M instr/s (**360× lower**) |
  | idle invalidate calls | 24 400/s | 15/s (**1 600× lower**) |
  | idle invalidate wall-clock | 500 ms/s (50 %) | 310 µs/s (0.03 %) |
  | CPU @ 360 MHz idle | ~6.4 % | ~0.024 % |
  | CPU @ 360 MHz under 4 Mbps RX | ~6.6 % | ~3.5 % retire + 12 % cache stall |
  RBU stays 0 and TCP byte-clean round-trip is preserved.

### Changed
- `dma_recovery_task` poll interval bumped from 1 ms to 50 ms in
  `embassy_tcp_stress_irq` and `embassy_tcp_echo_irq` (the two examples
  that run under `p4-time-driver-irq`). The recovery loop is empirically
  dead code under the IRQ time driver (rbu = 0 over 50 s of sustained
  3–4 Mbps RX + TX), and a 50 ms recovery window is well below TCP's
  retransmit timer (≥200 ms), so the slower interval cannot mask any
  real-world stall. **Caveat — measured idle CPU did not drop:**
  `minstret` baseline stayed at ~23 M instr/s before and after the
  change, suggesting the dominant idle overhead lives somewhere else
  (most likely the embassy-net `Runner` / smoltcp polling), not in the
  user-space recovery task. 50 ms is kept as the better default for a
  dead-code path even though the empirical CPU saving was nil; future
  investigation into idle-baseline cost should look at embassy-net's
  link-check / smoltcp poll-at scheduling.

### Added
- CPU instrumentation in `embassy_tcp_stress_irq` `stat_task`: per-second
  `mcycle` and `minstret` deltas. **Important:** on the ESP32-P4 Andes
  RV32 core, `mcycle` ticks at 40 MHz wall-clock (XTAL) and does NOT
  halt during `wfi` — useless for CPU%. `minstret` does halt during
  `wfi` (no instructions retire) and is the right signal. Empirical
  baseline: 23 M instructions/s idle, 24.5 M under 4 Mbps RX (≈ 3 000
  instructions per Ethernet frame for the full RX → smoltcp → TCP →
  socket → ACK chain). At assumed 360 MHz HP CPU clock that's ~6–7 %
  CPU under SmartBox-scale load.
- `examples/embassy_tcp_stress_irq.rs` — sustained-rate smoke test on the
  IRQ-driven path. Two independent TCP listeners (`:7780` RX-sink, `:7781`
  TX-source) plus a 1 s `stat_task` printing per-second deltas of bytes,
  frames, EMAC IRQs, RBU recoveries, and cumulative descriptor errors.
  First run on the Waveshare board (host through WSL NAT, 2 KB socket
  buffers): **RX ~4 Mbps × 25 s, TX ~3 Mbps × 25 s, `rbu = 0` end-to-end,
  IRQ-to-frame ratio 1:1, zero runt / oversized / framing errors.** The
  polling `dma_recovery_task` did not fire once over 50 s of continuous
  traffic, confirming it is dead code at SmartBox-scale workloads under
  `p4-time-driver-irq`. Throughput is window-limited (2 KB sockets +
  ~5 ms WSL NAT RTT), not driver-limited.
- **IRQ-driven `embassy-time` driver** (`src/time_driver_irq.rs`,
  `p4-time-driver-irq` cargo feature). SYSTIMER alarm 0 →
  `INTERRUPT_CORE0_SYSTIMER_TARGET0_INT_MAP_REG = 17` → CLIC entry 17 →
  `_p4_eth_trap_entry` (naked-asm, `.balign 256`) → `rust_trap_dispatch`.
  Tick rate 1 µs, drift ~0.002 % over 1 s on the Waveshare board (12×
  more accurate than the polling driver). Mutually exclusive with
  `p4-time-driver`.
- **IRQ-driven EMAC RX/TX wakers** (under `p4-time-driver-irq`). Synopsys
  SBD source (peripheral ID 92, `INTERRUPT_CORE0_BASE + 0x170`) → CLIC
  entry 18 → same trap entry → `Dma::read_interrupt_status()` →
  `crate::wake_rx_task` / `crate::wake_tx_task`. The `eth/mod.rs` `rx_fut`
  and `tx_fut` `wake_by_ref` paths are now gated
  `#[cfg(not(feature = "p4-time-driver-irq"))]` — under the IRQ feature
  they just `Poll::Pending` and let the executor sleep until the ISR
  signals.
- `src/clic.rs` module — CLIC + INTERRUPT_CORE0 register helpers
  (`route_to_clic`, `route_systimer_target0`, `route_emac_sbd`,
  `enable_cpu_int`, `disable_cpu_int`, `clear_pending`, `set_threshold`,
  `read_ctrl_word`). Constants: `CLIC_EXT_INTR_NUM_OFFSET = 16`,
  `INT_MAP_SYSTIMER_TARGET0`, `INT_MAP_EMAC_SBD`.
- `systimer::init_alarm0`, `arm_alarm0`, `disarm_alarm0`, `clear_alarm0`,
  `alarm0_pending` helpers. `init_alarm0` sets `SYSTIMER_CONF.target0_work_en`
  (bit 24) which is **off by default** — without it `INT_RAW.target0`
  never asserts.
- `time_driver_irq::EMAC_IRQS` / `EMAC_IRQ_RX` / `EMAC_IRQ_TX` public
  `AtomicU32` counters for diagnostics.
- `examples/clic_irq_smoke.rs` — bare-bones CLIC IRQ probe (no embassy,
  no driver crate dependency on the trap entry). Useful for any future
  ISR debugging on P4.
- `examples/embassy_irq_smoke.rs` — embassy executor + `Timer::after`
  driven by the IRQ time-driver. Confirms ~16_000_300 ticks per second
  with no polling task spawned.
- `examples/embassy_tcp_echo_irq.rs` — TCP echo on the full IRQ-driven
  path. Confirms RX/TX completion IRQs fire one-per-event (no trap
  storm) and TCP round-trip works end-to-end.
- `BoardConfig` struct grouping `RmiiPinConfig`, `MdioPinConfig`,
  `PhyResetPinConfig`, `RefClockPin`, and PHY MDIO address. Provides
  `BoardConfig::WAVESHARE_P4_ETH` constant for the Waveshare ESP32-P4-ETH
  dev board.
- `Ethernet::try_new_with_board`, `eth::new_with_board`, and
  `eth::new_from_static_resources_with_board` constructors — accept an
  explicit `BoardConfig` so the driver no longer hard-codes the Waveshare
  pin map.
- `embassy_tcp_echo` example: TCP listener on :7777 with byte-exact echo
  loop and `tx`/`rx` frame counters.
- `embassy_udp_echo` example: UDP listener on :7778 with diagnostic
  counters (`RX_LARGE_FRAMES`, `RX_LAST_LARGE_FRAME[16]`,
  `RX_LAST_FRAME_LEN`, `RX_RUNT_FRAMES_TOTAL`,
  `RX_OVERSIZED_FRAMES_TOTAL`, `RX_ERROR_FRAMES_TOTAL`) and a
  hex-dump of the most recent ≥200-byte RX frame.
- `embassy-net-udp` cargo feature gating `embassy-net/udp`.
- Atomic counters in the descriptor pipeline (
  `RX_RUNT_FRAMES_TOTAL`, `RX_OVERSIZED_FRAMES_TOTAL`,
  `RX_ERROR_FRAMES_TOTAL`, `RX_LAST_FRAME_LEN`, `RX_LAST_RDES0`,
  `RX_LARGE_FRAMES`) re-exported from `lib.rs` for downstream
  diagnostics.
- `RX_LAST_LARGE_FRAME[16]` + `RX_LAST_LARGE_FRAME_LEN` for hex-dumping
  the latest large RX frame from outside `ethernet_task`.
- README at the crate root with hardware-support matrix, pin map, quick
  start, mandatory P4-specific build invariants (RAM_DMA section,
  workspace `opt-level = 1`), Cargo features, examples, and known
  limitations.
- `.github/workflows/ci.yml` with host unit tests
  (`x86_64-unknown-linux-gnu`), target build checks
  (`riscv32imafc-unknown-none-elf`) for `embassy_static_ping`,
  `embassy_tcp_echo`, and `embassy_udp_echo`, plus
  `cargo fmt --check` and `cargo clippy`.
- Dual `LICENSE-MIT` + `LICENSE-APACHE` text files at workspace root.

### Changed
- `ethernet_task` now calls `time_driver_irq::enable_emac_irq()` after
  `eth.start()` under `#[cfg(feature = "p4-time-driver-irq")]`. The call
  reprograms `DMA_INTEN = TIE | RIE | NIE` (drops `AIE`) before unmasking
  the CLIC entry. See *Fixed* below for why.
- **Crate renamed** from `smartbox-emac` to `esp-p4-eth`. Library import
  path is now `use esp_p4_eth::…`. Workspace, crate `Cargo.toml`,
  examples, fuzz targets, and internal references all updated.
- **License** changed from `MIT` to `MIT OR Apache-2.0` to match the
  Rust ecosystem standard.
- `dma_op_mode_value()` now returns `TSF | FUF` (was `TSF | RSF`). See
  the *Fixed* section below for rationale.
- `Ethernet::try_new` is now a thin wrapper around
  `Ethernet::try_new_with_board(…, &BoardConfig::WAVESHARE_P4_ETH)` —
  call signature unchanged for the Waveshare board.
- `Cargo.toml` package metadata extended with `description`,
  `repository`, `homepage`, `documentation`, `readme`, `keywords`, and
  `categories`. `publish = false` removed.

### Fixed
- **`AIE` trap-storm under IRQ-driven mode.** When `enable_emac_irq()`
  was first wired up with the default `DMA_INTEN` (which includes `AIE`,
  the abnormal-summary enable), the CPU got stuck in a level-triggered
  trap loop right after `Ethernet::start()` because `RU` (RX Buffer
  Unavailable, sticky) goes high the moment the DMA core looks at its
  empty descriptor ring. Symptoms: `init: all tasks spawned` printed but
  nothing after — no link-up message, no `stat_task` ticks. Fix: the
  IRQ-driven enable path narrows `DMA_INTEN` to `TIE | RIE | NIE`
  (normal-summary only). Abnormal recovery stays with the polling
  `dma_recovery_task` which already pokes `RX_POLL_DEMAND` on RBU.
- **256-byte L2 frame cutoff in RX path.** With `DMA_OP_MODE.RSF = 1`
  (Receive Store and Forward) the GMAC waited for the entire frame in
  the on-chip RX FIFO before transferring it to memory; the P4 RX FIFO
  is too small to hold a full Ethernet frame, so any frame larger than
  the FIFO was silently dropped before reaching the DMA descriptor
  pipeline (`err = runt = oversized = 0`). Symptoms: `ping -l 200`
  worked, `ping -l 210` did not; `embassy_tcp_echo` hung on a single
  1400-byte send and reset after ~30 s; `embassy_udp_echo` timed out
  on any datagram ≥240 bytes. Switching to cut-through receive
  (`RSF = 0`, default RTC threshold of 64 bytes) makes every size from
  60 to MTU 1500 bytes round-trip byte-exact.
- Renamed unit test `dma_op_mode_enables_store_and_forward` →
  `dma_op_mode_uses_tx_store_forward_and_cut_through_rx`; the new test
  asserts `RSF` stays 0.

### Removed
- Repository-private `target_fresh_161820/` debug build directory.
- Stale `examples/dhcp_echo.rs~8728` OneDrive backup file.

## Older history

For pre-rename history (`smartbox-emac` era — bring-up of clock tree,
IO_MUX + GPIO matrix routing fixes, MDIO bus, IP101 PHY, embassy-net
integration, L2 cache region discovery, FUF + ACS-clear + 5M PHY wait,
SYSTIMER `embassy-time` driver), see the git log on `main` and the
project memos under `MIGRATION_PLAN/ESP32_P4.md` in the SmartBox repo.
