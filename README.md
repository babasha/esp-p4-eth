# esp-p4-eth

`#![no_std]` async Ethernet MAC driver for **ESP32-P4** RMII designs, plug-in
compatible with [`embassy-net`](https://crates.io/crates/embassy-net).

> **Status:** ready for **0.1.0** crates.io release as of 2026-04-29.
> End-to-end ping, TCP, and UDP all work on the Waveshare ESP32-P4-ETH dev
> board (IP101GRI PHY) at 100 Mbps full duplex. Both `embassy-time`
> (SYSTIMER) and EMAC RX/TX wakers can run IRQ-driven via the on-chip CLIC —
> the executor genuinely sleeps on `wfi` when there's no work to do.
> Cold-boot and warm-reboot reliability validated 100 % across stress runs
> (30/30 warm reboots on the `embassy_tcp_soak` harness, 5/5 power-cycle
> on the canonical examples). 180 host-side unit tests pass on
> `x86_64-unknown-linux-gnu`. The public API surface (`Ethernet`, `Device`,
> `Runner`, `BoardConfig`, the `pub mod diag` observability atomics)
> is what we expect to commit to for 0.x SemVer.

## Why this exists

As of late April 2026, the official [`esp-hal`](https://github.com/esp-rs/esp-hal)
does not include `soc/esp32p4/` EMAC support — the P4 simply isn't covered by
the upstream HAL crates yet. This repository is a self-contained, hand-rolled
Synopsys DesignWare GMAC driver targeting the P4 silicon directly: clock tree,
IO_MUX + GPIO matrix routing, DMA descriptor rings with cache coherency,
MDIO + IP101 PHY, and an `embassy-net-driver-channel` adaptor that drops into
any `embassy-net` stack.

## Features

- 100 Mbps full duplex RMII, `embassy-net` `Driver` via
  `embassy-net-driver-channel`
- Two `embassy-time` driver options on the P4 SYSTIMER:
  - **IRQ-driven** via SYSTIMER alarm 0 → CLIC entry 17, gated by the
    `p4-time-driver-irq` cargo feature. Executor drops to `wfi` between
    deadlines. Drift ~0.002 % over 1 s on the Waveshare board.
  - **Polling** via `time_polling_task`, gated by `p4-time-driver`. Simpler
    but burns one core; kept for comparison and as a `wfi`-free fallback.
- **IRQ-driven EMAC RX/TX wakers** under `p4-time-driver-irq` — DMA
  completion interrupts (Synopsys SBD source → CLIC entry 18) wake the
  embassy-net rx/tx futures. No `wake_by_ref` busy-spin under that feature.
- IPv4 / ARP / ICMP / TCP / UDP all verified end-to-end against a Windows host
  through a consumer router
- All RX frame sizes from 60 up to MTU 1500 bytes round-trip cleanly (single
  1472-byte UDP datagrams included)
- `BoardConfig` abstraction — bring your own pin map, ref-clock pad, and
  PHY MDIO address
- 180 host-side unit tests; the host build target is `x86_64-unknown-linux-gnu`

## Hardware support

| Board                                             | Status   | `BoardConfig`                       |
|---------------------------------------------------|----------|-------------------------------------|
| Waveshare ESP32-P4-ETH (IP101GRI PHY, 25 MHz XO)  | ✅ tested | `BoardConfig::WAVESHARE_P4_ETH`     |
| Other P4 + RMII PHY designs                       | should work | construct your own `BoardConfig` |

### Waveshare ESP32-P4-ETH default pin map

| Signal      | GPIO   |
|-------------|--------|
| TXD0        | 34     |
| TXD1        | 35     |
| TX_EN       | 49     |
| RXD0        | 30     |
| RXD1        | 29     |
| CRS_DV      | 28     |
| MDC         | 31     |
| MDIO        | 52     |
| PHY RESET   | 51 (active low) |
| REF_CLK in  | 50     |
| PHY MDIO addr | 1    |

## Quick start (Waveshare ESP32-P4-ETH)

Build a minimal "ping me" example:

```bash
cargo build \
  --no-default-features \
  --features p4-example,p4-time-driver,embassy-net-icmp \
  --example embassy_static_ping
```

Flash via [`espflash`](https://github.com/esp-rs/espflash) into RAM:

```bash
espflash flash --ram --no-stub --chip esp32p4 \
  --ignore_app_descriptor --monitor \
  target/riscv32imafc-unknown-none-elf/debug/examples/embassy_static_ping
```

The example brings the link up at 100 Mbps full duplex (~2.5 s after reset),
configures itself as `192.168.0.50/24`, and replies to ICMP echo requests.
Adjust `SELF_IP` and `GATEWAY` constants in the example for your subnet.

For the **IRQ-driven** path (recommended for real workloads — no polling
task burning a core, executor sleeps on `wfi`):

```bash
cargo build \
  --no-default-features \
  --features p4-example,p4-time-driver-irq,embassy-net-tcp \
  --example embassy_tcp_echo_irq
```

This routes SYSTIMER alarms through CLIC entry 17 and EMAC RX/TX completion
interrupts through CLIC entry 18, both dispatched by a single trap entry
(`_p4_eth_trap_entry`) defined in `src/time_driver_irq.rs`.

## Mandatory P4-specific build configuration

ESP32-P4 has two hardware constraints that the calling crate **must** honour:

1. **DMA-shared statics must live below `0x4FF80000`.** The upper 256 KB of
   HP SRAM is the L2 cache backing region; bus masters (the EMAC DMA) cannot
   read it. Any static that the DMA touches (descriptors, packet buffers,
   `StaticDmaResources`) must be placed in a linker section that resolves to
   the safe range. See `memory.x` and the
   `#[link_section = ".dma_bss"]` annotation in the bundled examples.
2. **Use a workspace `[profile.dev]` with `opt-level = 1`.** With `opt-level = 0`
   the debug `.text` overflows the 192 KB safe-DRAM slab; with `opt-level = "s"`
   inlining gets aggressive enough that the naked-counter MDIO BUSY-poll loop
   completes before the PHY answers and the bus times out.

The ready-made `embassy_static_ping`, `embassy_tcp_echo`, and
`embassy_udp_echo` examples already embed both invariants and serve as
templates.

## Cargo features

| Feature              | Default | Purpose                                                                       |
|----------------------|---------|-------------------------------------------------------------------------------|
| `mock-time`          | yes     | embassy-time mock driver — required for host tests; mutually exclusive with `p4-time-driver*` |
| `embassy-net-tcp`    | yes     | enables the `embassy-net/tcp` socket layer                                    |
| `embassy-net-udp`    | no      | enables the `embassy-net/udp` socket layer                                    |
| `embassy-net-icmp`   | yes     | enables `embassy-net/auto-icmp-echo-reply`                                    |
| `p4-time-driver`     | no      | SYSTIMER-backed polling `embassy-time` driver for `riscv32imafc` targets      |
| `p4-time-driver-irq` | no      | SYSTIMER+CLIC IRQ-driven `embassy-time` driver. Also routes EMAC RX/TX completion IRQs and removes the `wake_by_ref` paths in `eth/mod.rs`. Mutually exclusive with `p4-time-driver` |
| `p4-example`         | no      | gates `[[example]]` blocks; required by every P4 example                      |

For a target build, `--no-default-features` then add what you need, e.g.:

```bash
--features p4-example,p4-time-driver,embassy-net-icmp,embassy-net-udp
```

## Examples

The crate ships with three canonical examples that build with their
required features by default. Bring-up scratch examples (mdio_test,
clk_dump, systimer_probe, etc.) live under `examples/dev/` and are gated
behind the `dev-examples` feature so they don't pollute downstream builds.

### Canonical (`examples/`)

| Example                  | What it shows                                                                                       |
|--------------------------|-----------------------------------------------------------------------------------------------------|
| `embassy_static_ping`    | full embassy-net stack with static IP, ICMP echo reply via the driver (polling time driver)         |
| `embassy_dhcp`           | embassy-net DHCP client + diagnostic atomics dump (polling time driver)                             |
| `embassy_tcp_echo_irq`   | TCP listener on :7777 echoing bytes back, byte-exact for any size 1..1500, IRQ-driven path          |
| `embassy_tcp_soak`       | 4 parallel TCP echo listeners on :7780–:7783 with 60-second `stat_task` snapshots and hourly summary; pair with `examples/dev/soak_driver.py` from the host for byte-exact verification |

Build, e.g.:
```bash
cargo build --no-default-features \
  --features p4-example,p4-time-driver-irq,embassy-net-tcp \
  --example embassy_tcp_echo_irq
```

### Dev / bring-up (`examples/dev/`, requires `dev-examples` feature)

`mdio_test`, `phy_probe`, `clk_dump`, `systimer_probe`, `clic_irq_smoke`,
`embassy_smoke`, `embassy_irq_smoke`, `embassy_time_smoke`, polling
`embassy_tcp_echo` / `embassy_udp_echo`, `embassy_tcp_stress_irq`,
`phy_init_diag` (cold-boot diagnostic, requires `phy-init-debug` feature),
plus `soak_driver.py` host-side companion for `embassy_tcp_soak`.

Build any of them by adding `dev-examples` to the feature list, e.g.:
```bash
cargo build --no-default-features \
  --features p4-example,dev-examples \
  --example mdio_test
```

## P4 CLIC quirks (relevant if you fork the trap entry)

Three non-obvious facts learned the hard way during IRQ bring-up. They're
already encoded in `src/clic.rs` and `src/time_driver_irq.rs`, but worth
flagging if you're poking the trap path or routing additional IRQs:

1. **`mtvec.MODE` is forced to `11` (CLIC mode) in hardware.** Direct /
   Vectored RISC-V modes are not available — writing `addr | 0` reads back
   as `addr | 3`.
2. **Trap-entry base = `mtvec & ~0xFF`.** The low 8 bits are MODE/reserved
   and get clamped. The asm trap entry must be `.balign 256`.
3. **`INTERRUPT_CORE0_<peripheral>_INT_MAP_REG` accepts the CLIC index**
   (`cpu_int_line + 16`), not just the CPU INT line. Writing `1` to map a
   peripheral to "CPU line 1" silently no-ops; you must write `17` for it
   to land in `CLIC_INT_CTRL_REG[17]`.
4. **Don't enable `AIE` in `DMA_INTEN` under IRQ-driven mode.** The
   `RU` bit (RX Buffer Unavailable, sticky) goes high immediately after
   `Ethernet::start()` because the descriptor ring is empty; with AIE
   enabled the abnormal-summary line storms the trap. The driver programs
   `DMA_INTEN = TIE | RIE | NIE` only and leaves abnormal recovery to a
   polling `dma_recovery_task`.

## Performance & footprint

Numbers below are for `embassy_tcp_echo_irq` built in release mode against
crate version 0.1.0 on the Waveshare ESP32-P4-ETH dev board.

| Metric                       | Value                                | Notes                                    |
|------------------------------|--------------------------------------|------------------------------------------|
| `.text` (code)               | 71.5 KB                              | release, debuginfo stripped              |
| `.rodata`                    | 12.6 KB                              |                                          |
| `.bss` (CPU-side)            | 31.3 KB                              | embassy + smoltcp + sockets + diagnostics|
| `.dma_bss` (DMA buffers + descriptors) | 26.1 KB                    | 8 RX + 8 TX × 1536-byte buffers          |
| `.stack` budget              | 76.0 KB                              | embassy task pool reservation            |
| **Total runtime RAM**        | **~134 KB**                          | static + stack budget                    |
| Combined RAM-loaded image    | ~218 KB                              | the whole `--ram --no-stub` payload      |
| Idle CPU                     | **0.024 %** @ 360 MHz                | executor sleeps on `wfi`                 |
| Cold-boot to first link-up   | ~3 s                                 | includes 5 M-cycle PHY oscillator wait   |
| Sustained TCP echo validated | 4 Mbps × 50 s, byte-clean, RBU=0     | window-limited (2 KB sockets), not driver|
| Cold-boot reliability        | 100 % (5/5 power-cycle)              |                                          |
| Warm-reboot reliability      | 100 % (30/30 stress on soak harness) | after the L2 cache mode init fix         |

For an apples-to-apples comparison with the IDF `esp_eth` driver on the
same chip, `MIGRATION_PLAN/ESP32_P4.md` has the canonical IDF baseline
recipe; runtime numbers vs IDF are an open follow-up tracked for 0.2.0.

## Known limitations
- **Throughput ceiling not characterised.** Sustained TCP RX of ~4 Mbps and
  TX of ~3 Mbps over 25 s (Waveshare board, 2 KB socket buffers, host
  through WSL NAT) round-trip cleanly with `RBU = 0`, 1:1 IRQ-to-frame
  ratio, and no descriptor errors. Higher rates and multi-connection
  saturation have not been measured yet — the stress example
  (`embassy_tcp_stress_irq`) is window-limited by 2 KB sockets, and
  `embassy_tcp_soak` has 4 listeners × 4 KB each. A multi-MB/s
  characterisation with 32 KB sockets and a direct cable is a planned
  follow-up.
- **DMA buffer footprint vs IDF.** The driver currently allocates one
  1536-byte buffer per descriptor (8 + 8 = ~26 KB total). IDF defaults to
  20 RX + 10 TX × 512-byte buffers with descriptor chaining for jumbo
  frames (~16 KB). Switching to chained 512-byte buffers would shave
  ~10 KB of static RAM at the cost of more bookkeeping in `descriptors.rs`.
- **Cache writeback uses `_All` instead of `_Addr`.** The chip ROM
  `Cache_WriteBack_Addr` variant returns success but does not actually
  flush data to RAM on the `--ram --no-stub` boot path even after
  `Cache_Set_L2_Cache_Mode` has been re-applied. The driver therefore
  uses `Cache_WriteBack_All` per descriptor — correct, but ~100 µs/frame
  more expensive than IDF's per-address writeback. Identifying the
  remaining init step IDF runs that makes `_Addr` actually flush is a
  planned investigation.
- **DHCP not validated end-to-end.** The driver sends DHCP DISCOVER frames
  correctly but the lab consumer router silently drops them. Static IP works.
- **Half-IRQ EMAC error recovery (cosmetic).** Under `p4-time-driver-irq`,
  normal RX/TX completions are IRQ-driven but `RU` / abnormal recovery
  still lives in the polling `dma_recovery_task`. The stress example shows
  recovery never fires at 3–4 Mbps sustained (rbu+=0 over 50 s of
  continuous traffic), so this is not a correctness issue at SmartBox-scale
  rates — but re-enabling `AIE` with proper debouncing and removing the
  polling task is a future cleanup.

## Contributing

Issues and pull requests welcome. The driver is structured for
side-by-side comparison with ESP-IDF baseline register dumps — when adding
a fix, please include a brief explanation of which register/bit changes and
what wire-level test (ping, UDP echo size, TCP echo size) catches a
regression.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE))
- MIT license ([LICENSE-MIT](../LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
