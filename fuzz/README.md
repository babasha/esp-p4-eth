# esp-p4-eth fuzzing

Targets:
- `rx_receive`
- `tx_transmit`

Examples:

```bash
cargo fuzz run rx_receive --target aarch64-apple-darwin
cargo fuzz run tx_transmit --target aarch64-apple-darwin
```

This repository defaults to the embedded `riscv32imafc-unknown-none-elf` target.
Fuzzing is host-only, so use an explicit host target when running `cargo fuzz`
or checking this crate directly.
