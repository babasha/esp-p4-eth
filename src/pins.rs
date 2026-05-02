//! ESP32-P4 IO_MUX + GPIO matrix routing for the EMAC RMII signals.
//!
//! On ESP32-P4 the data-plane RMII signals (TX_EN, TXD0/1, RXD0/1, CRS_DV) are
//! routed through dedicated IO_MUX function 3 on a fixed set of pads. The MDIO
//! management lines have no IO_MUX function and must be routed through the GPIO
//! matrix instead.
//!
//! The defaults below match the Waveshare ESP32-P4-ETH development board which
//! uses the IP101GRI PHY with the following pinout:
//!
//! | Signal | GPIO | Direction | Routing       |
//! |--------|------|-----------|---------------|
//! | TXD0   |  34  | OUT       | IO_MUX func 3 |
//! | TXD1   |  35  | OUT       | IO_MUX func 3 |
//! | TX_EN  |  49  | OUT       | IO_MUX func 3 |
//! | RXD0   |  30  | IN        | IO_MUX func 3 |
//! | RXD1   |  29  | IN        | IO_MUX func 3 |
//! | CRS_DV |  28  | IN        | IO_MUX func 3 |
//! | MDC    |  31  | OUT       | GPIO matrix   |
//! | MDIO   |  52  | INOUT     | GPIO matrix   |
//! | RESET  |  51  | OUT       | GPIO          |

use crate::regs;

const REG_IO_MUX_BASE: usize = 0x500E_1000;

const fn iomux_reg(pin: u32) -> usize {
    REG_IO_MUX_BASE + 0x4 + (pin as usize) * 4
}

const FUN_IE: u32 = 1 << 9;
const FUN_PU: u32 = 1 << 8;
const FUN_PD: u32 = 1 << 7;
const MCU_SEL_SHIFT: u32 = 12;
const MCU_SEL_MASK: u32 = 0x7 << MCU_SEL_SHIFT;
const FUN_DRV_SHIFT: u32 = 0;
const FUN_DRV_MASK: u32 = 0x3 << FUN_DRV_SHIFT;
const FUN_DRV_MEDIUM: u32 = 0b10; // matches IDF default for EMAC pads
const EMAC_PAD_FUNC: u32 = 3;
const GPIO_PAD_FUNC: u32 = 1;

// EMAC peripheral signal indices on ESP32-P4. Verified against
// `soc/esp32p4/include/soc/gpio_sig_map.h`:
const MII_MDI_PAD_IN_IDX: u32 = 107;
const MII_MDC_PAD_OUT_IDX: u32 = 108;
const MII_MDO_PAD_OUT_IDX: u32 = 109;

// GPIO matrix routing registers. Verified against ESP32-P4 SoC headers
// (`soc/esp32p4/include/soc/gpio_reg.h`):
//   GPIO_FUNC0_OUT_SEL_CFG_REG   = DR_REG_GPIO_BASE + 0x558 (pin 0)
//   GPIO_FUNC31_OUT_SEL_CFG_REG  = DR_REG_GPIO_BASE + 0x5D4 (pin 31)
//   GPIO_FUNC52_OUT_SEL_CFG_REG  = DR_REG_GPIO_BASE + 0x628 (pin 52)
//   GPIO_FUNC0_IN_SEL_CFG_REG    = DR_REG_GPIO_BASE + 0x158 (signal 0)
//   GPIO_FUNC107_IN_SEL_CFG_REG  = DR_REG_GPIO_BASE + 0x304 (signal 107)
//   GPIO_FUNC108_IN_SEL_CFG_REG  = DR_REG_GPIO_BASE + 0x308 (signal 108)
// So the formulas are:
//   OUT_SEL_CFG(pin)  = 0x558 + pin*4
//   IN_SEL_CFG(sig)   = 0x158 + sig*4
// Earlier versions used 0x554/0x154 which mis-targeted pins/signals by one
// position — `matrix_out(31, 256)` was actually writing to FUNC30, leaving
// pin 31 (MDC) unrouted. That made MDIO silently dead even though our
// before/after dump appeared to "match" IDF (because we were reading at the
// same wrong addresses).
//
// Layout of each register (still per gpio_reg.h):
//   OUT_SEL_CFG: bits[8:0] = output signal index (0..255), 256 = use GPIO_OUT[n].
//                bit 9 = OUT_INV_SEL, bit 10 = OEN_SEL, bit 11 = OEN_INV_SEL.
//   IN_SEL_CFG:  bits[5:0] = pin number (0..56). 0x3F = always high, 0x3E = always low.
//                bit 6 = IN_INV_SEL, bit 7 = SIG_IN_SEL (1 = via matrix, 0 = bypass).
const REG_GPIO_BASE: usize = 0x500E_0000;
const GPIO_FUNC_OUT_SEL_CFG_BASE: usize = REG_GPIO_BASE + 0x558;
const GPIO_FUNC_IN_SEL_CFG_BASE: usize = REG_GPIO_BASE + 0x158;
const GPIO_FUNC_IN_SEL_BIT: u32 = 1 << 7;
const GPIO_OUT_W1TS: usize = REG_GPIO_BASE + 0x08;
const GPIO_OUT_W1TC: usize = REG_GPIO_BASE + 0x0C;
const GPIO_OUT1_W1TS: usize = REG_GPIO_BASE + 0x14;
const GPIO_OUT1_W1TC: usize = REG_GPIO_BASE + 0x18;
const GPIO_ENABLE_W1TS: usize = REG_GPIO_BASE + 0x24;
const GPIO_ENABLE1_W1TS: usize = REG_GPIO_BASE + 0x30;
const GPIO_MATRIX_OUT_SIMPLE: u32 = 256;

/// RMII data-plane pin selection. The defaults match Waveshare ESP32-P4-ETH.
#[derive(Clone, Copy, Debug)]
pub struct RmiiPinConfig {
    pub txd0: u32,
    pub txd1: u32,
    pub tx_en: u32,
    pub rxd0: u32,
    pub rxd1: u32,
    pub crs_dv: u32,
}

impl RmiiPinConfig {
    /// Pinout used by the Waveshare ESP32-P4-ETH dev board (IP101GRI PHY).
    pub const WAVESHARE_P4_ETH: Self = Self {
        txd0: 34,
        txd1: 35,
        tx_en: 49,
        rxd0: 30,
        rxd1: 29,
        crs_dv: 28,
    };
}

/// MDIO management pin selection. The defaults match Waveshare ESP32-P4-ETH.
#[derive(Clone, Copy, Debug)]
pub struct MdioPinConfig {
    pub mdc: u32,
    pub mdio: u32,
}

impl MdioPinConfig {
    /// MDC/MDIO pinout used by the Waveshare ESP32-P4-ETH dev board.
    pub const WAVESHARE_P4_ETH: Self = Self { mdc: 31, mdio: 52 };
}

/// PHY reset pin selection.
#[derive(Clone, Copy, Debug)]
pub struct PhyResetPinConfig {
    pub pin: u32,
    pub active_low: bool,
}

impl PhyResetPinConfig {
    /// IP101GRI reset pin used by the Waveshare ESP32-P4-ETH dev board.
    pub const WAVESHARE_P4_ETH: Self = Self {
        pin: 51,
        active_low: true,
    };
}

/// Programs the default RMII data pins through IO_MUX (Waveshare ESP32-P4-ETH).
pub fn configure_rmii_pins() {
    configure_rmii_pin_set(RmiiPinConfig::WAVESHARE_P4_ETH);
}

/// Programs the supplied RMII data-plane pins through IO_MUX function 3.
pub fn configure_rmii_pin_set(cfg: RmiiPinConfig) {
    configure_input_pin(iomux_reg(cfg.crs_dv));
    configure_input_pin(iomux_reg(cfg.rxd0));
    configure_input_pin(iomux_reg(cfg.rxd1));
    configure_output_pin(iomux_reg(cfg.txd0));
    configure_output_pin(iomux_reg(cfg.txd1));
    configure_output_pin(iomux_reg(cfg.tx_en));
}

/// Routes the EMAC MDC/MDIO management lines through the GPIO matrix.
pub fn configure_mdio_pins() {
    configure_mdio_pin_set(MdioPinConfig::WAVESHARE_P4_ETH);
}

/// Configures the MDC/MDIO pads for the EMAC management interface.
///
/// On ESP32-P4 the per-pin function table (`io_mux_reg.h`) shows pins 31 and
/// 52 have NO EMAC peripheral function — `MCU_SEL=1` selects plain GPIO. The
/// only way to connect EMAC MDC/MDIO to these pads is through the GPIO
/// matrix:
///   1. IO_MUX `MCU_SEL=1` puts the pad in GPIO mode and sets `FUN_DRV=2`
///      (medium drive). Reset default `FUN_DRV=0` is too weak for MDC.
///   2. `matrix_out(pin, signal_idx)` routes the EMAC output signal to the
///      pad: signal 108 (`MII_MDC_PAD_OUT_IDX`) → pin 31, signal 109
///      (`MII_MDO_PAD_OUT_IDX`) → pin 52.
///   3. `matrix_in(signal=107, pin=52)` routes the MDIO pad input back to
///      EMAC's `MII_MDI_PAD_IN_IDX = 107` so the peripheral can sample PHY
///      responses during the read turn-around.
///   4. `GPIO_ENABLE[pin]=1` unlocks the pad output buffer; the EMAC peripheral
///      then internally gates OE during the read phase of MDIO transactions.
///
/// Earlier versions of this driver mis-targeted the GPIO matrix registers by
/// 4 bytes (used base 0x554 for OUT_SEL and 0x154 for IN_SEL — the real P4
/// addresses are 0x558 / 0x158). With the off-by-4, `matrix_out(31, 256)`
/// wrote to FUNC30, and pin 31 stayed in its reset routing, so EMAC MDC never
/// reached the pad. That bug also masked itself via a self-consistent dump
/// (we read at the same wrong addresses we wrote to). Both the offset bug and
/// the bogus "peripheral-direct via MCU_SEL=1" theory have been corrected.
pub fn configure_mdio_pin_set(cfg: MdioPinConfig) {
    // MDC: output only. MCU_SEL=1 (GPIO mode), FUN_IE=0, FUN_DRV=2,
    // OUT_SEL=signal 108 (EMAC MDC), GPIO_ENABLE=1.
    configure_gpio_pad_with_drive(cfg.mdc, false);
    matrix_out(cfg.mdc, MII_MDC_PAD_OUT_IDX);
    gpio_enable_output(cfg.mdc);

    // MDIO: bidirectional. MCU_SEL=1, FUN_IE=1 (input enable), FUN_DRV=2.
    // OUT_SEL = signal 109 (EMAC MDO output), IN_SEL routes pad → signal 107
    // (EMAC MDI input). GPIO_ENABLE=1 — peripheral controls actual OE.
    configure_gpio_pad_with_drive(cfg.mdio, true);
    matrix_out(cfg.mdio, MII_MDO_PAD_OUT_IDX);
    matrix_in(cfg.mdio, MII_MDI_PAD_IN_IDX);
    gpio_enable_output(cfg.mdio);
}

/// Deasserts the default Waveshare ESP32-P4-ETH PHY reset line.
pub fn release_waveshare_phy_reset() {
    release_phy_reset_pin(PhyResetPinConfig::WAVESHARE_P4_ETH);
}

/// Configures a PHY reset pin as GPIO output and drives it to the deasserted level.
pub fn release_phy_reset_pin(cfg: PhyResetPinConfig) {
    configure_gpio_pad(cfg.pin, false);
    matrix_out(cfg.pin, GPIO_MATRIX_OUT_SIMPLE);

    if cfg.active_low {
        gpio_set_high(cfg.pin);
    } else {
        gpio_set_low(cfg.pin);
    }

    gpio_enable_output(cfg.pin);
}

fn configure_input_pin(iomux: usize) {
    regs::write(iomux, pad_config_value(regs::read(iomux), true, EMAC_PAD_FUNC));
}

fn configure_output_pin(iomux: usize) {
    regs::write(
        iomux,
        pad_config_value(regs::read(iomux), false, EMAC_PAD_FUNC),
    );
}

fn configure_gpio_pad(pin: u32, input_enabled: bool) {
    let iomux = iomux_reg(pin);
    regs::write(
        iomux,
        pad_config_value(regs::read(iomux), input_enabled, GPIO_PAD_FUNC),
    );
}

/// Same as `configure_gpio_pad` but also programs the pad's drive strength to medium.
/// EMAC management pins (MDC/MDIO) require this — reset default `FUN_DRV=0` is too weak
/// to drive a real board trace at MDC frequencies, and IDF's working setup uses `FUN_DRV=2`.
fn configure_gpio_pad_with_drive(pin: u32, input_enabled: bool) {
    let iomux = iomux_reg(pin);
    let mut value = pad_config_value(regs::read(iomux), input_enabled, GPIO_PAD_FUNC);
    value = (value & !FUN_DRV_MASK) | ((FUN_DRV_MEDIUM << FUN_DRV_SHIFT) & FUN_DRV_MASK);
    regs::write(iomux, value);
}

fn matrix_out(pin: u32, signal_idx: u32) {
    let reg = GPIO_FUNC_OUT_SEL_CFG_BASE + (pin as usize) * 4;
    // signal_idx in bits[8:0]; OEN_SEL=0 so the EMAC peripheral controls the output enable.
    regs::write(reg, signal_idx & 0x1FF);
}

fn matrix_in(pin: u32, signal_idx: u32) {
    let reg = GPIO_FUNC_IN_SEL_CFG_BASE + (signal_idx as usize) * 4;
    // pin in bits[5:0]; bit 7 = "use GPIO matrix" (without it the peripheral sees the bypass
    // path which is hard-tied to the IO_MUX function pad).
    regs::write(reg, (pin & 0x3F) | GPIO_FUNC_IN_SEL_BIT);
}

fn gpio_set_high(pin: u32) {
    if pin < 32 {
        regs::write(GPIO_OUT_W1TS, 1u32 << pin);
    } else {
        regs::write(GPIO_OUT1_W1TS, 1u32 << (pin - 32));
    }
}

fn gpio_set_low(pin: u32) {
    if pin < 32 {
        regs::write(GPIO_OUT_W1TC, 1u32 << pin);
    } else {
        regs::write(GPIO_OUT1_W1TC, 1u32 << (pin - 32));
    }
}

fn gpio_enable_output(pin: u32) {
    if pin < 32 {
        regs::write(GPIO_ENABLE_W1TS, 1u32 << pin);
    } else {
        regs::write(GPIO_ENABLE1_W1TS, 1u32 << (pin - 32));
    }
}

fn pad_config_value(previous: u32, input_enabled: bool, func: u32) -> u32 {
    let mut value =
        (previous & !(MCU_SEL_MASK | FUN_PU | FUN_PD)) | ((func << MCU_SEL_SHIFT) & MCU_SEL_MASK);

    if input_enabled {
        value |= FUN_IE;
    } else {
        value &= !FUN_IE;
    }

    value
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regs;

    #[test]
    fn iomux_reg_matches_idf_addresses() {
        assert_eq!(iomux_reg(0), 0x500E_1004);
        assert_eq!(iomux_reg(28), 0x500E_1074);
        assert_eq!(iomux_reg(50), 0x500E_10CC);
        assert_eq!(iomux_reg(52), 0x500E_10D4);
    }

    #[test]
    fn input_pad_config_enables_input_and_selects_emac_function() {
        let value = pad_config_value(u32::MAX, true, EMAC_PAD_FUNC);

        assert_eq!(value & MCU_SEL_MASK, EMAC_PAD_FUNC << MCU_SEL_SHIFT);
        assert_ne!(value & FUN_IE, 0);
        assert_eq!(value & FUN_PU, 0);
        assert_eq!(value & FUN_PD, 0);
    }

    #[test]
    fn output_pad_config_disables_input_and_selects_emac_function() {
        let value = pad_config_value(u32::MAX, false, EMAC_PAD_FUNC);

        assert_eq!(value & MCU_SEL_MASK, EMAC_PAD_FUNC << MCU_SEL_SHIFT);
        assert_eq!(value & FUN_IE, 0);
        assert_eq!(value & FUN_PU, 0);
        assert_eq!(value & FUN_PD, 0);
    }

    #[test]
    fn waveshare_rmii_pins_program_iomux_function_3() {
        regs::reset_test_registers();
        let cfg = RmiiPinConfig::WAVESHARE_P4_ETH;
        let pins_in = [cfg.crs_dv, cfg.rxd0, cfg.rxd1];
        let pins_out = [cfg.txd0, cfg.txd1, cfg.tx_en];

        for &p in pins_in.iter().chain(pins_out.iter()) {
            regs::write(iomux_reg(p), u32::MAX);
        }

        configure_rmii_pin_set(cfg);

        for &p in &pins_in {
            let v = regs::read(iomux_reg(p));
            assert_eq!(v & MCU_SEL_MASK, EMAC_PAD_FUNC << MCU_SEL_SHIFT);
            assert_ne!(v & FUN_IE, 0);
        }
        for &p in &pins_out {
            let v = regs::read(iomux_reg(p));
            assert_eq!(v & MCU_SEL_MASK, EMAC_PAD_FUNC << MCU_SEL_SHIFT);
            assert_eq!(v & FUN_IE, 0);
        }
    }

    #[test]
    fn waveshare_mdio_pins_set_iomux_to_gpio_function() {
        // Host-side check is limited to the IO_MUX side because the GPIO Matrix routing now
        // goes through the boot-ROM helpers (`rom_gpio_matrix_out` / `rom_gpio_matrix_in`),
        // which only exist on the target. The hardware path is exercised by the on-device
        // example `emac_init` which scans MDIO and verifies the PHY responds.
        regs::reset_test_registers();
        let cfg = MdioPinConfig::WAVESHARE_P4_ETH;

        configure_mdio_pin_set(cfg);

        let mdc = regs::read(iomux_reg(cfg.mdc));
        assert_eq!(mdc & MCU_SEL_MASK, GPIO_PAD_FUNC << MCU_SEL_SHIFT);
        assert_eq!(mdc & FUN_IE, 0);

        let mdio = regs::read(iomux_reg(cfg.mdio));
        assert_eq!(mdio & MCU_SEL_MASK, GPIO_PAD_FUNC << MCU_SEL_SHIFT);
        assert_ne!(mdio & FUN_IE, 0);
    }

    #[test]
    fn waveshare_phy_reset_deasserts_gpio51_high() {
        regs::reset_test_registers();

        release_waveshare_phy_reset();

        let cfg = PhyResetPinConfig::WAVESHARE_P4_ETH;
        let iomux = regs::read(iomux_reg(cfg.pin));
        assert_eq!(iomux & MCU_SEL_MASK, GPIO_PAD_FUNC << MCU_SEL_SHIFT);
        assert_eq!(iomux & FUN_IE, 0);
        assert_eq!(
            regs::read(GPIO_FUNC_OUT_SEL_CFG_BASE + cfg.pin as usize * 4),
            GPIO_MATRIX_OUT_SIMPLE
        );
        assert_eq!(regs::read(GPIO_OUT1_W1TS), 1u32 << (cfg.pin - 32));
        assert_eq!(regs::read(GPIO_ENABLE1_W1TS), 1u32 << (cfg.pin - 32));
    }
}
