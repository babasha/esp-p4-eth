//! EMAC clock-tree configuration for ESP32-P4.
//!
//! The P4 MAC is not usable until both the HP/LP clock gates and the RMII clock
//! mux are configured.
//!
//! Two board topologies are supported:
//!
//! - `configure_clock_ext_in()`: the PHY or an external oscillator drives the
//!   50 MHz `REF_CLK` into one of the RMII clock input pads.
//! - `configure_clock_mpll_out()`: P4 derives a 50 MHz `REF_CLK` from MPLL,
//!   drives it out on `GPIO23` or `GPIO39`, and expects that signal to be
//!   looped back externally into one of the RMII clock input pads. This follows
//!   the ESP-IDF guidance for `EMAC_CLK_OUT` on ESP32-P4.

use core::hint::spin_loop;

use crate::{regs, Speed};

const DR_REG_HP_SYS_CLKRST_BASE: usize = 0x500E_6000;
const HP_SYS_CLKRST_SOC_CLK_CTRL1_REG: usize = DR_REG_HP_SYS_CLKRST_BASE + 0x18;
const HP_SYS_CLKRST_REF_CLK_CTRL0_REG: usize = DR_REG_HP_SYS_CLKRST_BASE + 0x24;
const HP_SYS_CLKRST_REF_CLK_CTRL1_REG: usize = DR_REG_HP_SYS_CLKRST_BASE + 0x28;
const HP_SYS_CLKRST_PERI_CLK_CTRL00_REG: usize = DR_REG_HP_SYS_CLKRST_BASE + 0x30;
const HP_SYS_CLKRST_PERI_CLK_CTRL01_REG: usize = DR_REG_HP_SYS_CLKRST_BASE + 0x34;

const DR_REG_LP_CLKRST_BASE: usize = 0x5011_1000;
const LP_CLKRST_HP_CLK_CTRL_REG: usize = DR_REG_LP_CLKRST_BASE + 0x40;
const LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG: usize = DR_REG_LP_CLKRST_BASE + 0x4C;

const HP_SYS_CLKRST_REG_EMAC_SYS_CLK_EN: u32 = 1 << 13;
const HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_SHIFT: u32 = 0;
const HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_MASK: u32 =
    0xff << HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_SHIFT;
const HP_SYS_CLKRST_REG_REF_50M_CLK_EN: u32 = 1 << 27;
const HP_SYS_CLKRST_REG_PAD_EMAC_REF_CLK_EN: u32 = 1 << 24;
const HP_SYS_CLKRST_REG_EMAC_RMII_CLK_SRC_SEL_SHIFT: u32 = 25;
const HP_SYS_CLKRST_REG_EMAC_RMII_CLK_SRC_SEL_MASK: u32 =
    0x3 << HP_SYS_CLKRST_REG_EMAC_RMII_CLK_SRC_SEL_SHIFT;
const HP_SYS_CLKRST_REG_EMAC_RMII_CLK_EN: u32 = 1 << 27;
const HP_SYS_CLKRST_REG_EMAC_RX_CLK_SRC_SEL: u32 = 1 << 28;
const HP_SYS_CLKRST_REG_EMAC_RX_CLK_EN: u32 = 1 << 29;

const HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_SHIFT: u32 = 0;
const HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_MASK: u32 =
    0xff << HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_SHIFT;
const HP_SYS_CLKRST_REG_EMAC_TX_CLK_SRC_SEL: u32 = 1 << 8;
const HP_SYS_CLKRST_REG_EMAC_TX_CLK_EN: u32 = 1 << 9;
const HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_SHIFT: u32 = 10;
const HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_MASK: u32 =
    0xff << HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_SHIFT;

const LP_CLKRST_HP_PAD_EMAC_TX_CLK_EN: u32 = 1 << 13;
const LP_CLKRST_HP_PAD_EMAC_RX_CLK_EN: u32 = 1 << 14;
const LP_CLKRST_HP_PAD_EMAC_TXRX_CLK_EN: u32 = 1 << 15;
const LP_CLKRST_HP_MPLL_500M_CLK_EN: u32 = 1 << 28;
const LP_CLKRST_RST_EN_EMAC: u32 = 1 << 30;
const LP_CLKRST_FORCE_NORST_EMAC: u32 = 1 << 31;

const IO_MUX_GPIO23_REG: usize = 0x500E_1060;
const IO_MUX_GPIO32_REG: usize = 0x500E_1084;
const IO_MUX_GPIO39_REG: usize = 0x500E_10A0;
const IO_MUX_GPIO44_REG: usize = 0x500E_10B4;
const IO_MUX_GPIO50_REG: usize = 0x500E_10CC;

const FUN_IE: u32 = 1 << 9;
const FUN_PU: u32 = 1 << 8;
const FUN_PD: u32 = 1 << 7;
const MCU_SEL_SHIFT: u32 = 12;
const MCU_SEL_MASK: u32 = 0x7 << MCU_SEL_SHIFT;
const REF_50M_CLK_PAD_FUNC: u32 = 3;
const EMAC_RMII_CLK_PAD_FUNC: u32 = 3;

// After deasserting the EMAC reset and enabling the AHB / RMII / RX / TX
// clocks, downstream MMIO accesses to the EMAC must wait for the peripheral
// to actually leave reset and for the clock tree to stabilise. The previous
// 1 024-cycle wait (~3 µs at 360 MHz) was empirically insufficient; bumped
// to 100 000 (~280 µs) which lines up with the IDF baseline post-reset wait.
// The remaining 10 % cold-boot hang race seen at higher delays was unrelated
// — it was caused by the descriptor-ring constructors writing the DMA
// `*_DESC_LIST` peripheral registers BEFORE `dma_reset` had run; that is now
// fixed in `Ethernet::reset_dma`.
const CLOCK_CONFIG_DELAY_SPINS: usize = 100_000;
const RMII_CLK_SRC_EXTERNAL: u32 = 0;
const REF_50M_CLK_DIV_DEFAULT: u32 = 9;

/// GPIO options that can feed the RMII reference clock into the MAC.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RefClockPin {
    /// Route REF_CLK through GPIO32.
    Gpio32,
    /// Route REF_CLK through GPIO44.
    Gpio44,
    /// Route REF_CLK through GPIO50.
    Gpio50,
}

/// GPIO options that can emit the internally generated 50 MHz REF_CLK.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MpllClockOutPin {
    /// Route internally generated REF_CLK out through GPIO23.
    Gpio23,
    /// Route internally generated REF_CLK out through GPIO39.
    Gpio39,
}

/// Enables the EMAC clock tree for an externally supplied 50 MHz `REF_CLK`.
///
/// Use this mode when the PHY or a dedicated oscillator already drives the
/// RMII reference clock into one of the supported input pads.
pub fn configure_clock_ext_in(pin: RefClockPin) {
    select_rmii_phy_interface();
    enable_emac_clock_tree();
    configure_ref_clock_pin(pin);
    select_external_rmii_clock();
    configure_speed_divider(Speed::Mbps100);
}

/// Programs `HP_SYSTEM.sys_gmac_ctrl0` for RMII operation:
/// - `phy_intf_sel = 4` (RMII)
/// - `gmac_rst_clk_tx_n = 1` (active-low — deassert TX clock-domain reset)
/// - `gmac_rst_clk_rx_n = 1` (active-low — deassert RX clock-domain reset)
///
/// The default value of `phy_intf_sel` is 0 which selects MII. Without `tx/rx_clk_n` deasserted
/// the EMAC TX/RX clock domains stay in reset — MDC clock never gates through, MDIO transactions
/// formally complete (BUSY bit clears) but PHY responses arrive as all-zeros.
///
/// Confirmed against a working IDF v5.3 baseline dump on Waveshare ESP32-P4-ETH after `Ethernet
/// Link Up`: `HP_SYSTEM.SYS_GMAC_CTRL0 = 0xD0` (bits 4, 6, 7 set = phy_intf_sel=4 RMII, plus
/// both clock-domain resets deasserted).
fn select_rmii_phy_interface() {
    const HP_SYSTEM_SYS_GMAC_CTRL0_REG: usize = 0x500E_514C;
    const PHY_INTF_SEL_SHIFT: u32 = 2;
    const PHY_INTF_SEL_MASK: u32 = 0x7 << PHY_INTF_SEL_SHIFT;
    const PHY_INTF_SEL_RMII: u32 = 4;
    const GMAC_RST_CLK_TX_N: u32 = 1 << 6;
    const GMAC_RST_CLK_RX_N: u32 = 1 << 7;

    let value = (regs::read(HP_SYSTEM_SYS_GMAC_CTRL0_REG) & !PHY_INTF_SEL_MASK)
        | (PHY_INTF_SEL_RMII << PHY_INTF_SEL_SHIFT)
        | GMAC_RST_CLK_TX_N
        | GMAC_RST_CLK_RX_N;
    regs::write(HP_SYSTEM_SYS_GMAC_CTRL0_REG, value);
}

/// Disables the EMAC clock tree and asserts peripheral reset.
///
/// This is the low-level power-down primitive used by [`crate::Ethernet`] on
/// shutdown and drop.
pub fn disable_emac_clock_tree() {
    clear_bits(
        HP_SYS_CLKRST_PERI_CLK_CTRL00_REG,
        HP_SYS_CLKRST_REG_PAD_EMAC_REF_CLK_EN
            | HP_SYS_CLKRST_REG_EMAC_RMII_CLK_EN
            | HP_SYS_CLKRST_REG_EMAC_RX_CLK_EN,
    );
    clear_bits(
        HP_SYS_CLKRST_PERI_CLK_CTRL01_REG,
        HP_SYS_CLKRST_REG_EMAC_TX_CLK_EN,
    );
    clear_bits(
        LP_CLKRST_HP_CLK_CTRL_REG,
        LP_CLKRST_HP_PAD_EMAC_TX_CLK_EN
            | LP_CLKRST_HP_PAD_EMAC_RX_CLK_EN
            | LP_CLKRST_HP_PAD_EMAC_TXRX_CLK_EN,
    );
    clear_bits(
        HP_SYS_CLKRST_SOC_CLK_CTRL1_REG,
        HP_SYS_CLKRST_REG_EMAC_SYS_CLK_EN,
    );

    let mut reset = regs::read(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG);
    reset |= LP_CLKRST_RST_EN_EMAC;
    reset &= !LP_CLKRST_FORCE_NORST_EMAC;
    regs::write(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG, reset);
}

/// Enables the EMAC clock tree for a 50 MHz REF_CLK derived from MPLL.
///
/// ESP-IDF documents the ESP32-P4 `EMAC_CLK_OUT` topology as an output-only
/// `REF_CLK` source that must be externally looped back into one of the RMII
/// clock input pads. Because of that requirement, this helper configures both
/// the output pad and the input pad used for the external loopback.
pub fn configure_clock_mpll_out(output_pin: MpllClockOutPin, input_pin: RefClockPin) {
    select_rmii_phy_interface();
    enable_emac_clock_tree();
    enable_ref_50m_clock();
    configure_ref_50m_output_pin(output_pin);
    configure_ref_clock_pin(input_pin);
    select_external_rmii_clock();
    configure_speed_divider(Speed::Mbps100);
}

/// Programs the RMII TX/RX clock divider for the selected negotiated line speed.
///
/// ESP32-P4 uses divider `1` for 100 Mbps operation and divider `10` for
/// 10 Mbps operation.
pub fn configure_speed_divider(speed: Speed) {
    let divider = divider_for_speed(speed);
    let peri_clk_ctrl01 = regs::read(HP_SYS_CLKRST_PERI_CLK_CTRL01_REG);
    let peri_clk_ctrl01 = replace_field(
        peri_clk_ctrl01,
        HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_MASK,
        HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_SHIFT,
        divider,
    );
    let peri_clk_ctrl01 = replace_field(
        peri_clk_ctrl01,
        HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_MASK,
        HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_SHIFT,
        divider,
    );

    regs::write(HP_SYS_CLKRST_PERI_CLK_CTRL01_REG, peri_clk_ctrl01);
}

#[cfg(test)]
#[allow(dead_code)]
pub(crate) fn emac_clock_tree_enabled() -> bool {
    regs::read(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG) & HP_SYS_CLKRST_REG_EMAC_SYS_CLK_EN != 0
}

#[cfg(test)]
#[allow(dead_code)]
pub(crate) fn emac_reset_asserted() -> bool {
    regs::read(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG) & LP_CLKRST_RST_EN_EMAC != 0
}

/// Enables the EMAC clock tree.
///
/// Bits programmed here are matched bit-for-bit against a working IDF v5.3
/// baseline dump on Waveshare ESP32-P4-ETH after `Ethernet Link Up`. Three
/// "pad enable" bits that earlier versions of this driver set are NOT set by
/// IDF and are intentionally omitted here:
///
///   - `PAD_EMAC_REF_CLK_EN` (PERI_CLK_CTRL00 bit 24)
///   - `HP_PAD_EMAC_TX_CLK_EN` (LP_HP_CLK_CTRL bit 13)
///   - `HP_PAD_EMAC_RX_CLK_EN` (LP_HP_CLK_CTRL bit 14)
///
/// `LP_HP_CLK_CTRL` keeps only `HP_PAD_EMAC_TXRX_CLK_EN` (bit 15) which IDF
/// also sets. Setting the extra three bits drove our `PERI_CLK_CTRL00 = 0x29`
/// (vs IDF `0x28`) and `LP_HP_CLK_CTRL = 0x1FFFFFFC` (vs IDF `0x1FFF9FFD`)
/// — see clk_dump example output captured 2026-04-25.
///
/// Similarly, `LP_CLKRST_FORCE_NORST_EMAC` is no longer asserted: IDF leaves
/// `LP_EMAC_RST_CTRL = 0x00000000` after init while we previously had
/// `0x80000000`. Clearing the reset gate is sufficient; forcing the
/// "no-reset" override is not.
fn enable_emac_clock_tree() {
    set_bits(
        HP_SYS_CLKRST_SOC_CLK_CTRL1_REG,
        HP_SYS_CLKRST_REG_EMAC_SYS_CLK_EN,
    );
    set_bits(
        HP_SYS_CLKRST_PERI_CLK_CTRL00_REG,
        HP_SYS_CLKRST_REG_EMAC_RMII_CLK_EN | HP_SYS_CLKRST_REG_EMAC_RX_CLK_EN,
    );
    set_bits(
        HP_SYS_CLKRST_PERI_CLK_CTRL01_REG,
        HP_SYS_CLKRST_REG_EMAC_TX_CLK_EN,
    );
    // We rely on RAM-only loading via espflash --ram --no-stub which bypasses
    // the IDF 2nd-stage bootloader. ROM startup leaves bits 13/14 of
    // LP_HP_CLK_CTRL set; full IDF system_init clears them. We must clear them
    // explicitly to land on the same state as the IDF baseline. `set_bits`
    // alone (OR) cannot do that.
    let hp_clk = (regs::read(LP_CLKRST_HP_CLK_CTRL_REG)
        & !(LP_CLKRST_HP_PAD_EMAC_TX_CLK_EN | LP_CLKRST_HP_PAD_EMAC_RX_CLK_EN))
        | LP_CLKRST_HP_PAD_EMAC_TXRX_CLK_EN;
    regs::write(LP_CLKRST_HP_CLK_CTRL_REG, hp_clk);

    let mut reset = regs::read(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG);
    reset &= !(LP_CLKRST_RST_EN_EMAC | LP_CLKRST_FORCE_NORST_EMAC);
    regs::write(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG, reset);

    for _ in 0..CLOCK_CONFIG_DELAY_SPINS {
        spin_loop();
    }
}

fn configure_ref_clock_pin(pin: RefClockPin) {
    let iomux = input_pin_iomux_reg(pin);
    let value = (regs::read(iomux) & !(MCU_SEL_MASK | FUN_PU | FUN_PD))
        | FUN_IE
        | (EMAC_RMII_CLK_PAD_FUNC << MCU_SEL_SHIFT);

    regs::write(iomux, value);
}

fn configure_ref_50m_output_pin(pin: MpllClockOutPin) {
    let iomux = output_pin_iomux_reg(pin);
    let value = (regs::read(iomux) & !(MCU_SEL_MASK | FUN_IE | FUN_PU | FUN_PD))
        | (REF_50M_CLK_PAD_FUNC << MCU_SEL_SHIFT);

    regs::write(iomux, value);
}

fn enable_ref_50m_clock() {
    set_bits(LP_CLKRST_HP_CLK_CTRL_REG, LP_CLKRST_HP_MPLL_500M_CLK_EN);

    let ref_clk_ctrl0 = replace_field(
        regs::read(HP_SYS_CLKRST_REF_CLK_CTRL0_REG),
        HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_MASK,
        HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_SHIFT,
        REF_50M_CLK_DIV_DEFAULT,
    );
    regs::write(HP_SYS_CLKRST_REF_CLK_CTRL0_REG, ref_clk_ctrl0);
    set_bits(
        HP_SYS_CLKRST_REF_CLK_CTRL1_REG,
        HP_SYS_CLKRST_REG_REF_50M_CLK_EN,
    );
}

fn select_external_rmii_clock() {
    let peri_clk_ctrl00 = regs::read(HP_SYS_CLKRST_PERI_CLK_CTRL00_REG);
    let peri_clk_ctrl00 = replace_field(
        peri_clk_ctrl00,
        HP_SYS_CLKRST_REG_EMAC_RMII_CLK_SRC_SEL_MASK,
        HP_SYS_CLKRST_REG_EMAC_RMII_CLK_SRC_SEL_SHIFT,
        RMII_CLK_SRC_EXTERNAL,
    ) & !HP_SYS_CLKRST_REG_EMAC_RX_CLK_SRC_SEL;

    regs::write(HP_SYS_CLKRST_PERI_CLK_CTRL00_REG, peri_clk_ctrl00);

    let peri_clk_ctrl01 =
        regs::read(HP_SYS_CLKRST_PERI_CLK_CTRL01_REG) & !HP_SYS_CLKRST_REG_EMAC_TX_CLK_SRC_SEL;
    regs::write(HP_SYS_CLKRST_PERI_CLK_CTRL01_REG, peri_clk_ctrl01);
}

fn divider_for_speed(speed: Speed) -> u32 {
    match speed {
        Speed::Mbps100 => 1,
        Speed::Mbps10 => 10,
    }
}

fn input_pin_iomux_reg(pin: RefClockPin) -> usize {
    match pin {
        RefClockPin::Gpio32 => IO_MUX_GPIO32_REG,
        RefClockPin::Gpio44 => IO_MUX_GPIO44_REG,
        RefClockPin::Gpio50 => IO_MUX_GPIO50_REG,
    }
}

fn output_pin_iomux_reg(pin: MpllClockOutPin) -> usize {
    match pin {
        MpllClockOutPin::Gpio23 => IO_MUX_GPIO23_REG,
        MpllClockOutPin::Gpio39 => IO_MUX_GPIO39_REG,
    }
}

fn replace_field(value: u32, mask: u32, shift: u32, field: u32) -> u32 {
    (value & !mask) | ((field << shift) & mask)
}

fn set_bits(reg: usize, bits: u32) {
    regs::write(reg, regs::read(reg) | bits);
}

fn clear_bits(reg: usize, bits: u32) {
    regs::write(reg, regs::read(reg) & !bits);
}

#[cfg(test)]
mod tests {
    use super::{
        configure_clock_ext_in, configure_clock_mpll_out, configure_speed_divider,
        disable_emac_clock_tree, divider_for_speed, input_pin_iomux_reg, output_pin_iomux_reg,
        replace_field, MpllClockOutPin, RefClockPin, EMAC_RMII_CLK_PAD_FUNC, FUN_IE, FUN_PD,
        FUN_PU, HP_SYS_CLKRST_PERI_CLK_CTRL00_REG, HP_SYS_CLKRST_PERI_CLK_CTRL01_REG,
        HP_SYS_CLKRST_REF_CLK_CTRL0_REG, HP_SYS_CLKRST_REF_CLK_CTRL1_REG,
        HP_SYS_CLKRST_REG_EMAC_RMII_CLK_EN, HP_SYS_CLKRST_REG_EMAC_RMII_CLK_SRC_SEL_MASK,
        HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_MASK, HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_SHIFT,
        HP_SYS_CLKRST_REG_EMAC_RX_CLK_EN, HP_SYS_CLKRST_REG_EMAC_SYS_CLK_EN,
        HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_MASK, HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_SHIFT,
        HP_SYS_CLKRST_REG_EMAC_TX_CLK_EN, HP_SYS_CLKRST_REG_PAD_EMAC_REF_CLK_EN,
        HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_MASK, HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_SHIFT,
        HP_SYS_CLKRST_REG_REF_50M_CLK_EN, HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, IO_MUX_GPIO23_REG,
        IO_MUX_GPIO32_REG, IO_MUX_GPIO39_REG, IO_MUX_GPIO44_REG, IO_MUX_GPIO50_REG,
        LP_CLKRST_FORCE_NORST_EMAC, LP_CLKRST_HP_CLK_CTRL_REG, LP_CLKRST_HP_MPLL_500M_CLK_EN,
        LP_CLKRST_HP_PAD_EMAC_RX_CLK_EN, LP_CLKRST_HP_PAD_EMAC_TXRX_CLK_EN,
        LP_CLKRST_HP_PAD_EMAC_TX_CLK_EN, LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG,
        LP_CLKRST_RST_EN_EMAC, MCU_SEL_MASK, MCU_SEL_SHIFT, REF_50M_CLK_DIV_DEFAULT,
        REF_50M_CLK_PAD_FUNC,
    };
    use crate::{regs, Speed};

    #[test]
    fn ref_clock_pin_maps_to_expected_iomux_register() {
        assert_eq!(input_pin_iomux_reg(RefClockPin::Gpio32), IO_MUX_GPIO32_REG);
        assert_eq!(input_pin_iomux_reg(RefClockPin::Gpio44), IO_MUX_GPIO44_REG);
        assert_eq!(input_pin_iomux_reg(RefClockPin::Gpio50), IO_MUX_GPIO50_REG);
    }

    #[test]
    fn mpll_output_pin_maps_to_expected_iomux_register() {
        assert_eq!(
            output_pin_iomux_reg(MpllClockOutPin::Gpio23),
            IO_MUX_GPIO23_REG
        );
        assert_eq!(
            output_pin_iomux_reg(MpllClockOutPin::Gpio39),
            IO_MUX_GPIO39_REG
        );
    }

    #[test]
    fn speed_divider_matches_checklist() {
        assert_eq!(divider_for_speed(Speed::Mbps100), 1);
        assert_eq!(divider_for_speed(Speed::Mbps10), 10);
    }

    #[test]
    fn configure_speed_divider_updates_rx_and_tx_fields_only() {
        regs::reset_test_registers();
        regs::write(HP_SYS_CLKRST_PERI_CLK_CTRL01_REG, u32::MAX);

        configure_speed_divider(Speed::Mbps10);
        let value = regs::read(HP_SYS_CLKRST_PERI_CLK_CTRL01_REG);
        let divider_mask =
            HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_MASK | HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_MASK;

        assert_eq!(
            (value & HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_MASK)
                >> HP_SYS_CLKRST_REG_EMAC_RX_CLK_DIV_NUM_SHIFT,
            10
        );
        assert_eq!(
            (value & HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_MASK)
                >> HP_SYS_CLKRST_REG_EMAC_TX_CLK_DIV_NUM_SHIFT,
            10
        );
        assert_eq!(value & !divider_mask, u32::MAX & !divider_mask);
    }

    #[test]
    fn replace_field_overwrites_only_target_bits() {
        let value = replace_field(
            0xffff_ffff,
            MCU_SEL_MASK,
            MCU_SEL_SHIFT,
            EMAC_RMII_CLK_PAD_FUNC,
        );
        let expected_mcu_sel = EMAC_RMII_CLK_PAD_FUNC << MCU_SEL_SHIFT;

        assert_eq!(value & MCU_SEL_MASK, expected_mcu_sel);
        assert_eq!(value & FUN_IE, FUN_IE);
        assert_eq!(value & FUN_PU, FUN_PU);
        assert_eq!(value & FUN_PD, FUN_PD);
    }

    #[test]
    fn disable_clock_tree_clears_gates_and_asserts_reset() {
        regs::reset_test_registers();
        configure_clock_ext_in(RefClockPin::Gpio32);

        disable_emac_clock_tree();

        assert_eq!(
            regs::read(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG) & HP_SYS_CLKRST_REG_EMAC_SYS_CLK_EN,
            0
        );
        assert_eq!(
            regs::read(HP_SYS_CLKRST_PERI_CLK_CTRL00_REG)
                & (HP_SYS_CLKRST_REG_PAD_EMAC_REF_CLK_EN
                    | HP_SYS_CLKRST_REG_EMAC_RMII_CLK_EN
                    | HP_SYS_CLKRST_REG_EMAC_RX_CLK_EN),
            0
        );
        assert_eq!(
            regs::read(HP_SYS_CLKRST_PERI_CLK_CTRL01_REG) & HP_SYS_CLKRST_REG_EMAC_TX_CLK_EN,
            0
        );
        assert_eq!(
            regs::read(LP_CLKRST_HP_CLK_CTRL_REG)
                & (LP_CLKRST_HP_PAD_EMAC_TX_CLK_EN
                    | LP_CLKRST_HP_PAD_EMAC_RX_CLK_EN
                    | LP_CLKRST_HP_PAD_EMAC_TXRX_CLK_EN),
            0
        );
        assert_ne!(
            regs::read(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG) & LP_CLKRST_RST_EN_EMAC,
            0
        );
        assert_eq!(
            regs::read(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL_REG) & LP_CLKRST_FORCE_NORST_EMAC,
            0
        );
    }

    #[test]
    fn mpll_clock_out_enables_ref_50m_and_loopback_input() {
        regs::reset_test_registers();

        configure_clock_mpll_out(MpllClockOutPin::Gpio23, RefClockPin::Gpio32);

        assert_ne!(
            regs::read(LP_CLKRST_HP_CLK_CTRL_REG) & LP_CLKRST_HP_MPLL_500M_CLK_EN,
            0
        );
        assert_ne!(
            regs::read(HP_SYS_CLKRST_REF_CLK_CTRL1_REG) & HP_SYS_CLKRST_REG_REF_50M_CLK_EN,
            0
        );
        assert_eq!(
            (regs::read(HP_SYS_CLKRST_REF_CLK_CTRL0_REG)
                & HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_MASK)
                >> HP_SYS_CLKRST_REG_REF_50M_CLK_DIV_NUM_SHIFT,
            REF_50M_CLK_DIV_DEFAULT
        );
        assert_eq!(
            regs::read(HP_SYS_CLKRST_PERI_CLK_CTRL00_REG)
                & HP_SYS_CLKRST_REG_EMAC_RMII_CLK_SRC_SEL_MASK,
            0
        );
        assert_eq!(
            regs::read(IO_MUX_GPIO23_REG) & MCU_SEL_MASK,
            REF_50M_CLK_PAD_FUNC << MCU_SEL_SHIFT
        );
        assert_eq!(regs::read(IO_MUX_GPIO23_REG) & FUN_IE, 0);
        assert_eq!(
            regs::read(IO_MUX_GPIO32_REG) & MCU_SEL_MASK,
            EMAC_RMII_CLK_PAD_FUNC << MCU_SEL_SHIFT
        );
        assert_ne!(regs::read(IO_MUX_GPIO32_REG) & FUN_IE, 0);
    }
}
