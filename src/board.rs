//! Board-level configuration grouping pin maps, reference clock pin, and PHY
//! address. A single [`BoardConfig`] value is enough to bring up the EMAC on
//! a target hardware without per-call argument plumbing.
//!
//! Built-in board defaults live as `const` items, e.g. [`BoardConfig::WAVESHARE_P4_ETH`].
//! For custom hardware, construct your own [`BoardConfig`] literal.

use crate::clock::RefClockPin;
use crate::pins::{MdioPinConfig, PhyResetPinConfig, RmiiPinConfig};

/// Top-level hardware description used by the EMAC bring-up path.
#[derive(Clone, Copy, Debug)]
pub struct BoardConfig {
    /// RMII data-plane pin assignment (TXD0/1, RXD0/1, TX_EN, CRS_DV).
    pub rmii_pins: RmiiPinConfig,
    /// MDIO management bus pin assignment (MDC, MDIO).
    pub mdio_pins: MdioPinConfig,
    /// PHY hardware reset pin (active level encoded in the struct).
    pub phy_reset: PhyResetPinConfig,
    /// Pad that receives the 50 MHz RMII reference clock from the PHY.
    pub ref_clock: RefClockPin,
    /// MDIO address strap for the PHY (0..31).
    pub phy_addr: u8,
}

impl BoardConfig {
    /// Layout used by the Waveshare ESP32-P4-ETH dev board (IP101GRI PHY).
    ///
    /// - RMII data: TXD0=GPIO34, TXD1=GPIO35, TX_EN=GPIO49,
    ///   RXD0=GPIO30, RXD1=GPIO29, CRS_DV=GPIO28
    /// - MDIO: MDC=GPIO31, MDIO=GPIO52
    /// - PHY reset: GPIO51 (active low)
    /// - Reference clock input: GPIO50
    /// - PHY MDIO address: 1 (Waveshare strap)
    pub const WAVESHARE_P4_ETH: Self = Self {
        rmii_pins: RmiiPinConfig::WAVESHARE_P4_ETH,
        mdio_pins: MdioPinConfig::WAVESHARE_P4_ETH,
        phy_reset: PhyResetPinConfig::WAVESHARE_P4_ETH,
        ref_clock: RefClockPin::Gpio50,
        phy_addr: 1,
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::RefClockPin;

    /// MDIO Clause-22 reserves 5 bits for the address — only 0..31 is legal.
    /// A typo to e.g. 33 would silently pick `33 & 0x1F = 1` and address the
    /// wrong PHY without compile-time complaint, so guard the range.
    #[test]
    fn waveshare_phy_addr_is_one_and_in_legal_range() {
        let cfg = BoardConfig::WAVESHARE_P4_ETH;
        assert_eq!(cfg.phy_addr, 1, "Waveshare straps the IP101 to MDIO addr 1");
        assert!(cfg.phy_addr < 32, "MDIO addr must fit in 5 bits");
    }

    #[test]
    fn waveshare_rmii_pin_map_matches_dev_board_silkscreen() {
        let pins = BoardConfig::WAVESHARE_P4_ETH.rmii_pins;
        assert_eq!(pins.txd0, 34);
        assert_eq!(pins.txd1, 35);
        assert_eq!(pins.tx_en, 49);
        assert_eq!(pins.rxd0, 30);
        assert_eq!(pins.rxd1, 29);
        assert_eq!(pins.crs_dv, 28);
    }

    #[test]
    fn waveshare_mdio_pins_match_dev_board_silkscreen() {
        let pins = BoardConfig::WAVESHARE_P4_ETH.mdio_pins;
        assert_eq!(pins.mdc, 31);
        assert_eq!(pins.mdio, 52);
    }

    #[test]
    fn waveshare_phy_reset_is_gpio51_active_low() {
        let reset = BoardConfig::WAVESHARE_P4_ETH.phy_reset;
        assert_eq!(reset.pin, 51);
        assert!(
            reset.active_low,
            "IP101GRI RST_N is active low — flipping this strands the PHY in reset"
        );
    }

    #[test]
    fn waveshare_ref_clock_pad_is_gpio50() {
        assert!(matches!(
            BoardConfig::WAVESHARE_P4_ETH.ref_clock,
            RefClockPin::Gpio50
        ));
    }

    /// Compile-time check that `BoardConfig` is `Copy + Clone` so it can be
    /// freely passed by value through the bring-up API without `&BoardConfig`
    /// lifetime plumbing leaking into call sites.
    #[test]
    fn board_config_is_copy_clone() {
        fn assert_copy<T: Copy>() {}
        fn assert_clone<T: Clone>() {}
        assert_copy::<BoardConfig>();
        assert_clone::<BoardConfig>();

        let a = BoardConfig::WAVESHARE_P4_ETH;
        let b = a; // compiles only because Copy
        let _c = a.clone();
        assert_eq!(a.phy_addr, b.phy_addr);
    }

    /// All P4 GPIO numbers are < 56; anything higher is a typo. Catches
    /// transposition errors like 35 ↔ 53.
    #[test]
    fn all_waveshare_gpio_numbers_fit_p4_pinout() {
        let cfg = BoardConfig::WAVESHARE_P4_ETH;
        let pins = [
            cfg.rmii_pins.txd0,
            cfg.rmii_pins.txd1,
            cfg.rmii_pins.tx_en,
            cfg.rmii_pins.rxd0,
            cfg.rmii_pins.rxd1,
            cfg.rmii_pins.crs_dv,
            cfg.mdio_pins.mdc,
            cfg.mdio_pins.mdio,
            cfg.phy_reset.pin,
        ];
        for p in pins {
            assert!(p < 56, "P4 has GPIO 0..55; got {}", p);
        }
    }

    /// No two RMII / MDIO / reset pins may share a GPIO number — that would
    /// either deadlock the IO_MUX matrix or silently re-route one signal.
    #[test]
    fn waveshare_pin_assignments_are_unique() {
        let cfg = BoardConfig::WAVESHARE_P4_ETH;
        let pins = [
            cfg.rmii_pins.txd0,
            cfg.rmii_pins.txd1,
            cfg.rmii_pins.tx_en,
            cfg.rmii_pins.rxd0,
            cfg.rmii_pins.rxd1,
            cfg.rmii_pins.crs_dv,
            cfg.mdio_pins.mdc,
            cfg.mdio_pins.mdio,
            cfg.phy_reset.pin,
        ];
        let mut sorted = pins;
        sorted.sort();
        for w in sorted.windows(2) {
            assert_ne!(w[0], w[1], "duplicate GPIO assignment in BoardConfig: {}", w[0]);
        }
    }
}
