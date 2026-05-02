//! Clause 22 MDIO helpers and IP101 PHY integration.
//!
//! The current implementation targets the common IP101GRI PHY used in ESP32-P4
//! RMII designs. Link state is polled over MDIO and then reflected back into the
//! MAC speed/duplex configuration.

use core::hint::spin_loop;

use crate::{regs, Duplex, Ethernet};

/// Diagnostic UART writer for the cold-boot hang investigation.
///
/// Bangs raw bytes onto UART0's TX FIFO with the same MMIO layout the
/// `mdio_test` example uses, so we can keep the helper self-contained
/// inside `phy.rs` without taking on a `Write`-trait dependency on the
/// caller. ROM bootloader has UART0 already configured at the espflash
/// monitor baud rate by the time `Ip101::init` runs, so no init is needed
/// here.
#[cfg(feature = "phy-init-debug")]
mod debug_uart {
    use core::fmt;
    use core::ptr::{read_volatile, write_volatile};

    const UART0_BASE: usize = 0x500C_A000;
    const UART_FIFO: *mut u32 = UART0_BASE as *mut u32;
    const UART_STATUS: *const u32 = (UART0_BASE + 0x1C) as *const u32;
    const UART_TXFIFO_CNT_SHIFT: u32 = 16;
    const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
    const UART_TXFIFO_CAPACITY: u32 = 128;

    pub struct Writer;

    impl fmt::Write for Writer {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            for byte in s.bytes() {
                if byte == b'\n' {
                    write_byte(b'\r');
                }
                write_byte(byte);
            }
            Ok(())
        }
    }

    fn write_byte(byte: u8) {
        while txfifo_count() >= UART_TXFIFO_CAPACITY {
            core::hint::spin_loop();
        }
        unsafe {
            write_volatile(UART_FIFO, byte as u32);
        }
    }

    fn txfifo_count() -> u32 {
        let status = unsafe { read_volatile(UART_STATUS) };
        (status & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
    }
}

/// Step marker emitted to UART0 when the `phy-init-debug` feature is on.
/// Compiles to a no-op otherwise so the bring-up path stays cost-free in
/// production builds.
#[cfg(feature = "phy-init-debug")]
pub fn diag_log(msg: &str) {
    use core::fmt::Write;
    let mut uart = debug_uart::Writer;
    let _ = writeln!(uart, "{}", msg);
}

/// No-op fallback when `phy-init-debug` is disabled.
#[cfg(not(feature = "phy-init-debug"))]
#[inline(always)]
pub fn diag_log(_msg: &str) {}

// MDIO transaction at csr_clock_range=0 (= MDC ~250 kHz worst case) takes
// up to ~200 us. With opt-level >= 1 the polling loop body collapses to a
// few cycles, so 100 iterations finish in < 1 us and we'd time out before
// the PHY ever responds. 100_000 covers the worst case at any optimization
// level we ship (~5 ms upper bound at 360 MHz CPU, well above hardware
// transaction time).
const MDIO_TIMEOUT_POLLS: usize = 100_000;
const PHY_RESET_TIMEOUT_POLLS: usize = 500_000;

/// Basic Mode Control Register address.
pub const BMCR: u8 = 0x00;
/// Basic Mode Status Register address.
pub const BMSR: u8 = 0x01;
/// PHY ID register 1 address.
pub const PHYIDR1: u8 = 0x02;
/// PHY ID register 2 address.
pub const PHYIDR2: u8 = 0x03;
/// Auto-negotiation advertisement register address.
pub const ANAR: u8 = 0x04;
/// Auto-negotiation link partner ability register address.
pub const ANLPAR: u8 = 0x05;

/// Bit definitions for [`BMCR`].
pub mod bmcr {
    /// Software reset request bit.
    pub const RESET: u16 = 1 << 15;
    /// Selects 100 Mbps operation when auto-negotiation is disabled.
    pub const SPEED_100: u16 = 1 << 13;
    /// Enables auto-negotiation.
    pub const ANEN: u16 = 1 << 12;
    /// Restarts auto-negotiation.
    pub const RESTART_AN: u16 = 1 << 9;
    /// Selects full duplex when auto-negotiation is disabled.
    pub const FULL_DUPLEX: u16 = 1 << 8;
}

/// Bit definitions for [`BMSR`].
pub mod bmsr {
    /// Current link-status bit. On many PHYs this bit is latch-low.
    pub const LINK_STATUS: u16 = 1 << 2;
    /// Auto-negotiation complete bit.
    pub const AN_COMPLETE: u16 = 1 << 5;
}

/// Common capability bits read from [`ANLPAR`].
pub mod anlpar {
    /// Link partner supports 10BASE-T half duplex.
    pub const BASE_10_HALF: u16 = 1 << 5;
    /// Link partner supports 10BASE-T full duplex.
    pub const BASE_10_FULL: u16 = 1 << 6;
    /// Link partner supports 100BASE-TX half duplex.
    pub const BASE_100_HALF: u16 = 1 << 7;
    /// Link partner supports 100BASE-TX full duplex.
    pub const BASE_100_FULL: u16 = 1 << 8;
}

/// High-level link state exposed by the PHY layer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinkState {
    /// No link is currently active.
    Down,
    /// Link is up and includes the negotiated line mode.
    Up { speed: Speed, duplex: Duplex },
}

/// Line speed used by both the PHY and MAC layers.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Speed {
    /// 10 Mbps RMII operation.
    Mbps10,
    /// 100 Mbps RMII operation.
    Mbps100,
}

/// Minimal PHY abstraction used by [`crate::Ethernet`].
pub trait Phy {
    /// Performs one-time PHY initialization.
    fn init(&mut self, _eth: &mut Ethernet<'_>) -> Result<(), PhyError>;
    /// Polls current link state and, when possible, returns negotiated line mode.
    fn poll_link(&mut self, _eth: &mut Ethernet<'_>) -> LinkState;
}

/// PHY/MDIO errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhyError {
    /// A long PHY-side operation did not complete in time.
    Timeout,
    /// Reserved placeholder for MDIO bus-level failures.
    Bus,
    /// MDIO busy bit did not clear before timeout.
    MdioTimeout,
}

/// IP101 PHY model used by the current driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ip101 {
    /// Clause 22 PHY address on the MDIO bus.
    pub addr: u8,
}

impl Ip101 {
    /// Creates an IP101 handle for the given PHY address.
    pub const fn new(addr: u8) -> Self {
        Self { addr }
    }

    /// Resets the PHY and restarts auto-negotiation.
    pub fn init(&mut self, _eth: &mut Ethernet<'_>) -> Result<(), PhyError> {
        #[cfg(feature = "phy-init-debug")]
        {
            use core::fmt::Write;
            let mut uart = debug_uart::Writer;
            let _ = writeln!(uart, "[phy_init] addr={} starting", self.addr);

            // Pre-probe: snapshot PHY register state BEFORE we touch BMCR.
            // If MDIO is already broken (PHY not responding, MAC clock not
            // configured), these will return Err(MdioTimeout) or 0xFFFF.
            for (label, reg) in [("PHYIDR1", PHYIDR1), ("PHYIDR2", PHYIDR2),
                                 ("BMCR", BMCR), ("BMSR", BMSR)] {
                match mdio_read(self.addr, reg) {
                    Ok(v) => {
                        let _ = writeln!(uart, "[phy_init] pre {}=0x{:04X}", label, v);
                    }
                    Err(e) => {
                        let _ = writeln!(uart, "[phy_init] pre {} ERR={:?}", label, e);
                    }
                }
            }
        }

        let write_result = mdio_write(self.addr, BMCR, bmcr::RESET);

        #[cfg(feature = "phy-init-debug")]
        {
            use core::fmt::Write;
            let mut uart = debug_uart::Writer;
            match &write_result {
                Ok(()) => {
                    let _ = writeln!(uart, "[phy_init] BMCR<-RESET write OK, polling clear...");
                }
                Err(e) => {
                    let _ = writeln!(uart, "[phy_init] BMCR<-RESET write ERR={:?}", e);
                }
            }
        }
        write_result?;

        #[cfg(feature = "phy-init-debug")]
        let mut last_bmcr: u16 = 0xFFFF;

        for _i in 0..PHY_RESET_TIMEOUT_POLLS {
            let bmcr_value = match mdio_read(self.addr, BMCR) {
                Ok(v) => v,
                Err(e) => {
                    #[cfg(feature = "phy-init-debug")]
                    {
                        use core::fmt::Write;
                        let mut uart = debug_uart::Writer;
                        let _ = writeln!(
                            uart,
                            "[phy_init] poll iter={} mdio_read ERR={:?}",
                            _i, e
                        );
                    }
                    return Err(e);
                }
            };

            #[cfg(feature = "phy-init-debug")]
            {
                if bmcr_value != last_bmcr || _i == 0 || _i % 50_000 == 0 {
                    use core::fmt::Write;
                    let mut uart = debug_uart::Writer;
                    let _ = writeln!(
                        uart,
                        "[phy_init] iter={} BMCR=0x{:04X}",
                        _i, bmcr_value
                    );
                    last_bmcr = bmcr_value;
                }
            }

            if bmcr_value & bmcr::RESET == 0 {
                #[cfg(feature = "phy-init-debug")]
                {
                    use core::fmt::Write;
                    let mut uart = debug_uart::Writer;
                    let _ = writeln!(
                        uart,
                        "[phy_init] reset cleared at iter={}, writing ANEN|RESTART_AN",
                        _i
                    );
                }

                mdio_write(self.addr, BMCR, bmcr::ANEN | bmcr::RESTART_AN)?;

                #[cfg(feature = "phy-init-debug")]
                {
                    use core::fmt::Write;
                    let mut uart = debug_uart::Writer;
                    let _ = writeln!(uart, "[phy_init] DONE OK");
                }

                return Ok(());
            }

            spin_loop();
        }

        #[cfg(feature = "phy-init-debug")]
        {
            use core::fmt::Write;
            let mut uart = debug_uart::Writer;
            let _ = writeln!(
                uart,
                "[phy_init] TIMEOUT after {} polls (RESET bit never cleared)",
                PHY_RESET_TIMEOUT_POLLS
            );
        }

        Err(PhyError::Timeout)
    }

    /// Returns whether the PHY currently reports link-up.
    pub fn link_up(&mut self, _eth: &mut Ethernet<'_>) -> Result<bool, PhyError> {
        let _ = mdio_read(self.addr, BMSR)?;
        let status = mdio_read(self.addr, BMSR)?;
        Ok(status & bmsr::LINK_STATUS != 0)
    }

    /// Resolves negotiated speed/duplex from link partner abilities.
    pub fn negotiate(&mut self, _eth: &mut Ethernet<'_>) -> Result<(Speed, Duplex), PhyError> {
        let abilities = mdio_read(self.addr, ANLPAR)?;
        Ok(resolve_link_from_anlpar(abilities))
    }

    fn autoneg_complete(&mut self) -> Result<bool, PhyError> {
        let status = mdio_read(self.addr, BMSR)?;
        Ok(status & bmsr::AN_COMPLETE != 0)
    }
}

/// Writes one Clause 22 PHY register over MDIO.
pub fn mdio_write(phy_addr: u8, reg: u8, value: u16) -> Result<(), PhyError> {
    wait_for_mdio_idle(|| regs::read(regs::mac::MII_ADDR))?;

    regs::write(regs::mac::MII_DATA, u32::from(value));
    regs::write(
        regs::mac::MII_ADDR,
        mii_address_value(phy_addr, reg, current_csr_clock_range(), true),
    );
    #[cfg(test)]
    complete_test_mdio_write(reg, value);

    wait_for_mdio_idle(|| regs::read(regs::mac::MII_ADDR))
}

/// Reads one Clause 22 PHY register over MDIO.
pub fn mdio_read(phy_addr: u8, reg: u8) -> Result<u16, PhyError> {
    wait_for_mdio_idle(|| regs::read(regs::mac::MII_ADDR))?;

    regs::write(
        regs::mac::MII_ADDR,
        mii_address_value(phy_addr, reg, current_csr_clock_range(), false),
    );
    #[cfg(test)]
    complete_test_mdio_read(reg);

    wait_for_mdio_idle(|| regs::read(regs::mac::MII_ADDR))?;
    Ok(regs::read(regs::mac::MII_DATA) as u16)
}

fn current_csr_clock_range() -> u32 {
    (regs::read(regs::mac::MII_ADDR) & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK)
        >> regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT
}

fn mii_address_value(phy_addr: u8, reg: u8, csr_clock_range: u32, write: bool) -> u32 {
    let mut value = regs::bits::miiaddr::MII_BUSY
        | ((u32::from(phy_addr) & 0x1f) << regs::bits::miiaddr::PHY_ADDR_SHIFT)
        | ((u32::from(reg) & 0x1f) << regs::bits::miiaddr::REG_ADDR_SHIFT)
        | ((csr_clock_range << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT)
            & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK);

    if write {
        value |= regs::bits::miiaddr::MII_WRITE;
    }

    value
}

fn wait_for_mdio_idle<F>(mut read_miiaddr: F) -> Result<(), PhyError>
where
    F: FnMut() -> u32,
{
    for _ in 0..MDIO_TIMEOUT_POLLS {
        if read_miiaddr() & regs::bits::miiaddr::MII_BUSY == 0 {
            return Ok(());
        }
    }

    Err(PhyError::MdioTimeout)
}

#[cfg(test)]
fn complete_test_mdio_write(reg: u8, value: u16) {
    let data = match reg {
        BMCR if value & bmcr::RESET != 0 => 0,
        _ => u32::from(value),
    };
    regs::write(regs::mac::MII_DATA, data);
    regs::write(
        regs::mac::MII_ADDR,
        current_csr_clock_range() << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
    );
}

#[cfg(test)]
fn complete_test_mdio_read(reg: u8) {
    let value = match reg {
        BMCR => 0,
        BMSR => 0,
        ANLPAR => 0,
        PHYIDR1 => 0,
        PHYIDR2 => 0,
        _ => regs::read(regs::mac::MII_DATA),
    };

    regs::write(regs::mac::MII_DATA, value);
    regs::write(
        regs::mac::MII_ADDR,
        current_csr_clock_range() << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
    );
}

impl Phy for Ip101 {
    fn init(&mut self, eth: &mut Ethernet<'_>) -> Result<(), PhyError> {
        Ip101::init(self, eth)
    }

    fn poll_link(&mut self, eth: &mut Ethernet<'_>) -> LinkState {
        match (self.link_up(eth), self.autoneg_complete()) {
            (Ok(true), Ok(true)) => match self.negotiate(eth) {
                Ok((speed, duplex)) => LinkState::Up { speed, duplex },
                Err(_) => LinkState::Down,
            },
            _ => LinkState::Down,
        }
    }
}

fn resolve_link_from_anlpar(abilities: u16) -> (Speed, Duplex) {
    if abilities & anlpar::BASE_100_FULL != 0 {
        return (Speed::Mbps100, Duplex::Full);
    }

    if abilities & anlpar::BASE_100_HALF != 0 {
        return (Speed::Mbps100, Duplex::Half);
    }

    if abilities & anlpar::BASE_10_FULL != 0 {
        return (Speed::Mbps10, Duplex::Full);
    }

    (Speed::Mbps10, Duplex::Half)
}

#[cfg(test)]
mod tests {
    use super::{
        anlpar, bmcr, bmsr, mdio_read, mdio_write, mii_address_value, resolve_link_from_anlpar,
        wait_for_mdio_idle, PhyError, ANAR, ANLPAR, BMCR, BMSR, PHYIDR1, PHYIDR2,
    };
    use crate::regs;

    #[test]
    fn phy_id_register_constants_match_standard_register_map() {
        assert_eq!(BMCR, 0x00);
        assert_eq!(BMSR, 0x01);
        assert_eq!(PHYIDR1, 0x02);
        assert_eq!(PHYIDR2, 0x03);
        assert_eq!(ANAR, 0x04);
        assert_eq!(ANLPAR, 0x05);
    }

    #[test]
    fn phy_bit_constants_match_clause_22_layout() {
        assert_eq!(bmcr::RESET, 1 << 15);
        assert_eq!(bmcr::SPEED_100, 1 << 13);
        assert_eq!(bmcr::ANEN, 1 << 12);
        assert_eq!(bmcr::RESTART_AN, 1 << 9);
        assert_eq!(bmcr::FULL_DUPLEX, 1 << 8);
        assert_eq!(bmsr::LINK_STATUS, 1 << 2);
        assert_eq!(bmsr::AN_COMPLETE, 1 << 5);
    }

    #[test]
    fn mii_address_value_packs_write_transaction() {
        let value = mii_address_value(0x1f, 0x12, 0b0001, true);

        assert_ne!(value & regs::bits::miiaddr::MII_BUSY, 0);
        assert_ne!(value & regs::bits::miiaddr::MII_WRITE, 0);
        assert_eq!(
            (value & regs::bits::miiaddr::PHY_ADDR_MASK) >> regs::bits::miiaddr::PHY_ADDR_SHIFT,
            0x1f
        );
        assert_eq!(
            (value & regs::bits::miiaddr::REG_ADDR_MASK) >> regs::bits::miiaddr::REG_ADDR_SHIFT,
            0x12
        );
        assert_eq!(
            (value & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK)
                >> regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
            0b0001
        );
    }

    #[test]
    fn mii_address_value_packs_read_transaction_without_write_bit() {
        let value = mii_address_value(0x00, 0x03, 0b0100, false);

        assert_ne!(value & regs::bits::miiaddr::MII_BUSY, 0);
        assert_eq!(value & regs::bits::miiaddr::MII_WRITE, 0);
        assert_eq!(
            (value & regs::bits::miiaddr::REG_ADDR_MASK) >> regs::bits::miiaddr::REG_ADDR_SHIFT,
            0x03
        );
        assert_eq!(
            (value & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK)
                >> regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
            0b0100
        );
    }

    #[test]
    fn mii_address_value_masks_phy_register_and_clock_fields() {
        let value = mii_address_value(0xff, 0xfe, 0xff, true);

        assert_eq!(
            (value & regs::bits::miiaddr::PHY_ADDR_MASK) >> regs::bits::miiaddr::PHY_ADDR_SHIFT,
            0x1f
        );
        assert_eq!(
            (value & regs::bits::miiaddr::REG_ADDR_MASK) >> regs::bits::miiaddr::REG_ADDR_SHIFT,
            0x1e
        );
        assert_eq!(
            (value & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK)
                >> regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
            0x0f
        );
    }

    #[test]
    fn wait_for_mdio_idle_returns_ok_after_busy_clears() {
        let mut polls = 0usize;
        let result = wait_for_mdio_idle(|| {
            polls += 1;
            if polls < 3 {
                regs::bits::miiaddr::MII_BUSY
            } else {
                0
            }
        });

        assert_eq!(result, Ok(()));
        assert_eq!(polls, 3);
    }

    #[test]
    fn wait_for_mdio_idle_times_out_when_busy_stays_set() {
        let result = wait_for_mdio_idle(|| regs::bits::miiaddr::MII_BUSY);

        assert_eq!(result, Err(PhyError::MdioTimeout));
    }

    #[test]
    fn mdio_write_preserves_csr_clock_range_and_writes_data() {
        regs::reset_test_registers();
        regs::write(
            regs::mac::MII_ADDR,
            0b0101 << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
        );

        mdio_write(0x03, 0x1f, 0xCAFE).unwrap();

        assert_eq!(regs::read(regs::mac::MII_DATA), 0xCAFE);
        assert_eq!(
            regs::read(regs::mac::MII_ADDR) & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK,
            0b0101 << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT
        );
    }

    #[test]
    fn mdio_read_preserves_csr_clock_range_and_returns_data() {
        regs::reset_test_registers();
        regs::write(regs::mac::MII_DATA, 0xBEEF);
        regs::write(
            regs::mac::MII_ADDR,
            0b0011 << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT,
        );

        let value = mdio_read(0x03, 0x1f).unwrap();

        assert_eq!(value, 0xBEEF);
        assert_eq!(
            regs::read(regs::mac::MII_ADDR) & regs::bits::miiaddr::CSR_CLOCK_RANGE_MASK,
            0b0011 << regs::bits::miiaddr::CSR_CLOCK_RANGE_SHIFT
        );
    }

    #[test]
    fn mdio_transaction_returns_timeout_when_bus_is_busy() {
        regs::reset_test_registers();
        regs::write(regs::mac::MII_ADDR, regs::bits::miiaddr::MII_BUSY);

        assert_eq!(mdio_read(0, BMSR), Err(PhyError::MdioTimeout));
        assert_eq!(mdio_write(0, BMCR, 0), Err(PhyError::MdioTimeout));
    }

    #[test]
    fn negotiation_prefers_highest_common_mode() {
        assert_eq!(
            resolve_link_from_anlpar(anlpar::BASE_100_FULL),
            (super::Speed::Mbps100, super::Duplex::Full)
        );
        assert_eq!(
            resolve_link_from_anlpar(anlpar::BASE_100_HALF),
            (super::Speed::Mbps100, super::Duplex::Half)
        );
        assert_eq!(
            resolve_link_from_anlpar(anlpar::BASE_10_FULL),
            (super::Speed::Mbps10, super::Duplex::Full)
        );
        assert_eq!(
            resolve_link_from_anlpar(0),
            (super::Speed::Mbps10, super::Duplex::Half)
        );
    }

    #[test]
    fn negotiation_uses_ethernet_priority_order_for_combined_abilities() {
        let all_modes = anlpar::BASE_10_HALF
            | anlpar::BASE_10_FULL
            | anlpar::BASE_100_HALF
            | anlpar::BASE_100_FULL;

        assert_eq!(
            resolve_link_from_anlpar(all_modes),
            (super::Speed::Mbps100, super::Duplex::Full)
        );
        assert_eq!(
            resolve_link_from_anlpar(anlpar::BASE_10_FULL | anlpar::BASE_100_HALF),
            (super::Speed::Mbps100, super::Duplex::Half)
        );
    }
}
