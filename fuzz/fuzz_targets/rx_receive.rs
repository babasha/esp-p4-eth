#![no_main]

use libfuzzer_sys::fuzz_target;
use esp_p4_eth::{
    zeroed_rx_descriptors, OwnedBy, RDesRing, BUF_SIZE, MIN_RX_FRAME_SIZE, RX_DESC_COUNT,
};

fuzz_target!(|data: &[u8]| {
    let mut descriptors = zeroed_rx_descriptors();
    let mut buffers = [[0u8; BUF_SIZE]; RX_DESC_COUNT];
    let mut ring = RDesRing::new(&mut descriptors, &mut buffers);

    if data.len() < 6 {
        let _ = ring.receive();
        return;
    }

    let owner = if data[0] & 1 == 0 {
        OwnedBy::Cpu
    } else {
        OwnedBy::Dma
    };
    let mut status = (((u16::from(data[1]) << 8) | u16::from(data[2])) as u32) << 16;
    if data[0] & 0b0000_0010 != 0 {
        status |= 1 << 15;
    }
    if data[0] & 0b0000_0100 != 0 {
        status |= 1 << 9;
    }
    if data[0] & 0b0000_1000 != 0 {
        status |= 1 << 8;
    }

    let payload_len = usize::from(data[3]) % (BUF_SIZE + MIN_RX_FRAME_SIZE);
    let payload_end = 4 + payload_len.min(data.len().saturating_sub(4));
    ring.fuzz_seed_current(status, owner, &data[4..payload_end]);

    if let Some((len, frame)) = ring.receive() {
        assert_eq!(len, frame.len());
        assert!(len <= BUF_SIZE);
        ring.pop();
    }

    assert!(ring.fuzz_current_index() < RX_DESC_COUNT);
});
