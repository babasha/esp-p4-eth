#![no_main]

use libfuzzer_sys::fuzz_target;
use esp_p4_eth::{zeroed_tx_descriptors, DescriptorError, TDesRing, BUF_SIZE, TX_DESC_COUNT};

fuzz_target!(|data: &[u8]| {
    let mut descriptors = zeroed_tx_descriptors();
    let mut buffers = [[0u8; BUF_SIZE]; TX_DESC_COUNT];
    let mut ring = TDesRing::new(&mut descriptors, &mut buffers);

    match ring.transmit(data) {
        Ok(()) => {
            assert!(data.len() <= BUF_SIZE);
        }
        Err(DescriptorError::BufferTooLarge(_)) => {
            assert!(data.len() > BUF_SIZE);
        }
        Err(DescriptorError::RingFull) => {}
        Err(DescriptorError::NoFrame) => {}
    }

    assert!(ring.fuzz_current_index() < TX_DESC_COUNT);
});
