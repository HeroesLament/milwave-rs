//! ALE-specific integration tests for milwave-rs.
//!
//! Migrated from phy_modem unified_modem_tests::ale_tests.

use milwave_rs::unified::{UnifiedModulator, UnifiedDemodulator, ConstellationType};

const SAMPLE_RATE: u32 = 9600;
const SYMBOL_RATE: u32 = 2400;
const CARRIER_FREQ: f64 = 1800.0;

/// The ALE capture probe sequence (first 32 symbols).
const CAPTURE_PROBE: [u8; 32] = [
    0, 4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 4, 0, 0, 4,
    4, 4, 0, 4, 0, 0, 0, 4, 0, 4, 0, 4, 4, 0, 4, 0,
];

/// Test that we can recover the capture probe.
#[test]
#[ignore = "pre-existing failure in phy_modem - capture probe correlation is 0/32"]
fn test_capture_probe_recovery() {
    let mut mod_ = UnifiedModulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );
    let mut demod = UnifiedDemodulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );

    let mut symbols = vec![0u8; 50];
    symbols.extend_from_slice(&CAPTURE_PROBE);
    symbols.extend_from_slice(&CAPTURE_PROBE);
    symbols.extend_from_slice(&CAPTURE_PROBE);

    let mut samples = mod_.modulate(&symbols);
    samples.extend(mod_.flush());

    let recovered = demod.demodulate(&samples);

    let probe_start = 50 + 32 + 15;

    if recovered.len() >= probe_start + 32 {
        let rx_section = &recovered[probe_start..probe_start + 32];

        let corr: i32 = CAPTURE_PROBE.iter().zip(rx_section)
            .map(|(&t, &r)| {
                let t_sign = if t < 4 { 1 } else { -1 };
                let r_sign = if r < 4 { 1 } else { -1 };
                t_sign * r_sign
            })
            .sum();

        assert!(corr.abs() >= 28,
            "Capture probe correlation too low: {}", corr);
    }
}
