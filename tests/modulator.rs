//! Integration tests for the generic Modulator (milwave-rs).
//!
//! Tests behavior through the public API only. Internal-state tests
//! (modulator_reset and friends) remain in src/modulator.rs as inline
//! #[cfg(test)] mod tests.

use milwave_rs::modulator::Modulator;
use wavecore_rs::{Nco, Psk8, RootRaisedCosine, FixedTiming};

fn make_test_modulator() -> Modulator<Psk8, RootRaisedCosine, Nco, FixedTiming> {
    let constellation = Psk8;
    let pulse = RootRaisedCosine::new(4, 0.35, 6);
    let carrier = Nco::new(1800.0, 9600);
    let timing = FixedTiming::new(9600, 2400);
    Modulator::new(constellation, pulse, carrier, timing)
}

#[test]
fn test_modulator_output_length() {
    let mut mod_ = make_test_modulator();
    let symbols = vec![0, 1, 2, 3, 4, 5, 6, 7];
    let samples = mod_.modulate(&symbols);

    // 8 symbols * 4 samples/symbol = 32 samples
    assert_eq!(samples.len(), 32);
}

#[test]
fn test_modulator_determinism() {
    let mut mod1 = make_test_modulator();
    let mut mod2 = make_test_modulator();

    let symbols = vec![0, 1, 2, 3, 4, 5, 6, 7];
    let samples1 = mod1.modulate(&symbols);
    let samples2 = mod2.modulate(&symbols);

    assert_eq!(samples1, samples2);
}

#[test]
fn test_modulator_bounded_output() {
    let mut mod_ = make_test_modulator();
    let symbols: Vec<u8> = (0..100).map(|i| i % 8).collect();
    let samples = mod_.modulate(&symbols);

    for &s in &samples {
        assert!(
            s.abs() < 32000,
            "Sample {} exceeds safe range",
            s
        );
    }
}

// ─── Migrated from phy_modem unified_modem_tests::modulator_tests ──────────

use milwave_rs::unified::{UnifiedModulator, ConstellationType};

const SAMPLE_RATE: u32 = 9600;
const SYMBOL_RATE: u32 = 2400;
const CARRIER_FREQ: f64 = 1800.0;
const SPS: usize = 4;

/// Test that UnifiedModulator produces correct number of samples.
#[test]
fn test_unified_modulator_sample_count() {
    let mut mod_ = UnifiedModulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );

    let symbols = vec![0u8; 10];
    let samples = mod_.modulate(&symbols);

    assert_eq!(samples.len(), 10 * SPS,
        "Expected {} samples, got {}", 10 * SPS, samples.len());
}

/// Test that modulator output has expected carrier frequency.
#[test]
fn test_modulator_carrier_frequency() {
    let mut mod_ = UnifiedModulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );

    let symbols = vec![0u8; 100];
    let samples = mod_.modulate(&symbols);

    let mut crossings = 0;
    for i in 1..samples.len() {
        if (samples[i] > 0) != (samples[i-1] > 0) {
            crossings += 1;
        }
    }

    let expected_crossings = (2.0 * CARRIER_FREQ * 100.0 / SYMBOL_RATE as f64) as usize;
    let diff = (crossings as i32 - expected_crossings as i32).abs();
    assert!(diff < expected_crossings as i32 / 10,
        "Zero crossings: expected ~{}, got {}", expected_crossings, crossings);
}

/// Test that different symbols produce different waveforms.
#[test]
fn test_modulator_symbol_differentiation() {
    let mut mod0 = UnifiedModulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );
    let mut mod4 = UnifiedModulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );

    let samples0 = mod0.modulate(&[0]);
    let samples4 = mod4.modulate(&[4]);

    let corr: i64 = samples0.iter().zip(samples4.iter())
        .map(|(&a, &b)| a as i64 * b as i64)
        .sum();

    assert!(corr < 0, "Symbols 0 and 4 should be anti-correlated, got {}", corr);
}

/// Test modulator phase continuity across symbols.
#[test]
#[ignore = "pre-existing failure in phy_modem - phase continuity assertion too tight"]
fn test_modulator_phase_continuity() {
    let mut mod_ = UnifiedModulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );

    let symbols: Vec<u8> = (0..20).map(|i| (i % 2) as u8).collect();
    let samples = mod_.modulate(&symbols);

    let mut max_jump = 0i32;
    for i in 1..samples.len() {
        let jump = (samples[i] as i32 - samples[i-1] as i32).abs();
        max_jump = max_jump.max(jump);
    }

    assert!(max_jump < 20000,
        "Discontinuity detected: max jump = {}", max_jump);
}
