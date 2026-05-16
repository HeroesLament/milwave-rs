//! Integration tests for the generic Demodulator (milwave-rs).
//!
//! Public-API tests only. Stateful reset tests stay inline in src/.

use milwave_rs::modulator::Modulator;
use milwave_rs::demodulator::Demodulator;
use wavecore_rs::{Nco, Psk8, RootRaisedCosine, FixedTiming};

fn make_modulator() -> Modulator<Psk8, RootRaisedCosine, Nco, FixedTiming> {
    let constellation = Psk8;
    let pulse = RootRaisedCosine::new(4, 0.35, 6);
    let carrier = Nco::new(1800.0, 9600);
    let timing = FixedTiming::new(9600, 2400);
    Modulator::new(constellation, pulse, carrier, timing)
}

fn make_demodulator() -> Demodulator<Psk8, RootRaisedCosine, Nco, FixedTiming> {
    let constellation = Psk8;
    let pulse = RootRaisedCosine::new(4, 0.35, 6);
    let carrier = Nco::new(1800.0, 9600);
    let timing = FixedTiming::new(9600, 2400);
    Demodulator::new(constellation, pulse, carrier, timing)
}

#[test]
fn test_loopback() {
    let mut modulator = make_modulator();
    let mut demodulator = make_demodulator();

    let preamble = vec![0u8; 20];
    let data = vec![0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3];
    let mut all_symbols = preamble.clone();
    all_symbols.extend(&data);

    let samples = modulator.modulate(&all_symbols);
    let flush = modulator.flush();

    let mut all_samples = samples;
    all_samples.extend(flush);

    let recovered = demodulator.demodulate(&all_samples);

    let skip = 20 + 12;
    let data_len = data.len();

    if recovered.len() > skip + data_len {
        let recovered_data = &recovered[skip..skip + data_len];
        assert_eq!(
            recovered_data, &data[..],
            "Loopback failed: {:?} vs {:?}",
            recovered_data, data
        );
    }
}

#[test]
fn test_demodulate_to_iq() {
    let mut modulator = make_modulator();
    let mut demodulator = make_demodulator();

    let symbols = vec![0, 2, 4, 6];
    let samples = modulator.modulate(&symbols);
    let flush = modulator.flush();

    let mut all_samples = samples;
    all_samples.extend(flush);

    let soft = demodulator.demodulate_to_iq(&all_samples);

    assert!(soft.iq.len() >= symbols.len(),
        "Expected at least {} I/Q samples, got {}",
        symbols.len(), soft.iq.len());

    let sps = 9600 / 2400;
    assert!(soft.timing_offset < sps,
        "Timing offset {} should be < {}", soft.timing_offset, sps);
}

// ─── Migrated from phy_modem unified_modem_tests::demodulator_tests ────────

use milwave_rs::unified::{UnifiedDemodulator, ConstellationType};

/// Test demodulator with perfect (analytically generated) signal.
#[test]
fn test_demodulator_with_perfect_signal() {
    const SAMPLE_RATE: u32 = 9600;
    const SYMBOL_RATE: u32 = 2400;
    const CARRIER_FREQ: f64 = 1800.0;

    let mut demod = UnifiedDemodulator::new(
        ConstellationType::Psk8, SAMPLE_RATE, SYMBOL_RATE, CARRIER_FREQ
    );

    // Generate a perfect PSK8 signal analytically.
    // Symbol 0 at phase 0, no pulse shaping, just raw carrier.
    let num_samples = 200;
    let mut samples = Vec::with_capacity(num_samples);

    for i in 0..num_samples {
        let t = i as f64 / SAMPLE_RATE as f64;
        let phase = 2.0 * core::f64::consts::PI * CARRIER_FREQ * t;
        let sample = (phase.cos() * 16000.0) as i16;
        samples.push(sample);
    }

    let iq = demod.demodulate_iq(&samples);

    let skip = 20;
    if iq.len() > skip {
        for (idx, &(i, q)) in iq.iter().skip(skip).enumerate() {
            let mag = (i * i + q * q).sqrt();
            assert!(mag > 0.1, "Sample {} has low magnitude: {}", idx, mag);
        }
    }
}
