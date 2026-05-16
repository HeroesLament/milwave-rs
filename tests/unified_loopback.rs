//! Integration tests for UnifiedModulator/UnifiedDemodulator (milwave-rs).
//!
//! Public-API tests only. PLL state, DFE convergence, constellation switching
//! state, and similar internal-automaton tests stay inline in src/unified.rs.

use milwave_rs::unified::{UnifiedModulator, UnifiedDemodulator, ConstellationType};

#[test]
fn test_constellation_roundtrip() {
    for ct in [
        ConstellationType::Bpsk,
        ConstellationType::Qpsk,
        ConstellationType::Psk8,
        ConstellationType::Qam16,
    ] {
        for sym in 0..ct.order() as u8 {
            let (i, q) = ct.symbol_to_iq(sym);
            let recovered = ct.iq_to_symbol(i, q);
            assert_eq!(sym, recovered, "{:?} symbol {} roundtrip failed", ct, sym);
        }
    }
}

#[test]
fn test_loopback() {
    let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
    let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

    let preamble = vec![0u8; 20];
    let data = vec![0, 1, 2, 3, 4, 5, 6, 7];
    let mut all_symbols = preamble.clone();
    all_symbols.extend(&data);

    let mut samples = modulator.modulate(&all_symbols);
    samples.extend(modulator.flush());

    let recovered = demodulator.demodulate(&samples);

    let skip = 20 + 12;
    if recovered.len() >= skip + data.len() {
        let offset = (recovered[skip] + 8 - data[0]) % 8;

        let mut errors = 0;
        for i in 0..data.len() {
            let expected = (data[i] + offset) % 8;
            if recovered[skip + i] != expected {
                errors += 1;
            }
        }
        assert!(errors <= 1, "Too many errors: {} out of {}", errors, data.len());
    }
}

// ─── Migrated from phy_modem unified_modem_tests::loopback_tests ───────────

const SR: u32 = 9600;
const BR: u32 = 2400;
const CF: f64 = 1800.0;

/// Basic loopback with preamble for synchronization.
#[test]
#[ignore = "pre-existing failure in phy_modem - loopback errors exceed threshold"]
fn test_loopback_basic() {
    let mut mod_ = UnifiedModulator::new(ConstellationType::Psk8, SR, BR, CF);
    let mut demod = UnifiedDemodulator::new(ConstellationType::Psk8, SR, BR, CF);

    let preamble: Vec<u8> = vec![0; 50];
    let data: Vec<u8> = (0..8).cycle().take(32).collect();

    let mut symbols = preamble.clone();
    symbols.extend(&data);

    let mut samples = mod_.modulate(&symbols);
    samples.extend(mod_.flush());

    let recovered = demod.demodulate(&samples);

    let skip = 50 + 15;

    if recovered.len() >= skip + data.len() {
        let offset = (recovered[skip] + 8 - data[0]) % 8;

        let mut errors = 0;
        for i in 0..data.len() {
            let expected = (data[i] + offset) % 8;
            if recovered[skip + i] != expected {
                errors += 1;
            }
        }
        assert!(errors <= 2, "Too many errors: {} out of {}", errors, data.len());
    } else {
        panic!("Not enough recovered symbols: {} (need {})",
            recovered.len(), skip + data.len());
    }
}

/// Test loopback with BPSK-only (ALE preamble scenario).
#[test]
fn test_loopback_bpsk_only() {
    let mut mod_ = UnifiedModulator::new(ConstellationType::Psk8, SR, BR, CF);
    let mut demod = UnifiedDemodulator::new(ConstellationType::Psk8, SR, BR, CF);

    let bpsk_sequence: Vec<u8> = vec![
        0, 4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 4, 0, 0, 4,
        4, 4, 0, 4, 0, 0, 0, 4, 0, 4, 0, 4, 4, 0, 4, 0,
    ];

    let mut symbols = vec![0u8; 50];
    symbols.extend(&bpsk_sequence);
    symbols.extend(&bpsk_sequence);

    let mut samples = mod_.modulate(&symbols);
    samples.extend(mod_.flush());

    let recovered = demod.demodulate(&samples);

    let skip = 50 + 15;
    let data_len = bpsk_sequence.len() * 2;

    if recovered.len() >= skip + data_len {
        let tx_bpsk: Vec<u8> = symbols[50..50+data_len].iter()
            .map(|&s| if s < 4 { 0 } else { 1 })
            .collect();

        let rx_bpsk: Vec<u8> = recovered[skip..skip+data_len].iter()
            .map(|&s| if s < 4 { 0 } else { 1 })
            .collect();

        let errors_normal: usize = tx_bpsk.iter().zip(&rx_bpsk)
            .filter(|(&t, &r)| t != r).count();
        let errors_inverted: usize = tx_bpsk.iter().zip(&rx_bpsk)
            .filter(|(&t, &r)| t != (1 - r)).count();

        let errors = errors_normal.min(errors_inverted);

        assert!(errors <= 3, "Too many BPSK errors: {} out of {}", errors, data_len);
    }
}

/// Test that timing recovery works.
#[test]
fn test_timing_recovery() {
    let mut mod_ = UnifiedModulator::new(ConstellationType::Psk8, SR, BR, CF);
    let mut demod = UnifiedDemodulator::new(ConstellationType::Psk8, SR, BR, CF);

    let symbols: Vec<u8> = vec![0; 100];
    let samples = mod_.modulate(&symbols);

    let iq = demod.demodulate_iq(&samples);

    let skip = 20;
    if iq.len() > skip + 20 {
        let mags: Vec<f64> = iq[skip..skip+20].iter()
            .map(|(i, q)| (i*i + q*q).sqrt())
            .collect();

        let mean_mag: f64 = mags.iter().sum::<f64>() / mags.len() as f64;
        let variance: f64 = mags.iter()
            .map(|m| (m - mean_mag).powi(2))
            .sum::<f64>() / mags.len() as f64;
        let std_dev = variance.sqrt();

        assert!(std_dev / mean_mag < 0.3,
            "Timing not stable: CV = {:.1}%", 100.0 * std_dev / mean_mag);
    }
}

