//! Integration tests for ConstellationType (milwave-rs).
//!
//! All tests here use only the public ConstellationType API.
//! Migrated from phy_modem/src/modem/unified_modem_tests.rs `constellation_tests`.

use milwave_rs::unified::ConstellationType;
use core::f64::consts::PI;

/// Test that PSK8 constellation points are at the correct phases.
#[test]
fn test_psk8_constellation_phases() {
    for sym in 0..8u8 {
        let (i, q) = ConstellationType::Psk8.symbol_to_iq(sym);

        let expected_phase = sym as f64 * PI / 4.0;
        let expected_i = expected_phase.cos();
        let expected_q = expected_phase.sin();

        let i_err = (i - expected_i).abs();
        let q_err = (q - expected_q).abs();

        assert!(i_err < 1e-10, "PSK8 sym {} I: expected {}, got {}", sym, expected_i, i);
        assert!(q_err < 1e-10, "PSK8 sym {} Q: expected {}, got {}", sym, expected_q, q);

        let mag = (i * i + q * q).sqrt();
        assert!((mag - 1.0).abs() < 1e-10, "PSK8 sym {} magnitude: {}", sym, mag);
    }
}

/// Test that iq_to_symbol correctly inverts symbol_to_iq.
#[test]
fn test_psk8_roundtrip() {
    for sym in 0..8u8 {
        let (i, q) = ConstellationType::Psk8.symbol_to_iq(sym);
        let recovered = ConstellationType::Psk8.iq_to_symbol(i, q);
        assert_eq!(sym, recovered, "PSK8 roundtrip failed for sym {}: got {}", sym, recovered);
    }
}

/// Test decision boundaries for PSK8.
/// Each symbol occupies ±22.5° around its nominal phase.
#[test]
fn test_psk8_decision_boundaries() {
    for sym in 0..8u8 {
        let nominal_phase = sym as f64 * PI / 4.0;

        let i = nominal_phase.cos();
        let q = nominal_phase.sin();
        assert_eq!(ConstellationType::Psk8.iq_to_symbol(i, q), sym,
            "Nominal phase {} failed", sym);

        let offset = 20.0 * PI / 180.0;
        let i_plus = (nominal_phase + offset).cos();
        let q_plus = (nominal_phase + offset).sin();
        assert_eq!(ConstellationType::Psk8.iq_to_symbol(i_plus, q_plus), sym,
            "Sym {} at +20° failed", sym);

        let i_minus = (nominal_phase - offset).cos();
        let q_minus = (nominal_phase - offset).sin();
        assert_eq!(ConstellationType::Psk8.iq_to_symbol(i_minus, q_minus), sym,
            "Sym {} at -20° failed", sym);
    }
}

/// Test that BPSK uses symbols 0 and 4 correctly (needed for ALE).
#[test]
fn test_bpsk_phase_mapping() {
    let (i0, q0) = ConstellationType::Bpsk.symbol_to_iq(0);
    assert!((i0 - 1.0).abs() < 1e-10, "BPSK sym 0 I should be 1, got {}", i0);
    assert!(q0.abs() < 1e-10, "BPSK sym 0 Q should be 0, got {}", q0);

    let (i1, q1) = ConstellationType::Bpsk.symbol_to_iq(1);
    assert!((i1 + 1.0).abs() < 1e-10, "BPSK sym 1 I should be -1, got {}", i1);
    assert!(q1.abs() < 1e-10, "BPSK sym 1 Q should be 0, got {}", q1);
}

/// Verify PSK8 symbols 0 and 4 match BPSK symbols 0 and 1.
#[test]
fn test_psk8_bpsk_compatibility() {
    let (psk8_0_i, psk8_0_q) = ConstellationType::Psk8.symbol_to_iq(0);
    let (psk8_4_i, psk8_4_q) = ConstellationType::Psk8.symbol_to_iq(4);

    assert!((psk8_0_i - 1.0).abs() < 1e-10, "PSK8 sym 0 should be at phase 0");
    assert!(psk8_0_q.abs() < 1e-10);

    assert!((psk8_4_i + 1.0).abs() < 1e-10, "PSK8 sym 4 should be at phase 180");
    assert!(psk8_4_q.abs() < 1e-10);
}
