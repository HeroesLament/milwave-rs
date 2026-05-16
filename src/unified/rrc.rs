//! Root-Raised-Cosine (RRC) pulse shaping for 188-110D.
//!
//! This RRC implementation is intentionally separate from `wavecore_rs::RootRaisedCosine`
//! because the modem code currently uses a flat `Vec<f64>` of coefficients directly,
//! while wavecore's API wraps coefficients in a struct. Functional equivalence has not
//! been bit-exact-verified, so we keep the protocol-specific impl here.
//!
//! Both `RRC_SPAN` and `generate_rrc_coeffs` are `pub(crate)`: visible to the rest of
//! milwave-rs (the modulator/demodulator submodules and inline tests) but not part of
//! the public API. Consumers should use `wavecore_rs::RootRaisedCosine` instead.

use core::f64::consts::PI;

#[allow(unused_imports)]
use num_traits::Float;

#[cfg(feature = "std")]
use std::vec::Vec;

#[cfg(all(not(feature = "std"), feature = "alloc"))]
use alloc::vec::Vec;

const RRC_ALPHA: f64 = 0.35;

/// One-sided filter span in symbols. Total coefficient count is `2 * RRC_SPAN * sps + 1`.
pub(crate) const RRC_SPAN: usize = 6;

/// Generate energy-normalized RRC filter coefficients for the given samples-per-symbol.
///
/// Length is `2 * RRC_SPAN * sps + 1` (always odd, symmetric, linear phase).
/// Energy normalization ensures `sum(c²) = 1.0`.
pub(crate) fn generate_rrc_coeffs(sps: usize) -> Vec<f64> {
    let len = 2 * RRC_SPAN * sps + 1;
    let mut coeffs = vec![0.0; len];
    let center = (len - 1) / 2;

    for i in 0..len {
        let t = (i as f64 - center as f64) / sps as f64;
        coeffs[i] = rrc_sample(t, RRC_ALPHA);
    }

    // Normalize filter for unit energy.
    let energy: f64 = coeffs.iter().map(|x| x * x).sum();
    let norm = energy.sqrt();
    for c in &mut coeffs {
        *c /= norm;
    }

    coeffs
}

/// Closed-form RRC impulse response sample at time `t` (symbols), roll-off `alpha`.
///
/// Handles the two singular points:
/// - `t = 0`: `1 - alpha + 4*alpha/PI`
/// - `t = ±1/(4*alpha)`: derived l'Hôpital limit (the standard removable singularity)
fn rrc_sample(t: f64, alpha: f64) -> f64 {
    if t.abs() < 1e-10 {
        1.0 - alpha + 4.0 * alpha / PI
    } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-10 {
        alpha / 2.0_f64.sqrt() *
            ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin() +
             (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
    } else {
        let num = (PI * t * (1.0 - alpha)).sin() +
                  4.0 * alpha * t * (PI * t * (1.0 + alpha)).cos();
        let den = PI * t * (1.0 - (4.0 * alpha * t).powi(2));
        num / den
    }
}

// =============================================================================
// Tests
// =============================================================================
//
// These test the pub(crate) helpers above. They stay inline because moving them
// to integration tests would require promoting `generate_rrc_coeffs` and
// `RRC_SPAN` to public API, duplicating wavecore_rs::RootRaisedCosine's surface.

#[cfg(test)]
mod tests {
    use super::*;

    /// Test that RRC filter has correct length.
    #[test]
    fn test_rrc_length() {
        let sps = 4;
        let coeffs = generate_rrc_coeffs(sps);
        let expected_len = 2 * RRC_SPAN * sps + 1;
        assert_eq!(coeffs.len(), expected_len,
            "RRC length should be {}, got {}", expected_len, coeffs.len());
    }

    /// Test that RRC filter is normalized for unit energy.
    #[test]
    fn test_rrc_normalization() {
        let sps = 4;
        let coeffs = generate_rrc_coeffs(sps);

        let energy: f64 = coeffs.iter().map(|x| x * x).sum();
        assert!((energy - 1.0).abs() < 1e-6,
            "RRC energy should be 1.0, got {}", energy);
    }

    /// Test RRC symmetry (linear phase).
    #[test]
    fn test_rrc_symmetry() {
        let sps = 4;
        let coeffs = generate_rrc_coeffs(sps);
        let n = coeffs.len();

        for i in 0..n/2 {
            let diff = (coeffs[i] - coeffs[n - 1 - i]).abs();
            assert!(diff < 1e-10,
                "RRC not symmetric at {}: {} vs {}", i, coeffs[i], coeffs[n-1-i]);
        }
    }

    /// Test that cascaded TX+RX RRC = raised cosine (Nyquist criterion).
    #[test]
    fn test_rrc_cascade_is_nyquist() {
        let sps = 4;
        let rrc = generate_rrc_coeffs(sps);

        let rc_len = 2 * rrc.len() - 1;
        let mut rc = vec![0.0; rc_len];

        for (i, &h1) in rrc.iter().enumerate() {
            for (j, &h2) in rrc.iter().enumerate() {
                rc[i + j] += h1 * h2;
            }
        }

        let center = rc_len / 2;

        // Raised cosine should be near-zero at symbol intervals (except center).
        for k in 1..=RRC_SPAN {
            let idx_plus = center + k * sps;
            let idx_minus = center - k * sps;

            if idx_plus < rc_len {
                let val: f64 = rc[idx_plus] / rc[center];
                assert!(val.abs() < 0.05,
                    "RC not zero at +{} symbols: {}", k, val);
            }
            if idx_minus < rc_len {
                let val: f64 = rc[idx_minus] / rc[center];
                assert!(val.abs() < 0.05,
                    "RC not zero at -{} symbols: {}", k, val);
            }
        }
    }
}
