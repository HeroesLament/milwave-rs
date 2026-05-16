//! UnifiedModulator — the TX-side state machine for 188-110D.
//!
//! Handles RRC pulse shaping + NCO carrier modulation + output scaling.
//! Supports per-symbol or whole-frame constellation switching, which is required
//! for 188-110D where PSK8 mini-probes are interleaved with QAM data symbols
//! and filter state must be preserved across the switch.
//!
//! State preserved across calls:
//! - `nco_phase` — carrier phase accumulator
//! - `i_history` / `q_history` — RRC filter delay lines
//! - `constellation` — current modulation mode
//!
//! Calling `flush()` after the final `modulate()` drains the RRC filter tail
//! (adds `2 * RRC_SPAN` zero symbols). Calling `reset()` zeros the filter
//! state and NCO phase.

use core::f64::consts::PI;

#[cfg(feature = "std")]
use std::vec::Vec;

#[cfg(all(not(feature = "std"), feature = "alloc"))]
use alloc::vec::Vec;

#[allow(unused_imports)]
use num_traits::Float;

use super::constellation::ConstellationType;
use super::rrc::{generate_rrc_coeffs, RRC_SPAN};

pub struct UnifiedModulator {
    constellation: ConstellationType,
    sample_rate: u32,
    symbol_rate: u32,
    carrier_freq: f64,
    sps: usize,

    rrc_coeffs: Vec<f64>,
    i_history: Vec<f64>,
    q_history: Vec<f64>,

    nco_phase: f64,
    nco_phase_inc: f64,

    output_scale: f64,
}

impl UnifiedModulator {
    pub fn new(
        constellation: ConstellationType,
        sample_rate: u32,
        symbol_rate: u32,
        carrier_freq: f64,
    ) -> Self {
        let sps = (sample_rate / symbol_rate) as usize;
        let rrc_coeffs = generate_rrc_coeffs(sps);
        let filter_len = rrc_coeffs.len();

        Self {
            constellation,
            sample_rate,
            symbol_rate,
            carrier_freq,
            sps,
            rrc_coeffs,
            i_history: vec![0.0; filter_len],
            q_history: vec![0.0; filter_len],
            nco_phase: 0.0,
            nco_phase_inc: 2.0 * PI * carrier_freq / sample_rate as f64,
            output_scale: 32768.0,
        }
    }

    /// Switch constellation without resetting filter state.
    pub fn set_constellation(&mut self, constellation: ConstellationType) {
        self.constellation = constellation;
    }

    /// Get current constellation.
    pub fn constellation(&self) -> ConstellationType {
        self.constellation
    }

    /// Modulate symbols to audio samples.
    pub fn modulate(&mut self, symbols: &[u8]) -> Vec<i16> {
        let impulse_offset = self.sps / 2;
        let mut output = Vec::with_capacity(symbols.len() * self.sps);

        for &sym in symbols {
            let (i_val, q_val) = self.constellation.symbol_to_iq(sym);

            for sample_idx in 0..self.sps {
                self.i_history.rotate_left(1);
                self.q_history.rotate_left(1);

                let last = self.i_history.len() - 1;

                if sample_idx == impulse_offset {
                    self.i_history[last] = i_val;
                    self.q_history[last] = q_val;
                } else {
                    self.i_history[last] = 0.0;
                    self.q_history[last] = 0.0;
                }

                let i_filtered = self.apply_filter(&self.i_history);
                let q_filtered = self.apply_filter(&self.q_history);

                let cos_val = self.nco_phase.cos();
                let sin_val = self.nco_phase.sin();
                let sample = i_filtered * cos_val - q_filtered * sin_val;

                self.nco_phase += self.nco_phase_inc;
                if self.nco_phase > 2.0 * PI {
                    self.nco_phase -= 2.0 * PI;
                }

                output.push((sample * self.output_scale) as i16);
            }
        }

        output
    }

    /// Modulate with constellation specified per-symbol (for mini-probe interleaving).
    pub fn modulate_mixed(&mut self, symbols: &[(u8, ConstellationType)]) -> Vec<i16> {
        let impulse_offset = self.sps / 2;
        let mut output = Vec::with_capacity(symbols.len() * self.sps);

        for &(sym, constellation) in symbols {
            let (i_val, q_val) = constellation.symbol_to_iq(sym);

            for sample_idx in 0..self.sps {
                self.i_history.rotate_left(1);
                self.q_history.rotate_left(1);

                let last = self.i_history.len() - 1;

                if sample_idx == impulse_offset {
                    self.i_history[last] = i_val;
                    self.q_history[last] = q_val;
                } else {
                    self.i_history[last] = 0.0;
                    self.q_history[last] = 0.0;
                }

                let i_filtered = self.apply_filter(&self.i_history);
                let q_filtered = self.apply_filter(&self.q_history);

                let cos_val = self.nco_phase.cos();
                let sin_val = self.nco_phase.sin();
                let sample = i_filtered * cos_val - q_filtered * sin_val;

                self.nco_phase += self.nco_phase_inc;
                if self.nco_phase > 2.0 * PI {
                    self.nco_phase -= 2.0 * PI;
                }

                output.push((sample * self.output_scale) as i16);
            }
        }

        output
    }

    /// Flush filter tail by feeding `2 * RRC_SPAN` zero symbols.
    pub fn flush(&mut self) -> Vec<i16> {
        let flush_count = 2 * RRC_SPAN;
        let zeros = vec![0u8; flush_count];
        self.modulate(&zeros)
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        for x in &mut self.i_history { *x = 0.0; }
        for x in &mut self.q_history { *x = 0.0; }
        self.nco_phase = 0.0;
    }

    #[inline]
    fn apply_filter(&self, history: &[f64]) -> f64 {
        let mut sum = 0.0;
        for (h, c) in history.iter().zip(self.rrc_coeffs.iter()) {
            sum += h * c;
        }
        sum
    }
}

// =============================================================================
// Tests
// =============================================================================
//
// One inline test for mid-frame constellation switching, which exercises
// state that crosses the public API surface (RRC filter history preserved
// across `set_constellation()`).

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_modulator_constellation_switch() {
        let mut mod_ = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let psk_samples = mod_.modulate(&[0, 1, 2, 3]);
        assert!(!psk_samples.is_empty());

        mod_.set_constellation(ConstellationType::Qam16);
        let qam_samples = mod_.modulate(&[0, 1, 2, 3]);
        assert!(!qam_samples.is_empty());

        assert_ne!(psk_samples, qam_samples);
    }
}
