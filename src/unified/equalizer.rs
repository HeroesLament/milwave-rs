//! Decision Feedback Equalizer (DFE) for HF multipath channels.
//!
//! The DFE operates in two modes:
//!
//! 1. **CMA** (Constant Modulus Algorithm) — blind acquisition, no training
//!    needed. Works because PSK/QAM signals have approximately constant envelope.
//!    Minimizes `(|y|² - R²)²` where `R²` is the expected modulus squared.
//!
//! 2. **DD** (Decision-Directed LMS) — uses the slicer's symbol decisions as
//!    references for adaptation. Better steady-state performance than CMA, but
//!    requires good initial convergence (which CMA provides automatically).
//!
//! Transitions:
//! - CMA → DD when CMA cost drops below threshold (`cma_to_dd_threshold`).
//! - DD → CMA fallback when error power rises (decisions becoming unreliable).
//!
//! `train(i, q, known_symbol)` forces DD mode immediately, with 2× step size,
//! for fastest convergence using known reference symbols.

use wavecore_rs::Complex;

#[cfg(feature = "std")]
use std::vec::Vec;

#[cfg(all(not(feature = "std"), feature = "alloc"))]
use alloc::vec::Vec;

use super::constellation::ConstellationType;

/// Equalizer operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EqMode {
    /// Constant Modulus Algorithm - blind acquisition (no training needed)
    CMA,
    /// Decision-Directed LMS - requires good initial convergence
    DD,
}

/// Configuration for the Decision Feedback Equalizer
#[derive(Debug, Clone)]
pub struct DFEConfig {
    /// Number of feedforward filter taps (typically 11-21)
    pub ff_taps: usize,
    /// Number of feedback filter taps (typically 5-10)
    pub fb_taps: usize,
    /// LMS step size for DD mode (0.01 - 0.1)
    pub mu: f64,
    /// CMA step size (typically smaller, 0.001 - 0.01)
    pub mu_cma: f64,
    /// Leakage factor for coefficient updates (0.999 - 1.0)
    pub leakage: f64,
    /// Minimum signal magnitude to update coefficients
    pub update_threshold: f64,
    /// MSE threshold to switch from CMA to DD mode
    pub cma_to_dd_threshold: f64,
    /// Number of symbols before considering mode switch
    pub cma_min_symbols: usize,
}

impl Default for DFEConfig {
    fn default() -> Self {
        Self {
            ff_taps: 15,
            fb_taps: 7,
            mu: 0.03,
            mu_cma: 0.005,
            leakage: 0.9999,
            update_threshold: 0.1,
            cma_to_dd_threshold: 0.3,
            cma_min_symbols: 50,
        }
    }
}

impl DFEConfig {
    /// Configuration optimized for HF skywave channels (2-4ms delay spread)
    pub fn hf_skywave() -> Self {
        Self {
            ff_taps: 21,
            fb_taps: 10,
            mu: 0.03,
            mu_cma: 0.005,
            leakage: 0.9995,
            update_threshold: 0.05,
            cma_to_dd_threshold: 0.25,
            cma_min_symbols: 64,
        }
    }

    /// Configuration for ground wave (minimal multipath)
    pub fn ground_wave() -> Self {
        Self {
            ff_taps: 7,
            fb_taps: 3,
            mu: 0.05,
            mu_cma: 0.01,
            leakage: 1.0,
            update_threshold: 0.05,
            cma_to_dd_threshold: 0.2,
            cma_min_symbols: 30,
        }
    }

    /// Fast acquisition configuration (for training)
    pub fn fast_acquisition() -> Self {
        Self {
            ff_taps: 15,
            fb_taps: 7,
            mu: 0.1,
            mu_cma: 0.02,
            leakage: 0.999,
            update_threshold: 0.05,
            cma_to_dd_threshold: 0.3,
            cma_min_symbols: 32,
        }
    }
}

/// Per-symbol DFE telemetry snapshot
#[derive(Clone, Debug)]
pub struct DfeTelemetry {
    pub symbol_idx: u64,
    pub mse: f64,
    pub cma_cost: f64,
    pub out_mag_sq: f64,
    pub in_mag_sq: f64,
    pub tap_energy: f64,
    pub mode: u8,
}

pub struct DFE {
    config: DFEConfig,
    constellation: ConstellationType,
    mode: EqMode,
    ff_coeffs: Vec<Complex>,
    ff_history: Vec<Complex>,
    fb_coeffs: Vec<Complex>,
    fb_history: Vec<u8>,
    cma_r2: f64,
    total_symbols: u64,
    error_power_avg: f64,
    cma_cost_avg: f64,
    telemetry_enabled: bool,
    telemetry: Vec<DfeTelemetry>,
}

impl DFE {
    pub fn new(config: DFEConfig, constellation: ConstellationType) -> Self {
        let ff_taps = config.ff_taps;
        let fb_taps = config.fb_taps;
        let cma_r2 = Self::compute_cma_r2(constellation);

        let mut dfe = Self {
            config,
            constellation,
            mode: EqMode::CMA,
            ff_coeffs: vec![Complex::zero(); ff_taps],
            ff_history: vec![Complex::zero(); ff_taps],
            fb_coeffs: vec![Complex::zero(); fb_taps],
            fb_history: vec![0; fb_taps],
            cma_r2,
            total_symbols: 0,
            error_power_avg: 1.0,
            cma_cost_avg: 1.0,
            telemetry_enabled: false,
            telemetry: Vec::new(),
        };

        dfe.init_center_tap();
        dfe
    }

    pub fn new_hf(constellation: ConstellationType) -> Self {
        Self::new(DFEConfig::hf_skywave(), constellation)
    }

    fn compute_cma_r2(constellation: ConstellationType) -> f64 {
        let n = constellation.order();
        let mut sum_sq = 0.0;
        let mut sum_fourth = 0.0;

        for sym in 0..n {
            let (i, q) = constellation.symbol_to_iq(sym as u8);
            let mag_sq = i * i + q * q;
            sum_sq += mag_sq;
            sum_fourth += mag_sq * mag_sq;
        }

        (sum_fourth / n as f64) / (sum_sq / n as f64)
    }

    fn init_center_tap(&mut self) {
        let center = self.ff_coeffs.len() / 2;
        self.ff_coeffs[center] = Complex::new(1.0, 0.0);
    }

    pub fn reset(&mut self) {
        for c in &mut self.ff_coeffs { *c = Complex::zero(); }
        for c in &mut self.fb_coeffs { *c = Complex::zero(); }
        for h in &mut self.ff_history { *h = Complex::zero(); }
        for s in &mut self.fb_history { *s = 0; }
        self.init_center_tap();
        self.mode = EqMode::CMA;
        self.total_symbols = 0;
        self.error_power_avg = 1.0;
        self.cma_cost_avg = 1.0;
        self.telemetry.clear();
    }

    pub fn enable_telemetry(&mut self) {
        self.telemetry_enabled = true;
        self.telemetry.clear();
    }

    pub fn take_telemetry(&mut self) -> Vec<DfeTelemetry> {
        core::mem::take(&mut self.telemetry)
    }

    fn tap_energy(&self) -> f64 {
        self.ff_coeffs.iter().map(|c| c.mag_sq()).sum()
    }

    pub fn set_constellation(&mut self, constellation: ConstellationType) {
        self.constellation = constellation;
        self.cma_r2 = Self::compute_cma_r2(constellation);
    }

    pub fn mode(&self) -> EqMode {
        self.mode
    }

    pub fn set_dd_mode(&mut self) {
        self.mode = EqMode::DD;
    }

    /// Process one I/Q sample - automatically selects CMA or DD
    pub fn equalize(&mut self, i: f64, q: f64) -> u8 {
        let input = Complex::new(i, q);

        self.ff_history.rotate_right(1);
        self.ff_history[0] = input;

        let ff_out = self.compute_ff_output();
        let fb_out = self.compute_fb_output();
        let eq_out = ff_out - fb_out;

        let decision = self.constellation.iq_to_symbol(eq_out.re, eq_out.im);
        let (dec_i, dec_q) = self.constellation.symbol_to_iq(decision);
        let reference = Complex::new(dec_i, dec_q);

        if input.mag_sq() > self.config.update_threshold {
            match self.mode {
                EqMode::CMA => self.update_cma(eq_out),
                EqMode::DD => {
                    let error = eq_out - reference;
                    self.update_dd(error);
                }
            }
        }

        self.fb_history.rotate_right(1);
        self.fb_history[0] = decision;

        self.total_symbols += 1;
        let dd_error = eq_out - reference;
        self.error_power_avg = 0.99 * self.error_power_avg + 0.01 * dd_error.mag_sq();

        if self.mode == EqMode::CMA && self.should_switch_to_dd() {
            self.mode = EqMode::DD;
        }
        if self.mode == EqMode::DD && self.should_fallback_to_cma() {
            self.mode = EqMode::CMA;
            self.cma_cost_avg = 1.0;
        }

        decision
    }

    /// Process one I/Q sample and return equalized I/Q (soft output)
    pub fn equalize_iq(&mut self, i: f64, q: f64) -> (f64, f64) {
        let input = Complex::new(i, q);

        self.ff_history.rotate_right(1);
        self.ff_history[0] = input;

        let ff_out = self.compute_ff_output();
        let fb_out = self.compute_fb_output();
        let eq_out = ff_out - fb_out;

        let decision = self.constellation.iq_to_symbol(eq_out.re, eq_out.im);
        let (dec_i, dec_q) = self.constellation.symbol_to_iq(decision);
        let reference = Complex::new(dec_i, dec_q);

        if input.mag_sq() > self.config.update_threshold {
            match self.mode {
                EqMode::CMA => self.update_cma(eq_out),
                EqMode::DD => {
                    let error = eq_out - reference;
                    self.update_dd(error);
                }
            }
        }

        self.fb_history.rotate_right(1);
        self.fb_history[0] = decision;

        self.total_symbols += 1;
        let dd_error = eq_out - reference;
        self.error_power_avg = 0.99 * self.error_power_avg + 0.01 * dd_error.mag_sq();

        if self.mode == EqMode::CMA && self.should_switch_to_dd() {
            self.mode = EqMode::DD;
        }
        if self.mode == EqMode::DD && self.should_fallback_to_cma() {
            self.mode = EqMode::CMA;
            self.cma_cost_avg = 1.0;
        }

        if self.telemetry_enabled {
            self.telemetry.push(DfeTelemetry {
                symbol_idx: self.total_symbols,
                mse: self.error_power_avg,
                cma_cost: self.cma_cost_avg,
                out_mag_sq: eq_out.mag_sq(),
                in_mag_sq: input.mag_sq(),
                tap_energy: self.tap_energy(),
                mode: if self.mode == EqMode::CMA { 0 } else { 1 },
            });
        }

        (eq_out.re, eq_out.im)
    }

    /// Train on known symbol (supervised mode - fastest convergence)
    pub fn train(&mut self, i: f64, q: f64, known_symbol: u8) -> u8 {
        let input = Complex::new(i, q);

        self.ff_history.rotate_right(1);
        self.ff_history[0] = input;

        let ff_out = self.compute_ff_output();
        let fb_out = self.compute_fb_output();
        let eq_out = ff_out - fb_out;

        let (ref_i, ref_q) = self.constellation.symbol_to_iq(known_symbol);
        let reference = Complex::new(ref_i, ref_q);
        let error = eq_out - reference;

        self.update_dd_scaled(error, 2.0);

        self.fb_history.rotate_right(1);
        self.fb_history[0] = known_symbol;

        self.total_symbols += 1;
        self.error_power_avg = 0.99 * self.error_power_avg + 0.01 * error.mag_sq();
        self.mode = EqMode::DD;

        self.constellation.iq_to_symbol(eq_out.re, eq_out.im)
    }

    /// CMA update: minimize (|y|² - R²)²
    fn update_cma(&mut self, eq_out: Complex) {
        let mag_sq = eq_out.mag_sq();
        let cma_error = mag_sq - self.cma_r2;

        self.cma_cost_avg = 0.99 * self.cma_cost_avg + 0.01 * cma_error * cma_error;

        let mu = self.config.mu_cma;
        let leakage = self.config.leakage;
        let scale = 2.0 * cma_error;

        for (c, h) in self.ff_coeffs.iter_mut().zip(&self.ff_history) {
            let update = eq_out * h.conj() * (scale * mu);
            *c = *c * leakage - update;
        }
    }

    fn update_dd(&mut self, error: Complex) {
        self.update_dd_scaled(error, 1.0);
    }

    fn update_dd_scaled(&mut self, error: Complex, mu_scale: f64) {
        let mu = self.config.mu * mu_scale;
        let leakage = self.config.leakage;

        for (c, h) in self.ff_coeffs.iter_mut().zip(&self.ff_history) {
            let update = error * h.conj() * mu;
            *c = *c * leakage - update;
        }

        for (c, &sym) in self.fb_coeffs.iter_mut().zip(&self.fb_history) {
            let (i, q) = self.constellation.symbol_to_iq(sym);
            let past = Complex::new(i, q);
            let update = error * past.conj() * mu;
            *c = *c * leakage + update;
        }
    }

    fn should_switch_to_dd(&self) -> bool {
        if self.total_symbols < self.config.cma_min_symbols as u64 {
            return false;
        }
        self.cma_cost_avg < self.config.cma_to_dd_threshold
            && self.error_power_avg < 0.5
    }

    fn should_fallback_to_cma(&self) -> bool {
        self.error_power_avg > 0.15
    }

    pub fn mse(&self) -> f64 {
        self.error_power_avg
    }

    pub fn cma_cost(&self) -> f64 {
        self.cma_cost_avg
    }

    pub fn symbols_processed(&self) -> u64 {
        self.total_symbols
    }

    #[inline]
    fn compute_ff_output(&self) -> Complex {
        self.ff_coeffs.iter()
            .zip(&self.ff_history)
            .map(|(c, h)| *c * *h)
            .sum()
    }

    #[inline]
    fn compute_fb_output(&self) -> Complex {
        self.fb_coeffs.iter()
            .zip(&self.fb_history)
            .map(|(c, &sym)| {
                let (i, q) = self.constellation.symbol_to_iq(sym);
                *c * Complex::new(i, q)
            })
            .sum()
    }
}

// =============================================================================
// Tests
// =============================================================================
//
// Inline test for DFE multipath behavior. This test drives the DFE directly
// with synthesized ISI (Complex channel taps), exercising internal CMA/DD
// state transitions. Pure-DFE integration test (no modulator/demodulator).

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dfe_with_multipath() {
        let config = DFEConfig {
            ff_taps: 11,
            fb_taps: 5,
            mu: 0.05,
            mu_cma: 0.005,
            leakage: 0.999,
            update_threshold: 0.01,
            cma_to_dd_threshold: 0.3,
            cma_min_symbols: 50,
        };
        let mut dfe = DFE::new(config, ConstellationType::Psk8);

        // Simple ISI channel: h = [1.0, 0.3+j0.2]
        let h0 = Complex::new(1.0, 0.0);
        let h1 = Complex::new(0.3, 0.2);

        let probe: Vec<u8> = vec![
            0, 4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 4, 0, 0, 4,
            4, 4, 0, 4, 0, 0, 0, 4, 0, 4, 0, 4, 4, 0, 4, 0,
        ];

        // Extended training over 100 symbols.
        let training: Vec<u8> = probe.iter().cloned().cycle().take(100).collect();
        let mut prev_iq = Complex::zero();

        for &sym in &training {
            let (i, q) = ConstellationType::Psk8.symbol_to_iq(sym);
            let current = Complex::new(i, q);
            let rx = h0 * current + h1 * prev_iq;
            dfe.train(rx.re, rx.im, sym);
            prev_iq = current;
        }

        // Then test on the probe pattern.
        let mut results = Vec::new();
        for &sym in &probe {
            let (i, q) = ConstellationType::Psk8.symbol_to_iq(sym);
            let current = Complex::new(i, q);
            let rx = h0 * current + h1 * prev_iq;
            results.push(dfe.equalize(rx.re, rx.im));
            prev_iq = current;
        }

        let bpsk_correct = results.iter().zip(&probe)
            .filter(|(&r, &s)| (r < 4) == (s < 4))
            .count();

        assert!(bpsk_correct >= 28, "Expected at least 28/32 BPSK correct, got {}", bpsk_correct);
    }
}
