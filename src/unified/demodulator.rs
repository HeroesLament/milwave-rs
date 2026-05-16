//! UnifiedDemodulator — the RX-side state machine for 188-110D.
//!
//! Combines:
//! - **Timing recovery** — finds optimal symbol-center sample offset by max
//!   energy across `sps` candidate phases (first ~500 samples).
//! - **8th-power PLL** — carrier recovery for M-PSK without modulation aid.
//!   Removes the modulation by raising to the 8th power, then tracks the
//!   resulting phase via a 2nd-order Type-2 loop filter. Block-averaged
//!   estimator (default 8 symbols) for √N noise rejection.
//! - **RRC matched filter** — same coefficients as the modulator's TX filter.
//! - **Optional DFE** — see the `equalizer` submodule.
//!
//! Key invariants:
//! - The PLL updates _inside_ the sample loop, so corrections apply to
//!   subsequent samples in the same call. Required for tracking phase drift
//!   over long frames (e.g. 2.8s ALE Deep WALE with 0.12 Hz Doppler).
//! - `compute_phase_error` is `pub(crate)` because the phase-detector tests
//!   in this file exercise it directly.

use core::f64::consts::PI;

#[cfg(feature = "std")]
use std::vec::Vec;

#[cfg(all(not(feature = "std"), feature = "alloc"))]
use alloc::vec::Vec;

#[allow(unused_imports)]
use num_traits::Float;

use super::constellation::ConstellationType;
use super::rrc::{generate_rrc_coeffs, RRC_SPAN};
use super::equalizer::{DFE, DFEConfig, DfeTelemetry, EqMode};

/// Per-symbol PLL telemetry snapshot.
#[derive(Clone, Debug)]
pub struct PllTelemetry {
    pub symbol_idx: usize,
    pub phase: f64,
    pub freq: f64,
    pub integrator: f64,
    pub phase_error: f64,
    pub mag_sq: f64,
    pub lock_detect: f64,
}

pub struct UnifiedDemodulator {
    constellation: ConstellationType,
    sample_rate: u32,
    symbol_rate: u32,
    carrier_freq: f64,
    sps: usize,

    rrc_coeffs: Vec<f64>,
    i_history: Vec<f64>,
    q_history: Vec<f64>,

    pll_phase: f64,
    pll_freq: f64,
    pll_integrator: f64,
    pll_alpha: f64,
    pll_beta: f64,
    carrier_phase_inc: f64,

    phase_block_size: usize,
    phase_accum_re: f64,
    phase_accum_im: f64,
    phase_accum_count: usize,

    lock_detect: f64,
    lock_detect_alpha: f64,

    telemetry_enabled: bool,
    telemetry: Vec<PllTelemetry>,
    last_phase_error: f64,

    timing_phase: usize,
    timing_acquired: bool,

    equalizer: Option<DFE>,

    training_mode: bool,
    training_symbols: Vec<u8>,
    training_index: usize,
}

impl UnifiedDemodulator {
    pub fn new(
        constellation: ConstellationType,
        sample_rate: u32,
        symbol_rate: u32,
        carrier_freq: f64,
    ) -> Self {
        let sps = (sample_rate / symbol_rate) as usize;
        let rrc_coeffs = generate_rrc_coeffs(sps);
        let filter_len = rrc_coeffs.len();

        // PLL design: 2nd-order Type-2 loop, BL ≈ 3 Hz at 2400 baud
        let loop_bw_hz = 3.0;
        let wn = 2.0 * PI * loop_bw_hz;
        let ts = 1.0 / symbol_rate as f64;
        let zeta = 0.707;

        let pll_alpha = 2.0 * zeta * wn * ts;
        let pll_beta = wn * wn * ts * ts;
        let carrier_phase_inc = 2.0 * PI * carrier_freq / sample_rate as f64;

        Self {
            constellation,
            sample_rate,
            symbol_rate,
            carrier_freq,
            sps,
            rrc_coeffs,
            i_history: vec![0.0; filter_len],
            q_history: vec![0.0; filter_len],
            pll_phase: 0.0,
            pll_freq: 0.0,
            pll_integrator: 0.0,
            pll_alpha,
            pll_beta,
            carrier_phase_inc,
            phase_block_size: 8,
            phase_accum_re: 0.0,
            phase_accum_im: 0.0,
            phase_accum_count: 0,
            lock_detect: 0.0,
            lock_detect_alpha: 0.02,
            telemetry_enabled: false,
            telemetry: Vec::new(),
            last_phase_error: 0.0,
            timing_phase: 0,
            timing_acquired: false,
            equalizer: None,
            training_mode: false,
            training_symbols: Vec::new(),
            training_index: 0,
        }
    }

    pub fn with_equalizer(
        constellation: ConstellationType,
        sample_rate: u32,
        symbol_rate: u32,
        carrier_freq: f64,
        dfe_config: DFEConfig,
    ) -> Self {
        let mut demod = Self::new(constellation, sample_rate, symbol_rate, carrier_freq);
        demod.equalizer = Some(DFE::new(dfe_config, constellation));
        demod
    }

    pub fn with_hf_equalizer(
        constellation: ConstellationType,
        sample_rate: u32,
        symbol_rate: u32,
        carrier_freq: f64,
    ) -> Self {
        Self::with_equalizer(
            constellation, sample_rate, symbol_rate, carrier_freq,
            DFEConfig::hf_skywave(),
        )
    }

    pub fn enable_equalizer(&mut self, config: DFEConfig) {
        self.equalizer = Some(DFE::new(config, self.constellation));
    }

    pub fn disable_equalizer(&mut self) {
        self.equalizer = None;
    }

    pub fn has_equalizer(&self) -> bool {
        self.equalizer.is_some()
    }

    pub fn set_training_symbols(&mut self, symbols: Vec<u8>) {
        self.training_symbols = symbols;
        self.training_index = 0;
        self.training_mode = true;
    }

    pub fn reset_equalizer(&mut self) {
        if let Some(eq) = &mut self.equalizer {
            eq.reset();
        }
        self.training_index = 0;
        self.training_mode = false;
    }

    pub fn equalizer_mse(&self) -> Option<f64> {
        self.equalizer.as_ref().map(|eq| eq.mse())
    }

    pub fn equalizer_mode(&self) -> Option<EqMode> {
        self.equalizer.as_ref().map(|eq| eq.mode())
    }

    pub fn equalizer_cma_cost(&self) -> Option<f64> {
        self.equalizer.as_ref().map(|eq| eq.cma_cost())
    }

    pub fn set_constellation(&mut self, constellation: ConstellationType) {
        self.constellation = constellation;
        if let Some(eq) = &mut self.equalizer {
            eq.set_constellation(constellation);
        }
    }

    pub fn constellation(&self) -> ConstellationType {
        self.constellation
    }

    /// 8th-power phase error (blind, no decision needed).
    #[inline]
    pub(crate) fn compute_phase_error(&self, i_rx: f64, q_rx: f64) -> f64 {
        let mut real = i_rx;
        let mut imag = q_rx;

        for _ in 0..3 {
            let new_real = real * real - imag * imag;
            let new_imag = 2.0 * real * imag;
            real = new_real;
            imag = new_imag;
        }

        imag.atan2(real) / 8.0
    }

    /// Decision-directed phase error using a known reference symbol.
    /// More accurate than 8th-power because it doesn't amplify noise.
    #[inline]
    #[allow(dead_code)]
    fn compute_phase_error_dd(&self, i_rx: f64, q_rx: f64, known_symbol: u8) -> f64 {
        let (i_exp, q_exp) = self.constellation.symbol_to_iq(known_symbol);
        let cross = i_rx * q_exp - q_rx * i_exp;
        let dot = i_rx * i_exp + q_rx * q_exp;
        cross.atan2(dot)
    }

    /// Demodulate to I/Q pairs with live PLL updates.
    pub fn demodulate_iq(&mut self, samples: &[i16]) -> Vec<(f64, f64)> {
        if samples.is_empty() {
            return Vec::new();
        }

        let skip_samples = 2 * RRC_SPAN * self.sps;
        let max_freq_offset = 2.0 * PI * 50.0 / self.sample_rate as f64;

        // Phase 1: Timing acquisition.
        if !self.timing_acquired {
            let acq_samples = samples.len().min(500);
            let mut phase_energy = vec![0.0; self.sps];

            let mut temp_phase = self.pll_phase;
            let mut temp_i_hist = self.i_history.clone();
            let mut temp_q_hist = self.q_history.clone();

            for (i, &sample) in samples[..acq_samples].iter().enumerate() {
                let sample_f = sample as f64 / 32768.0;

                let lo_i = temp_phase.cos();
                let lo_q = -temp_phase.sin();
                let mixed_i = sample_f * lo_i * 2.0;
                let mixed_q = sample_f * lo_q * 2.0;

                temp_i_hist.rotate_left(1);
                temp_q_hist.rotate_left(1);
                let last = temp_i_hist.len() - 1;
                temp_i_hist[last] = mixed_i;
                temp_q_hist[last] = mixed_q;

                let fi = self.apply_filter(&temp_i_hist);
                let fq = self.apply_filter(&temp_q_hist);

                if i >= skip_samples {
                    let phase_idx = i % self.sps;
                    phase_energy[phase_idx] += fi * fi + fq * fq;
                }

                temp_phase += self.carrier_phase_inc;
                while temp_phase > 2.0 * PI { temp_phase -= 2.0 * PI; }
            }

            self.timing_phase = phase_energy
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .map(|(i, _)| i)
                .unwrap_or(0);

            self.timing_acquired = true;
        }

        // Phase 2: Single-pass demodulation with LIVE PLL updates.
        let mut iq_out = Vec::with_capacity(samples.len() / self.sps);
        let mut symbol_count = 0usize;

        for (i, &sample) in samples.iter().enumerate() {
            let sample_f = sample as f64 / 32768.0;

            let lo_i = self.pll_phase.cos();
            let lo_q = -self.pll_phase.sin();
            let mixed_i = sample_f * lo_i * 2.0;
            let mixed_q = sample_f * lo_q * 2.0;

            self.i_history.rotate_left(1);
            self.q_history.rotate_left(1);
            let last = self.i_history.len() - 1;
            self.i_history[last] = mixed_i;
            self.q_history[last] = mixed_q;

            let fi = self.apply_filter(&self.i_history);
            let fq = self.apply_filter(&self.q_history);

            if i % self.sps == self.timing_phase {
                if i >= skip_samples {
                    let mag_sq = fi * fi + fq * fq;

                    if mag_sq > 0.001 {
                        let mut real = fi;
                        let mut imag = fq;
                        for _ in 0..3 {
                            let new_real = real * real - imag * imag;
                            let new_imag = 2.0 * real * imag;
                            real = new_real;
                            imag = new_imag;
                        }
                        let weight = 1.0;
                        self.phase_accum_re += real * weight;
                        self.phase_accum_im += imag * weight;
                    }
                    self.phase_accum_count += 1;

                    if self.phase_accum_count >= self.phase_block_size {
                        let accum_mag = (self.phase_accum_re * self.phase_accum_re
                                       + self.phase_accum_im * self.phase_accum_im).sqrt();

                        if accum_mag > 1e-12 {
                            let phase_error = self.phase_accum_im.atan2(self.phase_accum_re) / 8.0;
                            self.last_phase_error = phase_error;

                            let lock_indicator = (8.0 * phase_error).cos();
                            self.lock_detect = self.lock_detect * (1.0 - self.lock_detect_alpha)
                                             + lock_indicator * self.lock_detect_alpha;

                            // Scale gains by block size to maintain constant loop bandwidth.
                            let n = self.phase_block_size as f64;
                            self.pll_integrator += phase_error * n;
                            self.pll_integrator = self.pll_integrator.clamp(-2.0 * PI, 2.0 * PI);
                            self.pll_freq = (self.pll_alpha * n * phase_error
                                           + self.pll_beta * self.pll_integrator) / self.sps as f64;
                            self.pll_freq = self.pll_freq.clamp(-max_freq_offset, max_freq_offset);
                        }

                        self.phase_accum_re = 0.0;
                        self.phase_accum_im = 0.0;
                        self.phase_accum_count = 0;
                    }

                    if self.telemetry_enabled {
                        self.telemetry.push(PllTelemetry {
                            symbol_idx: symbol_count,
                            phase: self.pll_phase,
                            freq: self.pll_freq * self.sps as f64,
                            integrator: self.pll_integrator,
                            phase_error: self.last_phase_error,
                            mag_sq,
                            lock_detect: self.lock_detect,
                        });
                    }

                    iq_out.push((fi, fq));
                    symbol_count += 1;
                } else {
                    iq_out.push((fi, fq));
                }
            }

            self.pll_phase += self.carrier_phase_inc + self.pll_freq;
            while self.pll_phase > 2.0 * PI { self.pll_phase -= 2.0 * PI; }
            while self.pll_phase < 0.0 { self.pll_phase += 2.0 * PI; }
        }

        iq_out
    }

    /// Demodulate to symbols (runs DFE if present, otherwise direct slicer).
    pub fn demodulate(&mut self, samples: &[i16]) -> Vec<u8> {
        let iq = self.demodulate_iq(samples);

        match &mut self.equalizer {
            Some(eq) => {
                let mut results = Vec::with_capacity(iq.len());

                for (i, q) in iq {
                    let symbol = if self.training_mode && self.training_index < self.training_symbols.len() {
                        let known = self.training_symbols[self.training_index];
                        self.training_index += 1;

                        if self.training_index >= self.training_symbols.len() {
                            self.training_mode = false;
                        }

                        eq.train(i, q, known)
                    } else {
                        eq.equalize(i, q)
                    };

                    results.push(symbol);
                }

                results
            }
            None => {
                iq.iter()
                    .map(|&(i, q)| self.constellation.iq_to_symbol(i, q))
                    .collect()
            }
        }
    }

    /// Demodulate to equalized I/Q (soft output for soft Walsh decoder).
    pub fn demodulate_eq_iq(&mut self, samples: &[i16]) -> Vec<(f64, f64)> {
        let iq = self.demodulate_iq(samples);

        match &mut self.equalizer {
            Some(eq) => {
                iq.into_iter()
                    .map(|(i, q)| eq.equalize_iq(i, q))
                    .collect()
            }
            None => iq
        }
    }

    pub fn enable_dfe_telemetry(&mut self) {
        if let Some(eq) = &mut self.equalizer {
            eq.enable_telemetry();
        }
    }

    pub fn take_dfe_telemetry(&mut self) -> Vec<DfeTelemetry> {
        if let Some(eq) = &mut self.equalizer {
            eq.take_telemetry()
        } else {
            Vec::new()
        }
    }

    pub fn reset(&mut self) {
        for x in &mut self.i_history { *x = 0.0; }
        for x in &mut self.q_history { *x = 0.0; }
        self.pll_phase = 0.0;
        self.pll_freq = 0.0;
        self.pll_integrator = 0.0;
        self.phase_accum_re = 0.0;
        self.phase_accum_im = 0.0;
        self.phase_accum_count = 0;
        self.lock_detect = 0.0;
        self.last_phase_error = 0.0;
        self.telemetry.clear();
        self.timing_phase = 0;
        self.timing_acquired = false;
        self.training_index = 0;
        self.training_mode = false;
        if let Some(eq) = &mut self.equalizer {
            eq.reset();
        }
    }

    pub fn enable_telemetry(&mut self) {
        self.telemetry_enabled = true;
        self.telemetry.clear();
    }

    pub fn disable_telemetry(&mut self) {
        self.telemetry_enabled = false;
    }

    pub fn take_telemetry(&mut self) -> Vec<PllTelemetry> {
        core::mem::take(&mut self.telemetry)
    }

    pub fn lock_detect(&self) -> f64 {
        self.lock_detect
    }

    pub fn set_phase_block_size(&mut self, size: usize) {
        self.phase_block_size = size.max(1);
        self.phase_accum_re = 0.0;
        self.phase_accum_im = 0.0;
        self.phase_accum_count = 0;
    }

    pub fn phase_block_size(&self) -> usize {
        self.phase_block_size
    }

    pub fn reset_pll(&mut self) {
        self.pll_phase = 0.0;
        self.pll_freq = 0.0;
        self.pll_integrator = 0.0;
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
// Inline tests for stateful demodulator behavior: PLL acquisition, integrator
// state, lock under Doppler / random phase wander, integrator drift under
// zero-mean noise, and the pub(crate) compute_phase_error helper.
//
// Also includes test_dfe_clean_channel, which exercises the UnifiedDemodulator's
// integration with the DFE (training-mode-then-equalize). The pure-DFE
// multipath test lives in equalizer.rs.

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::modulator::UnifiedModulator;

    // ------------------------------------------------------------------
    // Demodulator + DFE integration
    // ------------------------------------------------------------------

    #[test]
    fn test_dfe_clean_channel() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::with_hf_equalizer(
            ConstellationType::Psk8, 9600, 2400, 1800.0
        );

        // Capture probe (BPSK: symbols 0 and 4)
        let probe: Vec<u8> = vec![
            0, 4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 4, 0, 0, 4,
            4, 4, 0, 4, 0, 0, 0, 4, 0, 4, 0, 4, 4, 0, 4, 0,
        ];

        let preamble = vec![0u8; 20];
        let mut all_symbols = preamble.clone();
        all_symbols.extend(&probe);
        all_symbols.extend(&probe);

        // Training must account for filter warmup in demodulator output.
        let warmup_symbols = 12;
        let mut training = vec![0u8; warmup_symbols];
        training.extend(&preamble);
        training.extend(&probe);
        demodulator.set_training_symbols(training);

        let mut samples = modulator.modulate(&all_symbols);
        samples.extend(modulator.flush());

        let recovered = demodulator.demodulate(&samples);

        let skip = warmup_symbols + 20 + 32;
        if recovered.len() >= skip + 32 {
            let rx_section = &recovered[skip..skip + 32];

            let corr: i32 = probe.iter().zip(rx_section)
                .map(|(&t, &r)| {
                    let t_sign = if t < 4 { 1 } else { -1 };
                    let r_sign = if r < 4 { 1 } else { -1 };
                    t_sign * r_sign
                })
                .sum();

            assert!(corr.abs() >= 14, "BPSK correlation too low: {}", corr);
        }
    }

    // ------------------------------------------------------------------
    // PLL acquisition + tracking
    // ------------------------------------------------------------------

    #[test]
    fn test_pll_phase_tracking() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let preamble = vec![0u8; 30];
        let data: Vec<u8> = (0..8).cycle().take(50).collect();
        let mut all_symbols = preamble.clone();
        all_symbols.extend(&data);

        let mut samples = modulator.modulate(&all_symbols);
        samples.extend(modulator.flush());

        let recovered = demodulator.demodulate(&samples);

        let skip = 30 + 12;
        if recovered.len() >= skip + 20 {
            let offset = (recovered[skip] + 8 - data[0]) % 8;

            let errors: usize = recovered[skip..skip+20].iter()
                .zip(data.iter())
                .filter(|(&r, &d)| r != (d + offset) % 8)
                .count();

            assert!(errors <= 2, "Too many errors: {} out of 20 (offset={})", errors, offset);
        }
    }

    #[test]
    fn test_pll_with_small_frequency_offset() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let preamble = vec![0u8; 100];
        let data: Vec<u8> = (0..8).cycle().take(200).collect();
        let mut all_symbols = preamble.clone();
        all_symbols.extend(&data);

        let mut samples = modulator.modulate(&all_symbols);
        samples.extend(modulator.flush());

        let freq_offset_hz = 0.12;
        let phase_inc = 2.0 * PI * freq_offset_hz / 9600.0;

        for (i, sample) in samples.iter_mut().enumerate() {
            let phase = phase_inc * i as f64;
            let s = *sample as f64;
            *sample = (s * phase.cos()) as i16;
        }

        let recovered = demodulator.demodulate(&samples);

        let skip = 100 + 12;
        if recovered.len() >= skip + 50 {
            let offset = (recovered[skip] + 8 - data[0]) % 8;

            let errors: usize = recovered[skip..skip+50].iter()
                .zip(data.iter())
                .filter(|(&r, &d)| r != (d + offset) % 8)
                .count();

            assert!(errors <= 5, "Too many errors with 0.12Hz offset: {} out of 50", errors);
        }
    }

    #[test]
    fn test_pll_frequency_estimate_accuracy() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let symbols = vec![0u8; 1000];

        let mut samples = modulator.modulate(&symbols);
        samples.extend(modulator.flush());

        let recovered = demodulator.demodulate(&samples);

        let estimated_hz = demodulator.pll_freq * 9600.0 / (2.0 * PI);

        assert!(estimated_hz.abs() < 0.1,
                "PLL frequency should be near zero on clean channel: {:.3} Hz", estimated_hz);

        let skip = 20;
        if recovered.len() > skip + 100 {
            let errors: usize = recovered[skip..skip+100].iter()
                .filter(|&&s| s != 0)
                .count();
            assert!(errors <= 5, "Too many errors on clean channel: {}", errors);
        }
    }

    #[test]
    fn test_pll_acquisition_with_initial_phase_offset() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let preamble = vec![0u8; 50];
        let data: Vec<u8> = (0..8).cycle().take(80).collect();
        let mut all_symbols = preamble.clone();
        all_symbols.extend(&data);

        let mut samples = modulator.modulate(&all_symbols);
        samples.extend(modulator.flush());

        let recovered = demodulator.demodulate(&samples);

        let skip = 50 + 12;
        if recovered.len() >= skip + 40 {
            let offset = (recovered[skip] + 8 - data[0]) % 8;

            let mut consistent = 0;
            let mut total = 0;
            for i in 0..40 {
                let expected = (data[i] + offset) % 8;
                if recovered[skip + i] == expected {
                    consistent += 1;
                }
                total += 1;
            }

            assert!(consistent >= 35,
                    "Symbols not consistently offset: only {}/{} match with offset {}",
                    consistent, total, offset);
        }
    }

    #[test]
    fn test_pll_phase_coherence_over_long_frame() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let num_symbols = 2400 * 2;
        let symbols = vec![0u8; num_symbols];

        let mut samples = modulator.modulate(&symbols);
        samples.extend(modulator.flush());

        let freq_offset_hz = 0.12;
        let phase_inc = 2.0 * PI * freq_offset_hz / 9600.0;

        for (i, sample) in samples.iter_mut().enumerate() {
            let phase = phase_inc * i as f64;
            *sample = ((*sample as f64) * phase.cos()) as i16;
        }

        let recovered = demodulator.demodulate(&samples);

        let check_points = [100, 500, 1000, 2000, 4000];
        let mut all_good = true;

        for &point in &check_points {
            if recovered.len() > point + 50 {
                let most_common = recovered[point..point+50].iter()
                    .fold([0usize; 8], |mut acc, &s| { acc[s as usize] += 1; acc })
                    .iter()
                    .cloned()
                    .max()
                    .unwrap_or(0);

                if most_common < 40 {
                    all_good = false;
                }
            }
        }

        assert!(all_good, "Phase coherence lost during 2-second frame with 0.12Hz Doppler");
    }

    #[test]
    fn test_pll_integrator_state() {
        let mut demod = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        assert_eq!(demod.pll_freq, 0.0, "Initial pll_freq should be 0");
        assert_eq!(demod.pll_integrator, 0.0, "Initial integrator should be 0");

        demod.pll_freq = 0.001;
        demod.pll_integrator = 0.5;
        demod.reset();

        assert_eq!(demod.pll_freq, 0.0, "pll_freq should be 0 after reset");
        assert_eq!(demod.pll_integrator, 0.0, "integrator should be 0 after reset");
    }

    /// Simple deterministic PRNG for tests (xorshift32)
    struct TestRng(u32);
    impl TestRng {
        fn new(seed: u32) -> Self { Self(seed) }
        fn next(&mut self) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            self.0
        }
        fn next_f64(&mut self) -> f64 {
            (self.next() as f64 / u32::MAX as f64) * 2.0 - 1.0
        }
    }

    #[test]
    fn test_pll_with_random_phase_wandering() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let symbols = vec![0u8; 2000];

        let mut samples = modulator.modulate(&symbols);
        samples.extend(modulator.flush());

        let mut rng = TestRng::new(12345);
        let doppler_hz = 0.12;
        let sample_rate = 9600.0;
        let correlation_samples = (sample_rate / doppler_hz) as usize;

        let mut phase_offset = 0.0f64;
        let mut phase_velocity = 0.0f64;

        for (i, sample) in samples.iter_mut().enumerate() {
            if i % (correlation_samples / 10) == 0 {
                phase_velocity += rng.next_f64() * 0.001;
                phase_velocity = phase_velocity.clamp(-0.01, 0.01);
            }
            phase_offset += phase_velocity;

            let s = *sample as f64;
            let rotated = s * phase_offset.cos();
            *sample = rotated as i16;
        }

        let recovered = demodulator.demodulate(&samples);

        let check_points = [100, 500, 1000, 1500];
        let mut total_consistent = 0;
        let mut total_checked = 0;

        for &point in &check_points {
            if recovered.len() > point + 30 {
                let mut counts = [0usize; 8];
                for &s in &recovered[point..point+30] {
                    counts[s as usize] += 1;
                }
                let most_common = counts.iter().max().unwrap_or(&0);
                total_consistent += most_common;
                total_checked += 30;
            }
        }

        let consistency_rate = total_consistent as f64 / total_checked as f64;

        assert!(consistency_rate > 0.70,
                "PLL failed with random phase wandering: {:.1}% < 70%",
                consistency_rate * 100.0);
    }

    #[test]
    fn test_pll_integrator_drift_with_zero_mean_noise() {
        let mut modulator = UnifiedModulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);
        let mut demodulator = UnifiedDemodulator::new(ConstellationType::Psk8, 9600, 2400, 1800.0);

        let symbols = vec![0u8; 3000];

        let mut samples = modulator.modulate(&symbols);
        samples.extend(modulator.flush());

        let mut rng = TestRng::new(54321);
        let noise_amplitude = 0.05;

        for sample in samples.iter_mut() {
            let noise = rng.next_f64() * noise_amplitude;
            let s = *sample as f64;
            let noisy = s * (1.0 + noise * 0.1);
            *sample = noisy as i16;
        }

        let recovered = demodulator.demodulate(&samples);

        let start_point = 100;
        let end_point = recovered.len().saturating_sub(100);

        if end_point > start_point + 100 {
            let start_mode = recovered[start_point..start_point+50].iter()
                .fold([0usize; 8], |mut acc, &s| { acc[s as usize] += 1; acc })
                .iter().cloned().max().unwrap_or(0);

            let end_mode = recovered[end_point-50..end_point].iter()
                .fold([0usize; 8], |mut acc, &s| { acc[s as usize] += 1; acc })
                .iter().cloned().max().unwrap_or(0);

            assert!(start_mode >= 40, "Poor consistency at start: {}/50", start_mode);
            assert!(end_mode >= 35, "Integrator drifted - poor consistency at end: {}/50", end_mode);

            assert!(demodulator.pll_integrator.abs() < 1.0,
                    "Integrator accumulated too much: {:.3}", demodulator.pll_integrator);
        }
    }

    // ------------------------------------------------------------------
    // Phase detector (pub(crate) compute_phase_error helper)
    // ------------------------------------------------------------------

    #[test]
    fn test_phase_detector_8th_power() {
        let demod = UnifiedDemodulator::new(
            ConstellationType::Psk8, 9600, 2400, 1800.0
        );

        for phase_deg in [0.0, 10.0, 20.0, -10.0, -20.0] {
            let phase_rad: f64 = phase_deg * PI / 180.0;
            let i = phase_rad.cos();
            let q = phase_rad.sin();

            let error = demod.compute_phase_error(i, q);

            let error_deg = error * 180.0 / PI;
            assert!((error_deg - phase_deg).abs() < 5.0,
                "Phase detector error: input {}°, output {}°", phase_deg, error_deg);
        }
    }

    #[test]
    fn test_phase_detector_modulation_removal() {
        let demod = UnifiedDemodulator::new(
            ConstellationType::Psk8, 9600, 2400, 1800.0
        );

        for sym in 0..8u8 {
            let (i, q) = ConstellationType::Psk8.symbol_to_iq(sym);
            let error = demod.compute_phase_error(i, q);

            let error_deg = error.abs() * 180.0 / PI;
            assert!(error_deg < 1.0,
                "Symbol {} gave phase error {}° (should be ~0)", sym, error_deg);
        }
    }
}
