//! Unified Modulator/Demodulator for 188-110D waveforms.
//!
//! One concrete modulator/demodulator pair with constellation as a runtime
//! enum. Filter state is preserved across constellation switches, which is
//! essential for 110D where PSK8 mini-probes are interleaved with QAM data
//! mid-stream.
//!
//! ## Module structure
//!
//! - [`constellation`] — `ConstellationType` enum and the BPSK / QPSK / PSK8 /
//!   QAM16 / QAM32 / QAM64 IQ tables and slicers.
//! - [`rrc`] — Root-Raised-Cosine coefficient generator (`pub(crate)`,
//!   used by the modulator and demodulator).
//! - [`equalizer`] — `EqMode`, `DFEConfig`, `DFE`, `DfeTelemetry`. Handles HF
//!   multipath via CMA blind acquisition then DD-LMS tracking.
//! - [`modulator`] — `UnifiedModulator`: RRC pulse shaping + NCO carrier +
//!   per-symbol or per-frame constellation switching.
//! - [`demodulator`] — `UnifiedDemodulator`: timing recovery + 8th-power PLL
//!   + matched filter + optional DFE wiring. `PllTelemetry` lives here.
//!
//! ## Carrier Tracking (PLL)
//!
//! The demodulator includes an 8th-power PLL for carrier tracking. This is
//! essential for channels with phase rotation (fading, frequency offset).
//!
//! The 8th-power loop removes the M-PSK modulation before tracking:
//! - For any PSK symbol at phase φ: z^8 collapses to real (no phase)
//! - Phase error θ appears as: z^8 = A^8·exp(j·8θ)
//! - Extract phase of z^8, divide by 8 to get θ
//!
//! This avoids the 180° ambiguity of decision-directed loops, at the cost
//! of 8-fold (45°) ambiguity. The ALE receiver resolves this ambiguity by
//! correlating with the known capture probe sequence.
//!
//! ## Adaptive Equalization (DFE)
//!
//! For HF channels with multipath (2-4ms delay spread), the demodulator
//! optionally includes a DFE — see the [`equalizer`] submodule.

pub mod constellation;
pub mod rrc;
pub mod equalizer;
pub mod modulator;
pub mod demodulator;

// Public API surface — re-exported for ergonomic `milwave_rs::unified::X` access.
pub use constellation::ConstellationType;
pub use equalizer::{EqMode, DFEConfig, DFE, DfeTelemetry};
pub use modulator::UnifiedModulator;
pub use demodulator::{UnifiedDemodulator, PllTelemetry};
