//! milwave_rs - MIL-STD waveform engine for HF modems.
//!
//! Composes wavecore_rs primitives (constellations, pulse shapes,
//! carriers, timing, Complex) into protocol-specific modems for the
//! MIL-STD-188-110D serial-tone data waveform, MIL-STD-188-141
//! Automatic Link Establishment (ALE), and related STANAG waveforms.
//!
//! ## Layering
//!
//! ```text
//! wavecore_rs       <- DSP primitives (separate crate)
//!     ^
//! milwave_rs        <- this crate. MIL-STD waveforms.
//!     ^
//! per-platform NIF wrappers (desktop cdylib, Mob staticlib)
//!     ^
//! Elixir / Mob / Desktop applications
//! ```
//!
//! ## Public API (planned, in flight)
//!
//! - UnifiedModulator / UnifiedDemodulator - the main API. Supports
//!   188-110D mini-probe mode-switching (mixed QPSK probe + data-mode
//!   constellation symbols within a frame).
//! - DFE - decision-feedback equalizer with HF skywave / ground-wave
//!   default configurations.
//! - WalshCorrelator - 188-110D Walsh correlator for preamble sync.
//! - BcjrDecoder / turbo_decode - turbo product code FEC for the
//!   higher-rate 188-110D modes.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "alloc")]
extern crate alloc;

// Modules will land as code is migrated from minutemodem/phy_modem/src/modem/:
//   pub mod unified;
//   pub mod equalizers;
//   pub mod walsh;
//   pub mod turbo;

pub mod walsh;
pub mod turbo;
pub mod modulator;
pub mod demodulator;
pub mod unified;
