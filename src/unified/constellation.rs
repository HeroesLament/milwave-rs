//! ConstellationType enum + IQ tables for the 188-110D waveforms.
//!
//! Supports BPSK, QPSK, PSK8, QAM16, QAM32, QAM64. Provides:
//! - `symbol_to_iq` — map a symbol index to its (I, Q) constellation point.
//! - `iq_to_symbol` — slicer (decision) from a soft (I, Q) point back to a symbol.
//!
//! The QAM16/32/64 constellation tables follow MIL-STD-188-110D Tables D-VII,
//! D-VIII, and D-IX. QAM32 and QAM64 contain intentional duplicates per spec.

use core::f64::consts::PI;

#[allow(unused_imports)]
use num_traits::Float;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstellationType {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam32,
    Qam64,
}

impl ConstellationType {
    pub fn order(&self) -> usize {
        match self {
            Self::Bpsk => 2,
            Self::Qpsk => 4,
            Self::Psk8 => 8,
            Self::Qam16 => 16,
            Self::Qam32 => 32,
            Self::Qam64 => 64,
        }
    }

    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Self::Bpsk => 1,
            Self::Qpsk => 2,
            Self::Psk8 => 3,
            Self::Qam16 => 4,
            Self::Qam32 => 5,
            Self::Qam64 => 6,
        }
    }

    #[inline]
    pub fn symbol_to_iq(&self, sym: u8) -> (f64, f64) {
        match self {
            Self::Bpsk => bpsk_symbol_to_iq(sym),
            Self::Qpsk => qpsk_symbol_to_iq(sym),
            Self::Psk8 => psk8_symbol_to_iq(sym),
            Self::Qam16 => qam16_symbol_to_iq(sym),
            Self::Qam32 => qam32_symbol_to_iq(sym),
            Self::Qam64 => qam64_symbol_to_iq(sym),
        }
    }

    #[inline]
    pub fn iq_to_symbol(&self, i: f64, q: f64) -> u8 {
        match self {
            Self::Bpsk => bpsk_iq_to_symbol(i, q),
            Self::Qpsk => qpsk_iq_to_symbol(i, q),
            Self::Psk8 => psk8_iq_to_symbol(i, q),
            Self::Qam16 => qam16_iq_to_symbol(i, q),
            Self::Qam32 => qam32_iq_to_symbol(i, q),
            Self::Qam64 => qam64_iq_to_symbol(i, q),
        }
    }
}

#[inline]
fn bpsk_symbol_to_iq(sym: u8) -> (f64, f64) {
    if sym & 1 == 0 { (1.0, 0.0) } else { (-1.0, 0.0) }
}

#[inline]
fn bpsk_iq_to_symbol(i: f64, _q: f64) -> u8 {
    if i >= 0.0 { 0 } else { 1 }
}

#[inline]
fn qpsk_symbol_to_iq(sym: u8) -> (f64, f64) {
    const QPSK: [(f64, f64); 4] = [
        ( 0.7071067811865476,  0.7071067811865476),
        (-0.7071067811865476,  0.7071067811865476),
        (-0.7071067811865476, -0.7071067811865476),
        ( 0.7071067811865476, -0.7071067811865476),
    ];
    QPSK[(sym & 0x03) as usize]
}

#[inline]
fn qpsk_iq_to_symbol(i: f64, q: f64) -> u8 {
    match (i >= 0.0, q >= 0.0) {
        (true, true) => 0,
        (false, true) => 1,
        (false, false) => 2,
        (true, false) => 3,
    }
}

#[inline]
fn psk8_symbol_to_iq(sym: u8) -> (f64, f64) {
    let phase = (sym & 0x07) as f64 * PI / 4.0;
    (phase.cos(), phase.sin())
}

#[inline]
fn psk8_iq_to_symbol(i: f64, q: f64) -> u8 {
    let angle = q.atan2(i);
    let angle_pos = if angle < 0.0 { angle + 2.0 * PI } else { angle };
    let symbol = ((angle_pos + PI / 8.0) / (PI / 4.0)).floor() as u8;
    symbol & 0x07
}

/// MIL-STD-188-110D Table D-VII 16-QAM constellation
const QAM16_CONSTELLATION: [(f64, f64); 16] = [
    ( 0.866025,  0.500000),
    ( 1.000000,  0.000000),
    ( 0.500000,  0.866025),
    ( 0.258819,  0.258819),
    (-0.500000,  0.866025),
    ( 0.000000,  1.000000),
    (-0.866025,  0.500000),
    (-0.258819,  0.258819),
    ( 0.500000, -0.866025),
    ( 0.000000, -1.000000),
    ( 0.866025, -0.500000),
    ( 0.258819, -0.258819),
    (-0.866025, -0.500000),
    (-0.500000, -0.866025),
    (-1.000000,  0.000000),
    (-0.258819, -0.258819),
];

#[inline]
fn qam16_symbol_to_iq(sym: u8) -> (f64, f64) {
    QAM16_CONSTELLATION[(sym & 0x0F) as usize]
}

#[inline]
fn qam16_iq_to_symbol(i: f64, q: f64) -> u8 {
    let mut best_sym = 0u8;
    let mut best_dist = f64::MAX;
    for (sym, &(ci, cq)) in QAM16_CONSTELLATION.iter().enumerate() {
        let di = i - ci;
        let dq = q - cq;
        let dist = di * di + dq * dq;
        if dist < best_dist {
            best_dist = dist;
            best_sym = sym as u8;
        }
    }
    best_sym
}

/// MIL-STD-188-110D Table D-VIII 32-QAM constellation
const QAM32_CONSTELLATION: [(f64, f64); 32] = [
    ( 0.866380,  0.499386), ( 0.984849,  0.173415), ( 0.520246,  0.853972), ( 0.520246,  0.173415),
    (-0.173772,  0.984770), ( 0.173416,  0.984770), (-0.173772,  0.520089), ( 0.173416,  0.520089),
    ( 0.520246, -0.853972), ( 0.984849, -0.173415), ( 0.866380, -0.499386), ( 0.520246, -0.173415),
    (-0.173772, -0.520089), ( 0.173416, -0.520089), (-0.173772, -0.984770), ( 0.173416, -0.984770),
    (-0.520603,  0.853972), (-0.984849,  0.173415), (-0.866380,  0.499386), (-0.520603,  0.173415),
    (-0.866380, -0.499386), (-0.984849, -0.173415), (-0.520603, -0.853972), (-0.520603, -0.173415),
    // Duplicates 24..32 per spec
    ( 0.866380,  0.499386), ( 0.984849,  0.173415), ( 0.520246,  0.853972), ( 0.520246,  0.173415),
    (-0.173772,  0.984770), ( 0.173416,  0.984770), (-0.173772,  0.520089), ( 0.173416,  0.520089),
];

#[inline]
fn qam32_symbol_to_iq(sym: u8) -> (f64, f64) {
    QAM32_CONSTELLATION[(sym & 0x1F) as usize]
}

#[inline]
fn qam32_iq_to_symbol(i: f64, q: f64) -> u8 {
    let mut best_sym = 0u8;
    let mut best_dist = f64::MAX;
    for sym in 0..24u8 {
        let (ci, cq) = QAM32_CONSTELLATION[sym as usize];
        let di = i - ci;
        let dq = q - cq;
        let dist = di * di + dq * dq;
        if dist < best_dist {
            best_dist = dist;
            best_sym = sym;
        }
    }
    best_sym
}

/// MIL-STD-188-110D Table D-IX 64-QAM constellation
const QAM64_CONSTELLATION: [(f64, f64); 64] = [
    ( 1.000000,  0.000000), ( 0.822878,  0.568218), ( 0.821137,  0.152996), ( 0.932897,  0.360142),
    ( 0.000000,  1.000000), ( 0.568218,  0.822878), ( 0.152996,  0.821137), ( 0.360142,  0.932897),
    ( 0.000000, -1.000000), ( 0.568218, -0.822878), ( 0.152996, -0.821137), ( 0.360142, -0.932897),
    ( 0.822878, -0.568218), ( 1.000000,  0.000000), ( 0.821137, -0.152996), ( 0.932897, -0.360142),
    (-1.000000,  0.000000), (-0.822878,  0.568218), (-0.821137,  0.152996), (-0.932897,  0.360142),
    (-0.822878, -0.568218), (-1.000000,  0.000000), (-0.821137, -0.152996), (-0.932897, -0.360142),
    ( 0.000000,  1.000000), (-0.568218,  0.822878), (-0.152996,  0.821137), (-0.360142,  0.932897),
    ( 0.000000, -1.000000), (-0.568218, -0.822878), (-0.152996, -0.821137), (-0.360142, -0.932897),
    ( 0.821137,  0.152996), ( 0.570088,  0.414693), ( 0.466049,  0.000000), ( 0.570088,  0.152996),
    ( 0.152996,  0.821137), ( 0.414693,  0.570088), ( 0.000000,  0.466049), ( 0.152996,  0.570088),
    ( 0.152996, -0.821137), ( 0.414693, -0.570088), ( 0.000000, -0.466049), ( 0.152996, -0.570088),
    ( 0.570088, -0.414693), ( 0.821137, -0.152996), ( 0.466049,  0.000000), ( 0.570088, -0.152996),
    (-0.821137,  0.152996), (-0.570088,  0.414693), (-0.466049,  0.000000), (-0.570088,  0.152996),
    (-0.570088, -0.414693), (-0.821137, -0.152996), (-0.466049,  0.000000), (-0.570088, -0.152996),
    (-0.152996,  0.821137), (-0.414693,  0.570088), ( 0.000000,  0.466049), (-0.152996,  0.570088),
    (-0.152996, -0.821137), (-0.414693, -0.570088), ( 0.000000, -0.466049), (-0.152996, -0.570088),
];

#[inline]
fn qam64_symbol_to_iq(sym: u8) -> (f64, f64) {
    QAM64_CONSTELLATION[(sym & 0x3F) as usize]
}

#[inline]
fn qam64_iq_to_symbol(i: f64, q: f64) -> u8 {
    let mut best_sym = 0u8;
    let mut best_dist = f64::MAX;
    for (sym, &(ci, cq)) in QAM64_CONSTELLATION.iter().enumerate() {
        let di = i - ci;
        let dq = q - cq;
        let dist = di * di + dq * dq;
        if dist < best_dist {
            best_dist = dist;
            best_sym = sym as u8;
        }
    }
    best_sym
}
