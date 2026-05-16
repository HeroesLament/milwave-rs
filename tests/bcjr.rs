//! Integration tests for BcjrDecoder (milwave-rs).
//!
//! Tests the public BCJR API. Internal helpers (log_add, trellis construction,
//! interleaver permutation) are tested inline in src/turbo.rs.

use milwave_rs::turbo::BcjrDecoder;

#[test]
fn test_bcjr_trivial() {
    let bcjr = BcjrDecoder::new();

    let n = 10;
    let channel_llrs: Vec<(f64, f64)> = vec![(-5.0, -5.0); n];
    let apriori: Vec<f64> = vec![0.0; n];

    let (extrinsic, hard_bits) = bcjr.decode(&channel_llrs, &apriori);

    assert_eq!(hard_bits.len(), n);
    for &bit in &hard_bits {
        assert_eq!(bit, 0, "Expected all-zero decode");
    }
    assert_eq!(extrinsic.len(), n);
}
