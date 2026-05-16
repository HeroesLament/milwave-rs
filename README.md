# milwave-rs

MIL-STD waveform engine for HF modems.

Composes [wavecore-rs](https://github.com/HeroesLament/wavecore-rs) DSP
primitives into protocol-specific modems for:

- MIL-STD-188-110D serial-tone HF data waveform
- MIL-STD-188-141 Automatic Link Establishment (ALE)
- STANAG waveforms (planned)

## Layering

```text
wavecore-rs       <- DSP primitives (constellations, pulse shapes, carriers, timing)
    ^
milwave-rs        <- this crate. MIL-STD waveforms.
    ^
NIF wrappers      <- desktop cdylib, Mob staticlib for Android/iOS
    ^
Elixir / Mob / Desktop applications
```

## Status

Early extraction from the MinuteModem umbrella's phy_modem/src/modem/
module. Content will land as the migration progresses:

- UnifiedModulator / UnifiedDemodulator with mini-probe constellation
  switching for 188-110D
- Decision-feedback equalizer (DFE) with HF skywave / ground-wave configs
- Walsh correlator for preamble sync
- Turbo product code FEC

## Feature flags

| Feature | Default | Purpose |
|---------|---------|---------|
| std     | yes     | Full standard library, inherent f64 math |
| alloc   |         | Heap allocator (implicit under std) |
| libm    |         | Software f64 math for no_std builds |
| serde   |         | Optional Serialize/Deserialize derives |

For no_std use: `cargo build --no-default-features --features alloc,libm`

## License

Dual-licensed under MIT or Apache-2.0, at your option.
