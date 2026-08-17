# Noisy and Runaway Hardware Fixtures — 2026-07-18

[Back to the hardware data archive](../../README.md)

[Analysis workflow](../../../doc/testing/telemetry_analysis_cli.md) · [Manifest](manifest.json)

These retained **pre-v12** 50 Hz fixtures were extracted from the 400 Hz dashboard capture
`telemetry_20260718-182506_00.csv`. The raw file is no longer retained; its provenance is bound by
the SHA-256 digest in `manifest.json`.

The four windows label quiet operation, runaway onset, manual intervention, and
locked runaway. They document IMU noise, scheduler cadence, feedback delay, and
saturation behavior. They must not be used to trajectory-fit controller gains;
in particular, the manual-intervention window includes operator input that is
not observable in telemetry.

These are frozen, checksum-bound evidence files. The original extraction script was intentionally
discarded after review because its segment timestamps were specific to this session.

The retained CSVs are 50 Hz fixtures, while the manifest’s per-window statistics describe the
original 400 Hz source windows. The manifest records the unavailable source checksum and source
window statistics, but it does not record hashes for these four fixture files or an exact
source-to-fixture extraction map. You can inspect the frozen fixtures and their stated claim scope;
you cannot reconstruct the original extraction from this checkout alone.

## Files

| File | Contents |
| --- | --- |
| [`quiet_operation.csv`](quiet_operation.csv) | Selected stable neutral window |
| [`runaway_onset.csv`](runaway_onset.csv) | Slow velocity-growth window before intervention |
| [`manual_intervention.csv`](manual_intervention.csv) | Operator intervention window |
| [`locked_runaway.csv`](locked_runaway.csv) | Locked runaway window |
| [`manifest.json`](manifest.json) | Source checksum, availability, windows, and derived statistics |

The raw source is not retained. Reproducing the extraction requires an external copy of the source
CSV with the checksum recorded in the manifest.
