# Noisy and Runaway Hardware Fixtures — 2026-07-18

These retained 50 Hz fixtures were extracted from the 400 Hz dashboard capture
`telemetry_20260718-182506_00.csv`. The raw file is no longer retained; its provenance is bound by
the SHA-256 digest in `manifest.json`.

The four windows label quiet operation, runaway onset, manual intervention, and
locked runaway. They document IMU noise, scheduler cadence, feedback delay, and
saturation behavior. They must not be used to trajectory-fit controller gains;
in particular, the manual-intervention window includes operator input that is
not observable in telemetry.

These are frozen, checksum-bound evidence files. The original extraction script was intentionally
discarded after review because its segment timestamps were specific to this session.
