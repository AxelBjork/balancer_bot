# Hardware Data Archive

Raw dashboard logs are written to the ignored `data/server/` directory. Sessions worth retaining
are promoted into `data/hardware_sessions/` with a checksum-bound manifest, compact extracts, and
notes describing what the data can and cannot support.

A fresh checkout contains these retained extracts and manifests, but not necessarily the original
raw CSVs. Reproduction commands that name `data/server/` require that source capture separately.

## Retained sessions

| Session | Description | Recommended use |
| --- | --- | --- |
| [`20260719_wood_floor_neutral`](hardware_sessions/20260719_wood_floor_neutral/) | 62.43 s undisturbed balance on a wooden floor; bounded 2.3065 Hz rocking mode | Neutral-balance timing, noise, and frequency-domain reference |
| [`20260718_noisy_runaway`](hardware_sessions/20260718_noisy_runaway/) | Selected quiet, runaway, intervention, and locked-runaway windows | Failure-mode and saturation reference only |
| [`20260722_pitch_inertia`](hardware_sessions/20260722_pitch_inertia/) | Two provisional free-swing excerpts recorded with `controller_enabled = 0` | Physical-pendulum evidence; do not use as a final inertia or damping calibration |
| [`20260722_wood_floor_direct_drive`](hardware_sessions/20260722_wood_floor_direct_drive/) | Fault-free forward-command and release excerpt after removing direct velocity feed-forward | Qualitative drive and completed-step feedback reference only |

Do not fit controller or plant parameters directly from these sessions. The logs do not contain an
independently measured external force, hardware position, or complete motor-target telemetry.

The two 2026-07-22 sessions have the same evidence boundary: they are useful runtime observations,
not calibrated plant-identification data. The [Pi guide](../doc/Running_on_Pi.md) links the
pitch-inertia measurement procedure to its retained excerpts.

## Claim and reproducibility levels

Checksum coverage answers which bytes are being discussed; it does not by itself prove how an
extract was selected or transformed. Use the retained evidence at the following levels:

| Evidence state | What can be reproduced or claimed |
| --- | --- |
| Raw source is available, with source/selection and artifact metadata | Re-run the documented selection and compare the derived artifact or metric |
| Raw source is unavailable, but the retained artifact has a checksum and documented transformation | Recompute a metric from the retained artifact; do not claim that the original extraction can be recreated |
| Artifact or extraction metadata is incomplete | Treat the file as bounded qualitative evidence only and state the missing provenance |
| Any historical capture, regardless of checksum coverage | Treat it as historical runtime evidence until its manifest, PID digest, repository revision, and hardware context are compared with the current run |

The dashboard CSVs are received `SystemTelemetry` packet logs, not complete raw sensor or plant
captures. They may contain controller-time resets, receive gaps, missing fields, or simulator-only
columns that are consistently zero. Split or select a continuous session before deriving metrics.

## Evidence path

For a recorded hardware claim, follow this path:

1. [`balancer_pi`](../doc/Running_on_Pi.md) runs while the dashboard receives production telemetry.
2. The dashboard writes a raw CSV under the ignored `data/server/` directory.
3. Analysis selects a continuous session and time window using the [shared telemetry workflow](../doc/testing/telemetry_analysis_cli.md).
4. A reviewed result is promoted here as a checksum-bound extract with a `manifest.json`.
5. The extract supports only the claim stated in its session README; it does not become a plant or
   controller calibration automatically.

Some source captures are no longer available. In those cases the manifest preserves the source
filename and checksum, while the retained extract and its documented transformation are the audit
record.
