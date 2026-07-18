# Hardware Data Archive

Raw dashboard logs are written to the ignored `data/server/` directory. Sessions worth retaining
are promoted into `data/hardware_sessions/` with a checksum-bound manifest, compact extracts, and
notes describing what the data can and cannot support.

## Retained sessions

| Session | Description | Recommended use |
| --- | --- | --- |
| [`20260719_wood_floor_neutral`](hardware_sessions/20260719_wood_floor_neutral/) | 62.43 s undisturbed balance on a wooden floor; bounded 2.3065 Hz rocking mode | Neutral-balance timing, noise, and frequency-domain reference |
| [`20260718_noisy_runaway`](hardware_sessions/20260718_noisy_runaway/) | Selected quiet, runaway, intervention, and locked-runaway windows | Failure-mode and saturation reference only |

Do not fit controller or plant parameters directly from either session. The logs do not contain an
independently measured external force, hardware position, or complete motor-target telemetry.
