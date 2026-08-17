# Retained hardware data

This directory contains selected hardware evidence, not a calibration database.
Raw dashboard captures normally live in the ignored `data/server/` directory.
Only reviewed extracts with a manifest are retained here.

The authoritative interpretation boundary is:

- a session README says exactly what its extract supports;
- `manifest.json` binds source names, checksums, windows, and transformations;
- a historical PID digest does not describe the current [`pid.conf`](../pid.conf);
- no session is a plant/controller fit unless its manifest explicitly permits
  that use and contains the required external measurements.

Before calculating derivatives, correlations, lags, or spectra, select one
continuous controller-time session. Do not interpolate across resets, receive
gaps, or manual-support intervals. Use the shared
[telemetry-analysis workflow](../doc/testing/telemetry_analysis_cli.md), not a
new one-off CSV parser.

## Retained sessions

| Session | Evidence | Permitted use |
| --- | --- | --- |
| [`20260719_wood_floor_neutral`](hardware_sessions/20260719_wood_floor_neutral/) | Undisturbed wooden-floor balance with a bounded `2.3065 Hz` rocking mode | Neutral balance, noise, and frequency reference |
| [`20260718_noisy_runaway`](hardware_sessions/20260718_noisy_runaway/) | Quiet, runaway, intervention, and locked-runaway windows | Failure-mode and saturation evidence only |
| [`20260722_pitch_inertia`](hardware_sessions/20260722_pitch_inertia/) | Two supported passive-pendulum excerpts with `controller_enabled = 0` | Provisional pitch-inertia evidence only |
| [`20260722_wood_floor_direct_drive`](hardware_sessions/20260722_wood_floor_direct_drive/) | Historical forward-command/release excerpt from the pre-v12 outer path | Qualitative motion and completed-step feedback evidence only |
| [`motor_tracking`](hardware_sessions/motor_tracking/) | Optical free-wheel STEP response and event timing | `93–95 Hz` actuator-mode correlation only |

The v12 outer loop and common controller-facing equations are defined in
[`doc/arch/control_plant.md`](../doc/arch/control_plant.md). The detailed
electrical simulator realization is defined in
[`doc/testing/stepper_phase_electrical.md`](../doc/testing/stepper_phase_electrical.md).
The retained
hardware mode near `93–95 Hz` is separate from the approximately `2.3065 Hz`
closed-loop rocking mode in the neutral-floor capture; do not infer that the
controller notch should move from its existing approximately 29 Hz center to
the actuator mode.

## Evidence workflow

1. Run `balancer_pi` while the dashboard records production telemetry.
2. Preserve the raw CSV, PID/config snapshot, repository revision, and command
   semantics.
3. Select a continuous session and named window with the shared analysis tool.
4. Promote only the reviewed extract, manifest, and claim-limited README.
5. Compare a historical session with current behavior only after checking its
   PID digest and runtime context.

The session folders intentionally use a lower, evidence-focused documentation
standard than the architecture handbook: provenance, claim scope, limitations,
and files are more important than restating the controller design.
