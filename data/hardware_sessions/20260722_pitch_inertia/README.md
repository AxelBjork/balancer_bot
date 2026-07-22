# Passive Pitch-Inertia Measurement — 2026-07-22

These are two derived excerpts from the dashboard/server capture
`data/server/telemetry_20260722-000533_00.csv`. They preserve full reflected telemetry rows and
were selected as clean, continuous free-swing bursts after starting `balancer_pi` with
`controller_enabled = 0`: the controller and IMU telemetry ran, but the motor runner was absent
and the steppers remained de-energized.

The robot was supported through the wheel-axis line. The wheels were fixed against motor-rotor
resistance, so this is a provisional physical-pendulum result, not a friction-free bearing test.
Manual assistance occurred elsewhere in the capture; these two windows were chosen for their
closely matching periods, but they do not provide a defensible amplitude-decay estimate for a
damped correction.

| Extract | Session / controller-time window | Median period | Undamped J estimate |
| --- | --- | ---: | ---: |
| `session0_51p2_to_54p6.csv` | 0 / 51.2–54.6 s | 0.54383 s | 0.004551 kg m² |
| `session1_3p8_to_5p5.csv` | 1 / 3.8–5.5 s | 0.54186 s | 0.004518 kg m² |

Using `H = 0.06192 kg m` and `g = 9.81 m/s²`, the agreed one-decimal-millith precision
provisional value is **`J = 0.0045 kg m²`**. Do not use this fixture to tune controller gains.
Before replacing the simulator constant, obtain an unassisted small-angle run on a lower-friction
pivot and compare the result; motor detent/friction is the main remaining systematic uncertainty.

Reproduce either estimate without changing the source capture:

```bash
python3 tools/measure_pitch_inertia.py data/server/telemetry_20260722-000533_00.csv \
  --session 0 --start-s 51.2 --end-s 54.6
```

The source and derived-file checksums are recorded in `manifest.json`.
