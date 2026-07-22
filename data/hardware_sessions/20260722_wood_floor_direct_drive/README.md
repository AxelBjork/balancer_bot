# Wood-Floor Direct-Drive Reference — 2026-07-22

This is a short, fault-free excerpt from
`data/server/telemetry_20260722-193855_00.csv`, captured after removing the direct motor
velocity feed-forward from the controller allocation. The operator reported stable balancing,
small translational control, and steering on a wooden floor. This fixture is qualitative evidence
that the simplified controller can balance and move; it is not a calibrated plant-identification or
controller-tuning dataset.

`session1_55p8_to_58p0.csv` is the newest fault-free complete forward-command stage in the source:
session 1, controller time 55.802502–58.000000 s. The target ramps to +1200 SPS, the motor command
and completed-pulse feedback reach forward speed, and the target then returns to neutral. All 879
rows have zero controller and actuator fault flags; no controller saturation flag was set.

[`session1_55p8_to_58p0.svg`](session1_55p8_to_58p0.svg) plots the requested velocity, motor
command, completed-pulse speed, pitch/reference, pitch rate, and relative completed wheel steps for
this exact interval.

| Phase | Time (s) | Mean motor (SPS) | Mean feedback (SPS) | RMS motor (SPS) | RMS feedback (SPS) | Pitch RMS (°) | Rate RMS (°/s) | Left / right completed steps |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Forward command | 55.80–57.18 | +723.9 | +705.5 | 870.5 | 837.5 | 2.29 | 20.61 | +984 / +984 |
| Release to neutral | 57.18–58.00 | +272.0 | +345.8 | 481.4 | 518.5 | 2.32 | 10.85 | +242 / +242 |
| Complete excerpt | 55.80–58.00 | +555.3 | +571.3 | 749.3 | 734.8 | 2.30 | 17.61 | +1226 / +1226 |

The command window is intentionally a real joystick transient rather than a steady-state fit. Its
large RMS values reflect the acceleration and release, not a claim of settled drive performance.

The source was recorded while the temporary `applied_velocity_feedforward_sps` protocol field was
still present. That field is preserved only as historical capture data; it is not part of the live
protocol after the controller cleanup.

Do not use this fixture to fit PID values. Reproduce the extraction from the checksum-bound source
with session selection before applying the controller-time window; the source file contains a
controller-time reset between sessions 0 and 1.
