# StepperPhaseElectrical ringdown reference

This directory retains only the extracted optical signal and the commanded
event timing needed to recreate the small-step mechanical ringdown analysis.
It is a bounded hardware-correlation reference, not a controller calibration
dataset.

## Fixture

- Robot body/stator fixed in a supported inverted fixture.
- One wheel free and one motor excited.
- Small signed one- and two-STEP events.
- The two edges of a two-step event are separated by 100 µs.
- Optical response captured at high frame rate.

The retained `optical_video_raw.csv` contains the extracted frame timestamp,
laser centroid, confidence, and diagnostic brightness/width fields. The
retained `optical_event_timing.csv` contains the signed event direction,
amplitude, hardware event time, and observed transition timing.

## Recreating the frequency estimate

1. Read `video_time_s`, `video_pts`, and the laser centroid coordinates from
   `optical_video_raw.csv`.
2. Restrict the analysis to rows with valid centroid coordinates during the
   stationary-camera portion of the capture.
3. Project the centroid coordinates onto their dominant one-dimensional axis
   or use the signed centroid displacement directly after removing the
   stationary baseline.
4. Map the hardware event times in `optical_event_timing.csv` into the optical
   time base using the transition timestamps. Keep the fitted time mapping
   separate from the mechanical signal.
5. For each signed one- or two-step event, fit the short post-transition
   displacement to a decaying sinusoid. Report the fitted frequency together
   with repetition and window sensitivity; do not infer torque or absolute
   phase from pixels alone.

The retained data is the extracted measurement input, not the original video.
The source video and the transient analysis script are unavailable, so this
procedure recreates the mechanical-frequency calculation from the retained
signal rather than reproducing the original computer-vision extraction.

## Simulator comparison

Use the same event sizes and timing in the StepperPhaseElectrical fixed-field
fixture. Compare frequency, phase-error sign, amplitude decay, and current/
torque timing. Do not add a lookup-table correction for individual microsteps
and do not fit controller gains to this fixture.

The authoritative simulator geometry is 200 motor full steps/revolution,
32 microsteps/full step, 6400 STEP/revolution, and a 0.0412 m wheel radius.
The actuator equations, electrical constants, normalization, and current
controller comparison are maintained in the
[StepperPhaseElectrical testing profile](../../../doc/testing/stepper_phase_electrical.md).
