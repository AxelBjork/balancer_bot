# Telemetry Analysis

This page defines the required analysis workflow for production telemetry, simulator timelines, and
captured hardware sessions. Analysis is read-only: it must never rewrite or repair a source CSV, and
derived output must be written only to an explicit path under `build/` or another requested output
location.

While diagnosing live capture gaps, the dashboard status includes the maximum and average duration
of `TelemetryState.accept`, plus the approximate CSV queue depth. `udp_receive_pause` events indicate
that receive timestamps paused while controller time remained continuous; `csv_write_stall` events
identify a slow individual CSV write.

The live dashboard distinguishes fresh telemetry from Pi reachability: `telemetry_connected` drives
the **Streaming** state, while `pi_ready` identifies an online Pi whose telemetry has stopped. The
descriptive `connection_state` is not the online/offline boolean.

## Sources and Ownership

The telemetry dashboard is the primary production UDP peer. It validates `SystemTelemetry` datagrams
and writes fixed-schema CSV captures under `data/server/`; a successful live Start normally begins a
new file, while the existing 128 MiB limit remains a fallback for unusually long runs. Simulator
runs and `RunRecorder` write structured artifacts under `build/sim/`. Both sources are valid inputs
to the shared analysis package.

There must be one pandas-backed analysis implementation shared by:

- `tools/analyze_timeline.py`
- simulator and test artifact generation
- dashboard/offline analysis that needs normalized frames or common metrics
- the existing analysis helpers in `tests/python/support/run_artifacts.py`

The ownership boundary is:

- `tools/telemetry_analysis/` owns reusable frame normalization, metrics, and plotting primitives;
- `tools/analyze_timeline.py` is the supported command-line wrapper and does not implement a second
  CSV or metric stack;
- `tests/python/support/run_artifacts.py` is the simulator/test artifact adapter. It uses the shared
  package, while retaining artifact summaries, manifests, and test-facing row helpers; and
- the dashboard uses the shared package for CSV playback and offline analysis where applicable.

The current import path through `tests/python/support/run_artifacts.py` therefore does not make the
shared analysis package test-only. It reflects the adapter’s current ownership of simulator artifact
recording and summary metadata.

At function level, the current split is:

| Location | Current responsibility |
| --- | --- |
| `tools/telemetry_analysis/frames.py` | Canonical frame aliases, numeric normalization, and CSV read/write |
| `tools/telemetry_analysis/metrics.py` | Reusable band-response and actuator-stage metrics |
| `tools/telemetry_analysis/plotting.py` | Shared SVG plotting |
| `tests/python/support/run_artifacts.py` | Timeline lag/scale analysis, summaries, artifact metadata, and `RunRecorder` integration |
| `tools/analyze_timeline.py` | Thin CLI wrapper around the artifact adapter |
| `tools/measure_pitch_inertia.py` | Domain-specific period/inertia calculation using the shared frame loader |
| `tools/telemetry_dashboard/server.py` | Live CSV logging and playback integration |

This table describes the current implementation, not the desired long-term directory layout. The
lag/scale and timeline-summary functions remain candidates for promotion into the shared package
only if they become reusable beyond the current artifact workflows.

Use `tools/telemetry_analysis/` for reusable frame normalization, metrics, and plotting. Do not add
another CSV parser, signal catalog, metric implementation, or plotting stack for an individual
investigation. A one-off investigation may import the shared loader and use concise pandas in one
terminal session; reusable calculations belong in the shared package with regression coverage.

## Current Command

The supported command-line entry point is:

```bash
python3 tools/analyze_timeline.py FILE [--summary-json] [--output OUTPUT]
```

It accepts a simulator or dashboard-compatible CSV, loads it through the existing artifact support,
and emits JSON to stdout unless `--output` is supplied. The command currently provides the standard
timeline analysis and summary; the multi-command `telemetry-analysis inspect/rows/stats/...`
interface is a future extension, not an installed command.

The current `analyze_timeline.py` wrapper does not expose `--session`, `--start`, or `--end`
selection flags. For a multi-session or bounded-window investigation, select the continuous rows
before calling the command, use the shared Python loader directly, or use a purpose-built tool such
as [`measure_pitch_inertia.py`](../../tools/measure_pitch_inertia.py), which exposes session and
controller-time selection for its calculation.

Examples:

```bash
python3 tools/analyze_timeline.py build/sim/<run_id>/timeline.csv --summary-json
python3 tools/analyze_timeline.py data/server/telemetry_YYYYMMDD-HHMMSS_00.csv \
  --summary-json --output build/analysis/telemetry.json
```

The dashboard’s **Choose CSV** and upload paths use the same shared telemetry package for playback.
Install `requirements-dev.txt` when using pandas-backed playback or analysis.

The dashboard’s browser JSON is a presentation adapter, not the reflected wire schema. When a chart
is blank or a field appears unavailable, compare the browser path in
`tools/telemetry_dashboard/static/dashboard.js` with the keys emitted by `telemetry_view()` in
`tools/telemetry_dashboard/server.py`, then inspect the logged CSV and generated binding. Do not
infer field absence or a controller result from a missing chart alone.

## Analysis Rules

Analysis must distinguish these clocks:

- controller/simulator time for dynamics and derivatives
- receive wall time for operator-facing timestamps
- receive monotonic time for gap and freshness measurements

Sessions must be split or selected before calculating derivatives, correlations, lags, or spectra.
Material capture gaps and resets must not be interpolated across. Missing or consistently zero fields
must be reported as unavailable rather than treated as real measurements; this is especially
important for hardware captures containing simulator-only columns.

Every reported result should identify the exact source path, session or row selection, time window,
and transformation. Hardware logs are evidence of runtime behavior, not calibrated plant-identification
data, unless the capture manifest explicitly permits fitting.

## Shared Outputs

The shared package currently provides:

- canonical telemetry-frame normalization and CSV read/write helpers
- band RMS and frequency-response metrics
- actuator-stage metrics
- SVG plotting through the existing shared plotting implementation

Simulator artifact generation must continue to use the shared helpers through
`tests/python/support/run_artifacts.py`. New output formats or selection APIs should be added only
when they are needed by more than one workflow or by a regression test.

## Workflow for a Novel Investigation

1. Identify the source file, capture manifest, clock, and session boundaries.
2. Import the shared loader or use `tools/analyze_timeline.py`.
3. Check field availability, resets, gaps, cadence, and zero-only columns before deriving metrics.
4. Select a continuous session and explicit time window.
5. Write derived CSV, JSON, or SVG output only to an explicit build/output path and record its source
   and transformation.
6. Promote repeated calculations into `tools/telemetry_analysis/` and add focused tests.

The dashboard and analysis tools must remain separate from controller tuning and safety decisions.
They report evidence; they do not declare a PID configuration safe for hardware.
