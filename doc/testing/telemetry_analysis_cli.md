# Telemetry Analysis CLI Requirements

## Recommendation

A small CLI is worthwhile, but it should standardize the repetitive mechanics rather than try to
encode every control investigation. Most investigations still require a new derived expression or
a different way of dividing the run into windows. Pandas makes that exploratory work concise once
the CSV has been loaded and normalized correctly.

The useful split is:

- the CLI owns loading, schema normalization, session detection, window selection, common metrics,
  event/plateau detection, resampling, spectra, JSON output, and SVG generation
- an investigator can request normalized rows or import the same library and write a few lines of
  pandas for the question that is unique to the current run
- any exploratory calculation that is used more than once should be promoted into the shared
  library and covered by tests

This will eliminate most shell-embedded CSV parsers without turning the CLI into a control-system
expert. The CLI should default to concise JSON on stdout because JSON is easy for people, tests,
agents, and the dashboard to consume. Human-readable tables are an optional presentation mode.

## One Implementation, Including Existing Artifact Tooling

Do not create an independent parser or plotting program. The implementation must be merged with
the analysis and hand-written SVG support currently in
`tests/python/support/run_artifacts.py`.

The preferred end state is:

1. Move the reusable contents of `run_artifacts.py` into an importable package under
   `tools/telemetry_analysis/`.
2. Put CSV normalization, selections, metrics, JSON serialization, and the existing SVG renderer in
   that package.
3. Keep `tests/python/support/run_artifacts.py` as a thin compatibility import so existing simulator
   tests and `RunRecorder` callers do not need a large simultaneous rewrite.
4. Change `tools/analyze_timeline.py` into either the CLI entry point or another thin wrapper over
   the package.
5. Make simulator artifact generation and offline hardware analysis call the same functions with
   the same metric definitions.

Preserve the current dependency-free manual SVG renderer. Generalize it to accept the normalized
time column and arbitrary panel specifications. Do not add a separate matplotlib plotting path.

Pandas should be a declared development/tool dependency, not an undeclared property of one
developer environment. Pandas already brings NumPy, so NumPy may be used for vector operations and
FFT. Do not require SciPy for the base CLI. Algorithms with an optional SciPy implementation must
have a clearly tested NumPy/pandas fallback.

## Scope

The CLI is read-only unless an explicit output path is supplied. It accepts:

- dashboard/server CSVs under `data/server/`
- simulator `timeline.csv` files and rows emitted by `RunRecorder`
- standalone or reduced CSV captures containing any supported subset of columns
- files that are still being appended to by the telemetry server

It should not:

- tune PID values or declare a controller safe
- contain thresholds that only make sense for one captured incident
- guess missing signals from unrelated columns
- rewrite, truncate, or repair its input file
- grow another signal-name catalog separate from the dashboard catalog
- replace exploratory pandas analysis for genuinely novel questions

## Proposed CLI Surface

Keep the number of commands small. Options such as `--session`, `--time`, `--rows`, and `--where`
should work consistently across commands.

```text
telemetry-analysis inspect FILE
telemetry-analysis rows FILE [SELECTION] --columns SIGNAL...
telemetry-analysis stats FILE [SELECTION] --signals SIGNAL...
telemetry-analysis bins FILE [SELECTION] --width 1s --signals SIGNAL...
telemetry-analysis events FILE [SELECTION]
telemetry-analysis plateaus FILE [SELECTION] --signal SIGNAL --tolerance N --min-duration 1s
telemetry-analysis relate FILE [SELECTION] --source SIGNAL --response SIGNAL --max-lag 1s
telemetry-analysis spectrum FILE [SELECTION] --signals SIGNAL... --max-frequency 20
telemetry-analysis plot FILE [SELECTION] --preset overview --output overview.svg
```

Examples:

```sh
# Discover sessions, columns, unavailable fields, cadence, gaps, and flag counts.
telemetry-analysis inspect data/server/telemetry.csv

# Get compact one-second summaries without writing a file.
telemetry-analysis bins data/server/telemetry.csv \
  --time 7:17 --width 1s \
  --signals pitch_deg,raw_acc_pitch_deg,pitch_rate_dps,u_sps,measured_vel_sps

# Export an exact normalized window for free-form pandas work.
telemetry-analysis rows data/server/telemetry.csv \
  --session 2 --time 12:18 --columns time_s,pitch_deg,u_sps --format csv \
  --output build/analysis/window.csv

# Compare two aligned signals using common lag, scale, correlation, and residual definitions.
telemetry-analysis relate build/sim/run/timeline.csv \
  --source u_sps --response measured_vel_sps --max-lag 1s
```

`inspect`, `stats`, `bins`, `events`, `plateaus`, `relate`, and `spectrum` should emit JSON by
default. `rows` defaults to JSON Lines on stdout so large selections can stream. `--format table`
is for interactive use, while `--format csv` and `--output` create an explicit derived artifact.

## Stable JSON Contract

Every JSON result should include provenance and the resolved selection. Warnings belong in the
result rather than being lost in terminal prose.

```json
{
  "schema_version": 1,
  "analysis": "stats",
  "source": {
    "path": "data/server/telemetry.csv",
    "size_bytes_at_read": 123456,
    "complete_rows_read": 4000
  },
  "selection": {
    "session_ids": [0],
    "time_column": "t_sec",
    "time_start_s": 7.0,
    "time_end_s": 17.0,
    "source_row_start": 2800,
    "source_row_end": 6799
  },
  "warnings": [],
  "result": {}
}
```

Requirements:

- use JSON `null`, never non-standard `NaN` or `Infinity`
- include units and preprocessing beside metrics where they are not self-evident
- keep signal names stable and use the shared signal catalog for aliases, labels, units, colors,
  plot groups, and zero-centered plotting conventions
- include all thresholds supplied by the user or defaults applied by the command
- report why a calculation is unavailable instead of returning a plausible zero
- keep stdout machine-readable; progress and warnings intended only for people go to stderr

## Input Normalization

### Complete-row snapshots

When the source is growing, record its byte length when the command starts and read no further than
that snapshot. Ignore a final record that does not have the header's field count. Do not silently
skip malformed complete records in the middle of the file; report their source row numbers.

For operations that only need aggregates, use `pandas.read_csv(..., chunksize=...)` and combine
partial aggregates. Loading tens of megabytes into a DataFrame is acceptable for analyses that
require random access, resampling, or FFT, but it should be an explicit consequence of that
analysis rather than the only loading implementation.

The implementation should retain both the original frame and a numeric view:

```python
raw = pd.read_csv(snapshot, dtype=str)
numeric = raw.apply(pd.to_numeric, errors="coerce")
```

This prevents one non-numeric value from forcing an otherwise numeric telemetry column to object
dtype while preserving identifiers and malformed values for diagnostics.

### Time and sessions

Choose the first suitable controller/simulator time from `t_sec`, `sim_time_s`, `time_s`, or
`time`. Keep receive timestamps as separate clocks. Normalize the chosen analysis clock to
`time_s` without deleting its original column.

A new session begins on any of:

- controller/simulator time moving backward or resetting
- a `run_id` change
- a receive-time gap explicitly selected as a session boundary
- an input-file boundary when multiple files are supplied

Do not split merely because one 400 Hz sample is late. The default gap threshold should be derived
from cadence, for example `max(20 * median_dt, 0.25 s)`, and must be reported.

```python
t = numeric[time_key]
positive_dt = t.diff().where(lambda s: s > 0)
median_dt = positive_dt.median()
reset = t.diff().le(0)
gap = t.diff().gt(max(20 * median_dt, 0.25))
session_id = (reset | gap).fillna(False).cumsum().astype("int64")
```

### Missing and uninformative fields

Distinguish these states:

- `missing`: column is absent
- `non_numeric`: no values can be parsed as finite numbers
- `all_null`: column exists but every parsed value is null
- `all_zero`: all finite values are zero; often unavailable hardware telemetry, but sometimes a
  legitimate constant signal
- `constant`: one non-zero finite value
- `variable`: suitable for general analysis

Never automatically substitute one signal for another. A command can accept an ordered alias list,
but it must report the resolved field.

```python
def classify(series: pd.Series) -> str:
    finite = pd.to_numeric(series, errors="coerce").replace([np.inf, -np.inf], np.nan).dropna()
    if finite.empty:
        return "all_null"
    if np.isclose(finite.to_numpy(), 0.0).all():
        return "all_zero"
    if finite.nunique() == 1:
        return "constant"
    return "variable"
```

## Selection Primitives

All commands should share one selection object supporting:

- source row range, preserving the original zero-based source row number
- controller/simulator time range
- receive timestamp range
- session number
- exact flag value or bit mask for controller faults and saturation
- a pandas-style conjunction of simple comparisons, such as
  `abs(pitch_rate_dps) < 5 and abs(measured_vel_sps) < 200`
- detected constant-command plateau and plateau sign

Arbitrary Python evaluation is not acceptable in the CLI. Implement `--where` using a small safe
comparison grammar or a constrained DataFrame query parser. The selected frame must retain
`source_row` and `session_id`.

Reusable contiguous-window detection is more valuable than dozens of named incident detectors:

```python
def contiguous_runs(mask: pd.Series) -> pd.DataFrame:
    groups = mask.ne(mask.shift(fill_value=False)).cumsum()
    selected = mask.groupby(groups).first()
    spans = (
        mask.to_frame("selected")
        .assign(group=groups)
        .query("selected")
        .groupby("group")
        .agg(first_row=("selected", "idxmin"), last_row=("selected", "idxmax"))
    )
    return spans.loc[selected[selected].index]
```

The real implementation should also attach start/end time, duration, row count, and preceding and
following values.

## Reusable Metrics

### Generic statistics

For each requested signal provide count, null count, min, max, mean, standard deviation, RMS,
median, p05, p95, p95 absolute value, first, and last. Do not compute RMS after demeaning unless the
metric explicitly says so.

```python
def signal_stats(s: pd.Series) -> dict[str, float | int | None]:
    x = pd.to_numeric(s, errors="coerce").replace([np.inf, -np.inf], np.nan).dropna()
    if x.empty:
        return {"count": 0}
    a = x.to_numpy(dtype=float)
    return {
        "count": int(a.size),
        "min": float(a.min()),
        "max": float(a.max()),
        "mean": float(a.mean()),
        "std": float(a.std(ddof=0)),
        "rms": float(np.sqrt(np.mean(a * a))),
        "median": float(np.median(a)),
        "p05": float(np.quantile(a, 0.05)),
        "p95": float(np.quantile(a, 0.95)),
        "p95_abs": float(np.quantile(np.abs(a), 0.95)),
        "first": float(a[0]),
        "last": float(a[-1]),
    }
```

### Time bins

Time-binned summaries are the fastest way to understand a long run before looking for a specific
event. They should work with any signals and use elapsed time rather than assuming exactly 400
rows per second.

```python
width_s = 1.0
bin_id = np.floor((df["time_s"] - df["time_s"].iloc[0]) / width_s).astype("int64")
summary = df.groupby(bin_id)[signals].agg(["count", "mean", "std", "min", "max"])
```

Add RMS and fraction-of-rows predicates such as non-zero fault flags or saturation. Emit each bin's
actual start/end time and sample count so capture gaps remain visible.

### Cadence and gaps

Report median, p05, p95, and maximum positive `dt`, non-positive steps, counts above configurable
multiples of median cadence, and the largest gaps with source rows and both timestamps.

```python
dt = df.groupby("session_id")["time_s"].diff()
median_dt = dt.where(dt > 0).median()
gaps = df.loc[dt > 5 * median_dt, ["source_row", "session_id", "time_s"]].assign(dt_s=dt)
```

Analyze controller time and receive time separately. A receive-time gap with continuous controller
time means capture or delivery delay, not necessarily a controller stall.

### Flags and event transitions

Treat fault and saturation columns as integer bit fields. Report union, counts and duration per
value and per bit, plus transition windows.

```python
flags = pd.to_numeric(df["controller_fault_flags"], errors="coerce").fillna(0).astype("uint32")
transitions = df.loc[flags.ne(flags.shift()), ["source_row", "time_s"]].assign(
    previous=flags.shift().loc[lambda s: s.index.isin(transitions.index)],
    current=flags.loc[lambda s: s.index.isin(transitions.index)],
)
```

The production implementation should avoid the self-reference in this compact sketch and use a
named transition mask. Bit names should come from one declared mapping matching the C++ protocol.

### Wrapped angles and integrated rates

Angle subtraction must be circular. A normal subtraction around `-180`/`180` creates fake 360°
events.

```python
def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0

gravity_error_deg = wrap_deg(df["raw_acc_pitch_deg"] - df["fused_pitch_deg"])
fused_rate_dps = wrap_deg(df["fused_pitch_deg"].diff()) / df["time_s"].diff()
correction_rate_dps = fused_rate_dps - df["gyro_pitch_rate_dps"]
```

Integrate a rate without SciPy using the trapezoidal rule:

```python
t = df["time_s"].to_numpy(dtype=float)
g = df["gyro_pitch_rate_dps"].to_numpy(dtype=float)
integrated = np.r_[0.0, np.cumsum(0.5 * (g[1:] + g[:-1]) * np.diff(t))]
```

Useful generic angle diagnostics include wrapped error statistics, time spent outside an error
band, recovery slope after large disagreement, and whether an angle returns near its initial value
after one or more complete rotations.

### Lag, scale, sign, and residuals

Preserve and generalize `estimate_lag_scale()` from `run_artifacts.py`. It should:

- operate within one session
- optionally resample irregular data to a uniform grid
- search positive and negative lags unless causality is explicitly constrained
- optionally analyze levels or first differences
- return lag, scale, signed correlation, RMSE, sample count, and confidence warnings
- reject constant or mostly unavailable signals

For quick exploratory work after uniform resampling:

```python
best = None
for lag in range(-max_steps, max_steps + 1):
    shifted = response.shift(-lag)
    pair = pd.concat({"source": source, "response": shifted}, axis=1).dropna()
    corr = pair.corr().iloc[0, 1]
    scale = np.dot(pair.source, pair.response) / np.dot(pair.source, pair.source)
    rmse = np.sqrt(np.mean((scale * pair.source - pair.response) ** 2))
    candidate = {"lag_steps": lag, "correlation": corr, "scale": scale, "rmse": rmse}
    if best is None or abs(corr) > abs(best["correlation"]):
        best = candidate
```

Always report whether signals were demeaned or differenced. Correlation of two trending signals can
look excellent while saying little about dynamic response.

### Plateaus and response metrics

Plateau detection should be generic: quantize a signal using a caller-provided tolerance, run-length
encode it, then retain spans longer than a minimum duration.

```python
level = (df[signal] / tolerance).round() * tolerance
group = level.ne(level.shift()).cumsum()
plateaus = (
    df.assign(level=level, group=group)
    .groupby("group")
    .agg(
        level=("level", "median"),
        start_s=("time_s", "first"),
        end_s=("time_s", "last"),
        rows=("time_s", "size"),
    )
)
plateaus["duration_s"] = plateaus.end_s - plateaus.start_s
```

Given a detected step or plateau, reusable response metrics are delay to first correct-sign
response, 10–90% rise time, peak and overshoot, time in the wrong direction, settled mean/RMS,
steady error, and stopping or reversal time. Positive/negative symmetry compares matched absolute
plateau levels and reports ratios as well as signed differences.

### Spectra, phase, and coherence

FFT requires uniform sampling. Split at session boundaries and large gaps, interpolate only within
a contiguous selected window, detrend, apply a named window, and report sample rate and bin width.
A NumPy-only amplitude spectrum is sufficient for the base CLI:

```python
rate_hz = 100.0
uniform_t = np.arange(t[0], t[-1], 1.0 / rate_hz)
uniform_y = np.interp(uniform_t, t, y)
q = np.arange(uniform_y.size)
detrended = uniform_y - np.polyval(np.polyfit(q, uniform_y, 1), q)
window = np.hanning(detrended.size)
amplitude = 2.0 * np.abs(np.fft.rfft(detrended * window)) / window.sum()
frequency_hz = np.fft.rfftfreq(detrended.size, d=1.0 / rate_hz)
```

Return dominant peaks with configurable frequency separation rather than simply returning adjacent
bins from one broad peak. Cross-spectral phase and magnitude-squared coherence can be implemented
with a documented NumPy Welch calculation; SciPy must remain optional. Never interpolate across a
session reset or material capture gap.

### Safe JSON conversion

Pandas and NumPy scalar types and non-finite floats need normalization before `json.dumps`:

```python
def json_value(value):
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    return value
```

## SVG and Artifact Output

Generalize `_write_svg_multiplot()` rather than replacing it. It should consume normalized frames
or a small frame-to-row adapter, use the shared signal catalog, and support:

- arbitrary signal panels from CLI arguments or named presets
- the resolved analysis time instead of requiring `sim_time_s`
- explicit y ranges or zero-centered automatic ranges
- gap-aware polylines that do not draw through missing sessions
- min/max envelope downsampling so short spikes are not discarded
- titles containing source and resolved selection

`RunRecorder.write_csv_json_plots()` should call the same public plot and summary functions as the
CLI. Existing `overview_plot.svg` and `actuator_plot.svg` outputs remain supported. Optional CSV,
JSON, or SVG artifacts are written only when an explicit output directory or path is supplied.

## Practical Workflow for Novel Investigations

The expected workflow is deliberately simple:

1. Run `inspect` to identify sessions, gaps, valid fields, and obvious fault intervals.
2. Run `bins` over the whole session to locate interesting time ranges.
3. Use `stats`, `events`, `plateaus`, `relate`, or `spectrum` if the question matches a reusable
   metric.
4. Use `rows --format csv` or import `load_telemetry()` for a unique pandas calculation.
5. Promote the calculation only if it recurs or belongs in regression tests.

The package API is as important as the executable:

```python
from tools.telemetry_analysis import load_telemetry, Selection

run = load_telemetry("data/server/telemetry.csv")
window = run.select(Selection(session=0, time=(7.0, 17.0)))
df = window.numeric
```

This is the escape hatch for questions that do not justify a permanent CLI option. It still reuses
safe ingestion, normalized time, sessions, source rows, aliases, and availability diagnostics.

## Regression Coverage

Tests should cover:

- server, simulator, reduced, concatenated, growing, and incomplete-final-row CSVs
- time-column selection, controller-time resets, receive-time gaps, and multiple sessions
- absent, malformed, all-null, all-zero, constant, and variable fields
- row/time/flag/query/plateau selections with exact provenance
- irregular cadence and no interpolation across gaps
- wrapped angle differences and crossings at `-180`/`180`
- lag and scale with known synthetic delay, sign, offset, noise, and irregular sampling
- spectra with known frequencies and amplitudes, including peak separation
- valid strict JSON without `NaN`
- stable SVG generation and preservation of spikes during downsampling
- compatibility of existing simulator `RunRecorder` callers and artifact filenames

## Domain Lessons Worth Keeping Generic

Circular-angle diagnostics should compare gravity-derived and fused angles with wrapped
differences. Include detection of folded accelerometer estimates that behave like
`asin(sin(angle))`: they agree near upright but reflect beyond `+90` or `-90` instead of continuing
around the circle.

Command-authority analysis should be built from generic plateau and relationship primitives. For a
selected command plateau it can compare requested target, governed target, final actuator command,
measured response, and controller terms. Report response delay, wrong-sign intervals, cancellation
or reinforcement, settled authority, and positive/negative symmetry. Keep the signal list
configurable so the same analysis works for simulator and hardware schemas.
