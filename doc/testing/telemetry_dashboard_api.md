# Telemetry Dashboard API

This guide is for a developer or agent running inside the Podman Codex environment who needs to
observe a live hardware session and apply session-only controller overrides through the host dashboard. The client only
uses HTTP; it does not need access to the host process memory or the Pi's SSH keys.

## Access from the Codex container

Start the dashboard on the host, using the same repository checkout and the Pi target configured for
the host:

```bash
python3 tools/telemetry_dashboard/server.py --pi rpi4 --port 8080 --listen-lan
```

`--listen-lan` binds the server to `0.0.0.0`. Without it, the default bind address is
`127.0.0.1`, which is the host loopback interface and is not reachable from the container.

From inside the Codex/Podman environment, use:

```text
http://host.containers.internal:8080
```

Inside the container, `localhost` means the container itself. The host's LAN address, such as
`192.168.0.196`, is a different network path and may not be routable from the isolated container.
In the current Podman environment, `host.containers.internal` resolves to the host gateway and is
the intended connection name. Verify the mapping and endpoint before an experiment:

```bash
BASE=http://host.containers.internal:8080
getent hosts host.containers.internal
curl -fsS "$BASE/api/source"
```

[`tools/codex-gui`](../../tools/codex-gui) is the host launcher for the Codex Podman container. It
mounts this repository at `/workspaces/balancer_bot` and starts the containerized GUI; it does not
start the telemetry dashboard. The dashboard remains a host process, and the host-gateway URL is
the HTTP boundary between the two environments.

The current server has no authentication or TLS layer. Any client that can reach port 8080 can
invoke deployment, start, abort, joystick, and PID operations, not just read telemetry. `--listen-lan`
exposes the dashboard on host interfaces. Use a trusted or firewall-restricted network, enable it
only for the experiment, and stop the dashboard or remove the option afterward.

The Pi bridge accepts one active UDP peer. Stop another dashboard, SIL client, or UDP observer before
starting this session, or telemetry may be delivered to a different peer.

## Read-only endpoints

Set a base URL in examples as follows:

```bash
BASE=http://host.containers.internal:8080
```

### `GET /api/source`

Returns the selected source and dashboard display metadata:

```json
{
  "mode": "live",
  "name": "rpi4",
  "duration_s": null,
  "sample_cadence_s": null,
  "configured_pi": "rpi4",
  "connection_state": "configured",
  "connection_message": "Waiting to resolve and register with the Pi.",
  "run_limit_s": 120.0,
  "display_sample_hz": 50.0,
  "display_run": 3
}
```

The connection fields above are illustrative and change as resolution, heartbeat, and telemetry
events occur.

This response describes the source, but live/freshness state is reported by the `status` events
from `/api/stream`.

### `GET /api/pid`

Returns the dashboard's in-memory controller snapshot, the local `pid.conf` baseline loaded when the
server started, and the last delivery status:

The maps are abbreviated below for readability; the actual response contains all 12 numeric fields in both
`values` and `baseline`.

```jsonc
{
  "ok": true,
  "values": { "pitch_gain": 6000.0 /* ...11 more fields... */ },
  "baseline": { "pitch_gain": 6000.0 /* ...11 more fields... */ },
  "override_active": false,
  "pending": false,
  "last_status": null
}
```

The complete numeric snapshot has these 12 fields, presented in the same order as `pid.conf`:

```text
pitch_gain, pitch_rate_gain, pitch_accel_gain,
drive_max_acceleration_mps2, velocity_damping_per_s, velocity_pitch_limit_deg, velocity_I,
velocity_I_limit_deg, velocity_control_cutoff_hz, drive_max_sps,
turn_max_sps, balance_max_sps
```

This is presentation order only; the nested binary PID payload retains its established wire
layout for compatibility.

### `GET /api/history`

Returns the bounded display-rate telemetry history. Query parameters are optional:

| Parameter | Default | Effective range | Meaning |
| --- | ---: | ---: | --- |
| `seconds` | `120` | `1` to `120` for live sessions | Receive-time window |
| `max_points` | `6000` | `10` to `6000` | Maximum returned samples |
| `end` | latest sample | — | Receive-time endpoint for the window |

`seconds` and `max_points` outside their limits are clamped; malformed numeric values use their
defaults. The `end` value is capped internally when selecting samples, while `requested_end` can
still echo the caller's value. If there are more samples than `max_points`, the response decimates
them and sets `decimated` to `true`.
`received_at` and `end` use the server's monotonic receive clock. `t_sec` is the controller clock;
keep those clocks distinct when analyzing dynamics. See the
[telemetry analysis guide](telemetry_analysis_cli.md).

The response includes `earliest`, `latest`, window/cache bounds, `display_run`, and `samples`.
Each sample is the browser-facing normalized object emitted by the server, with `attitude`, `rate`,
`motion`, `controller`, `timing`, and `flags` groups.

Live samples also carry source timing metadata at the top level:

```json
{
  "run_id": 123456,
  "packet_seq": 8123,
  "loop_seq": 8123,
  "sender_monotonic_ns": 998877665544
}
```

`run_id` changes when the hardware process restarts. `packet_seq` identifies the logical packet
produced by the controller, `loop_seq` identifies the physics tick, and
`sender_monotonic_ns` is the sender's clock domain; it must not be subtracted directly from
`received_at`, which is the dashboard host's monotonic clock.

Examples:

```bash
curl -fsS "$BASE/api/source"
curl -fsS "$BASE/api/history?seconds=10&max_points=500"
```

### `GET /api/stream`

This is an HTTP Server-Sent Events stream. Use `curl -N` to keep the connection open:

```bash
curl -N "$BASE/api/stream"
```

The stream currently emits two event types:

- `telemetry` — normalized samples containing `sequence`, `received_at`, `t_sec`, the grouped
  telemetry values described above, and the sender metadata above.
- `status` — dashboard and connection state. Relevant fields include `run_active`,
  `telemetry_connected`, `pi_ready`, `last_packet_age_ms`, `connection_state`,
  `connection_message`, `display_run`, `pid_status`, telemetry gap counters, sender identifiers,
  effective UDP receive-buffer size, CSV writer metrics, dashboard-loop/SSE timing metrics, and
  latched flags.

The diagnostic status fields are `sender_run_id`, `sender_packet_seq`, `sender_loop_seq`,
`sender_monotonic_ns`, `sender_time_gap_ms`, `receiver_gap_ms`,
`telemetry_sender_reset_count`, `telemetry_packet_sequence_gap_count`,
`telemetry_loop_sequence_gap_count`, and `last_gap_classification`. CSV backpressure is exposed
as `csv_queue_depth`, `csv_write_stall_count`, and `csv_write_max_ms`; dashboard delivery is
exposed as `dashboard_display_lateness_max_ms`, `dashboard_display_work_max_ms`,
`dashboard_display_stall_count`, `dashboard_sse_stall_count`, and `dashboard_sse_flush_max_ms`.
`udp_receive_buffer_bytes` reports the effective socket receive buffer after applying the 4 MiB
target.

Gap events retain the existing `udp_receive_pause`, `telemetry_packet_gap`, and `telemetry_gap`
event names. Their JSON data now includes `classification`, `sender_time_gap_s`, sequence deltas,
and packet/loop gap counts. Classifications are `receiver_or_network_pause`,
`sender_control_dispatch_pause`, `packet_or_transport_gap`, `loop_without_packet_gap`, or
`receiver_and_sender_gap`. A `telemetry_run_reset` event records a sender run boundary and is not
counted as a freeze.

For a live experiment, use these meanings rather than `connection_state` as an online boolean:

```text
telemetry_connected == true                 Streaming
telemetry_connected == false, pi_ready true Pi online; telemetry stopped
telemetry_connected == false, pi_ready false Pi offline
```

`telemetry_connected` becomes false when no valid packet has arrived for one second. `pi_ready` is
the independent heartbeat result for the Pi's SSH port. `run_active` is the dashboard's run
lifecycle flag. A fresh status event should be observed before changing PID values.

The dashboard's `Freeze diagnostics` view is an opt-in plot group and is hidden at startup. When
enabled, it plots receiver interarrival gaps, sender-time gaps, browser SSE gaps, append/render
durations, and the latest sample age; it is not an always-on status readout.

## Session-only PID tuning

`POST /api/pid` accepts a complete numeric controller snapshot. The recommended request shape is:

```json
{
  "values": {
    "pitch_gain": 6000.0,
    "pitch_rate_gain": 500.0,
    "pitch_accel_gain": 0.0,
    "drive_max_acceleration_mps2": 1.0,
    "velocity_damping_per_s": 8.0,
    "velocity_pitch_limit_deg": 4.0,
    "velocity_I": 0.001,
    "velocity_I_limit_deg": 4.0,
    "velocity_control_cutoff_hz": 3.0,
    "drive_max_sps": 1200.0,
    "turn_max_sps": 1200.0,
    "balance_max_sps": 16000.0
  }
}
```

The values must be numeric and finite. The server rejects unknown or missing fields, negative values
where non-negative values are required, non-positive required limits, `velocity_pitch_limit_deg`
above 90° (zero is the explicit diagnostic no-limit value), and `drive_max_sps`, `turn_max_sps`,
or `balance_max_sps` above the current verified-1/32 limit of `16000`.

The immediate response reports `request_id`, `sent`, and `pending`. Delivery and validation on the
Pi are asynchronous. The eventual `pid_status` in a `status` SSE event, or `last_status` from
`GET /api/pid`, contains `state` (`pending`, `applied`, or `rejected`), `accepted`, `result_code`,
and a human-readable `message`. Wait for `accepted: true` before treating a change as applied.

The override is held in memory only. It is not written to `pid.conf`; it is retained across Start
and Abort in the same dashboard session and re-sent after Start or when a Pi address becomes
available. Changing the selected target resets the session override to the dashboard's local
baseline. The dashboard's **Load** action has the same baseline restore behavior; it does not read
the Pi's file.

A compact Python example that preserves the complete current snapshot, changes one value, waits
for the Pi acknowledgement, pauses for feedback collection, and then restores the original values:

```python
import json
import time
from urllib.request import Request, urlopen

base = "http://host.containers.internal:8080"

def get_pid():
    with urlopen(f"{base}/api/pid", timeout=2) as response:
        return json.load(response)

def submit(values):
    request = Request(
        f"{base}/api/pid",
        data=json.dumps({"values": values}).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=2) as response:
        return json.load(response)

def wait_for_applied(request_id, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = get_pid().get("last_status")
        if status and status.get("request_id") == request_id and status.get("state") != "pending":
            if not status.get("accepted"):
                raise RuntimeError(status)
            return status
        time.sleep(0.05)
    raise TimeoutError(f"Timed out waiting for PID request {request_id}")

snapshot = get_pid()
original_values = dict(snapshot["values"])
values = dict(original_values)
values["pitch_rate_gain"] = 520.0  # make one small, intentional change
result = submit(values)
print("applied:", wait_for_applied(result["request_id"]))

input("Collect the feedback window now; press Enter to restore the original values. ")
result = submit(original_values)
print("restored:", wait_for_applied(result["request_id"]))
```

Validation and other request failures return HTTP 400 with a JSON body of the form
`{"ok": false, "error": "..."}`. If the Pi address is temporarily unavailable, the server returns
HTTP 200 with `sent: false` and keeps the request pending for a later address-ready event. In all
cases, a successful HTTP response means the snapshot was accepted or queued by the server; it does
not mean that the Pi has applied it. Wait for the matching `pid_status` acknowledgement.

## Safe experiment workflow

1. Confirm the host dashboard is running with `--listen-lan`, then query `/api/source` and open
   `/api/stream`.
2. Confirm `mode` is `live`, `run_active` is true, and `telemetry_connected` is true with a small
   `last_packet_age_ms`. Keep `pi_ready` visible as a separate reachability signal.
3. Save the original response from `GET /api/pid`. Change one complete-snapshot value at a time
   with `POST /api/pid`, using conservative increments.
4. Wait for the matching `pid_status.request_id` and `accepted: true`, then collect a bounded
   `/api/history?seconds=...` window for feedback. Record the PID snapshot, time window, run number,
   and any telemetry gaps or latched flags.
5. Restore the saved snapshot and wait for its acknowledgement before ending the experiment.

The default agent workflow is observation and PID tuning only. The intentionally omitted routes are:

| Method and route | Effect | Default agent policy |
| --- | --- | --- |
| `POST /api/deploy` | Copies the Pi binary and `pid.conf` over SSH | Do not call |
| `POST /api/start` | Starts the balancer over SSH and begins a run | Do not call |
| `POST /api/abort` | Stops the balancer over SSH | Do not call |
| `POST /api/joystick` | Sends normalized joystick commands over UDP | Do not call; neutral cleanup only |
| `POST /api/connect` | Changes the selected live target | Leave to operator |
| `POST /api/source/csv` | Replaces the live source with an upload | Leave to operator |
| `POST /api/ping` | Performs a read-only SSH connectivity probe | Leave to operator |
| `GET /api/clear-flags` | Clears latched telemetry flags | Leave to operator |
| `GET /api/deploy-info` | Reports deployment availability | Read-only |

The first four routes can deploy software, start or stop the physical balancer, or command wheel
motion. Direct HTTP access does not expose host process memory or Pi SSH keys, but it does expose
these dashboard capabilities to any client that can reach the port.

## Lifecycle and shutdown

The dashboard process owns the UDP registration, live CSV capture, heartbeat, and SSE history. A
successful live Start begins a new raw CSV capture and display run; the display history is bounded
to 120 seconds or 6000 points, while raw CSV retention is separate. Stopping the host dashboard
closes those workers and the HTTP listener; it does not replace the Pi process lifecycle.

When the experiment is finished, restore the original PID snapshot, confirm its asynchronous status,
close any SSE/curl client, and stop the host dashboard with `Ctrl-C` or the host process supervisor.
For the broader hardware bring-up and dashboard behavior, see the
[Raspberry Pi guide](../Running_on_Pi.md), and for UDP message boundaries see the
[generated protocol documentation](../ipc/protocol.md).
