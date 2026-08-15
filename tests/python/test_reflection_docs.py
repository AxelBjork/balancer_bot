from pathlib import Path


def test_protocol_documents_nested_struct_formats():
    protocol = (Path(__file__).parents[2] / "doc" / "ipc" / "protocol.md").read_text(
        encoding="utf-8"
    )

    sim_start = protocol.index("### `MsgId::SimStartRun`")
    disturbance = protocol.index("#### Sub-struct: `SimDisturbancePayload`", sim_start)
    joy_segments = protocol.index("#### Sub-struct: `SimJoySegmentPayload`", disturbance)
    assert disturbance < joy_segments
    assert "| `kind` | `uint8_t` | `int` | 1 | 0 |  |" in protocol[disturbance:joy_segments]
    assert "| `force_n_end` | `double` | `float` | 8 | 40 |  |" in protocol[disturbance:joy_segments]

    pid_override = protocol.index("### `MsgId::PidConfigOverride`")
    pid_values = protocol.index("#### Sub-struct: `ConfigPidValuesPayload`", pid_override)
    assert "| `drive_max_acceleration_mps2` | `double` | `float` | 8 | 0 |  |" in protocol[pid_values:]
    assert "| `pitch_gain` | `double` | `float` | 8 | 64 |  |" in protocol[pid_values:]


def test_protocol_keeps_full_udp_bridge_description():
    protocol = (Path(__file__).parents[2] / "doc" / "ipc" / "protocol.md").read_text(
        encoding="utf-8"
    )

    assert (
        "The simulator scenario service uses a separate UDP endpoint on port `9001` and is not "
        "this bridge."
    ) in protocol
