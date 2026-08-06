# RAVEN Teach and Play Demo

`teach_and_play_demo` is a one-shot, in-memory keyframe demonstration. Every
run starts with an empty pose list. It records calibrated joint positions while
all motors remain disabled, then replays them with rest-to-rest quintic
segments. No motion YAML is read or written.

ROS 2 is not used for control. The separate read-only
`raven_joint_state_bridge` can run in parallel for RViz visualization.

## Build and run

```bash
cmake -B build -S . -DBUILD_GUI=OFF
cmake --build build --target teach_and_play_demo -j$(nproc)
sudo ./build/teach_and_play_demo
```

The application always uses `can0`, `config/joint_limits.yaml`, and
`config/motor_config.yaml`.

## Teach mode

- Motors remain disabled and Type 17 mechanical positions are polled.
- `SPACE` captures the current calibrated three-joint pose in memory.
- `X` or Backspace deletes the most recent pose.
- `C` clears every captured pose.
- `ENTER` finishes teaching after at least one pose has been captured.
- `Q` cancels and discards the in-memory recording.

Support the mechanism while moving it by hand. If the disabled robot cannot be
backdriven safely, do not use passive Teach mode; use a separately reviewed
active teaching design instead.

## Playback

Before enabling, the app checks every taught pose, every quintic segment, peak
joint velocity, and every controlled-stop envelope against the configured
limits. A requested 3.0 second transition is extended automatically when the
configured `max_slew_rate_rad_s` requires more time.

- `SPACE` at preflight captures the execution-time start pose, enables, and
  begins playback.
- Each taught pose is held for 0.7 seconds.
- `Q` during motion performs a 1.0 second controlled stop and holds the
  resulting pose.
- Natural completion returns to the execution-time start pose.
- `G` ramps gravity compensation on or off using the existing policy.
- Final Hold continues until `SPACE` disables all motors and exits.

Type 2 timeout keeps the existing latched Feedback Hold policy. Hard limits,
motor faults, and CAN failures keep the existing `MotorDriver` safety behavior.
Playback CSV files are still written to
`logs/teach_and_play_one_shot_<timestamp>.csv`, including available Type 17
bus-voltage telemetry. The taught keyframes themselves are never persisted.

## Faster scripted presentation

The independent scripted demonstration remains available:

```bash
sudo ./build/scripted_motion_demo config/demos/presentation_fast.yaml
```
