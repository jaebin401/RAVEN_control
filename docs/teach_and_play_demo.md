# RAVEN Teach and Play Demo

`teach_and_play_demo` records calibrated joint-space keyframes while all motors
remain disabled, saves them as YAML, and replays them with rest-to-rest quintic
segments. ROS 2 is not used for control; the separate read-only
`raven_joint_state_bridge` can run in parallel for RViz visualization.

## Build

```bash
cmake -B build -S . -DBUILD_GUI=OFF
cmake --build build --target teach_and_play_demo -j$(nproc)
```

## Create a recording

Runtime recordings are ignored by Git. Copy the versioned template first:

```bash
mkdir -p recordings
cp config/demos/teach_and_play_template.yaml recordings/my_demo.yaml
sudo ./build/teach_and_play_demo recordings/my_demo.yaml
```

When an existing YAML contains poses, `P` plays it and `T` replaces it with a
new teaching session. An empty or new YAML enters Teach mode directly.

### Teach mode

- Motors remain disabled and Type 17 mechanical positions are polled.
- `SPACE` captures the current calibrated three-joint pose.
- `X` or Backspace deletes the most recent pose.
- `ENTER` saves at least one pose and advances to playback preflight.
- `Q` cancels without changing the existing YAML.

Support the mechanism while moving it by hand. If the disabled robot cannot be
backdriven safely, do not use passive Teach mode; use a separately reviewed
active teaching design instead.

### Playback

Before enabling, the app checks every taught pose, every quintic segment, peak
joint velocity, and the controlled-stop envelope against the configured limits.
Durations that are too short are extended automatically. The current pose at
playback approval is the return pose.

- `SPACE` at preflight captures the start pose, enables, and begins playback.
- `Q` during motion performs a controlled stop and holds the resulting pose.
- Natural completion returns to the execution-time start pose.
- `G` ramps gravity compensation on or off using the existing policy.
- Final Hold continues until `SPACE` disables all motors and exits.

Type 2 timeout keeps the existing latched Feedback Hold policy. Hard limits,
motor faults, and CAN failures keep the existing `MotorDriver` safety behavior.
Playback CSV files are written to `logs/teach_and_play_<name>_<timestamp>.csv`,
including the available Type 17 bus-voltage telemetry.

## Faster scripted presentation

The additional scripted sequence ends at calibrated joint zero and uses shorter
quintic transitions without exceeding the default 0.524 rad/s slew limit:

```bash
sudo ./build/scripted_motion_demo config/demos/presentation_fast.yaml
```
