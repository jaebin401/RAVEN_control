# Gravity feedforward validation

`gravity_feedforward_validation` is a Linux/SocketCAN commissioning app for
J1 (`upperArm_Joint`). It never generates an automatic position trajectory.
J1 is commanded with Kp=Kd=0 while J0 and J2 retain their configured PD gains
to keep the test geometry fixed.

This program is not a general demo. It exists to preserve evidence for the
complete path from URDF gravity calculation to the torque field transmitted
to each RS02.

## Safety gates

The program refuses to arm unless all of the following are true:

- gravity compensation is explicitly enabled and is not in dry-run mode;
- gravity scale and every joint torque limit are positive;
- the configured ramp is at least 1000 ms;
- all joint limits are confirmed;
- J0 and J2 have a nonzero configured PD hold gain;
- torque limits do not exceed the app commissioning caps: J0 0.5 N.m,
  J1 3.0 N.m, J2 1.0 N.m;
- the captured pose is inside every soft limit;
- the initial model prediction fits inside the configured torque limits.

During a live run it stops on a motor fault, hard-limit violation, stale Type
2 feedback, feedforward torque clamp, CAN failure, bus voltage above 35 V, or
motor temperature above 60 C. These checks do not absorb regenerative energy.
The arm must remain physically supported because the current power system has
no regeneration protection and J1 intentionally has no PD torque.

## Configuration

The tracked example and local `motor_config.yaml` use safe defaults that make
the validation app refuse to arm. Prepare the local ignored configuration
deliberately. A conservative first direction check can use a partial scale;
it is not a final gravity-compensation validation.

```yaml
gravity_compensation:
  enabled: true
  dry_run: false
  scale: 0.25
  ramp_duration_ms: 3000
  max_joint_torque_nm:
    shoulder_Joint: 0.1
    upperArm_Joint: 2.0
    foreArm_Joint: 0.3
```

Verify the URDF path, joint zero offsets, position signs, torque limits, cable
clearance, and physical support before changing the scale toward 1.0. The app
does not modify the configuration.

## Build and run

Pinocchio, yaml-cpp, and Linux SocketCAN are required.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gravity_feedforward_validation -j

RAVEN_GIT_COMMIT="$(git rev-parse HEAD)" \
  ./build/gravity_feedforward_validation \
  can0 \
  config/joint_limits.yaml \
  config/motor_config.yaml \
  logs/gravity_validation
```

Before enable, the first `Space` captures a preview pose. The program prints
that pose, raw Pinocchio gravity torque, scaled joint torque,
motor-coordinate torque, exact 16-bit MIT torque field, and the joint torque
decoded from that field. A second `Space` is required after reviewing those
values. The app refuses to enable if any joint moved more than 0.1 degree
between preview and final confirmation.

Controls:

- Before enable: first `Space` captures the preview and second `Space` confirms
  it and arms once; `Q` quits.
- Live: `Space` sends stop and exits normally; `Q` aborts and sends stop.
- After `manual_ready`: move only J1 slowly, release it, then press `M` to
  record a five-second measurement window.

The sequence is fixed:

```text
gravity_ramp -> command_gate (30 s) -> manual_ready
                                      -> measurement_N (5 s)
                                      -> manual_ready
```

No state contains an automatic position transition.

## Evidence bundle

Each armed run creates a timestamped directory containing:

```text
logs/gravity_validation/<timestamp>_gravity_feedforward_j1/
├── metadata.yaml
├── run_status.yaml
├── motor_config.yaml
├── joint_limits.yaml
└── raw.csv
```

The CSV includes raw, ramped, limited and sent joint torque; motor-coordinate
torque; exact encoded torque; decoded joint torque; Kp/Kd; Type 2 measured
state; bus voltage; temperature; feedback validity; and timing. The
`feedforward_audit_valid` column distinguishes logs that populated the wire
audit fields from older motion apps that did not.
