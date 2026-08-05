# RAVEN joint characterization

`joint_characterization` is a Linux/SocketCAN hardware test runner for the
current three-axis stabilization baseline. It uses the same motor calibration,
hard/soft limits, feedback watchdog, gravity model, and CSV logger as the
control demos. It does not modify `config/motor_config.yaml`.

## Safety behavior

- Every configured limit must be confirmed before enable.
- Every target must remain inside the soft limits plus the test-plan clearance.
- Quintic and constant-velocity plans are rejected if they exceed the joint's
  configured slew rate.
- A step test may move exactly one joint and is capped by `max_step_deg`.
- RS02 faults, stale feedback, CAN errors, `Q`, Ctrl-C, and SIGTERM terminate
  the test and send stop commands to every motor.
- RS02 bus voltage parameter `0x701C` is requested throughout the run and
  written to the CSV. This is monitoring, not an energy-absorption mechanism.

The YAML plans are starting templates. Check fixture clearance, payload, cable
routing, and the power-supply settings before every run.

## Build on Linux

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Included plans

| Purpose | Plan |
| --- | --- |
| Quasi-static gravity torque map | `config/tests/gravity_identification.yaml` |
| J0 friction | `config/tests/friction_identification_j0.yaml` |
| J1 friction | `config/tests/friction_identification.yaml` |
| J2 friction | `config/tests/friction_identification_j2.yaml` |
| J0 gain steps | `config/tests/joint_gain_tuning_j0.yaml` |
| J1 gain steps | `config/tests/joint_gain_tuning.yaml` |
| J2 gain steps | `config/tests/joint_gain_tuning_j2.yaml` |
| Gravity compensation comparison | `config/tests/gravity_validation.yaml` |
| Ten-cycle repeatability run | `config/tests/repeatability_acceptance.yaml` |

Run one plan first into the ignored scratch-log directory:

```sh
RAVEN_GIT_COMMIT="$(git rev-parse HEAD)" \
  ./build/joint_characterization \
  config/tests/gravity_identification.yaml
```

The program shows the current pose and waits for Space before enabling. A run
creates the following bundle:

```text
logs/characterization/20260806_120000_gravity_identification/
├── metadata.yaml
├── run_status.yaml
├── test_plan.yaml
├── joint_limits.yaml
├── motor_config.yaml
├── notes.md
└── raw.csv
```

Use a tracked output root only for a run that should be transferred through
Git:

```sh
RAVEN_GIT_COMMIT="$(git rev-parse HEAD)" \
  ./build/joint_characterization \
  config/tests/gravity_identification.yaml \
  tests/results
```

Do not use `sudo` unless the entire build and repository are intentionally
owned by root. Give the Linux user permission to use SocketCAN instead.

## Create the summary

After the run, generate `summary.yaml` with the standard-library analysis tool:

```sh
python3 tools/analyze_characterization.py \
  tests/results/<run-directory> \
  --max-vbus-v 35 \
  --max-temperature-c 60 \
  --max-deadline-miss-percent 1
```

Position-error thresholds can be supplied after a baseline has been measured:

```sh
python3 tools/analyze_characterization.py \
  tests/results/<run-directory> \
  --max-rms-error-deg 0.5 \
  --max-peak-error-deg 2.0
```

The analyzer reports per-analysis-phase tracking error, drift, velocity, and
measured torque, plus global VBUS, temperature, fault, feedback-validity, and
control-loop timing metrics.

## Git handoff

Commit the complete run directory without changing its contents. The plan and
configuration snapshots are required even when the source commit is recorded.
On macOS, pull the commit and provide the run directory under `tests/results/`
for fitting, plots, gain selection, and acceptance review.

