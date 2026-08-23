# Pinocchio URDF gravity model

`PinocchioGravityModel` parses the RAVEN URDF once at construction and
reuses the resulting Pinocchio `Model` and `Data` for gravity calculations.
It is a standalone C++ component and does not depend on `rclcpp`, topics, or
ROS executors.

## Build dependency

Pinocchio dynamics are enabled by default:

```bash
cmake -S . -B build
cmake --build build
```

CMake discovers Pinocchio through its exported target:

```cmake
find_package(pinocchio CONFIG REQUIRED)
target_link_libraries(target PUBLIC pinocchio::pinocchio)
```

For a development machine without Pinocchio, the legacy analytic gravity
model and all non-Pinocchio tests remain available:

```bash
cmake -S . -B build -DRAVEN_ENABLE_PINOCCHIO_DYNAMICS=OFF
```

## Runtime data flow

```text
RAVEN.urdf -- parsed once --> Pinocchio Model + Data
measured q ----------------> computeGeneralizedGravity()
                           --> joint-coordinate gravity torque g(q)
```

The model loads the fixed-base robot and validates these three required
joints by name:

1. `shoulder_Joint`
2. `upperArm_Joint`
3. `foreArm_Joint`

The name mapping prevents URDF storage order from being confused with RAVEN
motor order. The URDF must contain exactly three configuration and velocity
degrees of freedom for this model.

`compute()` reuses mutable Pinocchio workspace, so a model instance must only
be called by one control thread at a time.

## Model comparison

The existing analytic `GravityCompensator` remains a reference
implementation. Build tests compare both implementations at multiple poses:

```bash
ctest --test-dir build --output-on-failure
```

The comparison tool accepts the URDF path and optional joint angles in
degrees:

```bash
./build/pinocchio_gravity_model_check \
  ../RAVEN_hardware/urdf/urdf/RAVEN.urdf

./build/pinocchio_gravity_model_check \
  ../RAVEN_hardware/urdf/urdf/RAVEN.urdf 0 45 45
```

`RAVEN_URDF_PATH` is a CMake cache path used only to locate the URDF for
automated tests. Runtime motion applications load `urdf_path` from the
top-level `gravity_compensation` section of `motor_config.yaml`.

## Runtime integration

`coordinated_motion_demo`, `scripted_motion_demo`, and
`teach_and_play_demo` construct the model once and use measured Type 2 joint
positions on every control cycle. `GravityFeedforwardController` applies the
configured scale, enable ramp, and per-joint torque limits. In dry-run mode it
records the calculated values but commands zero feedforward torque.

The controller output remains in the joint coordinate system. The existing
`MotorDriver::sendMitCommand()` path performs the virtual-work-consistent
conversion

```text
motor torque = position_sign * joint torque / joint_to_motor_ratio
```

before the RS02 MIT command is packed and sent. No application-local sign
table is used.

For commissioning, retain `dry_run: true` until measured joint angles,
Pinocchio torque signs, URDF inertial parameters, and the configured joint
limits have been checked on the target mechanism. Then start live testing
with a small scale and supported links.
