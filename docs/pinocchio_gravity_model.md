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
automated tests. Runtime applications should pass the URDF path to
`PinocchioGravityModel` from their own command-line or configuration layer.

## Current integration boundary

This implementation produces joint-coordinate `g(q)`. It intentionally does
not perform motor sign conversion, ramping, torque limiting, MIT command
packing, or CAN transmission. The motor feedforward path should switch to
this model only after the analytic comparison passes on the target Ubuntu
system and joint/motor torque signs are validated with supported hardware.
