# ROS 2 joint-state bridge

`raven_joint_state_bridge` is a read-only telemetry process. It listens for
RS02 communication Type 2 operation-feedback frames and publishes calibrated
joint coordinates as `sensor_msgs/msg/JointState` on `/joint_states`.

It never sends a CAN frame and cannot enable, stop, or command a motor. ROS 2
is not part of the motor-control or safety feedback path.

## Build

Source the installed ROS 2 distribution before configuring CMake. CMake only
creates the bridge target when `ament_cmake`, `rclcpp`, and `sensor_msgs` are
available.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cmake -S . -B build -DRAVEN_BUILD_ROS2_BRIDGE=ON
cmake --build build -j"$(nproc)"
```

If CMake was configured before ROS 2 was sourced, run the configure command
again after sourcing ROS 2.

## Run

```bash
./build/raven_joint_state_bridge --ros-args \
  -p can_interface:=can0 \
  -p motor_config_path:=config/motor_config.yaml \
  -p publish_rate_hz:=50
```

The bridge publishes only after it has received one fresh Type 2 frame from
every configured joint. When any joint becomes stale it stops publishing and
warns once per second. RViz then remains at its last valid pose.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `can_interface` | `can0` | SocketCAN interface to observe |
| `motor_config_path` | `config/motor_config.yaml` | Motor ID and joint-coordinate calibration |
| `publish_rate_hz` | `50` | `/joint_states` output rate, range 1..200 |
| `feedback_timeout_ms` | motor YAML value | Maximum age of every joint in a complete snapshot |

Do not run `joint_state_publisher_gui` or the `raven_sim_demo` publisher at the
same time as this bridge. They all publish `/joint_states`.
