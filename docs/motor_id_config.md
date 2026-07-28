# RobStride Motor ID / Zero Configuration

`motor_id_config` is a SocketCAN maintenance tool for the RobStride private
protocol. It does not enable the motor or transmit motion commands.

The implementation follows section 4.1 of `RS02User Manual 260112`:

| Operation | Communication type | 29-bit extended CAN ID |
|---|---:|---|
| Get device ID | 0 | `0x00_00_HOST_MOTOR` |
| Stop motor | 4 | `0x04_00_HOST_MOTOR` |
| Set mechanical zero | 6 | `0x06_00_HOST_MOTOR`, data byte 0 = `01` |
| Set motor CAN ID | 7 | `0x07_NEW_HOST_OLD` |
| Read mechanical position | 17 | `0x11_00_HOST_MOTOR`, index `0x7019` |

The project default host ID is `0xFD`. Motor IDs are limited to `0..127`, as
specified by the manual's `CAN_ID` parameter table.

## Build

On Ubuntu 22.04 or another Linux system with SocketCAN headers:

```bash
cmake -B build -S . -DBUILD_GUI=OFF
cmake --build build --target motor_id_config -j$(nproc)
```

Bring up the CAN interface at 1 Mbps before running the tool:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## Commands

Discover every responding ID:

```bash
./build/motor_id_config scan
```

Scan a smaller inclusive range or query one known ID:

```bash
./build/motor_id_config scan 120 127
./build/motor_id_config get 127
```

Change ID 127 to ID 1:

```bash
./build/motor_id_config set-id 127 1
```

The tool first confirms that ID 127 responds and ID 1 is unused. It then stops
the target, changes the ID, and verifies the target at its new ID. Destructive
commands prompt for confirmation; use `--yes` only in a controlled script.

Set the current shaft position as mechanical zero:

```bash
./build/motor_id_config set-zero 1
```

The target is identified by its MCU UID, stopped, zeroed, acknowledged, and its
`mechPos` parameter is read back when supported by the firmware.

To inspect the exact frames without opening or writing to a CAN interface:

```bash
./build/motor_id_config --dry-run set-id 127 1
./build/motor_id_config --dry-run set-zero 1
```

Expected default frames for those examples are:

```text
cansend can0 0400FD7F#0000000000000000
cansend can0 0701FD7F#0000000000000000

cansend can0 0400FD01#0000000000000000
cansend can0 0600FD01#0100000000000000
```

Use `--interface can1`, `--host-id 0xFD`, `--timeout 50`, or `--verbose` when
the adapter or bus setup differs. A scan that finds nothing usually indicates
motor power, CAN-H/CAN-L wiring, termination, bitrate, or protocol-mode issues.
