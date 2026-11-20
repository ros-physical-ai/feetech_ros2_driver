# User Guide

> **⚠️ Migration notice:** The `offset` parameter is deprecated and ignored. The driver now always centers at 2048 (midpoint of the 0–4095 tick range). Use `homing_offset` instead — it writes directly to the servo's EEPROM so the centering happens in hardware. To migrate: `homing_offset = old_offset - 2048`. The driver will log a warning if it detects the old `offset` parameter.

## ros2_control urdf tag

The feetech system interface has a few `ros2_control` urdf tags to customize its behavior.

#### Hardware Parameters

* `usb_port` (required). Example: `<param name="usb_port">/dev/ttyUSB0</param>`.
* `joint_config_file` (optional): Path to a YAML file with per-joint parameters. If omitted, only URDF params are used (backward-compatible). See [YAML Joint Configuration](#yaml-joint-configuration-file) below.

#### Per-joint Parameters

Make sure to look at [Memory table](https://docs.google.com/spreadsheets/d/1GVs7W1VS1PqdhA1nW-abeyAHhTUxKUdR/edit?gid=364516031#gid=364516031) for a detailed explanation of the parameters.

* `id` (**required**, must be in URDF): Servo ID on the bus. The driver uses this to address the servo and to match YAML config entries. Example: `<param name="id">1</param>`.
* `p_coefficient` (optional): Proportional coefficient of the PID controller. Example: `<param name="p_coefficient">8</param>`.
* `i_coefficient` (optional): Integral coefficient of the PID controller. Example: `<param name="i_coefficient">0</param>`.
* `d_coefficient` (optional): Derivative coefficient of the PID controller. Example: `<param name="d_coefficient">32</param>`.
* `homing_offset` (optional): Signed offset written to the servo's EEPROM. The servo firmware applies `Present_Position = Actual_Position - Homing_Offset`, so setting `homing_offset = actual_position - 2048` makes the servo report 2048 (center) at your desired physical center. If migrating from the old `offset` parameter: `homing_offset = old_offset - 2048` (since the old offset was effectively the actual position at center).
* `range_min` (optional): Minimum angle limit (raw ticks, after homing offset is applied).
* `range_max` (optional): Maximum angle limit (raw ticks, after homing offset is applied).
* `max_torque_limit` (optional): Maximum torque limit.
* `protection_current` (optional): Protection current threshold.
* `overload_torque` (optional): Overload torque threshold.
* `return_delay_time` (optional): Response delay time.
* `acceleration` (optional): Acceleration value.

### Example

Take a look at [ros2_so_arm100](https://github.com/JafarAbdi/ros2_so_arm100/blob/main/so_arm100_description/control/so_arm100.ros2_control.xacro) for an example of how to use the URDF tags.

---

## YAML Joint Configuration File

As an alternative (or addition) to URDF `<param>` tags, joint parameters can be loaded from a YAML file. This is useful for calibration values that change between robots (like `homing_offset`) without modifying the URDF.

The `id` parameter **must** be defined in the URDF (it is the hardware identity used to address the servo on the bus). YAML entries are matched to URDF joints by servo `id`, not by joint name — so the YAML joint name is just a human-friendly label. When both URDF params and YAML are provided for the same `id`, YAML values take precedence.

### Format

```yaml
joints:
  joint_name:
    id: 1
    homing_offset: 530
    range_min: 866
    range_max: 3231
    p_coefficient: 16
    i_coefficient: 0
    d_coefficient: 32
    return_delay_time: 0
    acceleration: 254
```

### URDF Integration

Pass the YAML file path as a hardware parameter:

```xml
<param name="joint_config_file">$(find my_robot_bringup)/config/joints.yaml</param>
```

### Examples

* URDF-only setup: [ros2_so_arm100](https://github.com/JafarAbdi/ros2_so_arm100/blob/main/so_arm100_description/control/so_arm100.ros2_control.xacro)
* YAML config setup: [so101-ros-physical-ai](https://github.com/legalaspro/so101-ros-physical-ai) — see [follower](https://github.com/legalaspro/so101-ros-physical-ai/blob/main/so101_bringup/config/hardware/follower_joints.yaml) and [leader](https://github.com/legalaspro/so101-ros-physical-ai/blob/main/so101_bringup/config/hardware/leader_joints.yaml) arm configs.

## SCS0009 position system (Jazzy and newer)

Select `feetech_ros2_driver/Scs0009System` for a bus of SCS0009 motors.
These servos use big-endian words and a 10-bit encoder with a 300-degree span.
The hardware parameters are `port`, `baudrate`, `serial_timeout` (seconds),
`goal_speed` (rad/s), `motor_min_deg`, `motor_max_deg` (relative to the
calibration), and `disable_torque_on_shutdown` (true or false). Supply all
seven. Each joint takes an `id`, a `middle_deg` calibration offset, a bounded
position command interface and position, velocity and effort state interfaces.
Velocity is estimated from position samples. Effort is reported as zero.

Uncoupled joints use calibrated motor radians. Optional ros2_control
DifferentialTransmission entries use the declared joint and actuator order.
Each joint's id identifies the actuator at the same index in its transmission.
On activation the plugin reads all positions, writes the measured hold targets
and the configured speed, then enables and checks torque. A serial fault
returns a hardware error and attempts to disable torque on the entire bus.
