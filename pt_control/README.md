# pt_control

The `ros2_control` setup for the Pan Tilt 100: controllers, configuration, joystick teleop and the launch file that brings them up on real hardware, in mock mode, or in the MuJoCo simulation.

## Contents

| Path | Purpose |
|------|---------|
| `launch/pantilt.launch.py` | The control stack: `robot_state_publisher`, the controller manager, the two controllers, joystick teleop, optional motor diagnostics, and the extra nodes the simulation needs |
| `launch/teleop.launch.py` | `joy_teleop` on its own |
| `config/urdf_config.yaml` | Serial port, baud rate, mock mode, servo speed profile: the values passed to the URDF |
| `config/pantilt_config.yaml` | Controller manager (50 Hz) and the joint state broadcaster |
| `config/pantilt_controller.yaml` | The position controller: its type, joints and interface |
| `config/teleop_config.yaml` | Joystick buttons and axes |

## Running

```bash
ros2 launch pt_control pantilt.launch.py                        # real hardware
ros2 launch pt_control pantilt.launch.py use_mock:=true         # simulated motors, no hardware
ros2 launch pt_control pantilt.launch.py ros2_control_hardware_type:=mujoco   # MuJoCo (normally started through pt_bringup with sim:=true)
ros2 launch pt_control teleop.launch.py                         # joystick teleop alone
```

This is the dedicated-bus setup: it runs its own controller manager on its own serial port. To run it with the camera, use `pt_bringup`. The `joy` node is not started here; run `ros2 run joy joy_node` first.

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `sts_serial_port` | `""` | Serial port; empty uses `urdf_config.yaml` |
| `use_mock` | `""` | `true` or `false`; empty uses `urdf_config.yaml` |
| `diagnostics` | `false` | Also start the motor diagnostics node (`/base/diagnostics`) |
| `pantilt_config` | `pt101` | Mesh variant, `pt100` or `pt101` |
| `use_sim_time` | `false` | Use `/clock` instead of system time |
| `ros2_control_hardware_type` | `real` | `real` or `mujoco` (`gazebo` exists in the xacro but is not wired into this launch file) |
| `mujoco_scene` | `flat` | `mujoco` only: `flat`, `none` or a scene file |
| `mujoco_model` | `""` | `mujoco` only: a pre-built MJCF; empty generates one at launch time with `pt_mujoco` |
| `mujoco_headless` | `false` | `mujoco` only: skip the MuJoCo viewer window |

## Configuration

**`urdf_config.yaml`** holds the connection settings (`serial_port`, `baud_rate`, `use_mock`, `use_sync_write`, `sts3215_max_vel_steps`, `proportional_vel_max`) and the servo speed profile (`internal_max_vel`, `internal_max_acc`, `internal_acc_coeff`). Only `sts_serial_port` and `use_mock` can also be set on the command line; edit the file for the rest. The default profile, 65 / 50 / 0, is slow and smooth, and hosts that share a bus (such as lekiwi_ros2) read these values too. Motor IDs, centres and joint limits are not here: they are calibration constants in [`pt_description`](../pt_description/README.md#calibration-and-limits).

**`pantilt_controller.yaml`** defines `pantilt_controller`, a `ForwardCommandController` on the `position` interface for `shoulder_pan_joint` and `tilt_joint`. The launch file passes it to the spawner with `--param-file`, so a host robot that runs its own controller manager can use the same file without merging it into its parameters. Position limits come from the URDF.

**`pantilt_config.yaml`** sets the controller manager update rate (50 Hz) and the joint state broadcaster's joints. The broadcaster publishes standard joint states to `/joint_states` and the extra motor data (voltage, temperature, current, moving flag) to `/dynamic_joint_states`.

**`teleop_config.yaml`** maps the joystick:

| Input | Action |
|-------|--------|
| L1 (button 9), held | Deadman: commands are sent only while it is held |
| Left stick X / Y | Pan / tilt to an absolute position; full deflection is ±π/2 rad |
| A (button 0) | Call `/emergency_stop` with `true` |
| B (button 1) | Call `/emergency_stop` with `false` |

The sticks set absolute positions, not speeds. The velocity of the motion comes from the servo's own profile.

## How it works

The launch file expands the URDF from `pt_description` with the values in `urdf_config.yaml`, then starts everything against it. The controllers are started a couple of seconds after the controller manager, and the motor diagnostics after those. The controller only forwards each commanded position to the hardware interface, which turns it into motor steps; the acceleration and speed profile is handled by the servo firmware.

In `mujoco` mode the launch file starts `mujoco_ros2_control`'s own `ros2_control_node`, which runs the simulation in the same process, instead of the standard controller manager. It first builds the MuJoCo model from the URDF with `pt_mujoco`, then loads that package's camera and emergency-stop plugin configuration, and adds three helper nodes: a compressed-image republisher for `/oak/rgb/image_raw`, the static transform from `oak_link` to the camera's optical frame, and `depthimage_to_laserscan` for `/oak/scan`. See the [`pt_mujoco` README](../pt_mujoco/README.md).

## Using it from another robot

`pt_bringup` and this launch file run their own controller manager, so a robot that shares the servo bus does not include them. It runs one controller manager for everything and reuses this package's controller file, servo profile and teleop mapping. [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) does this; see the [repository README](../README.md#using-it-on-another-robot).

## Tests

```bash
pytest test -q
```

The tests check the launch arguments (including the simulation ones), that the simulation configuration is loaded from `pt_mujoco`, and that the servo profile in `urdf_config.yaml` is within range, matches the URDF defaults and is passed to the URDF.
