# Pan Tilt MuJoCo

MuJoCo models of the Pan Tilt mechanism (`pt100`, `pt101`) with Orbbec Gemini 2 camera geometry by default, or the supported OAK-D S2 alternative, generated from the URDF in `pt_description`. The package gives you three ways to use them:

- **Standalone:** a native viewer, a model builder and a benchmark, with no ROS needed.
- **ROS simulation:** `sim:=true` runs the same controllers and teleop as the real pan-tilt against the model in `mujoco_ros2_control`.
- **Payload for another robot:** hosts such as [lekiwi_ros2](https://github.com/adityakamath/lekiwi_ros2) attach the model to their own.

## Contents

| Path | Purpose |
|------|---------|
| `pt_mujoco/build_mujoco_models.py` | Builds the model from the URDF, and `build_payload_spec` for host robots |
| `pt_mujoco/mujoco_parameters.py` | Applies the servo profile from `config/mujoco.yaml` to the model |
| `pt_mujoco/simulation.py` | The simulation without ROS: reset, command, step |
| `pt_mujoco/mujoco_preview.py`, `benchmark_mujoco.py` | Native viewer and step-response benchmark |
| `config/` | Physics and servo profile, ROS plugin configuration, depth-to-scan slice |
| `mjcf/` | MJCF sources, the `scenes/` floor and four pre-built body/camera combinations |
| `mjcf/oakd_s2_subtree.xml`, `mjcf/gemini2_subtree.xml` | Camera-specific tilt assemblies selected by `camera_config` |

## Requirements

- Python packages, installed into the interpreter that ROS and colcon use: `pip install -r requirements.txt` (`mujoco`, `numpy`, `xacro`, `PyYAML`). Add `pytest<8` to run the tests.
- `pt_description` (URDF and meshes) and `pt_control` (configuration files) from this repository.
- For the ROS simulation only, on Kilted:
  - `sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins ros-kilted-image-transport-plugins` (0.1.2 or newer; older releases have no camera plugin)
  - [mujoco_ros2_plugins](https://github.com/adityakamath/mujoco_ros2_plugins), cloned into the same workspace. It provides the simulated `/emergency_stop`, and launch fails if it is missing.

## Running

### Standalone

Run these from this directory (`mjpython` instead of `python3` on macOS for the viewer):

```sh
python3 -m pt_mujoco.mujoco_preview --variant pt101      # native viewer
python3 -m pt_mujoco.build_mujoco_models --variant pt101 --output /tmp/pt.xml --absolute
python3 -m pt_mujoco.benchmark_mujoco --output step_response.json
```

In the viewer, click the window first, then use Left/Right for pan, Up/Down for tilt, Space to toggle the emergency stop (disables torque on both motors, matching `mujoco_ros2_plugins/EmergencyStopPlugin`'s real behavior - the pan-tilt drifts freely rather than holding), X to reset and P to pause. The same tools are installed as commands (`ros2 run pt_mujoco <tool>` or `pip install -e .`).

Always build models with `build_mujoco_models` rather than plain xacro: it takes frames, mesh origins, inertias and joint limits from the URDF. `--scene` selects the environment (`flat`, `none` or a scene file). With no arguments it regenerates the committed `mjcf/pt100_gemini2.xml` and `mjcf/pt101_gemini2.xml`, for the default camera. Also run with `--camera oakd_s2` after any change to the URDF, config or MJCF to refresh all four models.

From Python:

```python
import mujoco
from pt_mujoco.build_mujoco_models import build
from pt_mujoco.simulation import Simulation

sim = Simulation(mujoco.MjModel.from_xml_path(str(build("pt101", "/tmp/pt.xml", absolute=True))))
sim.reset()
sim.command(0.5, -0.3)     # pan, tilt targets in radians
sim.step(500)
print(sim.positions())
```

### ROS simulation

Build the workspace as described in the [repository README](../README.md#installation), then:

```sh
ros2 launch pt_bringup pantilt.launch.py sim:=true                    # headless, no display needed
ros2 launch pt_bringup pantilt.launch.py sim:=true mujoco_gui:=true   # with the MuJoCo viewer
```

`pantilt_config` (`pt100` or `pt101`), `camera_config` (`oakd_s2` or `gemini2`) and `mujoco_scene` choose the model. To watch a headless run from another machine, start `foxglove_bridge` and connect Foxglove to `ws://<host>:8765`. Command it as on the real pan-tilt, with the joystick or:

```sh
ros2 topic pub /pantilt_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3]}"
```

| Topic or service | What it is |
|---|---|
| `/joint_states`, `/pantilt_controller/commands` | Same as the real robot, from ros2_control on the simulated motors |
| `/gemini2/color/image_raw`, `/gemini2/color/camera_info` | Gemini 2 camera in frame `gemini2_color_optical_frame`, a nominal 640×360 RGB view without measured Gemini intrinsics |
| `/gemini2/color/image_raw/compressed` | Compressed version of the RGB image |
| `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/rgb/camera_info`, `/oak/rgb/image_raw/compressed` | OAK-D S2 camera in frame `oak_rgb_camera_optical_frame` (`--camera oakd_s2`) |
| `/oak/scan` | OAK-D S2 only: laser scan sliced from the depth image, as on the real bringup |
| `/emergency_stop` (`std_srvs/SetBool`) | While enabled, torque is disabled on both motors and commands are ignored, matching the real robot's `sts_hardware_interface`; the pan-tilt drifts freely rather than holding position; releasing it hands control back |

The standalone MuJoCo camera plugin requests 30 Hz for both camera geometries; real camera bringup defaults to 15 Hz. A previous headless Raspberry Pi 5 run delivered about 19 Hz while the simulation stayed in real time; actual throughput depends on the scene and host.

## Configuration

| File | What it sets |
|---|---|
| `config/mujoco.yaml` | Timestep, integrator and solver settings, and the pan and tilt servo profile (gain, damping, armature, friction) |
| `config/mujoco_ros2_control_plugins.yaml` | The ROS plugins shared by both cameras: the camera plugin's rate and the emergency stop |
| `config/mujoco_camera_gemini2.yaml`, `config/mujoco_camera_oakd_s2.yaml` | The selected camera's render camera (`gemini2_rgb` / `oak_rgb`), topics and frame |
| `config/mujoco_depth_to_scan.yaml` | Range and height of the `/oak/scan` slice |
| `mjcf/` | The MJCF model sources (entry file, pan-tilt body, servo defaults, camera) and the `scenes/` floor. Frames, inertias and limits in them are overwritten from the URDF at build time |

Changes to `config/mujoco.yaml` are applied when the model is built, so rebuild (or relaunch) to see them. The servo profile was identified from a real STS3215 servo; change the gains there if you want a different response.

If the package cannot find its files, point it at them with `PT_MUJOCO_SHARE`, `PT_DESCRIPTION_SHARE` or `PT_CONTROL_SHARE`.

## Limitations

Only the servo profile is measured (a real STS3215 at 12 V). Inertias, the torque limit and everything else are approximations, and there is no torque-speed curve, backlash or sensor noise. A step response settles in about 0.3 s with 4% (pan) to 8% (tilt) overshoot. The camera renders headless; the MuJoCo viewer (`mujoco_gui:=true` and `mujoco_preview`) needs a display.

## Using it on another robot

```python
from pt_mujoco.build_mujoco_models import build_payload_spec

payload = build_payload_spec(variant, urdf, joint_limits, description_dir)
payload.default.name = 'pt_payload'
frame = host.body('base_link').add_frame(name='pantilt_mount')   # placed from the host's URDF
host.attach(payload, prefix='', frame=frame)
```

`urdf` is the host's own expanded URDF, which must contain the pan-tilt, so the host stays the source of truth for where the pan-tilt is mounted. `joint_limits` is an optional dictionary of joint limit overrides (`{}` to use the URDF's limits). Everything else, meaning inertias, torque limits, servo parameters and the camera with its orientation, comes from this package. lekiwi_mujoco does exactly this. A host also loads `config/mujoco_ros2_control_plugins.yaml` for the camera plugin, and may lower the camera rate with `camera_publish_rate`.

## Tests

```sh
pytest pt_mujoco/test -q       # about 15 seconds
```

The launch arguments are covered by `pt_control/test/test_launch.py`.

## Gemini 2 models

All standalone tools default to Gemini 2 and accept `--camera oakd_s2` for the supported alternative. Explicit Gemini 2 examples:

```sh
python3 -m pt_mujoco.mujoco_preview --variant pt101 --camera gemini2
python3 -m pt_mujoco.build_mujoco_models --camera gemini2
python3 -m pt_mujoco.benchmark_mujoco --camera gemini2 --output gemini_steps.json
ros2 launch pt_bringup pantilt.launch.py sim:=true camera_config:=gemini2
```

Regenerate both camera sets after geometry changes:

```sh
python3 -m pt_mujoco.build_mujoco_models --camera oakd_s2
python3 -m pt_mujoco.build_mujoco_models --camera gemini2
```

This produces `mjcf/pt{100,101}_{oakd_s2,gemini2}.xml` with portable mesh paths.
Python callers default to `camera_config="gemini2"` and can pass `camera_config="oakd_s2"` to `build`, `build_spec`, or
`build_robot_spec`. Host payload construction derives the mesh choice from its URDF.
The Gemini camera flip, seating offset and centered tilt mount come from Xacro;
the camera is charcoal and the bracket uses the shared light-grey material.

The Gemini model uses Gemini-specific names throughout (`gemini2_link`, `gemini2_link_model_origin`, the `gemini2_rgb` camera and `/gemini2/*` topics); the OAK-D S2 model keeps `oak_*`. The Gemini model has a nominal 640×360 RGB view and ideal co-located IMU sensors, not measured Gemini intrinsics or extrinsics. Camera inertia and hardware calibration remain future work.
`--model` in the viewer uses the supplied model as-is rather than applying `--camera`.

`mjcf/oakd_s2_subtree.xml` and `mjcf/gemini2_subtree.xml` define the camera-specific
tilt assemblies. The builder selects the subtree from the URDF camera mesh, then
synchronizes transforms and geometry from the URDF as before.

Real Gemini 2 bringup uses OrbbecSDK_ROS2 with native `/gemini2/*` topics and device calibration. The simulated Gemini camera uses the same `/gemini2/color/*` names; the real driver is never launched in simulation. See [real camera bringup](../README.md#gemini-2-camera).


The Gemini 2 generated model uses a nominal 640×360, 55° vertical RGB view
(approximately 86° horizontal), upright at the home pose, plus co-located
`gemini2_accelerometer` and `gemini2_gyroscope` sensors. These ideal sensor
extrinsics are not hardware calibration. LeKiwi bringup configures the SDK-style
RGB/depth, pointcloud and synchronized IMU publishers; the renderer's internal
camera is named `gemini2_rgb`.
