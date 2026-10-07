# Adding a robot

A robot is a directory under `arena_robots/arena_robots/robots/<name>/`. The
directory name is the robot's canonical identifier: task_generator, launch
files, and the `arena feature robots` CLI all look robots up by this name.

## Required files

```
<robot>/
├── model_params.yaml    # robot-wide identity (robot_model, base_frame, z_offset, sensors)
├── control.yaml         # ros2_control configuration
├── mappings.yaml        # sim ⇄ ROS2 topic bridge
└── caps/
    └── mobile.yaml      # required: every robot has a mobile base
```

### `model_params.yaml`

Robot-wide identity. Cap data does not live here except for fields that apply
to any robot regardless of cap: `base_frame` (every robot has a base link),
`z_offset` (spawn-placement offset, defaults to 0.0), and `sensors` (hardware
sensor declarations, defaults to `[]`).

```yaml
robot_model: my_robot       # required; matches the directory name
base_frame: base_link       # required; TF frame of the robot's base link
z_offset: 0.37              # optional; metres to lift the robot above the ground plane at spawn
sensors:                    # optional; declared sensors parsed into SensorSpec entries
  - {name: lidar, type: laserscan, topic: ${namespace}/scan, frame: base_scan}
```

Parsed by [`arena_robots.Robot.ModelParams`](../arena_robots/Robot.py). Additional keys pass
through unchanged; nothing in-tree consumes them.

`sensors` also drives the Nav2 costmap sources ([`arena_robots.nav2`](../arena_robots/nav2.py)).
The local costmap raytraces planar lidars in a 2D obstacle layer and runs multi-ring lidars
and depth cameras through a spatio-temporal voxel layer, which clears inside a frustum fitted
to the sensor's field of view. Field of view and range are read from the gz `<sensor>` element
of the rendered URDF that the entry's `sensor:` key names, or its `name` when the key is absent
([`arena_robots.sensor_geometry`](../arena_robots/sensor_geometry.py)). A sensor without such
an element counts as planar when it only declares a `laserscan`. Otherwise it marks without
clearing and its obstacles expire after the layer's `voxel_decay`.

### `caps/`: capability declarations

Each `caps/<cap>.yaml` declares one capability the robot physically has. **File
presence IS the advertisement**: `caps/arm.yaml` existing means this robot
advertises `arm`. Derived at load time as `robot_view.caps.available`, a
`frozenset[str]` of cap stems.

Adapters declare `requires: frozenset[str]` via `@BringupMeta.attach(requires=frozenset({...}))` on the `Bringup` subclass; the broker gates binding on `adapter.requires ⊆ robot.caps.available`. A cap must have a documented vocabulary entry before any adapter cites it in `requires`.

#### Active cap vocabulary

| Cap | File | Shape | Meaning |
|---|---|---|---|
| `mobile` | `caps/mobile.yaml` | **flat** (singleton) | drivable chassis (wheeled / tracked / omni) |
| `arm` | `caps/arm.yaml` | **dict** (named instances) | serial manipulator with an IK-solvable tip |
| `lift` | `caps/lift.yaml` | **dict** (named instances) | prismatic vertical positioner (Ewellix column, torso lift, scissor) |

Reserved for future: `gripper`, `ptu_head`, `dual_arm`. Continuum arms either
discretize into the serial-chain shape or use a future `kind: continuum`
variant of `caps/arm.yaml`.

#### `caps/mobile.yaml`: flat, singleton

A robot has exactly one mobile base by construction. All robot-physical
primitives live at top level; adapter sub-blocks are reserved for
adapter-specific wiring only. `actions` and `laser` are top-level fields. The
`nav2:` sub-block carries only planner plugin wiring.

```yaml
odom_frame: odom
sensor_frame: base_scan
radius: 0.5
is_holonomic: false

footprint: [[0.5, 0.35], [0.5, -0.35], [-0.5, -0.35], [-0.5, 0.35]]
footprint_padding: 0.1        # optional
inflation_radius: 0.25        # optional
polygons_dict:
  StopPolygon: {type: polygon, points: [...], action_type: stop}

velocity_limits:
  linear: {min: -2.0, max: 2.0}
  angular: {min: -4.0, max: 4.0}
acceleration_limits:
  linear: 3.0
  angular: 3.5

actions:
  continuous: {linear: {min: -2.0, max: 2.0}, angular: {min: -4.0, max: 4.0}}
  discrete: [{name: move_forward, linear: 0.3, angular: 0.0}, ...]

laser: {angle: {min: -2.35619, max: 2.35619}, num_beams: 720, range: 30.0, update_rate: 10}

nav2:                      # planner plugin wiring only
  planner_plugins: [GridBased]
  planner_plugins_dict: {...}
```

`velocity_limits` and `acceleration_limits` flow into nav2 controller configs
via `Nav2KinematicsDerivedYAML` substitution; controller YAMLs reference them
as `${max_linear_vel:-default}` etc. To override a specific controller without
touching `mobile.yaml`, replace the substitution with a literal value in that
controller YAML.

#### `caps/arm.yaml`, `caps/lift.yaml`, `caps/gripper.yaml`: dict-keyed

Always a dict of named instances. Single-arm robots use a dict with one entry
(by convention named `arm`); dual-arm uses two entries (e.g. `left`, `right`).
No single-instance shorthand, uniform shape keeps the loader trivial.

```yaml
# caps/arm.yaml (single-arm)
arm:
  base_link: chassis_link
  tip_link: arm_tool0
  chain: [arm_shoulder_pan_joint, arm_shoulder_lift_joint, arm_elbow_joint,
          arm_wrist_1_joint, arm_wrist_2_joint, arm_wrist_3_joint]
  controller: arm_controller
  moveit:                  # adapter-specific; consumed by the moveit adapter
    package: ur_moveit_config
    args: {ur_type: ur5e, safety_limits: true}
```

For platforms that ship an upstream SRDF, structural primitives can be derived
from it; `controller:` and adapter sub-blocks stay authored:

```yaml
arm:
  srdf: $(find ur_moveit_config)/srdf/ur.srdf.xacro
  controller: arm_controller
  moveit: {package: ur_moveit_config, args: {ur_type: ur5e}}
```

Loader precedence per field: explicit > SRDF-derived > raise. No URDF-alone
derivation (structural-without-semantics is ambiguous).

`caps/lift.yaml`:

```yaml
lift:
  joint: robot_lift_ewellix_lift_top_joint
  controller: lift_controller
```

`caps/gripper.yaml` (when a platform gets one) uses a back-reference to its
arm:

```yaml
gripper:
  arm: arm                 # key into caps/arm.yaml
  joint: arm_gripper_joint
  controller: arm_gripper_controller
```

#### Adapter sub-blocks are namespaced, not dispatch

`moveit:`, `nav2:`, `rl:`, `drl_grasp:`, ... inside a cap file are pure data
organization. **No in-file handler declaration**: adapters are selected at
runtime (CLI / setup YAML) and are handed the cap YAML, reading whichever
primitives + adapter-sub-block they know about.

### `control.yaml`

ROS 2 `controller_manager` configuration. Define `joint_state_broadcaster`
plus whichever controller drives the robot (e.g.
`diff_drive_controller/DiffDriveController` for wheeled, `JointTrajectoryController`
for an arm). See [`husky/control.yaml`](husky/control.yaml) for a diff-drive
example and [`rbkairos_plus/control.yaml`](rbkairos_plus/control.yaml) for an
arm-extended example.

### `mappings.yaml`

Simulator ⇄ ROS2 topic bridge declarations, as a JSON array. Each entry:

```yaml
{
  "gz_topic":  "/model/{robot_name}/cmd_vel",    # simulator-side topic
  "ros_topic": "cmd_vel",                         # ROS2-side topic
  "gz_type":   "gz.msgs.Twist",
  "ros_type":  "geometry_msgs/msg/Twist",
  "direction": "]",                               # "[" sim→ros, "]" ros→sim
}
```

`{robot_name}` and `{world}` are substituted at runtime. See
[`husky/mappings.yaml`](husky/mappings.yaml).

Hand-authored rows carry only what the catalog cannot derive: `pose`, ros2_control
plumbing (`odometry`/`cmd_vel`/`joint_states`), and `contact` sensors. Bridge rows for
catalog-derivable sensor types (laserscan, pointcloud, imu, image, depth, camera_info)
are generated at runtime from `effective_sensors` and deduped against this file, so
never duplicate them here.

### `assembly.yaml`: component-catalog robots only

Sibling of `model_params.yaml` (deliberately not inside it: the existing scalar
`priority:` field would collide). Its presence declares the robot's sensors are sourced
from the component catalog: sensors come from `components/` via mounts, and the
chassis xacro carries no sensor invocations.

```yaml
prefix: ""            # frame-prefix convention; omit for the Robotnik "robot_" default
mounts:
  front_laser:        # mount names adopt the chassis xacro's own frame names
    parent: chassis_link
    xyz: [0.53, 0.33, 0.1145]     # verbatim from the chassis xacro's sensor invocation
    rpy: [3.141592653589793, 0, 0.7853981633974483]
    accepts: [lidar]              # a set: a mount may accept more than one part type
defaults:
  lidar:
    - variant: sick_s300
      mount: front_laser
      overrides: {name: lidar_rear, topic: scan/rear}   # per-instance tuning lives HERE
```

`accepts` is a set, not a scalar: a versatile mounting point (e.g. a top plate) may
accept several part types. When more than one mount accepts the same type, allocation
preference among them follows mount DECLARATION ORDER in this file, first-declared
wins. There is no separate `priority:` field; reorder the `mounts:` block itself to
change preference.

Requests (`robot:=name[lidar=x,...]`) resolve against this file with replace-on-touch
semantics; see the grammar section in
`task_generator/task_generator/manager/README.md`.

A sensor that is physically present but deliberately left unbridged (e.g. a GPS/navsat
mast) stays inline in the robot's own xacro; it gets no mount or component entry here.

### Moving a robot's sensors to the component catalog

1. **Golden capture first**: render the robot's current URDF via xacro in-container and
   save to `tests/golden/<name>_default.urdf` (strip the container wrapper's non-XML
   stdout prefix; verify it parses).
2. Author/reuse `components/` entries (see [components/README.md](../components/README.md)).
3. Write `assembly.yaml` with mount origins copied verbatim from the xacro invocations
   (verify each sensor's actual parent link; robots differ).
4. Strip ONLY the sensor invocations + now-unused includes from the chassis xacro,
   leaving no comment behind; never delete macro files, other robots may include
   them. Ensure
   the wrapper-contract args exist (`namespace`, `prefix`, `gazebo_classic`,
   `gazebo_ignition`) with defaults preserving the chassis's existing values.
5. `tests/test_default_morphology.py` parametrizes over every `tests/golden/*_default.urdf`
   automatically, so the robot is now covered: effective-sensors consistency
   (`effective_sensors({}) == model_params.sensors`) and canonicalized URDF
   structural comparison against the golden from step 1.
6. The golden reproduces known drift byte-identically; this move is not a bug-fix
   opportunity. Dead (env-gated Gazebo-Classic) blocks stay untouched.

## Optional files

| Path | Purpose |
| --- | --- |
| `urdf/<name>.urdf.xacro` | robot description; mesh refs use `package://arena_robots/robots/<name>/meshes/…` or a fixed upstream package (`package://jackal_description/…`) |
| `meshes/` | STL/DAE/OBJ files, normally a git submodule under `github.com/arena-robots/<name>` (opt-in via `arena feature robots add <name>`) |
| `launch/` | robot-specific launch files (e.g. nav2 overrides) |
| `README.md` | free-form robot docs (upstream references, env vars, etc.) |

## Meshes

Every robot that needs per-robot geometry has a `meshes/` git submodule pinned
via `.gitmodules`, pointing at `github.com/arena-robots/<name>.git` with
`update = none`. Running `arena feature robots add <name>` clones it; config
edits you make under the robot dir stay in the main Arena repo. Robots that
use upstream geometry (jackal, turtlebot) have no `meshes/` submodule,
their URDFs reference `package://jackal_description/…` etc., supplied by
`deps/jackal` or `deps/turtlebot4`.

## Checklist

1. `mkdir -p arena_robots/arena_robots/robots/<name>/caps`
2. Write `model_params.yaml` (minimal identity), `control.yaml`, `mappings.yaml`.
3. Write `caps/mobile.yaml` (every robot has a mobile base). Add `caps/arm.yaml`
   and/or `caps/lift.yaml` if the robot has those subsystems.
4. (Optional) add `urdf/<name>.urdf.xacro` and/or a `meshes/` submodule.
   For component-catalog sensors, add `assembly.yaml` + a golden (see "Moving a
   robot's sensors to the component catalog" above).
5. If the robot ships an upstream ROS package, add it as a submodule with
   `robot = <name>` (and `update = none`) in `.gitmodules`.
6. `arena feature robots add <name>` to fetch any submodules.
7. `arena feature robots check` to verify every `package://arena_robots/…`
   URI resolves on disk.
8. `arena launch … robot:=<name>` to bring it up.

## Adding a new cap

When a new subsystem kind (e.g. `gripper`, `ptu_head`) becomes needed:

1. Add an entry to the Active cap vocabulary table above with its file, shape,
   meaning, and list of robots that have it.
2. Define the primitive fields (what every adapter-agnostic consumer can count
   on) and document adapter sub-block conventions if any exist.
3. If adding a typed accessor is warranted, extend
   [`caps.py`](../arena_robots/caps.py) with a `<Cap>Spec` subclass. The loader dict-keyed
   pattern is uniform, see `ArmSpec`/`LiftSpec` for the template.
