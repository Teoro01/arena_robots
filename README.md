# arena_robots

Per-robot configuration, URDFs, and meshes for Arena, plus the launch-side of
navigation adapters. Everything robot-related that simulators and task-generator
consume lives here.

## Guides

- [Adding a robot](arena_robots/robots/README.md): directory layout, required
  YAMLs, how meshes are fetched, validation.
- [Navigation adapters](arena_robots/launch/adapters/README.md): Adapter ABC,
  selection via `mobile:`/`arm:` in setup YAML, adding a new one.
- [Robot setup files](arena_robots/config/setup/README.md): `robot_setup.yaml`
  schema for listing robots to spawn in a scenario.
- [Task kinds](arena_robots/arena_robots/task_server_handlers/README.md):
  `task_server` endpoints, `TaskKind` registry, adding a new kind.

## CLI

The `arena feature robots` CLI manages per-robot submodules (meshes + upstream
deps):

```
arena feature robots ls                 # [x] ready / [ ] pending per robot
arena feature robots add <name...>      # fetch robot's submodules
arena feature robots add --all          # fetch every robot
arena feature robots rm [-f] <name...>  # deinit robot's submodules (-f: shared paths too)
arena feature robots check [--all]      # verify package:// URIs resolve
arena feature robots check --sim <name...|--all> [sim:=...]  # drive each robot through map_empty's preflight scenario, verdict wedged/weak/ok
arena feature robots update             # refresh initialized submodules
```

Robot-owning submodules are identified hybrid: either the path begins with
`arena_robots/arena_robots/robots/<name>/`, or the submodule entry in
`.gitmodules` carries a `robot = <name>` attribute. Mesh assets live in
individual repos under [github.com/arena-robots](https://github.com/arena-robots).

## Internals

Two `Identifier` types from `arena_simulation_setup.tree` are the entry
points for everything downstream:

- **`RobotIdentifier`** ([Robot.py](arena_robots/arena_robots/Robot.py)): resolves a
  robot name to its directory via `SimplePathResolver` pointed at
  `get_package_share_path('arena_robots') / 'robots'`, and `.load()` returns
  a `RobotView` that lazy-reads `model_params.yaml` / `control.yaml` /
  `mappings.yaml` / `caps/*.yaml` and exposes a `ModelWrapper` covering URDF
  and USD. Cap advertisement is derived from the `caps/` tree, see the
  [robot-authoring guide](arena_robots/robots/README.md).
- **`RobotSetupIdentifier`** ([SetupFile.py](arena_robots/arena_robots/SetupFile.py)):
  resolves a setup name to `config/setup/<name>.yaml` and returns a
  `list[Config]`, one per spawned instance (after `count` expansion). See
  the [setup files guide](arena_robots/config/setup/README.md).

Both `Identifier`s are registered at import time, so third-party code can
use them by name without touching this package's filesystem layout
directly.

The robot audio contract shared by acoustic simulators and hearing stacks
lives here too: [audio.py](arena_robots/arena_robots/audio.py) holds the
microphone array geometry (`ArraySpec`, presets in `config/audio/arrays/`),
level math, `gcc_phat` and the `<robot>/audio/<stream>` topic names, and
[fleet.py](arena_robots/arena_robots/fleet.py) binds `RobotFleet` members
to their model-derived frames, odometry topics and drivetrain facts.
Audio travels as `arena_robots_msgs/AudioFrame`, sound detections as
`arena_robots_msgs/SoundDetection`, and `type: audio` (`SensorType.AUDIO`) is
the sensor type of a microphone array in a robot's `sensors:` list.

## Driving a robot standalone

Launch, CLI, and Python examples for sending goals to a robot without
task_generator: see [DRIVING.md](DRIVING.md).
