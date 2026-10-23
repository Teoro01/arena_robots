# Component catalog

`components/<type>/<variant>/component.yaml`, one directory per `(type, variant)`,
loaded by [`arena_robots.catalog.Catalog`](../arena_robots/catalog.py). See that
module's docstring for the authoritative `component.yaml` schema and the
sensor-template substitution context.

## Schema at a glance

```yaml
xacro:                    # how the wrapper invokes this component
  include: <path>         # robot-tree-relative, OR "$(find pkg)/..." / absolute (passed verbatim)
  macro: <macro name>     # must accept an <origin> block param (*origin)
  attach: {...}           # macro args; ${mount}/${parent}/${prefix}/${key:-default} templated
  args: {...}             # declared overridable arg surface with defaults (documentation)
sensor:
  gz:                     # LIST of bridged outputs (one component may emit several)
    - name: "${name:-lidar}"
      type: laserscan     # BRIDGE_TYPES in Sensor.py drive derived bridge rows
      topic: "${topic:-scan}"
      frame: "${prefix}${mount}_link"
ros2_control:             # joint-bearing parts only (arms): control-joint patch
  joints: [...]           # rendered by catalog.render_control_joints, injected post-render
control: {...}            # controller block deep-merged into the chassis control.yaml
caps: {...}               # full caps/<cap>.yaml shape, rendered per placement
frames: {...}             # named frames this component exports (e.g. a lift's `top`),
                          # for another mount's Mount.parent to chain onto: "@<mount>:<frame>"
variants: [...]           # family component: variant names this one dir serves, rendered
                          # per placement off ${variant} (e.g. arm/ur for the UR family)
```

## Conventions

- **Variants encode capability, not SKU** (`d435_color` vs a hypothetical `d435_rgbd`).
- **Include paths**: a robot-tree-relative `include` only resolves for the robot that owns
  the macro file; any component shared across robots must use a package-qualified
  `$(find arena_robots)/...` include.
- **Origin contract**: the wrapper appends an `<origin>` block to every invocation. Macros
  taking scalar offsets instead (the `arena_default_*` family) get a small `*origin` shim
  macro in the component dir; the original macro files are never modified.
- **Multi-output components**: `sensor.gz` is a list, one macro may contribute several
  sensors (e.g. an RGBD camera bundling an IMU).
- **Sim-only sensors** (rendered under gz but unbridged, e.g. GPS/navsat): the macro renders
  them, `sensor.gz` omits them; no SensorSpec, no bridge row.
- **Parent link**: every attach template names its parent `${prefix}${parent}`, so the
  component hangs off the chassis link whatever the robot's frame prefix.
- **Arm components** contribute four artifacts (`xacro`, `ros2_control.joints`, `control`,
  `caps`). The chassis and every placed component each render their own `ros2_control`
  tag normally (no gating xacro:arg needed on either side); `ros2_control.joints` is a
  separate control-joint patch (`catalog.render_control_joints`, computed straight from
  `ResolvedAssembly`, not by parsing xacro output) that arena_simulation_setup's urdf
  loader (`_inject_ros2_control_joints`) merges post-render into the chassis's own tag
  (the one whose plugin is `gz_ros2_control/GazeboSimSystem`), dropping every other
  `ros2_control` tag (an arm's own native one, superseded by the patch).
- Per-instance tuning lives in `assembly.yaml defaults[].params/overrides`, never in the
  request grammar.
- **Chained mounts**: a mount's `parent` may be `"@<mount>:<frame>"`, resolved to
  the referenced mount's placed component's `frames.<frame>` template (e.g. an arm parented
  on a lift's `top`). A placed part chained through an unpopulated mount is an
  `AssemblyError`; chained-parent references must form a DAG.
- **Family components**: a dir whose `variants:` lists the variant names it serves resolves
  any of them through the one shared spec (`Catalog` falls back when no dedicated dir
  exists; an exact dir still wins). `${variant}` joins the render context per placement;
  per-variant data files live beside the spec keyed by `${variant}` (e.g.
  `arm/ur/joint_limits/${variant}.yaml`).
- **Gripper components** chain onto an arm's exported `tip` frame (a gripper mount's
  `parent: "@<arm mount>:tip"`). The actuated joint carries command interfaces; mimic
  joints ride along state-only with `mimic: true` (the explicit jazzy ros2_control
  attribute; humble-era `<param name="mimic">` entries leave the joints NaN). Known
  jazzy gz limitation: mimic fingers stay limp in sim (gz_ros2_control 1.2 registers
  but does not enforce mimic, dartsim has no mimic constraint, bullet-featherstone
  breaks position control, and world-plugin JointPositionReset enforcement aborts the
  server), so only the actuated finger moves until the stack catches up.
  `caps.moveit.srdf` names a fragment merged into the composed SRDF per placement
  (`moveit_factory._gripper_srdf_extras`, context adds `arm` = the carrying arm's stem).
