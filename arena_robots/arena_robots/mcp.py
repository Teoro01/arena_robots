from __future__ import annotations

import asyncio
import math
import re
import time
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Any

from pydantic import Field

from arena_mcp.core import JOBS, LATCHED, RELIABLE, ROS, SENSOR, Domain, ToolError, cli, fleet, media, msgs, robot_ns
from arena_mcp.core.jobs import RUNNING

if TYPE_CHECKING:
    from geometry_msgs.msg import PoseStamped

    from arena_mcp.core import JobHandle
    from arena_robots.assembly import RequestPart
    from arena_robots.caps import RobotCaps
    from arena_robots.Robot import RobotView
    from arena_robots.Sensor import SensorSpec
    from arena_robots.task_kinds import TaskKind

domain = Domain(
    "robots",
    """Robot models and live robots: catalog, definitions and validation offline, fleet, task actions, velocity commands, pose, sensors, scans, cameras and power in a running env.
    Live tools take env_id and robot, both optional when only one exists (robots_fleet lists them).
    robots_goto, robots_reach, robots_gesture and robots_drive return a job: poll jobs_status for feedback, stop with jobs_cancel, which cancels the goal.
    Velocity commands go to <robot_ns>/cmd_vel, the topic the controllers' twist_stamper and flatsim read.""",
    requires=("arena_robots_msgs", "arena_simulation_setup", "ament_index_python"),
)

EnvId = Annotated[int | None, Field(description="env to act on, omit when only one env runs")]
RobotName = Annotated[str | None, Field(description="robot name or namespace in the env, omit when the env has one robot")]

TWIST = "geometry_msgs/msg/Twist"
TWIST_STAMPED = "geometry_msgs/msg/TwistStamped"
_DRIVES: dict[str, str] = {}


def _view(model: str) -> RobotView:
    from arena_robots.Robot import RobotIdentifier

    try:
        view = RobotIdentifier(model).resolve_sync()
    except FileNotFoundError as exc:
        raise ToolError(f"no robot model {model!r}", hint=f"models: {', '.join(_models())}") from exc
    if not (view.path / "model_params.yaml").is_file():
        raise ToolError(f"{model!r} is not a robot definition (no model_params.yaml)", hint=f"models: {', '.join(_models())}")
    return view


def _models() -> list[str]:
    from arena_robots.Robot import RobotIdentifier

    return sorted(identifier.name for identifier in RobotIdentifier.listall() if (identifier.resolve_sync().path / "model_params.yaml").is_file())


def _error(exc: Exception) -> str:
    return f"{type(exc).__name__}: {exc}"


def _attrs(value: object) -> object:
    import attrs

    return msgs.jsonable(attrs.asdict(value)) if value is not None else None


def _sensor_dict(spec: SensorSpec, namespace: str | None = None) -> dict[str, Any]:
    from arena_robots.Sensor import resolve_topic

    out = {"name": spec.name, "type": str(spec.type), "topic": resolve_topic(spec, namespace) if namespace else spec.topic, "frame": spec.frame}
    if spec.sensor is not None:
        out["sensor"] = spec.sensor
    return out


def _summary(view: RobotView) -> dict[str, Any]:
    entry: dict[str, Any] = {"model": view.name}
    try:
        caps = view.effective_caps({})
        available = sorted(caps.available)
        sensors = view.effective_sensors({})
        mobile = view.mobile
        arms = caps.arm or {}
    except Exception as exc:
        entry["error"] = _error(exc)
        return entry
    entry["caps"] = available
    entry["sensor_types"] = sorted({str(spec.type) for spec in sensors})
    entry["assembly"] = view.assembly is not None
    entry["priority"] = msgs.jsonable(view.model_params.priority)
    parts = []
    if mobile is not None:
        drive = "holonomic" if mobile.is_holonomic else "non-holonomic"
        parts.append(f"mobile ({drive}, radius {mobile.radius} m)" if mobile.radius is not None else f"mobile ({drive})")
    if arms:
        parts.append(f"arm x{len(arms)}")
    parts.extend(cap for cap in available if cap not in ("mobile", "arm"))
    entry["summary"] = ", ".join(parts) + (f", sensors: {', '.join(entry['sensor_types'])}" if sensors else "")
    return entry


def _mobile(caps: RobotCaps) -> dict[str, Any] | None:
    if "mobile" not in caps.available:
        return None
    mobile = caps.mobile
    out: dict[str, Any] = {"odom_frame": mobile.odom_frame, "sensor_frame": mobile.sensor_frame, "radius": mobile.radius, "is_holonomic": mobile.is_holonomic}
    for field in ("footprint", "footprint_padding", "inflation_radius"):
        try:
            out[field] = getattr(mobile, field)
        except Exception as exc:
            out[field] = {"error": _error(exc)}
    for field in ("velocity_limits", "acceleration_limits", "planar", "laser", "actions"):
        try:
            out[field] = _attrs(getattr(mobile, field))
        except Exception as exc:
            out[field] = {"error": _error(exc)}
    out["adapter_blocks"] = sorted(key for key, value in mobile.raw.items() if isinstance(value, dict) and key not in ("polygons_dict", "planar", "actions", "laser", "velocity_limits", "acceleration_limits"))
    return msgs.jsonable(out)


def _shared_gestures() -> list[str]:
    from arena_simulation_setup.tree.Gesture import GestureIdentifier

    return sorted({Path(identifier.name).stem for identifier in GestureIdentifier.listall()})


def _gestures(arm: object) -> dict[str, Any]:
    from arena_simulation_setup.tree.Gesture import GestureIdentifier

    poses = set(arm.named_poses)
    out: dict[str, Any] = {}
    for name in _shared_gestures():
        try:
            required = GestureIdentifier(name).resolve_sync().required_poses()
        except Exception as exc:
            out[name] = {"source": "shared", "error": _error(exc)}
            continue
        out[name] = {"source": "shared", "playable": required <= poses, "missing_poses": sorted(required - poses)}
    for name, spec in arm.gestures.items():
        required = spec.required_poses()
        out[name] = {"source": "robot", "playable": required <= poses, "missing_poses": sorted(required - poses)}
    return out


def _instances(caps: RobotCaps) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for cap in ("arm", "lift", "gripper"):
        try:
            instances = getattr(caps, cap)
        except Exception as exc:
            out[cap] = {"error": _error(exc)}
            continue
        if not instances:
            continue
        entries = {}
        for name, instance in instances.items():
            entry: dict[str, Any] = {"raw": msgs.jsonable(instance.raw)}
            if cap == "arm":
                try:
                    entry["named_poses"] = sorted(instance.named_poses)
                    entry["gestures"] = _gestures(instance)
                except Exception as exc:
                    entry["error"] = _error(exc)
            entries[name] = entry
        out[cap] = entries
    return out


def _action_name(action: type) -> str:
    return f"{action.__module__.split('.')[0]}/action/{action.__name__}"


def _actions(available: frozenset[str]) -> dict[str, Any]:
    from arena_robots.bringup import BRINGUPS
    from arena_robots.task_kinds import PUBLIC_SUFFIX, TaskKind, action_type, cap_for_task_kind

    endpoints = {kind.value: {"endpoint": f"<robot_ns>/{PUBLIC_SUFFIX[kind]}", "action": _action_name(action_type(kind)), "cap": cap_for_task_kind(kind)} for kind in TaskKind if cap_for_task_kind(kind) in available}
    bringups: dict[str, Any] = {}
    for cap, registry in BRINGUPS.items():
        if cap not in available:
            continue
        kinds: dict[str, Any] = {}
        for kind in sorted(registry.keys()):
            try:
                bringup = registry.get(kind)
                requires = frozenset(bringup._bringup_meta.requires)
                served = sorted(task_kind.value for task_kind in bringup.task_handlers)
            except Exception as exc:
                kinds[kind] = {"error": _error(exc)}
                continue
            kinds[kind] = {"requires": sorted(requires), "compatible": requires <= available, "task_kinds": served}
        bringups[cap] = kinds
    return {"endpoints": endpoints, "bringups": bringups}


def _component_variants(type_: str) -> list[str]:
    from ament_index_python.packages import get_package_share_path

    from arena_robots.catalog import ComponentSpec

    root = get_package_share_path("arena_robots") / "components" / type_
    if not root.is_dir():
        return []
    variants: list[str] = []
    for directory in sorted(root.iterdir()):
        manifest = directory / "component.yaml"
        if manifest.is_file():
            variants.extend(ComponentSpec.from_yaml(manifest).variants or [directory.name])
    return sorted(variants)


def _assembly(view: RobotView) -> dict[str, Any] | None:
    spec = view.assembly
    if spec is None:
        return None
    out = _attrs(spec)
    out["variants"] = {type_: _component_variants(type_) for type_ in sorted({type_ for mount in spec.mounts.values() for type_ in mount.accepts})}
    return out


def _definition(view: RobotView) -> dict[str, Any]:
    params = view.model_params
    caps = view.effective_caps({})
    out: dict[str, Any] = {
        "model": view.name,
        "path": str(view.path),
        "base_frame": params.base_frame,
        "odom_frame": params.odom_frame,
        "z_offset": params.z_offset,
        "control": _attrs(params.control),
        "caps": sorted(caps.available),
        "mobile": _mobile(caps),
        **_instances(caps),
        "sensors": [_sensor_dict(spec) for spec in view.effective_sensors({})],
        "actions": _actions(caps.available),
        "assembly": _assembly(view),
        "power": msgs.jsonable(view.power_profile({})),
        "model_params": msgs.jsonable(dict(params)),
    }
    return out


@domain.tool(read_only=True)
def robots_catalog() -> list[dict[str, Any]]:
    """Every robot model the robot resolver finds, with its caps, sensor types and a one-line summary."""
    from arena_robots.Robot import RobotIdentifier

    out = []
    for identifier in RobotIdentifier.listall():
        view = identifier.resolve_sync()
        if (view.path / "model_params.yaml").is_file():
            out.append(_summary(view))
    return sorted(out, key=lambda entry: entry["model"])


@domain.tool(read_only=True)
def robots_definition(model: str) -> dict[str, Any]:
    """One robot model resolved offline: frames, control, caps (mobile limits, footprint, arm instances with poses and gestures), default sensors, task actions, assembly, power profile."""
    return _definition(_view(model))


@domain.resource("arena://robots/catalog", mime_type="application/json", description="Every robot model with caps and a one-line summary")
def robots_catalog_resource() -> list[dict[str, Any]]:
    return robots_catalog()


@domain.resource("arena://robots/definition/{model}", mime_type="application/json", description="One robot model resolved offline")
def robots_definition_resource(model: str) -> dict[str, Any]:
    return robots_definition(model)


def _finding(robot: str, check: str, message: str, severity: str = "error", **extra: object) -> dict[str, Any]:
    return {"robot": robot, "check": check, "severity": severity, "message": message, **extra}


def _definition_findings(view: RobotView) -> list[dict[str, Any]]:
    from arena_robots.sensors import SENSOR_TYPES

    name = view.name
    try:
        params = view.model_params
    except Exception as exc:
        return [_finding(name, "model_params", _error(exc))]
    findings: list[dict[str, Any]] = []
    if params.get("robot_model") != name:
        findings.append(_finding(name, "model_params", f"robot_model is {params.get('robot_model')!r}, not the directory name", "warning"))
    try:
        caps = view.effective_caps({})
        available = caps.available
    except Exception as exc:
        findings.append(_finding(name, "assembly", _error(exc)))
        caps, available = view.caps, view.caps.available
    if "mobile" not in available:
        findings.append(_finding(name, "caps", "caps/mobile.yaml is missing, every robot needs a mobile cap"))
    else:
        for field in ("velocity_limits", "acceleration_limits", "actions", "planar", "laser", "footprint", "polygons_dict"):
            try:
                getattr(caps.mobile, field)
            except Exception as exc:
                findings.append(_finding(name, "caps.mobile", f"{field}: {_error(exc)}"))
    try:
        sensors = view.effective_sensors({})
    except Exception as exc:
        findings.append(_finding(name, "sensors", _error(exc)))
        sensors = []
    for spec in sensors:
        if str(spec.type) not in SENSOR_TYPES:
            findings.append(_finding(name, "sensors", f"sensor {spec.name!r} has type {spec.type!r}, not one of {sorted(SENSOR_TYPES)}"))
        if "${namespace}" not in spec.topic:
            findings.append(_finding(name, "sensors", f"sensor {spec.name!r} topic {spec.topic!r} lacks the ${{namespace}} placeholder", "warning"))
    try:
        control = params.control
    except Exception as exc:
        findings.append(_finding(name, "control", _error(exc)))
        control = None
    if control is not None and control.is_ros2_control:
        if not control.controllers:
            findings.append(_finding(name, "control", "control.mode is ros2_control but no controllers are declared"))
        try:
            if not view.control:
                findings.append(_finding(name, "control", "control.yaml is empty"))
        except Exception as exc:
            findings.append(_finding(name, "control", _error(exc)))
    try:
        arms = caps.arm or {}
    except Exception as exc:
        findings.append(_finding(name, "caps.arm", _error(exc)))
        arms = {}
    for instance, arm in arms.items():
        for field in ("controller", "named_poses", "gestures"):
            try:
                getattr(arm, field)
            except Exception as exc:
                findings.append(_finding(name, "caps.arm", f"{instance}.{field}: {_error(exc)}"))
    return findings


_OWNER = re.compile(r"^  \[(.+)\]$")
_URI = re.compile(r"^    (\S.*)$")
_SOURCE = re.compile(r"^      \S+ (.+)$")


def _uri_check(include_uninitialized: bool) -> dict[str, Any]:
    done = cli.run("feature", "robots", "check", *(["--all"] if include_uninitialized else []), timeout=300, check=False)
    if done["exit_code"] not in (0, 1) or (done["exit_code"] == 1 and "FAIL:" not in done["stdout"]):
        tail = (done["stderr"] or done["stdout"]).strip().splitlines()[-10:]
        raise ToolError(f"{done['command']} failed with exit code {done['exit_code']}:\n" + "\n".join(tail))
    skipped: list[str] = []
    dynamic = 0
    missing: dict[tuple[str, str], list[str]] = {}
    owner: str | None = None
    uri: str | None = None
    for line in done["stdout"].splitlines():
        if line.startswith("skipping uninitialized: "):
            skipped = [item.strip() for item in line.removeprefix("skipping uninitialized: ").split(",") if item.strip()]
        elif line.startswith("dynamic refs: "):
            dynamic = int(line.removeprefix("dynamic refs: ").split(",", 1)[0])
        elif match := _OWNER.match(line):
            owner, uri = match.group(1), None
        elif (match := _SOURCE.match(line)) and owner is not None and uri is not None:
            missing[(owner, uri)].append(match.group(1))
        elif (match := _URI.match(line)) and owner is not None:
            uri = match.group(1)
            missing[(owner, uri)] = []
    return {"command": done["command"], "skipped_uninitialized": skipped, "dynamic_refs": dynamic, "missing": [{"owner": key[0], "uri": key[1], "sources": sources} for key, sources in missing.items()]}


def _component_owners(view: RobotView) -> set[str]:
    from ament_index_python.packages import get_package_share_path

    from arena_robots.catalog import ComponentSpec

    spec = view.assembly
    if spec is None:
        return set()
    root = get_package_share_path("arena_robots") / "components"
    owners: set[str] = set()
    for type_, parts in spec.defaults.items():
        for part in parts:
            if (root / type_ / part.variant / "component.yaml").is_file():
                owners.add(f"{type_}/{part.variant}")
                continue
            for manifest in sorted((root / type_).glob("*/component.yaml")):
                if part.variant in ComponentSpec.from_yaml(manifest).variants:
                    owners.add(f"{type_}/{manifest.parent.name}")
    return owners


@domain.tool(read_only=True)
def robots_check(
    model: Annotated[str | None, Field(description="one robot model, all models when omitted")] = None,
    include_uninitialized: Annotated[bool, Field(description="also check mesh URIs of robots whose mesh submodules are not checked out (arena feature robots check --all)")] = False,
) -> dict[str, Any]:
    """Validate robot definitions (params, caps, sensors, control, arms) and run `arena feature robots check` for mesh URIs, as structured findings."""
    views = [_view(model)] if model else [_view(name) for name in _models()]
    findings = [finding for view in views for finding in _definition_findings(view)]
    uri = _uri_check(include_uninitialized)
    owners = ({views[0].name} | _component_owners(views[0])) if model else None
    for miss in uri["missing"]:
        if owners is None or miss["owner"] in owners:
            findings.append(_finding(miss["owner"], "mesh_uri", f"package://arena_robots/{miss['uri']} does not resolve", uri=miss["uri"], sources=miss["sources"]))
    return {
        "ok": not any(finding["severity"] == "error" for finding in findings),
        "models": [view.name for view in views],
        "findings": findings,
        "skipped_uninitialized": uri["skipped_uninitialized"],
        "dynamic_refs": uri["dynamic_refs"],
        "uri_check": uri["command"],
    }


async def _member(env_id: int | None, robot: str | None) -> dict[str, Any]:
    ns = await robot_ns(env_id, robot)
    for entry in await fleet(env_id):
        if entry["ns"] == ns:
            return entry
    raise ToolError(f"robot {ns} left the fleet", hint="robots_fleet lists the current robots")


def _topic(ns: str, relative: str) -> str:
    return f"{ns.rstrip('/')}/{relative}"


def _task_kinds() -> list[TaskKind]:
    from arena_robots.task_kinds import TaskKind

    return list(TaskKind)


def _served(entry: dict[str, Any], actions: list[str]) -> dict[str, Any]:
    from arena_robots.task_kinds import cap_for_task_kind, endpoint

    caps = {cap["cap"] for cap in entry["caps"]}
    out = {}
    for kind in _task_kinds():
        name = endpoint(entry["ns"], kind)
        out[kind.value] = {"endpoint": name, "served": name in actions, "cap": cap_for_task_kind(kind), "cap_present": cap_for_task_kind(kind) in caps}
    return out


@domain.tool(read_only=True)
async def robots_fleet(env_id: EnvId = None) -> list[dict[str, Any]]:
    """The live robots of an env: name, model, namespace, frames, caps with bound adapters, morphology, which task actions are served right now and cmd_vel subscribers."""
    robots = await fleet(env_id)
    graph = await ROS.graph()
    node = await ROS.node()
    for entry in robots:
        entry["actions"] = _served(entry, graph["actions"])
        cmd_vel = _topic(entry["ns"], "cmd_vel")
        entry["cmd_vel"] = {"topic": cmd_vel, "subscribers": node.count_subscribers(cmd_vel)}
    return robots


def _hint(entry: dict[str, Any], served: dict[str, Any]) -> str:
    live = [kind for kind, state in served.items() if state["served"]]
    adapters = ", ".join(f"{cap['cap']}={cap['adapter']}" for cap in entry["caps"])
    return f"{entry['name']} serves {live or 'no task actions'} (adapters: {adapters or 'none'}), robots_fleet shows the live endpoints"


class _Feedback:
    """Feedback from the ROS thread, applied to the job on the server loop at most twice a second."""

    def __init__(self, convert: Callable[[object], dict[str, Any]]) -> None:
        self._loop = asyncio.get_running_loop()
        self._convert = convert
        self._handle: JobHandle | None = None
        self._pending: dict[str, Any] | None = None
        self._last = 0.0

    def __call__(self, message: object) -> None:
        self._loop.call_soon_threadsafe(self._apply, self._convert(message.feedback))

    def _apply(self, fields: dict[str, Any]) -> None:
        self._pending = fields
        if self._handle is not None and time.monotonic() - self._last >= 0.5:
            self.flush()

    def attach(self, handle: JobHandle) -> None:
        self._handle = handle
        self.flush()

    def flush(self) -> None:
        if self._handle is not None and self._pending is not None:
            self._handle.progress(**self._pending)
            self._pending = None
            self._last = time.monotonic()


async def _action_job(kind: str, title: str, entry: dict[str, Any], task_kind: TaskKind, goal: object, convert: Callable[[object], dict[str, Any]]) -> dict[str, Any]:
    from action_msgs.msg import GoalStatus

    from arena_robots.task_kinds import action_type, endpoint

    action = action_type(task_kind)
    name = endpoint(entry["ns"], task_kind)
    feedback = _Feedback(convert)
    hint = _hint(entry, _served(entry, (await ROS.graph())["actions"]))
    wrapper, handle = await ROS.send_goal(action, name, goal, feedback=feedback, hint=hint)
    statuses = msgs.constants(action.Result, "STATUS_")
    goal_statuses = msgs.constants(GoalStatus, "STATUS_")

    async def body(job: JobHandle) -> dict[str, Any]:
        feedback.attach(job)
        job.log(f"goal accepted by {name}")
        response = await wrapper.await_result(handle)
        feedback.flush()
        result = msgs.to_dict(response.result)
        result["status"] = statuses.get(response.result.status, response.result.status)
        result["goal_status"] = goal_statuses.get(response.status, response.status)
        result["endpoint"] = name
        job.log(f"{name} ended {result['status']}: {result.get('reason', '')}")
        if response.result.status != 0:
            raise ToolError(f"{task_kind.value} ended {result['status']}: {result.get('reason') or 'no reason given'}", hint=f"final goal status {result['goal_status']}")
        return result

    async def cancel() -> None:
        await wrapper.cancel(handle)

    return JOBS.start_task(kind, title, body, on_cancel=cancel).as_dict()


def _quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


async def _stamped(frame: str, x: float, y: float, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> PoseStamped:
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = (await ROS.node()).sim_time.to_msg()
    msg.pose = msgs.pose(x, y, yaw, z)
    orientation = msg.pose.orientation
    orientation.x, orientation.y, orientation.z, orientation.w = _quaternion(roll, pitch, yaw)
    return msg


def _goto_feedback(feedback: object) -> dict[str, Any]:
    return msgs.jsonable({"distance_remaining": feedback.distance_remaining, "eta_seconds": feedback.eta_seconds, "current_pose": msgs.pose_dict(feedback.current_pose.pose), "frame": feedback.current_pose.header.frame_id})


def _arm_feedback(feedback: object) -> dict[str, Any]:
    return msgs.jsonable({"state": feedback.state, "progress": feedback.progress})


@domain.tool()
async def robots_goto(
    x: float,
    y: float,
    yaw: float = 0.0,
    env_id: EnvId = None,
    robot: RobotName = None,
    frame: Annotated[str, Field(description="frame of the target, the env map by default")] = "map",
    pose_tolerance: Annotated[float, Field(ge=0, description="arrival radius in m, 0 = adapter default")] = 0.0,
    yaw_tolerance: Annotated[float, Field(ge=0, description="heading tolerance in rad, 0 = adapter default")] = 0.0,
) -> dict[str, Any]:
    """Send a mobile robot to (x, y, yaw) through its goto_pose action as a job: progress carries distance_remaining and eta_seconds, jobs_cancel cancels the goal."""
    from arena_robots_msgs.action import GotoPose

    from arena_robots.task_kinds import TaskKind

    entry = await _member(env_id, robot)
    goal = GotoPose.Goal()
    goal.target = await _stamped(frame, x, y, yaw=yaw)
    goal.pose_tolerance = float(pose_tolerance)
    goal.yaw_tolerance = float(yaw_tolerance)
    return await _action_job("goto", f"goto {entry['name']} ({x:.2f}, {y:.2f}, {yaw:.2f})", entry, TaskKind.GOTO_POSE, goal, _goto_feedback)


@domain.tool()
async def robots_reach(
    env_id: EnvId = None,
    robot: RobotName = None,
    named: Annotated[str | None, Field(description="named pose from caps/arm.yaml named_poses, instead of a Cartesian target")] = None,
    x: float | None = None,
    y: float | None = None,
    z: float | None = None,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    frame: Annotated[str | None, Field(description="frame of the Cartesian target, the robot's base frame by default")] = None,
    instance: Annotated[str, Field(description="arm instance (mount name), empty for the only arm")] = "",
    position_tolerance: Annotated[float, Field(ge=0, description="m, 0 = adapter default")] = 0.0,
    orientation_tolerance: Annotated[float, Field(ge=0, description="rad, 0 = adapter default")] = 0.0,
    planning_time: Annotated[float, Field(ge=0, description="s, 0 = adapter default")] = 0.0,
) -> dict[str, Any]:
    """Move an arm to a named pose or a Cartesian target through its reach_pose action as a job with state and progress feedback, jobs_cancel cancels the goal."""
    from arena_robots_msgs.action import ReachPose

    from arena_robots.task_kinds import TaskKind

    if named is None and None in (x, y, z):
        raise ToolError("pass named or all of x, y and z", hint="robots_definition lists the arm's named_poses")
    entry = await _member(env_id, robot)
    goal = ReachPose.Goal()
    if named is not None:
        goal.named_target = named
    else:
        goal.target = await _stamped(frame or entry["base_frame"], x, y, z, roll, pitch, yaw)
    goal.instance = instance
    goal.position_tolerance = float(position_tolerance)
    goal.orientation_tolerance = float(orientation_tolerance)
    goal.planning_time = float(planning_time)
    target = named if named is not None else f"({x:.2f}, {y:.2f}, {z:.2f})"
    return await _action_job("reach", f"reach {entry['name']} {target}", entry, TaskKind.REACH_POSE, goal, _arm_feedback)


@domain.tool()
async def robots_gesture(
    name: Annotated[str, Field(description="gesture from the shared library or the arm's own gestures, robots_definition lists which are playable")],
    env_id: EnvId = None,
    robot: RobotName = None,
    instance: Annotated[str, Field(description="arm instance (mount name), empty for the only arm")] = "",
) -> dict[str, Any]:
    """Play a gesture on an arm through its play_gesture action as a job with state and progress feedback, jobs_cancel cancels the goal."""
    from arena_robots_msgs.action import PlayGesture

    from arena_robots.task_kinds import TaskKind

    entry = await _member(env_id, robot)
    goal = PlayGesture.Goal()
    goal.gesture = name
    goal.instance = instance
    return await _action_job("gesture", f"gesture {entry['name']} {name}", entry, TaskKind.PLAY_GESTURE, goal, _arm_feedback)


async def _cmd_vel(entry: dict[str, Any]) -> tuple[str, str, Callable[[float, float, float], Awaitable[None]]]:
    from geometry_msgs.msg import Twist, TwistStamped

    topic = _topic(entry["ns"], "cmd_vel")
    node = await ROS.node()
    types = dict(node.get_topic_names_and_types()).get(topic, [])
    stamped = TWIST_STAMPED in types and TWIST not in types

    async def send(vx: float, vy: float, wz: float) -> None:
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = float(vx), float(vy), float(wz)
        if stamped:
            msg = TwistStamped()
            msg.header.stamp = node.sim_time.to_msg()
            msg.header.frame_id = entry["base_frame"]
            msg.twist = twist
            await ROS.publish(TwistStamped, topic, msg)
        else:
            await ROS.publish(Twist, topic, twist)

    return topic, TWIST_STAMPED if stamped else TWIST, send


async def _zero(send: Callable[[float, float, float], Awaitable[None]]) -> None:
    for _ in range(5):
        await send(0.0, 0.0, 0.0)
        await asyncio.sleep(0.05)


@domain.tool()
async def robots_stop(env_id: EnvId = None, robot: RobotName = None) -> dict[str, Any]:
    """Cancel the robot's running robots_drive job and publish zero velocity on its cmd_vel, a planner driving it keeps commanding until its goal is cancelled."""
    entry = await _member(env_id, robot)
    cancelled = None
    job_id = _DRIVES.pop(entry["ns"], None)
    if job_id is not None and JOBS.get(job_id).state == RUNNING:
        cancelled = (await JOBS.cancel(job_id)).id
    topic, kind, send = await _cmd_vel(entry)
    await _zero(send)
    node = await ROS.node()
    return {"robot": entry["name"], "topic": topic, "type": kind, "subscribers": node.count_subscribers(topic), "cancelled_drive": cancelled}


@domain.tool()
async def robots_drive(
    vx: Annotated[float, Field(description="forward velocity in m/s")],
    vy: Annotated[float, Field(description="lateral velocity in m/s, holonomic robots only")] = 0.0,
    wz: Annotated[float, Field(description="yaw rate in rad/s")] = 0.0,
    duration_s: Annotated[float, Field(gt=0, le=600, description="wall-clock seconds to keep commanding")] = 1.0,
    rate_hz: Annotated[float, Field(gt=0, le=100)] = 20.0,
    env_id: EnvId = None,
    robot: RobotName = None,
) -> dict[str, Any]:
    """Publish a constant velocity on the robot's cmd_vel at rate_hz for duration_s as a job, then zero, jobs_cancel or robots_stop ends it early."""
    entry = await _member(env_id, robot)
    topic, kind, send = await _cmd_vel(entry)
    period = 1.0 / rate_hz
    previous = _DRIVES.get(entry["ns"])
    if previous is not None and JOBS.get(previous).state == RUNNING:
        await JOBS.cancel(previous)

    async def body(job: JobHandle) -> dict[str, Any]:
        sent = 0
        start = time.monotonic()
        try:
            while (elapsed := time.monotonic() - start) < duration_s:
                await send(vx, vy, wz)
                sent += 1
                if sent % max(1, int(rate_hz)) == 0:
                    job.progress(sent=sent, elapsed_s=round(elapsed, 2))
                await asyncio.sleep(period)
        finally:
            await _zero(send)
        return {"robot": entry["name"], "topic": topic, "type": kind, "messages": sent, "duration_s": duration_s, "command": {"vx": vx, "vy": vy, "wz": wz}}

    job = JOBS.start_task("drive", f"drive {entry['name']} vx={vx} vy={vy} wz={wz} for {duration_s}s", body)
    _DRIVES[entry["ns"]] = job.id
    return job.as_dict()


async def _maybe[T](awaitable: Awaitable[T]) -> tuple[T | None, str | None]:
    try:
        return await awaitable, None
    except ToolError as exc:
        return None, exc.message


def _path_length(points: list[object]) -> float:
    total = 0.0
    for first, second in zip(points, points[1:], strict=False):
        total += math.hypot(second.pose.position.x - first.pose.position.x, second.pose.position.y - first.pose.position.y)
    return total


@domain.tool(read_only=True)
async def robots_pose(env_id: EnvId = None, robot: RobotName = None) -> dict[str, Any]:
    """The robot's task pose (what the judge sees), odometry pose and twist, and its trail length this episode."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as PathMsg

    entry = await _member(env_id, robot)
    ns = entry["ns"]
    (task, task_error), (odom, odom_error), (trail, _) = await asyncio.gather(
        _maybe(ROS.latest(PoseStamped, _topic(ns, "task_pose"), qos=SENSOR, timeout=2.0)),
        _maybe(ROS.latest(Odometry, _topic(ns, "odom"), qos=SENSOR, timeout=2.0)),
        _maybe(ROS.latest(PathMsg, _topic(ns, "trail"), qos=LATCHED, timeout=1.0)),
    )
    if task is None and odom is None:
        raise ToolError(f"{entry['name']} publishes neither task_pose nor odom", hint=f"{task_error}, {odom_error}")
    out: dict[str, Any] = {"robot": entry["name"], "sim_time": msgs.seconds((await ROS.node()).sim_time.to_msg())}
    if task is not None:
        out["task_pose"] = {**msgs.pose_dict(task.pose), "frame": task.header.frame_id, "stamp": msgs.seconds(task.header.stamp)}
    else:
        out["task_pose"] = {"error": task_error}
    if odom is not None:
        twist = odom.twist.twist
        out["odom"] = {
            **msgs.pose_dict(odom.pose.pose),
            "frame": odom.header.frame_id,
            "child_frame": odom.child_frame_id,
            "stamp": msgs.seconds(odom.header.stamp),
            "twist": {"vx": twist.linear.x, "vy": twist.linear.y, "wz": twist.angular.z},
        }
    else:
        out["odom"] = {"error": odom_error}
    if trail is not None:
        out["trail"] = {"points": len(trail.poses), "length_m": round(_path_length(list(trail.poses)), 3)}
    return msgs.jsonable(out)


def _parts(view: RobotView, params: dict[str, str]) -> dict[str, list[RequestPart]]:
    from arena_robots.assembly import RequestPart

    spec = view.assembly
    if spec is None:
        return {}
    defaults = {(type_, part.mount, part.variant): part for type_, parts in spec.defaults.items() for part in parts}
    request: dict[str, list[RequestPart]] = {}
    for key, variant in params.items():
        type_, sep, mount = key.partition("@")
        if not sep:
            continue
        default = defaults.get((type_, mount, variant)) or defaults.get((type_, None, variant))
        request.setdefault(type_, []).append(RequestPart(variant=variant, mount=mount, overrides=dict(default.overrides) if default is not None else {}))
    return request


def _live_sensors(entry: dict[str, Any]) -> list[dict[str, Any]]:
    view = _view(entry["model"])
    parts = _parts(view, entry["params"])
    try:
        specs = view.effective_sensors(parts)
    except Exception:
        specs = view.effective_sensors({})
    return [_sensor_dict(spec, entry["ns"]) for spec in specs]


def _pick(sensors: list[dict[str, Any]], types: tuple[str, ...], name: str | None, entry: dict[str, Any]) -> dict[str, Any]:
    candidates = [sensor for sensor in sensors if sensor["type"] in types]
    if name is not None:
        for sensor in sensors:
            if name in (sensor["name"], sensor["topic"]):
                if sensor["type"] not in types:
                    raise ToolError(f"sensor {name!r} is a {sensor['type']}, not {' or '.join(types)}", hint=f"candidates: {[candidate['name'] for candidate in candidates]}")
                return sensor
        raise ToolError(f"{entry['name']} has no sensor {name!r}", hint=f"{' or '.join(types)} sensors: {[candidate['name'] for candidate in candidates]}")
    if not candidates:
        raise ToolError(f"{entry['name']} ({entry['model']}) declares no {' or '.join(types)} sensor", hint="robots_sensors lists its sensors")
    return candidates[0]


@domain.tool(read_only=True)
async def robots_sensors(
    env_id: EnvId = None,
    robot: RobotName = None,
    probe_s: Annotated[float, Field(ge=0, le=10, description="when > 0, wait this long for one message per sensor (subscribes to every sensor topic)")] = 0.0,
) -> list[dict[str, Any]]:
    """The robot's sensors with live topics, publisher counts and graph types, optionally probing that each one delivers messages."""
    from rosidl_runtime_py.utilities import get_message

    entry = await _member(env_id, robot)
    sensors = _live_sensors(entry)
    node = await ROS.node()
    types = dict(node.get_topic_names_and_types())
    for sensor in sensors:
        sensor["publishers"] = node.count_publishers(sensor["topic"])
        sensor["ros_types"] = types.get(sensor["topic"], [])

    async def probe(sensor: dict[str, Any]) -> None:
        if not sensor["publishers"] or len(sensor["ros_types"]) != 1:
            sensor["received"] = False
            return
        try:
            await ROS.next(get_message(sensor["ros_types"][0]), sensor["topic"], qos=SENSOR, timeout=probe_s)
        except ToolError:
            sensor["received"] = False
        else:
            sensor["received"] = True

    if probe_s > 0:
        await asyncio.gather(*(probe(sensor) for sensor in sensors))
    return sensors


def _scan_summary(msg: object, sectors: int) -> dict[str, Any]:
    import numpy as np

    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    out: dict[str, Any] = {
        "frame": msg.header.frame_id,
        "stamp": msgs.seconds(msg.header.stamp),
        "beams": int(ranges.size),
        "valid": int(valid.sum()),
        "angle_min_deg": math.degrees(msg.angle_min),
        "angle_max_deg": math.degrees(msg.angle_max),
        "range_min": msg.range_min,
        "range_max": msg.range_max,
    }
    if valid.any():
        nearest = int(np.flatnonzero(valid)[np.argmin(ranges[valid])])
        out["nearest"] = {"range": float(ranges[nearest]), "bearing_deg": round(math.degrees(float(angles[nearest])), 2)}
    edges = np.linspace(msg.angle_min, msg.angle_min + ranges.size * msg.angle_increment, sectors + 1)
    out["sectors"] = []
    for low, high in zip(edges[:-1], edges[1:], strict=True):
        inside = valid & (angles >= min(low, high)) & (angles < max(low, high))
        out["sectors"].append({"center_deg": round(math.degrees(float((low + high) / 2)), 1), "min_range": float(ranges[inside].min()) if inside.any() else None, "valid": int(inside.sum())})
    return msgs.jsonable(out)


def _scan_image(msg: object, side: int = 480) -> object:
    import numpy as np

    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    reach = float(ranges[valid].max()) if valid.any() else float(msg.range_max)
    scale = (side / 2 - 4) / max(reach, 1e-3)
    pixels = np.full((side, side), 255, dtype=np.uint8)
    center = side // 2
    rows = np.clip((center - ranges[valid] * np.cos(angles[valid]) * scale).astype(int), 1, side - 2)
    cols = np.clip((center - ranges[valid] * np.sin(angles[valid]) * scale).astype(int), 1, side - 2)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            pixels[rows + dr, cols + dc] = 0
    pixels[center - 3 : center + 4, center - 3 : center + 4] = 128
    pixels[center - 12 : center - 3, center - 1 : center + 2] = 128
    return media.array_png(pixels)


@domain.tool(read_only=True)
async def robots_scan(
    env_id: EnvId = None,
    robot: RobotName = None,
    sensor: Annotated[str | None, Field(description="laserscan sensor name or topic, the first one by default")] = None,
    sectors: Annotated[int, Field(ge=1, le=72, description="equal angular sectors to report the minimum range of")] = 8,
    image: Annotated[bool, Field(description="also return a top-down image of the points, robot at the center facing up")] = False,
) -> object:
    """A summary of the robot's latest laser scan: nearest return with bearing, minimum range per sector, optionally a top-down image."""
    from sensor_msgs.msg import LaserScan

    entry = await _member(env_id, robot)
    chosen = _pick(_live_sensors(entry), ("laserscan",), sensor, entry)
    msg = await ROS.latest(LaserScan, chosen["topic"], qos=SENSOR, timeout=3.0, hint="is the simulator running and unpaused? robots_sensors shows publisher counts")
    summary = {"robot": entry["name"], "sensor": chosen["name"], "topic": chosen["topic"], **_scan_summary(msg, sectors)}
    return [summary, _scan_image(msg)] if image else summary


@domain.tool(read_only=True)
async def robots_camera(
    env_id: EnvId = None,
    robot: RobotName = None,
    sensor: Annotated[str | None, Field(description="image or depth sensor name or topic, the first image sensor by default")] = None,
) -> object:
    """The latest frame of one of the robot's image or depth sensors as image content, with its size, encoding and stamp."""
    from sensor_msgs.msg import CompressedImage, Image

    entry = await _member(env_id, robot)
    chosen = _pick(_live_sensors(entry), ("image", "depth"), sensor, entry)
    topic = chosen["topic"]
    node = await ROS.node()
    types = dict(node.get_topic_names_and_types()).get(topic, [])
    hint = "is the simulator running and unpaused? robots_sensors shows publisher counts"
    if "sensor_msgs/msg/CompressedImage" in types and "sensor_msgs/msg/Image" not in types:
        msg = await ROS.latest(CompressedImage, topic, qos=SENSOR, timeout=3.0, hint=hint)
        info = {"format": msg.format}
        content = media.compressed_msg(msg)
    else:
        msg = await ROS.latest(Image, topic, qos=SENSOR, timeout=3.0, hint=hint)
        info = {"width": msg.width, "height": msg.height, "encoding": msg.encoding}
        content = media.image_msg(msg)
    return [{"robot": entry["name"], "sensor": chosen["name"], "topic": topic, "frame": msg.header.frame_id, "stamp": msgs.seconds(msg.header.stamp), **info}, content]


@domain.tool(read_only=True)
async def robots_power(env_id: EnvId = None, robot: RobotName = None) -> dict[str, Any]:
    """The robot's latest power draw (static, mechanical, thermal, per joint) and consumed energy with battery state of charge."""
    from arena_robots_msgs.msg import Energy, Power

    entry = await _member(env_id, robot)
    power_topic = _topic(entry["ns"], "power_publisher/power")
    energy_topic = _topic(entry["ns"], "power_publisher/energy")
    topics = (await ROS.graph())["topics"]
    if power_topic not in topics and energy_topic not in topics:
        raise ToolError(f"{entry['name']} has no power publisher", hint="it runs only for models with telemetry/power.yaml or an assembly power block, robots_definition shows the power profile")
    (power, power_error), (energy, energy_error) = await asyncio.gather(
        _maybe(ROS.latest(Power, power_topic, qos=RELIABLE, timeout=3.0)),
        _maybe(ROS.latest(Energy, energy_topic, qos=RELIABLE, timeout=3.0)),
    )
    if power is None and energy is None:
        raise ToolError(f"no power or energy message from {entry['name']}", hint=f"{power_error}, the publisher only reports while joint_states arrive")
    return {
        "robot": entry["name"],
        "power": msgs.to_dict(power) if power is not None else {"error": power_error},
        "energy": msgs.to_dict(energy) if energy is not None else {"error": energy_error},
    }
