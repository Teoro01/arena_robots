from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, ClassVar

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import Action

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.Sensor import SensorSpec, SensorType, resolve_topic
from arena_robots.task_kinds import TaskKind

_STAIR_STEP = 0.2
_FLAT_STEP = 0.1
_STAIR_SLOPE = 1.3
_FLAT_SLOPE = 0.35
_INFLATION_MARGIN = 0.5
_PLANNERS = ("dijkstra", "cvp")
_LIDAR_RANGE = 10.0
_OBSTACLE_RANGE_MARGIN = 1.0
_RAY_MARGIN = 0.3
_OBSTACLES_TOPIC = "meshnav/obstacles"
_RANGING = (SensorType.LASERSCAN.value, SensorType.POINTCLOUD.value)


def _load_goto_pose_meshnav() -> type:
    from arena_robots.task_server_handlers.goto_pose.meshnav import GotoPoseHandlerMeshNav

    return GotoPoseHandlerMeshNav


def _merged(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    """`base` with `overlay` merged in, mappings recursively, every other value replaced."""
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        result[key] = _merged(result[key], value) if isinstance(value, dict) and isinstance(result.get(key), dict) else copy.deepcopy(value)
    return result


def _footprint_frame(base_frame: str) -> str:
    """The frame MBF tracks the robot by, the base frame dropped to the ground by the robot's z_offset."""
    return f"{base_frame}_footprint"


def _ranging(sensors: list[SensorSpec]) -> dict[str, list[SensorSpec]]:
    """One range sensor spec per backing sensor, its point cloud when it has one, grouped by type."""
    backing: dict[str, dict[str, SensorSpec]] = {}
    for spec in sensors:
        if str(spec.type) in _RANGING:
            backing.setdefault(spec.sensor or spec.name, {})[str(spec.type)] = spec
    grouped: dict[str, list[SensorSpec]] = {kind: [] for kind in _RANGING}
    for by_type in backing.values():
        kind = SensorType.POINTCLOUD.value if SensorType.POINTCLOUD.value in by_type else SensorType.LASERSCAN.value
        grouped[kind].append(by_type[kind])
    return grouped


def _inscribed(footprint: list[list[float]], padding: float) -> float:
    """Radius of the largest circle about the base origin inside the padded footprint polygon."""
    edges = zip(footprint, [*footprint[1:], footprint[0]], strict=True)
    return min(abs(ax * by - ay * bx) / math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in edges) + padding


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class MeshNavBringup(Bringup):
    kind = "meshnav"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_meshnav}

    @property
    def native_action_name(self) -> str:
        return self.namespace("move_base_flex", "move_base")

    @property
    def planner(self) -> str:
        """The MBF planner plugin goals are sent to, `dijkstra` or `cvp`."""
        planner = str(self.settings["planner"])
        if planner not in _PLANNERS:
            raise ValueError(f"{self.robot.name}: meshnav planner {planner!r} is none of {list(_PLANNERS)}")
        return planner

    @property
    def self_radius(self) -> float:
        """Radius about the base origin within which range returns hit the robot's own body."""
        mobile = self.robot.caps.mobile
        if mobile.footprint:
            return max(math.hypot(x, y) for x, y in mobile.footprint) + (mobile.footprint_padding or 0.0)
        return mobile.radius or 0.3

    @property
    def settings(self) -> dict[str, Any]:
        """The robot's `meshnav:` block over defaults derived from its padded footprint or radius and the stair kinds it climbs."""
        mobile = self.robot.caps.mobile
        climbs = "stairs" in mobile.climbs
        radius = _inscribed(mobile.footprint, mobile.footprint_padding or 0.0) if mobile.footprint else mobile.radius or 0.3
        laser = mobile.laser
        defaults = {
            "planner": "dijkstra",
            "resolution": 0.1,
            "max_step": _STAIR_STEP if climbs else _FLAT_STEP,
            "max_slope": _STAIR_SLOPE if climbs else _FLAT_SLOPE,
            "inscribed_radius": radius,
            "inflation_radius": radius + _INFLATION_MARGIN,
            "obstacles": any(_ranging(self.robot.model_params.sensors).values()),
            "robot_height": 0.7,
            "min_obstacle_height": 0.1,
            "obstacle_range": (laser.range if laser is not None else _LIDAR_RANGE) - _OBSTACLE_RANGE_MARGIN,
            "obstacle_decay": 1.0,
            "planner_frequency": 1.0,
            "params": {},
        }
        return {**defaults, **mobile.sub("meshnav")}

    def parameters(self, *, odom_frame: str, base_frame: str, mesh_file: str, sensors: list[SensorSpec]) -> dict[str, Any]:
        """The parameters of the robot's MBF mesh server."""
        settings = self.settings
        with open(os.path.join(get_package_share_directory("arena_robots"), "config", "meshnav", "meshnav.yaml"), encoding="utf-8") as f:
            params = yaml.safe_load(f)["move_base_flex"]["ros__parameters"]
        velocity = self.robot.caps.mobile.raw.get("velocity_limits", {})
        mesh_map = params["mesh_map"]
        mesh_map["height_diff"]["threshold"] = float(settings["max_step"])
        mesh_map["steepness"]["threshold"] = float(settings["max_slope"])
        params["cvp"]["step_width"] = float(settings["resolution"]) / 2.0
        for name in ("static_inflation", "obstacle_inflation"):
            mesh_map[name]["inscribed_radius"] = float(settings["inscribed_radius"])
            mesh_map[name]["inflation_radius"] = float(settings["inflation_radius"])
        if self.sees_obstacles(sensors):
            mesh_map["obstacle"]["topic"] = str(self.namespace(_OBSTACLES_TOPIC))
            mesh_map["obstacle"]["robot_height"] = float(settings["robot_height"]) + _RAY_MARGIN
            mesh_map["obstacle"]["axis_frame"] = odom_frame
        else:
            mesh_map["layers"] = [layer for layer in mesh_map["layers"] if layer not in ("obstacle", "obstacle_inflation")]
            mesh_map["final"]["inputs"] = [layer for layer in mesh_map["final"]["inputs"] if layer != "obstacle_inflation"]
            del mesh_map["obstacle"], mesh_map["obstacle_inflation"]
        if "linear" in velocity:
            params["mesh_controller"]["max_lin_velocity"] = float(max(abs(v) for v in velocity["linear"]))
        if "angular" in velocity:
            params["mesh_controller"]["max_ang_velocity"] = float(max(abs(v) for v in velocity["angular"]))
        params = _merged(params, settings["params"])
        layers = json.dumps(params["mesh_map"], sort_keys=True).encode()
        mesh = Path(mesh_file)
        params["mesh_map"]["mesh_file"] = mesh_file
        params["mesh_map"]["mesh_working_file"] = str(mesh.with_name(f"{mesh.stem}_{hashlib.sha1(layers).hexdigest()[:12]}.h5"))
        params["mesh_map"]["global_frame"] = odom_frame
        params.update(global_frame=odom_frame, odom_frame=odom_frame, robot_frame=_footprint_frame(base_frame), odom_topic="odom")
        params["planner_frequency"] = float(settings["planner_frequency"])
        return params

    def sees_obstacles(self, sensors: list[SensorSpec]) -> bool:
        """Whether the obstacle layer is on and fed by at least one of the range sensors."""
        return bool(self.settings["obstacles"]) and any(_ranging(sensors).values())

    def obstacle_parameters(self, *, odom_frame: str, base_frame: str, mesh_file: str, sensors: list[SensorSpec]) -> dict[str, Any]:
        """The parameters of the robot's obstacle cloud node."""
        settings = self.settings
        params: dict[str, Any] = {
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "mesh_file": mesh_file,
            "resolution": float(settings["resolution"]),
            "min_obstacle_height": float(settings["min_obstacle_height"]),
            "max_obstacle_height": float(settings["robot_height"]),
            "max_step": float(settings["max_step"]),
            "obstacle_range": float(settings["obstacle_range"]),
            "obstacle_decay": float(settings["obstacle_decay"]),
            "self_radius": float(self.self_radius),
        }
        for kind, specs in _ranging(sensors).items():
            if specs:
                params[f"{kind}_topics"] = [resolve_topic(spec, str(self.namespace)) for spec in specs]
        return params

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        task_generator_node: str = "",
        env_namespace: str = "",
        sensors: list[SensorSpec] | None = None,
        mesh_file: str = "",
        odom_frame: str = "",
        base_frame: str = "",
        **_: object,
    ) -> list[Action]:
        from launch_ros.actions import Node

        if not mesh_file:
            raise ValueError(f"meshnav bringup for {self.robot.name!r}: no mesh_file, the task generator builds it from the spawned world before launch")
        sensors = sensors or []
        params = self.parameters(odom_frame=odom_frame, base_frame=base_frame, mesh_file=mesh_file, sensors=sensors)
        obstacles = (
            [
                Node(
                    package="arena_robots",
                    executable="meshnav_obstacles",
                    name="meshnav_obstacles",
                    namespace=str(self.namespace),
                    output="screen",
                    parameters=[{**self.obstacle_parameters(odom_frame=odom_frame, base_frame=base_frame, mesh_file=mesh_file, sensors=sensors), "use_sim_time": use_sim_time}],
                )
            ]
            if self.sees_obstacles(sensors)
            else []
        )
        return [
            *obstacles,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="meshnav_footprint",
                namespace=str(self.namespace),
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["--z", str(-self.robot.model_params.z_offset), "--frame-id", base_frame, "--child-frame-id", _footprint_frame(base_frame)],
            ),
            Node(
                package="mbf_mesh_nav",
                executable="mbf_mesh_nav",
                name="move_base_flex",
                namespace=str(self.namespace),
                output="screen",
                parameters=[{**params, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="twist_stamper",
                executable="twist_unstamper",
                name="meshnav_unstamper",
                namespace=str(self.namespace),
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel_in", "move_base_flex/cmd_vel"), ("cmd_vel_out", "cmd_vel")],
            ),
        ]
