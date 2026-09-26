from __future__ import annotations

import json
from typing import ClassVar

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.Sensor import SensorSpec, SensorType
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_nav2() -> type:
    from arena_robots.task_server_handlers.goto_pose.nav2 import GotoPoseHandlerNav2

    return GotoPoseHandlerNav2


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class Nav2Bringup(Bringup):
    kind = "nav2"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_nav2}

    @property
    def native_action_name(self) -> str:
        return self.namespace("navigate_to_pose")

    @property
    def bt_node_name(self) -> str:
        return self.namespace("bt_navigator")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        global_planner: str = "navfn",
        local_planner: str = "regulated_pure_pursuit",
        inter_planner: str = "default",
        train_mode: bool = False,
        task_generator_node: str = "",
        env_namespace: str = "",
        sensors: list[SensorSpec] | None = None,
        params_overlay: str = "",
        **_: object,
    ) -> list[Action]:
        launch_file = PathJoinSubstitution(
            [
                FindPackageShare("arena_robots"),
                "launch",
                "adapters",
                "mobile",
                "nav2.launch.py",
            ]
        )
        launch_arguments = {
            "robot": self.robot.name,
            "namespace": self.namespace,
            "use_sim_time": str(use_sim_time).lower(),
            "frame": frame,
            "global_planner": global_planner,
            "local_planner": local_planner,
            "inter_planner": inter_planner,
            "train_mode": str(train_mode).lower(),
            "task_generator_node": task_generator_node,
            "env_namespace": env_namespace,
            "params_overlay": params_overlay,
        }
        if sensors is not None:
            launch_arguments["sensors_json"] = json.dumps(
                [
                    {
                        "name": s.name,
                        "type": s.type.value if isinstance(s.type, SensorType) else str(s.type),
                        "topic": s.topic,
                    }
                    for s in sensors
                ]
            )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments=launch_arguments.items(),
            )
        ]
