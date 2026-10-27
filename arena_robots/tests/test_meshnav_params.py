from __future__ import annotations

import pytest
from arena_robots.bringup.mobile.meshnav import MeshNavBringup
from arena_robots.Robot import RobotIdentifier
from arena_robots.Sensor import SensorSpec, SensorType


def _params(robot: str, mesh_file: str = "/tmp/arena_meshnav/abc.ply") -> tuple[MeshNavBringup, dict]:
    view = RobotIdentifier(robot).resolve_sync()
    bringup = MeshNavBringup(view, f"/arena/env_0/task_generator_node/{robot}")
    return bringup, bringup.parameters(
        odom_frame=f"env_0/{robot}/odom",
        base_frame=f"env_0/{robot}/base",
        mesh_file=mesh_file,
        sensors=view.model_params.sensors,
    )


def test_a_stair_climber_tolerates_risers_a_wheeled_robot_does_not():
    _, go1 = _params("go1")
    _, jackal = _params("jackal")
    assert go1["mesh_map"]["height_diff"]["threshold"] == pytest.approx(0.2)
    assert jackal["mesh_map"]["height_diff"]["threshold"] == pytest.approx(0.1)
    assert go1["mesh_map"]["steepness"]["threshold"] > jackal["mesh_map"]["steepness"]["threshold"]


def test_inflation_follows_the_padded_footprint():
    _, jackal = _params("jackal")
    _, go1 = _params("go1")
    assert jackal["mesh_map"]["static_inflation"]["inscribed_radius"] == pytest.approx(0.2078)
    assert jackal["mesh_map"]["static_inflation"]["inflation_radius"] == pytest.approx(0.7078)
    assert go1["mesh_map"]["static_inflation"]["inscribed_radius"] == pytest.approx(0.09 + 0.23)


def test_frames_are_the_robot_odom_and_base():
    _, params = _params("jackal")
    assert params["global_frame"] == params["odom_frame"] == params["mesh_map"]["global_frame"] == "env_0/jackal/odom"
    assert params["robot_frame"] == "env_0/jackal/base_footprint"


def _bringup(robot: str, meshnav: dict | None = None) -> MeshNavBringup:
    view = RobotIdentifier(robot).resolve_sync()
    if meshnav is not None:
        view.caps.mobile.raw["meshnav"] = meshnav
    return MeshNavBringup(view, f"/arena/env_0/task_generator_node/{robot}")


def _obstacle_params(bringup: MeshNavBringup, sensors: list[SensorSpec] | None = None) -> dict:
    name = bringup.robot.name
    return bringup.obstacle_parameters(
        odom_frame=f"env_0/{name}/odom",
        base_frame=f"env_0/{name}/base",
        mesh_file="/tmp/arena_meshnav/abc.ply",
        sensors=bringup.robot.model_params.sensors if sensors is None else sensors,
    )


def test_the_obstacle_layer_reads_the_filtered_cloud_by_default():
    bringup, params = _params("jackal")
    obstacle = params["mesh_map"]["obstacle"]
    assert obstacle["topic"] == "/arena/env_0/task_generator_node/jackal/meshnav/obstacles"
    assert obstacle["robot_height"] > bringup.settings["robot_height"]
    assert obstacle["axis_frame"] == "env_0/jackal/odom"
    assert obstacle["max_obstacle_dist"] >= 1.0e6
    assert "obstacle_inflation" in params["mesh_map"]["final"]["inputs"]


def test_the_obstacle_layer_is_off_without_range_sensors():
    bringup = _bringup("jackal")
    imu = [spec for spec in bringup.robot.model_params.sensors if spec.type == SensorType.IMU]
    params = bringup.parameters(odom_frame="env_0/jackal/odom", base_frame="env_0/jackal/base", mesh_file="/tmp/arena_meshnav/abc.ply", sensors=imu)
    assert imu
    assert not bringup.sees_obstacles(imu)
    assert "obstacle" not in params["mesh_map"]
    assert params["mesh_map"]["final"]["inputs"] == ["static_inflation"]


def test_the_obstacle_layer_is_off_when_the_robot_turns_it_off():
    bringup = _bringup("jackal", {"obstacles": False})
    params = bringup.parameters(odom_frame="env_0/jackal/odom", base_frame="env_0/jackal/base", mesh_file="/tmp/arena_meshnav/abc.ply", sensors=bringup.robot.model_params.sensors)
    assert "obstacle" not in params["mesh_map"]


def test_the_obstacle_node_reads_one_namespaced_stream_per_range_sensor_preferring_clouds():
    params = _obstacle_params(_bringup("jackal"))
    assert params["pointcloud_topics"] == ["/arena/env_0/task_generator_node/jackal/lidar/points"]
    assert "laserscan_topics" not in params
    scan_only = [spec for spec in _bringup("jackal").robot.model_params.sensors if spec.type == SensorType.LASERSCAN]
    params = _obstacle_params(_bringup("jackal"), scan_only)
    assert params["laserscan_topics"] == ["/arena/env_0/task_generator_node/jackal/lidar"]
    assert "pointcloud_topics" not in params


def test_the_obstacle_node_defaults_follow_nav2():
    bringup = _bringup("jackal")
    params = _obstacle_params(bringup)
    laser = bringup.robot.caps.mobile.laser
    assert laser is not None
    assert params["min_obstacle_height"] == pytest.approx(0.1)
    assert params["max_obstacle_height"] == pytest.approx(bringup.settings["robot_height"])
    assert params["obstacle_range"] == pytest.approx(laser.range - 1.0)
    assert params["obstacle_decay"] == pytest.approx(1.0)
    assert params["max_step"] == pytest.approx(bringup.settings["max_step"])
    assert params["resolution"] == pytest.approx(bringup.settings["resolution"])
    assert params["self_radius"] >= bringup.settings["inscribed_radius"]
    assert params["odom_frame"] == "env_0/jackal/odom"
    assert params["base_frame"] == "env_0/jackal/base"


def test_meshnav_settings_override_the_obstacle_node_and_layer():
    bringup = _bringup("jackal", {"min_obstacle_height": 0.2, "robot_height": 1.0, "obstacle_range": 4.0, "obstacle_decay": 0.5})
    params = _obstacle_params(bringup)
    assert params["min_obstacle_height"] == pytest.approx(0.2)
    assert params["max_obstacle_height"] == pytest.approx(1.0)
    assert params["obstacle_range"] == pytest.approx(4.0)
    assert params["obstacle_decay"] == pytest.approx(0.5)
    layer = bringup.parameters(odom_frame="o", base_frame="b", mesh_file="/tmp/arena_meshnav/abc.ply", sensors=bringup.robot.model_params.sensors)
    assert layer["mesh_map"]["obstacle"]["robot_height"] > 1.0


def test_mbf_replans_at_the_planner_frequency():
    _, params = _params("jackal")
    assert params["planner_frequency"] == pytest.approx(1.0)
    bringup = _bringup("jackal", {"planner_frequency": 2.5})
    assert bringup.parameters(odom_frame="o", base_frame="b", mesh_file="/tmp/arena_meshnav/abc.ply", sensors=[])["planner_frequency"] == pytest.approx(2.5)


def test_the_launch_runs_the_obstacle_node_only_when_the_layer_is_on():
    bringup = _bringup("jackal")
    kwargs = {"mesh_file": "/tmp/arena_meshnav/abc.ply", "odom_frame": "env_0/jackal/odom", "base_frame": "env_0/jackal/base"}
    with_sensors = bringup._launch_actions(sensors=bringup.robot.model_params.sensors, **kwargs)
    without = bringup._launch_actions(sensors=[], **kwargs)
    assert "meshnav_obstacles" in [action.node_executable for action in with_sensors]
    assert "meshnav_obstacles" not in [action.node_executable for action in without]


def test_robots_with_other_limits_keep_separate_working_files():
    _, go1 = _params("go1")
    _, jackal = _params("jackal")
    assert go1["mesh_map"]["mesh_file"] == jackal["mesh_map"]["mesh_file"]
    assert go1["mesh_map"]["mesh_working_file"] != jackal["mesh_map"]["mesh_working_file"]
    assert go1["mesh_map"]["mesh_working_file"].endswith(".h5")


def test_velocity_limits_cap_the_mesh_controller():
    bringup, params = _params("go1")
    limits = bringup.robot.caps.mobile.raw["velocity_limits"]
    assert params["mesh_controller"]["max_lin_velocity"] == pytest.approx(max(abs(v) for v in limits["linear"]))
    assert params["mesh_controller"]["max_ang_velocity"] == pytest.approx(max(abs(v) for v in limits["angular"]))


def test_the_planner_steps_half_a_mesh_cell():
    bringup, params = _params("go1")
    assert params["cvp"]["step_width"] == pytest.approx(bringup.settings["resolution"] / 2.0)


def test_goals_go_to_the_dijkstra_planner_unless_the_robot_picks_cvp():
    bringup, params = _params("go1")
    assert bringup.planner == "dijkstra"
    assert set(params["planners"]) == {"dijkstra", "cvp"}
    view = RobotIdentifier("jackal").resolve_sync()
    view.caps.mobile.raw["meshnav"] = {"planner": "cvp"}
    assert MeshNavBringup(view, "/arena/env_0/task_generator_node/jackal").planner == "cvp"
