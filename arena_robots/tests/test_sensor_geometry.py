from __future__ import annotations

import math
from pathlib import Path

import pytest
from arena_robots.Robot import RobotIdentifier
from arena_robots.sensor_geometry import parse_sensor_geometry, sensor_geometry

PKG = Path(__file__).resolve().parents[2] / "arena_robots"
ROBOTS_DIR = PKG / "robots"
GOLDEN_DIR = Path(__file__).parent / "golden"
GOLDEN_ROBOTS = sorted(p.name.removesuffix("_default.urdf") for p in GOLDEN_DIR.glob("*_default.urdf"))
ALL_ROBOTS = sorted(p.name for p in ROBOTS_DIR.iterdir() if (p / "model_params.yaml").is_file())
_RANGING = {"laserscan", "pointcloud"}


def _golden(robot: str) -> dict:
    return parse_sensor_geometry((GOLDEN_DIR / f"{robot}_default.urdf").read_text())


def test_jackal_lidar_is_a_16_ring_360_degree_scanner():
    lidar = _golden("jackal")["gpu_lidar"]
    assert lidar.kind == "lidar"
    assert lidar.horizontal_fov == pytest.approx(2 * math.pi, abs=1e-4)
    assert (lidar.vertical_min, lidar.vertical_max) == pytest.approx((-0.261799, 0.261799))
    assert (lidar.min_range, lidar.max_range) == (0.08, 12.0)
    assert not lidar.planar


def test_boxer_lasers_are_planar_and_its_realsense_is_a_camera():
    shapes = _golden("boxer")
    assert shapes["front_laser"].planar
    assert shapes["rear_laser"].horizontal_fov == pytest.approx(math.pi / 2)
    camera = shapes["realsense_front_camera"]
    assert camera.kind == "camera"
    assert camera.horizontal_fov == pytest.approx(1.5184351666666667)
    assert math.tan(camera.vertical_max) == pytest.approx(math.tan(1.5184351666666667 / 2) * 480 / 640)
    assert (camera.min_range, camera.max_range) == (0.105, 8.0)


def test_turtlebot_cliff_rays_and_rgbd_camera_are_parsed():
    shapes = _golden("turtlebot")
    assert shapes["rgbd_camera"].max_range == 100.0
    assert shapes["cliff_front_left"].max_range == 0.15


@pytest.mark.parametrize("robot", GOLDEN_ROBOTS)
def test_every_ranging_sensor_has_golden_geometry(robot):
    shapes = _golden(robot)
    view = RobotIdentifier(robot).resolve_sync()
    backing = {s.sensor for s in view.model_params.sensors if str(s.type) in _RANGING}
    assert backing, robot
    assert backing <= set(shapes), sorted(backing - set(shapes))


@pytest.mark.parametrize("robot", ALL_ROBOTS)
def test_rendered_urdf_geometry_covers_every_ranging_sensor(robot):
    pytest.importorskip("xacro")
    view = RobotIdentifier(robot).resolve_sync()
    shapes = sensor_geometry(view, {})
    backing = {s.sensor for s in view.effective_sensors({}) if str(s.type) in _RANGING}
    assert backing <= set(shapes), sorted(backing - set(shapes))
    if robot in GOLDEN_ROBOTS:
        golden = _golden(robot)
        assert {name: shapes[name] for name in backing} == {name: golden[name] for name in backing}
