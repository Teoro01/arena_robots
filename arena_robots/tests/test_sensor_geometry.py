from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from arena_robots.assembly import AssemblySpec, RequestPart
from arena_robots.catalog import ComponentSpec
from arena_robots.Robot import RobotIdentifier
from arena_robots.sensor_geometry import parse_sensor_geometry, render_urdf, sensor_geometry

PKG = Path(__file__).resolve().parents[2] / "arena_robots"
ROBOTS_DIR = PKG / "robots"
GOLDEN_DIR = Path(__file__).parent / "golden"
GOLDEN_ROBOTS = sorted(p.name.removesuffix("_default.urdf") for p in GOLDEN_DIR.glob("*_default.urdf"))
ALL_ROBOTS = sorted(p.name for p in ROBOTS_DIR.iterdir() if (p / "model_params.yaml").is_file())
COMPONENTS = sorted(p.parent.relative_to(PKG / "components").as_posix() for p in (PKG / "components").glob("*/*/component.yaml"))
ASSEMBLIES = {p.parent.name: AssemblySpec.parse(yaml.safe_load(p.read_text()) or {}) for p in sorted(ROBOTS_DIR.glob("*/assembly.yaml"))}
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
    assert shapes["rear_laser"].horizontal_fov == pytest.approx(math.radians(80))
    camera = shapes["realsense_front_camera"]
    assert camera.kind == "camera"
    assert camera.horizontal_fov == pytest.approx(1.5184351666666667)
    assert math.tan(camera.vertical_max) == pytest.approx(math.tan(1.5184351666666667 / 2) * 480 / 640)
    assert (camera.min_range, camera.max_range) == (0.105, 8.0)


def test_turtlebot_cliff_rays_and_rgbd_camera_are_parsed():
    shapes = _golden("turtlebot")
    assert shapes["oakd"].max_range == 100.0
    assert shapes["cliff_front_left"].max_range == 0.15


@pytest.mark.parametrize("robot", GOLDEN_ROBOTS)
def test_caps_laser_matches_the_rendered_lidar(robot):
    view = RobotIdentifier(robot).resolve_sync()
    laser = view.mobile.laser
    scan = next(s for s in view.model_params.sensors if str(s.type) == "laserscan")
    shape = _golden(robot)[scan.sensor]
    fov = laser.angle.max - laser.angle.min
    assert fov == pytest.approx(shape.horizontal_fov, abs=1e-4)
    assert laser.num_beams == shape.samples
    assert laser.range == pytest.approx(shape.max_range)
    assert laser.update_rate == int(shape.update_rate)
    assert abs(laser.angle.increment * (laser.num_beams - 1) - fov) <= laser.angle.increment + 1e-6


@pytest.mark.parametrize("robot", GOLDEN_ROBOTS)
def test_every_ranging_sensor_has_golden_geometry(robot):
    shapes = _golden(robot)
    view = RobotIdentifier(robot).resolve_sync()
    backing = {s.sensor for s in view.model_params.sensors if str(s.type) in _RANGING}
    assert backing, robot
    assert backing <= set(shapes), sorted(backing - set(shapes))


@pytest.mark.parametrize("robot", ALL_ROBOTS)
def test_rendered_urdf_geometry_covers_every_ranging_sensor(robot, require_installed):
    pytest.importorskip("xacro")
    require_installed(robot)
    view = RobotIdentifier(robot).resolve_sync()
    shapes = sensor_geometry(view, {})
    backing = {s.sensor for s in view.effective_sensors({}) if str(s.type) in _RANGING}
    assert backing <= set(shapes), sorted(backing - set(shapes))
    if robot in GOLDEN_ROBOTS:
        golden = _golden(robot)
        assert {name: shapes[name] for name in backing} == {name: golden[name] for name in backing}


@pytest.mark.parametrize("robot", ALL_ROBOTS)
def test_pinned_default_request_renders_like_the_defaults(robot, require_installed):
    pytest.importorskip("xacro")
    require_installed(robot)
    view = RobotIdentifier(robot).resolve_sync()
    resolved = view._resolved({})
    if resolved is None:
        pytest.skip(f"{robot} has no assembly.yaml")
    pinned: dict[str, list[RequestPart]] = {}
    for placement in resolved.placements:
        pinned.setdefault(placement.type, []).append(RequestPart(variant=placement.variant, mount=placement.mount.name, overrides=placement.overrides))
    assert sensor_geometry(view, pinned) == sensor_geometry(view, {})


def _dangling_parents(urdf: str) -> list[str]:
    root = ET.fromstring(urdf)
    links = {link.get("name") for link in root.iter("link")}
    return sorted({parent.get("link") for joint in root.iter("joint") for parent in joint.iter("parent")} - links)


def _prefixed_host(part: str, variant: str, robot_local: bool) -> tuple[str, str] | None:
    """A robot with a frame prefix and an unchained mount taking the part, one whose defaults place it for a robot-local include."""
    for name, spec in ASSEMBLIES.items():
        if spec.prefix and (not robot_local or any(default.variant == variant for default in spec.defaults.get(part, []))):
            for mount in spec.mounts.values():
                if part in mount.accepts and mount.chained_parent is None:
                    return name, mount.name
    return None


@pytest.mark.parametrize("component", COMPONENTS)
def test_component_on_a_prefixed_chassis_hangs_off_its_prefixed_parent(component, require_installed):
    pytest.importorskip("xacro")
    part, name = component.split("/")
    spec = ComponentSpec.from_yaml(PKG / "components" / component / "component.yaml")
    variant = spec.variants[0] if spec.variants else name
    host = _prefixed_host(part, variant, not spec.xacro_include.startswith("$(") and not Path(spec.xacro_include).is_absolute())
    if host is None:
        pytest.skip(f"no robot with a frame prefix mounts {component}")
    robot, mount = host
    require_installed(robot)
    assert _dangling_parents(render_urdf(RobotIdentifier(robot).resolve_sync(), {part: [RequestPart(variant=variant, mount=mount)]})) == []
