"""Default-morphology snapshot tests: each robot's default assembly must render exactly
the golden URDF (links, joints, gz sensors), and its effective sensors must match
model_params.yaml, which robots_manager reads at runtime."""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from arena_robots.assembly import resolve
from arena_robots.catalog import render_wrapper_xacro
from arena_robots.Robot import RobotIdentifier

GOLDEN_DIR = Path(__file__).parent / "golden"
ROBOT_NAMES = sorted(p.name.removesuffix("_default.urdf") for p in GOLDEN_DIR.glob("*_default.urdf"))


class TestEffectiveSensorsMatchModelParams:
    """task_generator's robots_manager reads ``model_params.sensors`` at runtime, so a
    robot's default-request sensor resolution must reproduce it field-for-field,
    order-sensitive."""

    @pytest.mark.parametrize("robot_name", ROBOT_NAMES)
    def test_effective_sensors_matches_model_params(self, robot_name: str) -> None:
        view = RobotIdentifier(robot_name).resolve_sync()
        assert view.assembly is not None, f"{robot_name} has no assembly.yaml"
        assert view.effective_sensors({}) == view.model_params.sensors


def _origin(joint: ET.Element) -> tuple[tuple[float, ...], tuple[float, ...]]:
    origin = joint.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    return tuple(round(float(v), 6) for v in xyz.split()), tuple(round(float(v), 6) for v in rpy.split())


def _sensor_key(sensor: ET.Element) -> tuple[str | None, ...]:
    """A few type-specific key params: enough to distinguish sensors without pinning
    every noise/rate literal (those are unchanged, vendored macro internals)."""
    stype = sensor.get("type")
    if stype == "gpu_lidar":
        horiz = sensor.find("lidar/scan/horizontal")
        if horiz is None:
            return ()
        return (horiz.findtext("samples"), horiz.findtext("min_angle"), horiz.findtext("max_angle"))
    if stype in ("camera", "rgbd_camera"):
        cam = sensor.find("camera")
        return (cam.findtext("horizontal_fov"),) if cam is not None else ()
    return ()


def _canonical_urdf(path: Path) -> tuple[frozenset, frozenset, frozenset]:
    """(links, joints, gz sensors) canonicalized for order-insensitive comparison
    (components append after the chassis)."""
    root = ET.parse(path).getroot()
    links = frozenset(link.get("name") for link in root.iter("link"))
    joints = frozenset(
        (
            joint.get("name"),
            joint.get("type"),
            joint.find("parent").get("link") if joint.find("parent") is not None else None,
            joint.find("child").get("link") if joint.find("child") is not None else None,
            *_origin(joint),
        )
        for joint in root.iter("joint")
    )
    sensors = frozenset((sensor.get("name"), sensor.get("type"), _sensor_key(sensor)) for gz in root.iter("gazebo") for sensor in gz.findall("sensor"))
    return links, joints, sensors


_XACRO = shutil.which("xacro")


@pytest.mark.skipif(_XACRO is None, reason="xacro CLI not on PATH; run under the Arena container (bash arena -c pytest)")
class TestDefaultMorphologyMatchesGolden:
    """The wrapper-rendered URDF for a robot's bare default request must canonicalize to
    the same links/joints/gz-sensors as its golden snapshot; element order may
    legitimately differ (components append after the chassis)."""

    @pytest.mark.parametrize("robot_name", ROBOT_NAMES)
    def test_wrapper_render_matches_golden(self, robot_name: str, tmp_path: Path) -> None:
        view = RobotIdentifier(robot_name).resolve_sync()
        resolved = resolve(view.assembly, {})
        wrapper_path = tmp_path / f"{robot_name}_wrapper.urdf.xacro"
        wrapper_path.write_text(render_wrapper_xacro(view, resolved))

        rendered = subprocess.run([_XACRO, str(wrapper_path)], capture_output=True, text=True, check=True).stdout
        rendered_path = tmp_path / f"{robot_name}_wrapper.urdf"
        rendered_path.write_text(rendered)

        golden_links, golden_joints, golden_sensors = _canonical_urdf(GOLDEN_DIR / f"{robot_name}_default.urdf")
        wrapper_links, wrapper_joints, wrapper_sensors = _canonical_urdf(rendered_path)
        assert wrapper_links == golden_links
        assert wrapper_joints == golden_joints
        assert wrapper_sensors == golden_sensors
