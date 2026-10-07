"""Viewing volume of a robot's ranging sensors, read from its rendered URDF."""

from __future__ import annotations

import math
import tempfile
import typing
import xml.etree.ElementTree as ET
from pathlib import Path

import attrs

if typing.TYPE_CHECKING:
    from arena_robots.assembly import RequestPart
    from arena_robots.Robot import RobotView

_LIDAR_TYPES = frozenset({"gpu_lidar", "gpu_ray", "ray", "lidar"})
_CAMERA_TYPES = frozenset({"depth_camera", "depth", "rgbd_camera", "rgbd"})
_FULL_TURN = 2.0 * math.pi


@attrs.define(frozen=True)
class SensorGeometry:
    """Field of view (rad) and range (m) of one ranging sensor in its x-forward frame."""

    kind: typing.Literal["lidar", "camera"]
    horizontal_fov: float
    vertical_min: float
    vertical_max: float
    min_range: float
    max_range: float

    @property
    def planar(self) -> bool:
        return self.vertical_max <= self.vertical_min


def _float(element: ET.Element, path: str, default: float) -> float:
    found = element.find(path)
    if found is None or found.text is None or not found.text.strip():
        return default
    return float(found.text)


def _lidar(sensor: ET.Element) -> SensorGeometry | None:
    block = sensor.find("lidar")
    if block is None:
        block = sensor.find("ray")
    if block is None:
        return None
    h_min = _float(block, "scan/horizontal/min_angle", 0.0)
    h_max = _float(block, "scan/horizontal/max_angle", 0.0)
    v_min = v_max = 0.0
    if _float(block, "scan/vertical/samples", 1.0) > 1:
        v_min = _float(block, "scan/vertical/min_angle", 0.0)
        v_max = _float(block, "scan/vertical/max_angle", 0.0)
    return SensorGeometry(
        kind="lidar",
        horizontal_fov=min(abs(h_max - h_min), _FULL_TURN),
        vertical_min=min(v_min, v_max),
        vertical_max=max(v_min, v_max),
        min_range=_float(block, "range/min", 0.0),
        max_range=_float(block, "range/max", 0.0),
    )


def _camera(sensor: ET.Element) -> SensorGeometry | None:
    camera = sensor.find("camera")
    if camera is None:
        return None
    horizontal_fov = _float(camera, "horizontal_fov", 1.047)
    width = _float(camera, "image/width", 320.0)
    height = _float(camera, "image/height", 240.0)
    vertical_half = math.atan(math.tan(horizontal_fov / 2.0) * height / width)
    near = _float(camera, "clip/near", 0.1)
    far = _float(camera, "clip/far", 100.0)
    return SensorGeometry(
        kind="camera",
        horizontal_fov=horizontal_fov,
        vertical_min=-vertical_half,
        vertical_max=vertical_half,
        min_range=_float(camera, "depth_camera/clip/near", near),
        max_range=_float(camera, "depth_camera/clip/far", far),
    )


def parse_sensor_geometry(urdf_xml: str) -> dict[str, SensorGeometry]:
    """Geometry of every gz lidar and depth camera in `urdf_xml`, keyed by `<sensor name>`."""
    root = ET.fromstring(urdf_xml)
    out: dict[str, SensorGeometry] = {}
    for gazebo in root.iter("gazebo"):
        for sensor in gazebo.findall("sensor"):
            name = sensor.get("name")
            sensor_type = sensor.get("type")
            if name is None:
                continue
            if sensor_type in _LIDAR_TYPES:
                geometry = _lidar(sensor)
            elif sensor_type in _CAMERA_TYPES:
                geometry = _camera(sensor)
            else:
                continue
            if geometry is not None:
                out[name] = geometry
    return out


def render_urdf(robot: RobotView, parts: dict[str, list[RequestPart]]) -> str:
    """The robot's URDF for a parts request, rendered the way the simulators render it."""
    import xacro

    from arena_robots.catalog import render_wrapper_xacro

    resolved = robot._resolved(parts)
    if resolved is None:
        return xacro.process_file(str(robot.path / "urdf" / f"{robot.name}.urdf.xacro")).toxml()
    with tempfile.NamedTemporaryFile("w", suffix=".urdf.xacro", delete=False) as wrapper:
        wrapper.write(render_wrapper_xacro(robot, resolved))
    try:
        return xacro.process_file(wrapper.name).toxml()
    finally:
        Path(wrapper.name).unlink()


def sensor_geometry(robot: RobotView, parts: dict[str, list[RequestPart]]) -> dict[str, SensorGeometry]:
    """Geometry of the robot's ranging sensors for a parts request, keyed by backing `<sensor name>`."""
    return parse_sensor_geometry(render_urdf(robot, parts))
