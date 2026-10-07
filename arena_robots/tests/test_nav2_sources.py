from __future__ import annotations

import math
from pathlib import Path

import launch
import pytest
import yaml
from arena_rclpy_mixins.yaml_replace import YAMLReplacer
from arena_robots.nav2 import (
    SensorsDerivedYAML,
    compile_sensors_to_collision_monitor,
    compile_sensors_to_nav2,
    compile_sensors_to_stvl,
    split_planar_sensors,
)
from arena_robots.Sensor import SensorSpec, SensorType
from arena_robots.sensor_geometry import SensorGeometry

PKG = Path(__file__).resolve().parents[2] / "arena_robots"
ROBOTS_DIR = PKG / "robots"
NAV2_YAML = PKG / "config" / "nav2" / "nav2.yaml"


def _spec(name: str, type_: SensorType, topic: str) -> SensorSpec:
    return SensorSpec(name=name, type=type_, topic=topic, frame="")


def test_collision_sources_typed_by_sensor():
    sources = compile_sensors_to_collision_monitor(
        [
            _spec("lidar", SensorType.LASERSCAN, "ns/lidar"),
            _spec("rgbd_camera_points", SensorType.POINTCLOUD, "ns/rgbd_camera/points"),
            _spec("rgbd_camera_image", SensorType.IMAGE, "ns/rgbd_camera/image"),
        ]
    )
    assert sources["lidar"] == {"type": "scan", "topic": "ns/lidar"}
    assert sources["rgbd_camera_points"] == {
        "type": "pointcloud",
        "topic": "ns/rgbd_camera/points",
        "min_height": 0.1,
        "max_height": 2.0,
    }
    assert "rgbd_camera_image" not in sources


def _lidar_spec(name: str, sensor: str, *, cloud: bool = True) -> list[SensorSpec]:
    scan = SensorSpec(name=name, type=SensorType.LASERSCAN, topic=f"ns/{name}", frame="", sensor=sensor)
    points = SensorSpec(name=f"{name}_points", type=SensorType.POINTCLOUD, topic=f"ns/{name}/points", frame="", sensor=sensor)
    return [scan, points] if cloud else [scan]


_LIDAR_3D = SensorGeometry("lidar", 2 * math.pi, -0.2618, 0.2618, 0.08, 12.0)
_LIDAR_2D = SensorGeometry("lidar", 4.5379, 0.0, 0.0, 0.1, 10.0)
_CAMERA = SensorGeometry("camera", 1.5184, -0.6, 0.6, 0.105, 8.0)


def _stvl(specs: list[SensorSpec], geometry: dict[str, SensorGeometry], default_range: float) -> dict:
    planar, volumetric = split_planar_sensors(specs, geometry)
    assert planar == []
    return compile_sensors_to_stvl(volumetric, geometry, default_range=default_range)


def test_stvl_marks_each_sensor_once_from_its_cloud():
    sources = _stvl(_lidar_spec("lidar", "gpu_lidar"), {"gpu_lidar": _LIDAR_3D}, 30.0)
    assert list(sources) == ["lidar_points", "lidar_points_clearing"]
    marking, frustum = sources["lidar_points"], sources["lidar_points_clearing"]
    assert (marking["marking"], marking["clearing"]) == (True, False)
    assert (frustum["marking"], frustum["clearing"]) == (False, True)
    assert marking["topic"] == frustum["topic"] == "ns/lidar/points"
    assert marking["obstacle_range"] == 11.0
    assert marking["min_obstacle_height"] == 0.1
    assert (frustum["min_z"], frustum["max_z"]) == (0.08, 12.0)
    assert frustum["vertical_fov_angle"] == pytest.approx(0.5236)
    assert frustum["horizontal_fov_angle"] > 6.27


def test_planar_lidar_raytraces_its_scan_outside_the_voxel_layer():
    specs = _lidar_spec("front", "front_laser")
    planar, volumetric = split_planar_sensors(specs, {"front_laser": _LIDAR_2D})
    assert (planar, volumetric) == ([specs[0]], [])
    source = compile_sensors_to_nav2(planar, max_range=10.0)["front"]
    assert (source["data_type"], source["marking"], source["clearing"]) == ("LaserScan", True, True)
    assert source["min_obstacle_height"] == 0.05


def test_planar_lidar_without_a_scan_raytraces_its_cloud():
    cloud = _lidar_spec("front", "front_laser")[1]
    assert split_planar_sensors([cloud], {"front_laser": _LIDAR_2D}) == ([cloud], [])


def test_scan_only_sensor_without_geometry_is_planar():
    specs = _lidar_spec("rear", "rear_laser", cloud=False)
    assert split_planar_sensors(specs, {}) == (specs, [])


def test_stvl_camera_frustum_stays_inside_the_image_corners():
    camera = SensorSpec(name="cam_points", type=SensorType.POINTCLOUD, topic="ns/cam/points", frame="", sensor="cam")
    frustum = _stvl([camera], {"cam": _CAMERA}, 30.0)["cam_points_clearing"]
    assert frustum["horizontal_fov_angle"] == pytest.approx(1.5184)
    half = frustum["vertical_fov_angle"] / 2
    assert math.tan(half) == pytest.approx(math.tan(0.6) * math.cos(1.5184 / 2))
    assert (frustum["min_z"], frustum["max_z"]) == (0.105, 8.0)


def test_stvl_sensor_without_geometry_marks_only():
    sources = _stvl(_lidar_spec("lidar", "gpu_lidar"), {}, 12.0)
    assert list(sources) == ["lidar_points"]
    assert sources["lidar_points"]["obstacle_range"] == 11.0


def test_stvl_lidar_looking_only_upward_marks_only():
    upward = SensorGeometry("lidar", 2 * math.pi, 0.1, 0.5, 0.1, 10.0)
    assert list(_stvl(_lidar_spec("lidar", "gpu_lidar"), {"gpu_lidar": upward}, 30.0)) == ["lidar_points"]


def test_obstacle_layer_cloud_source_marks_and_clears():
    sources = compile_sensors_to_nav2([_spec("lidar_points", SensorType.POINTCLOUD, "ns/lidar/points")], max_range=12.0)
    assert list(sources) == ["lidar_points"]
    assert (sources["lidar_points"]["marking"], sources["lidar_points"]["clearing"]) == (True, True)


def _costmap_layers(robot: str) -> tuple[dict, dict, dict]:
    pytest.importorskip("xacro")
    robot_dir = ROBOTS_DIR / robot
    derived = SensorsDerivedYAML(str(robot_dir / "model_params.yaml"), str(robot_dir / "caps" / "mobile.yaml")).perform(launch.LaunchContext())
    variables = {"namespace": "ns", **yaml.safe_load(Path(derived).read_text())}
    nav2 = yaml.safe_load(NAV2_YAML.read_text())
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["plugins"] == ["obstacle_layer", "voxel_layer", "inflation_layer"]
    global_ = nav2["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    replacer = YAMLReplacer(variables)
    return replacer.replace(local["obstacle_layer"]), replacer.replace(local["voxel_layer"]), replacer.replace(global_)


@pytest.mark.parametrize(
    ("robot", "planar_sources", "voxel_sources", "global_sources"),
    [
        ("jackal", [], ["lidar_points", "lidar_points_clearing"], ["lidar", "lidar_points"]),
        ("a1", [], ["camera_face_points", "camera_face_points_clearing", "lidar_points", "lidar_points_clearing"], ["lidar_points"]),
        ("mpo700", ["lidar", "lidar_rear"], [], ["lidar", "lidar_points", "lidar_rear", "lidar_rear_points"]),
        (
            "boxer",
            ["front_laser", "rear_laser"],
            ["rgbd_camera_points", "rgbd_camera_points_clearing"],
            ["front_laser", "front_laser_points", "rear_laser", "rear_laser_points", "rgbd_camera_points"],
        ),
    ],
)
def test_costmap_layers_list_their_own_sources(robot, planar_sources, voxel_sources, global_sources):
    planar, voxel, global_ = _costmap_layers(robot)
    assert planar["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert voxel["plugin"] == "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    assert isinstance(planar["observation_sources"], str)
    assert isinstance(voxel["observation_sources"], str)
    assert planar["observation_sources"].split() == planar_sources
    assert voxel["observation_sources"].split() == voxel_sources
    assert global_["observation_sources"].split() == global_sources
    for layer in (planar, voxel, global_):
        for name in layer["observation_sources"].split():
            assert layer[name]["topic"].startswith("ns/"), name
    for layer, names in ((planar, planar_sources), (global_, global_sources)):
        for name in names:
            assert (layer[name]["marking"], layer[name]["clearing"]) == (True, True)
