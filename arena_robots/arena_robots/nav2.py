"""Nav2 launch helpers."""

import json
import math
import tempfile
import typing
from pathlib import Path

import launch
import yaml
from arena_bringup.substitutions import YAMLFileSubstitution

from arena_robots.caps import MobileSpec, stringify_float_matrix
from arena_robots.Robot import ModelParams, RobotIdentifier
from arena_robots.Sensor import SensorSpec, SensorType
from arena_robots.sensor_geometry import SensorGeometry, sensor_geometry

_TYPE_TO_NAV2: dict[str, str] = {
    SensorType.LASERSCAN.value: "LaserScan",
    SensorType.POINTCLOUD.value: "PointCloud2",
}

# Fallback raytrace/obstacle range when caps/mobile.yaml carries no `laser:` block.
_DEFAULT_LIDAR_RANGE = 10.0

_CLEARING_SUFFIX = "_clearing"
_STVL_X_FORWARD_FRUSTUM = 1
_STVL_FULL_TURN = 6.29


def compile_sensors_to_nav2(
    sensors: list[SensorSpec],
    *,
    max_range: float,
    obstacle_range_margin: float = 1.0,
    max_obstacle_height: float = 2.0,
    pointcloud_min_obstacle_height: float = 0.1,
    laserscan_min_obstacle_height: float = 0.05,
    clearing: bool = True,
    marking: bool = True,
    inf_is_valid: bool = True,
    extra_per_source: dict[str, typing.Any] | None = None,
) -> dict[str, dict[str, typing.Any]]:
    """Compile SensorSpec entries with a nav2 costmap data_type into nav2's observation_sources_dict shape.

    `max_range` drives `raytrace_max_range` (clearing) so the costmap tracks the full
    sensor range rather than nav2's 3.0 m default. `obstacle_max_range` sits a margin
    below it: Isaac's 3D lidar emits phantom max-range points for missed rays, and a
    margin keeps those from leaking into the costmap as concentric arcs that no later
    raytrace ever clears. `inf_is_valid` lets no-return beams clear out to `max_range`.
    `pointcloud_min_obstacle_height` is a height floor for 3D cloud sources, so their
    ground returns are dropped instead of marked. `laserscan_min_obstacle_height` is a
    lower floor for 2D scans: level returns always sit at beam height, so it only drops
    the ground strikes swept by a momentarily tilted robot (spawn bounce, door sills).
    `extra_per_source` is merged onto every emitted source last, letting callers layer
    layer-specific tunables (e.g. `observation_persistence` for the global costmap).
    """
    obstacle_max_range = max(0.0, max_range - obstacle_range_margin)
    out: dict[str, dict[str, typing.Any]] = {}
    for spec in sensors:
        type_str = spec.type.value if isinstance(spec.type, SensorType) else str(spec.type)
        data_type = _TYPE_TO_NAV2.get(type_str)
        if data_type is None:
            continue
        source: dict[str, typing.Any] = {
            "topic": spec.topic,
            "data_type": data_type,
            "max_obstacle_height": max_obstacle_height,
            "clearing": clearing,
            "marking": marking,
            "obstacle_max_range": obstacle_max_range,
            "raytrace_max_range": max_range,
            "inf_is_valid": inf_is_valid,
        }
        if data_type == "PointCloud2":
            source["min_obstacle_height"] = pointcloud_min_obstacle_height
        elif data_type == "LaserScan":
            source["min_obstacle_height"] = laserscan_min_obstacle_height
        if extra_per_source:
            source.update(extra_per_source)
        out[spec.name] = source
    return out


def _type_str(spec: SensorSpec) -> str:
    return spec.type.value if isinstance(spec.type, SensorType) else str(spec.type)


def split_planar_sensors(
    sensors: list[SensorSpec],
    geometry: typing.Mapping[str, SensorGeometry],
) -> tuple[list[SensorSpec], list[SensorSpec]]:
    """One (planar, volumetric) costmap spec per backing sensor: its scan when planar, else its cloud."""
    groups: dict[str, dict[str, SensorSpec]] = {}
    for spec in sensors:
        type_str = _type_str(spec)
        if type_str in _TYPE_TO_NAV2:
            groups.setdefault(spec.sensor or spec.name, {})[type_str] = spec
    planar: list[SensorSpec] = []
    volumetric: list[SensorSpec] = []
    for backing, by_type in groups.items():
        scan = by_type.get(SensorType.LASERSCAN.value)
        cloud = by_type.get(SensorType.POINTCLOUD.value)
        shape = geometry.get(backing)
        is_planar = shape.planar if shape is not None else cloud is None
        if is_planar:
            planar.append(scan or cloud)
        else:
            volumetric.append(cloud or scan)
    return planar, volumetric


def _clearing_frustum(geometry: SensorGeometry) -> tuple[float, float] | None:
    """(vertical, horizontal) full angles of an x-forward frustum inside the sensor's view."""
    horizontal = geometry.horizontal_fov
    if geometry.kind == "camera":
        vertical_half = math.atan(math.tan(geometry.vertical_max) * math.cos(horizontal / 2.0))
    else:
        vertical_half = min(-geometry.vertical_min, geometry.vertical_max)
    if vertical_half <= 0.0:
        return None
    if horizontal >= 2.0 * math.pi - 1e-3:
        horizontal = _STVL_FULL_TURN
    return 2.0 * vertical_half, horizontal


def compile_sensors_to_stvl(
    sensors: list[SensorSpec],
    geometry: typing.Mapping[str, SensorGeometry],
    *,
    default_range: float,
    obstacle_range_margin: float = 1.0,
    max_obstacle_height: float = 0.8,
    min_obstacle_height: float = 0.1,
    decay_acceleration: float = 5.0,
) -> dict[str, dict[str, typing.Any]]:
    """Compile volumetric specs into voxel-layer marking sources plus `<name>_clearing` frustums inside each sensor's view."""
    out: dict[str, dict[str, typing.Any]] = {}
    for spec in sensors:
        data_type = _TYPE_TO_NAV2[_type_str(spec)]
        shape = geometry.get(spec.sensor or spec.name)
        max_range = shape.max_range if shape is not None else default_range
        out[spec.name] = {
            "topic": spec.topic,
            "data_type": data_type,
            "marking": True,
            "clearing": False,
            "obstacle_range": max(0.0, max_range - obstacle_range_margin),
            "min_obstacle_height": min_obstacle_height,
            "max_obstacle_height": max_obstacle_height,
            "clear_after_reading": True,
        }
        frustum = _clearing_frustum(shape) if shape is not None else None
        if shape is None or frustum is None:
            continue
        vertical, horizontal = frustum
        out[f"{spec.name}{_CLEARING_SUFFIX}"] = {
            "topic": spec.topic,
            "data_type": data_type,
            "marking": False,
            "clearing": True,
            "model_type": _STVL_X_FORWARD_FRUSTUM,
            "min_z": shape.min_range,
            "max_z": max_range,
            "vertical_fov_angle": vertical,
            "horizontal_fov_angle": horizontal,
            "decay_acceleration": decay_acceleration,
        }
    return out


def compile_sensors_to_collision_monitor(
    sensors: list[SensorSpec],
    *,
    pointcloud_min_height: float = 0.1,
    pointcloud_max_height: float = 2.0,
) -> dict[str, dict[str, typing.Any]]:
    """Compile SensorSpec entries into collision_monitor source shape.

    Unlike costmap observation sources (`data_type: LaserScan|PointCloud2`), collision
    monitor sources declare `type: scan|pointcloud` and fall back to scan when the key
    is missing, silently subscribing pointcloud topics with the wrong message type.
    """
    out: dict[str, dict[str, typing.Any]] = {}
    for spec in sensors:
        type_str = spec.type.value if isinstance(spec.type, SensorType) else str(spec.type)
        if type_str == SensorType.LASERSCAN.value:
            out[spec.name] = {"type": "scan", "topic": spec.topic}
        elif type_str == SensorType.POINTCLOUD.value:
            out[spec.name] = {
                "type": "pointcloud",
                "topic": spec.topic,
                "min_height": pointcloud_min_height,
                "max_height": pointcloud_max_height,
            }
    return out


def _load_mobile(path_str: str) -> MobileSpec:
    with open(path_str) as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path_str}: mobile.yaml must be a mapping at top level")
    return MobileSpec(path=Path(path_str), raw=data)


class SensorsDerivedYAML(YAMLFileSubstitution):
    """Emit `observation_sources{,_string,_string_planar,_string_global,_dict,_dict_planar,_dict_global}`
    and `collision_sources{,_dict}` from sensors+laser range.

    Local uses the full lidar range, with planar sensors under the `_planar` keys and
    volumetric ones under the bare keys. Global uses shorter capped ranges and pulls per-source
    overrides (`raytrace_max_range`, `obstacle_max_range`, `observation_persistence`) from
    the optional `nav2.global_observation` block in caps/mobile.yaml.
    """

    _GLOBAL_DEFAULT_RAYTRACE = 6.0
    _GLOBAL_DEFAULT_OBSTACLE = 5.0
    _GLOBAL_DEFAULT_PERSISTENCE = 0.0
    _GLOBAL_DEFAULT_MIN_HEIGHT = 0.1

    def __init__(
        self,
        model_params_path: launch.SomeSubstitutionsType,
        mobile_path: launch.SomeSubstitutionsType,
        sensors_json: launch.SomeSubstitutionsType = '',
    ):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(model_params_path)
        self._mobile_path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)
        self._sensors_json = launch.utilities.normalize_to_list_of_substitutions(sensors_json)

    def perform(self, context: launch.LaunchContext) -> str:
        mobile_str = launch.utilities.perform_substitutions(context, self._mobile_path)
        mobile = _load_mobile(mobile_str)
        max_range = mobile.laser.range if mobile.laser is not None else _DEFAULT_LIDAR_RANGE

        sensors_json_str = launch.utilities.perform_substitutions(context, self._sensors_json)
        if sensors_json_str:
            entries = json.loads(sensors_json_str)
            sensors = [SensorSpec(name=d['name'], type=d['type'], topic=d['topic'], frame='', sensor=d.get('sensor')) for d in entries]
            geometry = {d['sensor']: SensorGeometry(**d['geometry']) for d in entries if d.get('geometry')}
        else:
            path_str = launch.utilities.perform_substitutions(context, self._path)
            sensors = ModelParams.from_yaml(path_str).sensors
            geometry = sensor_geometry(RobotIdentifier(Path(path_str).parent.name).resolve_sync(), {})

        planar, volumetric = split_planar_sensors(sensors, geometry)
        planar_sources = compile_sensors_to_nav2(planar, max_range=max_range)
        local_sources = compile_sensors_to_stvl(volumetric, geometry, default_range=max_range)

        overrides = mobile.raw.get('nav2', {}).get('global_observation', {}) or {}
        g_raytrace = float(overrides.get('raytrace_max_range', min(max_range, self._GLOBAL_DEFAULT_RAYTRACE)))
        g_obstacle = float(overrides.get('obstacle_max_range', min(g_raytrace, self._GLOBAL_DEFAULT_OBSTACLE)))
        g_persistence = float(overrides.get('observation_persistence', self._GLOBAL_DEFAULT_PERSISTENCE))
        g_min_height = float(overrides.get('min_obstacle_height', self._GLOBAL_DEFAULT_MIN_HEIGHT))
        g_include = overrides.get('include')
        if g_include is not None:
            include_names = set(g_include)
            global_sensors = [s for s in sensors if s.name in include_names]
        else:
            g_exclude = {str(t) for t in overrides.get('exclude_types', ())}
            global_sensors = [s for s in sensors if (s.type.value if isinstance(s.type, SensorType) else str(s.type)) not in g_exclude]
        global_sources = compile_sensors_to_nav2(
            global_sensors,
            max_range=g_raytrace,
            obstacle_range_margin=max(0.0, g_raytrace - g_obstacle),
            pointcloud_min_obstacle_height=g_min_height,
            extra_per_source={'observation_persistence': g_persistence},
        )

        collision_sources = compile_sensors_to_collision_monitor(sensors)

        derived = {
            'observation_sources_string': ' '.join(local_sources.keys()),
            'observation_sources_string_planar': ' '.join(planar_sources.keys()),
            'observation_sources_string_global': ' '.join(global_sources.keys()),
            'observation_sources': [*planar_sources.keys(), *local_sources.keys()],
            'observation_sources_dict': local_sources,
            'observation_sources_dict_planar': planar_sources,
            'observation_sources_dict_global': global_sources,
            'collision_sources': list(collision_sources.keys()),
            'collision_sources_dict': collision_sources,
        }
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(derived, tmp)
        tmp.close()
        return tmp.name


class Nav2SubBlockYAML(YAMLFileSubstitution):
    """Extract the `nav2:` sub-block from caps/mobile.yaml and emit it at top level
    as a temp YAML file. Lets YAMLMergeSubstitution treat adapter-specific config
    (footprint, polygons*, planner_plugins*) as flat merge-time keys while they
    stay nested in the authored file."""

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        raw = _load_mobile(path_str).sub('nav2')
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(raw, tmp)
        tmp.close()
        return tmp.name


class Nav2KinematicsDerivedYAML(YAMLFileSubstitution):
    """Emit controller-agnostic velocity/acceleration keys from the top-level
    ``velocity_limits``/``acceleration_limits`` in caps/mobile.yaml.

    These are the planner envelope (what nav2 may sample), not the hardware
    envelope (motor firmware / ``diff_drive_controller`` clip downstream).

    Controller plugin configs reference these via ``${max_linear_vel}`` etc.,
    letting each controller map the generic envelope onto its plugin-specific
    field names. Controllers wanting a lower cap can drop the ``${...}`` ref
    and hardcode the literal instead.
    """

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        mobile = _load_mobile(path_str)

        out: dict[str, typing.Any] = {}
        vel = mobile.velocity_limits
        if vel is not None:
            out['min_linear_vel'] = vel.linear.min
            out['max_linear_vel'] = vel.linear.max
            out['min_angular_vel'] = vel.angular.min
            out['max_angular_vel'] = vel.angular.max
            if vel.lateral is not None:
                out['min_lateral_vel'] = vel.lateral.min
                out['max_lateral_vel'] = vel.lateral.max

        acc = mobile.acceleration_limits
        if acc is not None:
            out['linear_acc'] = acc.linear
            out['angular_acc'] = acc.angular
            out['linear_decel'] = -acc.linear
            out['angular_decel'] = -acc.angular
            if acc.lateral is not None:
                out['lateral_acc'] = acc.lateral
                out['lateral_decel'] = -acc.lateral

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(out if out else {}, tmp)
        tmp.close()
        return tmp.name


class Nav2CollisionDerivedYAML(YAMLFileSubstitution):
    """Compile top-level `footprint` and `polygons_dict` from caps/mobile.yaml
    into the stringified form nav2's collision_monitor expects, overriding any
    raw float lists emitted by the preceding YAMLFileSubstitution(mobile_path)."""

    def __init__(self, mobile_path: launch.SomeSubstitutionsType):
        super().__init__(path=[], default={}, substitute=False)
        self._path = launch.utilities.normalize_to_list_of_substitutions(mobile_path)

    def perform(self, context: launch.LaunchContext) -> str:
        path_str = launch.utilities.perform_substitutions(context, self._path)
        mobile = _load_mobile(path_str)
        raw = mobile.raw

        out: dict[str, typing.Any] = {}

        footprint_raw = raw.get('footprint')
        if isinstance(footprint_raw, list):
            out['footprint'] = stringify_float_matrix([[float(c) for c in pt] for pt in footprint_raw])

        padding = mobile.footprint_padding
        if padding is not None:
            out['footprint_padding'] = padding

        polygons_raw = raw.get('polygons_dict')
        if isinstance(polygons_raw, dict) and polygons_raw:
            out['polygons'] = list(polygons_raw.keys())
            compiled: dict[str, typing.Any] = {}
            for name, entry in polygons_raw.items():
                ptype = entry.get('type')
                polygon_entry: dict[str, typing.Any] = {}
                for field in ('type', 'action_type', 'polygon_pub_topic', 'min_points', 'visualize', 'enabled', 'slowdown_ratio'):
                    if field in entry:
                        polygon_entry[field] = entry[field]
                if ptype == 'polygon':
                    pts = entry.get('points')
                    if isinstance(pts, list):
                        polygon_entry['points'] = stringify_float_matrix([[float(c) for c in pt] for pt in pts])
                    else:
                        polygon_entry['points'] = pts
                elif ptype == 'circle':
                    if 'radius' in entry:
                        polygon_entry['radius'] = entry['radius']
                compiled[name] = polygon_entry
            out['polygons_dict'] = compiled

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(out if out else {}, tmp)
        tmp.close()
        return tmp.name
