from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from arena_robots.catalog import Catalog, render_control_joints
from arena_robots.Robot import RobotIdentifier
from arena_robots.sensor_geometry import render_urdf
from arena_simulation_setup.utils.models.urdf import _inject_ros2_control_joints, _registered_hardware_plugins, _strip_unregistered_ros2_control

PKG = Path(__file__).resolve().parents[2] / "arena_robots"
ALL_ROBOTS = sorted(p.name for p in (PKG / "robots").iterdir() if (p / "model_params.yaml").is_file())


@pytest.mark.parametrize("robot", ALL_ROBOTS)
def test_every_ros2_control_block_runs_on_the_gazebo_sim_system(robot, require_installed):
    pytest.importorskip("xacro")
    require_installed(robot)
    view = RobotIdentifier(robot).resolve_sync()
    root = ET.fromstring(render_urdf(view, {}))
    _strip_unregistered_ros2_control(root, _registered_hardware_plugins())
    resolved = view._resolved({})
    if resolved is not None and view.assembly is not None:
        _inject_ros2_control_joints(root, render_control_joints(resolved, Catalog(root=PKG / "components"), prefix=view.assembly.prefix))
    plugins = [el.findtext("hardware/plugin", "").strip() for el in root.findall("ros2_control")]
    assert plugins
    assert set(plugins) == {"gz_ros2_control/GazeboSimSystem"}, plugins
