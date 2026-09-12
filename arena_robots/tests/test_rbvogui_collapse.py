"""Composing ``rbvogui[arm=ur5e]`` must reproduce the factory-integrated rbvogui_plus
variant's arm subtree and merged ros2_control tag. Sensor placement/FOV drift between
the pair (lasers parented at chassis_link vs base_link, FOV 2.2689 vs 2.1) is a
documented owner decision, same bucket as rbkairos, and deliberately outside this
gate."""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from arena_robots.assembly import RequestPart, resolve
from arena_robots.catalog import Catalog, render_control_joints, render_wrapper_xacro
from arena_robots.Robot import RobotIdentifier
from arena_simulation_setup.utils.models.urdf import _inject_ros2_control_joints

COMPONENTS_ROOT = Path(__file__).resolve().parent.parent / "components"

_XACRO = shutil.which("xacro")


def _render(path: Path) -> ET.Element:
    rendered = subprocess.run([_XACRO, str(path)], capture_output=True, text=True, check=True).stdout
    return ET.fromstring(rendered)


def _norm(name: str) -> str:
    return name.replace("arm0_", "arm_")


def _canon(el: ET.Element) -> dict:
    out: dict = {"tag": el.tag, "attrs": {k: _canon_value(v) for k, v in sorted(el.attrib.items())}}
    kids = [_canon(c) for c in el]
    if kids:
        out["kids"] = sorted(kids, key=repr)
    if el.text and el.text.strip():
        out["text"] = el.text.strip()
    return out


def _canon_value(value: str) -> str | tuple[float, ...]:
    parts = value.split()
    try:
        return tuple(round(float(p), 6) for p in parts)
    except ValueError:
        return _norm(value)


def _arm_elements(root: ET.Element) -> dict[tuple[str, str], dict]:
    return {(el.tag, _norm(el.get("name", ""))): _canon(el) for el in root if el.tag in ("link", "joint") and "arm" in el.get("name", "")}


@pytest.mark.skipif(_XACRO is None, reason="xacro CLI not on PATH; run under the Arena container (bash arena -c pytest)")
class TestRbvoguiArmCollapseMatchesPlus:
    @pytest.fixture(scope="class")
    def rendered(self, tmp_path_factory: pytest.TempPathFactory) -> tuple[ET.Element, ET.Element]:
        view = RobotIdentifier("rbvogui").resolve_sync()
        assert view.assembly is not None
        resolved = resolve(view.assembly, {"arm": [RequestPart(variant="ur5e")]})
        catalog = Catalog(root=COMPONENTS_ROOT)
        wrapper_path = tmp_path_factory.mktemp("collapse") / "rbvogui_arm.urdf.xacro"
        wrapper_path.write_text(render_wrapper_xacro(view, resolved, catalog=catalog))
        mine = _render(wrapper_path)
        # reproduce the loader's post-render ros2_control merge
        # (arena_simulation_setup.utils.models.urdf._inject_ros2_control_joints) over the
        # separately-rendered chassis and arm tags.
        _inject_ros2_control_joints(mine, render_control_joints(resolved, catalog, prefix=view.assembly.prefix))
        plus_view = RobotIdentifier("rbvogui_plus").resolve_sync()
        plus = _render(plus_view.path / "urdf" / "rbvogui_plus.urdf.xacro")
        return mine, plus

    def test_arm_subtree_structurally_identical(self, rendered: tuple[ET.Element, ET.Element]) -> None:
        mine, plus = rendered
        assert _arm_elements(mine) == _arm_elements(plus)

    def test_one_merged_ros2_control_tag_with_plus_joint_set(self, rendered: tuple[ET.Element, ET.Element]) -> None:
        mine, plus = rendered
        mine_tags = mine.findall("ros2_control")
        plus_tags = plus.findall("ros2_control")
        assert len(mine_tags) == 1
        assert len(plus_tags) == 1
        mine_joints = {_norm(j.get("name", "")) for j in mine_tags[0].iter("joint")}
        plus_joints = {j.get("name", "") for j in plus_tags[0].iter("joint")}
        assert mine_joints == plus_joints
        plugins = [p.text for p in mine_tags[0].iter("plugin")]
        assert plugins == ["gz_ros2_control/GazeboSimSystem"]

    def test_gz_sensor_inventory_matches(self, rendered: tuple[ET.Element, ET.Element]) -> None:
        mine, plus = rendered

        def inventory(root: ET.Element) -> set[tuple[str, str]]:
            return {(sensor.get("name", ""), sensor.get("type", "")) for gz in root.iter("gazebo") for sensor in gz.findall("sensor")}

        assert inventory(mine) == inventory(plus)
