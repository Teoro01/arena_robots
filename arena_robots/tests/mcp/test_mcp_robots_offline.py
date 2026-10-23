from __future__ import annotations

import json
import os
from pathlib import Path

import pytest
import yaml

pytest.importorskip("arena_mcp")

from arena_mcp.core import ToolError
from arena_robots import mcp
from arena_robots.Robot import RobotView

ROBOTS_DIR = Path(__file__).resolve().parents[2] / "robots"


def test_catalog_lists_every_robot_directory_with_its_caps():
    catalog = mcp.robots_catalog()
    assert [entry["model"] for entry in catalog] == sorted(path.parent.name for path in ROBOTS_DIR.glob("*/model_params.yaml"))
    jackal = next(entry for entry in catalog if entry["model"] == "jackal")
    assert "error" not in jackal
    assert jackal["caps"] == ["mobile"]
    assert "laserscan" in jackal["sensor_types"]
    assert jackal["summary"].startswith("mobile (non-holonomic, radius 0.267 m)")
    kairos = next(entry for entry in catalog if entry["model"] == "rbkairos_plus")
    assert "arm x1" in kairos["summary"]


def test_jackal_definition_carries_limits_sensors_and_only_the_goto_endpoint():
    definition = mcp.robots_definition("jackal")
    authored = yaml.safe_load((ROBOTS_DIR / "jackal" / "caps" / "mobile.yaml").read_text())
    assert definition["mobile"]["radius"] == authored["radius"]
    assert definition["mobile"]["footprint"] == authored["footprint"]
    limits = definition["mobile"]["velocity_limits"]
    assert [limits["linear"]["min"], limits["linear"]["max"]] == authored["velocity_limits"]["linear"]
    assert definition["mobile"]["planar"]["acceleration"]["lateral"] is None
    assert definition["control"]["cmd_vel_topic"] == "jackal_velocity_controller/cmd_vel"
    sensors = {sensor["name"]: sensor for sensor in definition["sensors"]}
    assert sensors["lidar"]["type"] == "laserscan"
    assert sensors["lidar"]["topic"].startswith("${namespace}/")
    assert set(definition["actions"]["endpoints"]) == {"goto_pose"}
    assert definition["actions"]["endpoints"]["goto_pose"]["action"] == "arena_robots_msgs/action/GotoPose"
    nav2 = definition["actions"]["bringups"]["mobile"]["nav2"]
    assert nav2["compatible"] and "goto_pose" in nav2["task_kinds"]
    assert "lidar" in definition["assembly"]["variants"]
    assert "jackal_default" in definition["assembly"]["variants"]["lidar"]


def test_arm_robot_definition_lists_named_poses_gestures_and_arm_endpoints():
    definition = mcp.robots_definition("rbkairos_plus")
    authored = yaml.safe_load((ROBOTS_DIR / "rbkairos_plus" / "caps" / "arm.yaml").read_text())
    arm = definition["arm"]["arm"]
    assert arm["named_poses"] == sorted(authored["arm"]["named_poses"])
    assert arm["raw"]["controller"] == authored["arm"]["controller"]
    assert arm["gestures"], "the shared gesture library is empty"
    assert all(gesture["playable"] == (not gesture["missing_poses"]) for gesture in arm["gestures"].values() if "error" not in gesture)
    assert set(definition["actions"]["endpoints"]) == {"goto_pose", "reach_pose", "play_gesture"}
    assert any(sensor["type"] == "image" for sensor in definition["sensors"])


@pytest.mark.parametrize("model", sorted(path.parent.name for path in ROBOTS_DIR.glob("*/model_params.yaml")))
def test_every_model_resolves_to_a_json_definition(model):
    definition = mcp.robots_definition(model)
    assert definition["model"] == model and "mobile" in definition["caps"]
    json.dumps(definition, allow_nan=False)
    bringup_errors = {cap: {kind: entry["error"] for kind, entry in kinds.items() if "error" in entry} for cap, kinds in definition["actions"]["bringups"].items()}
    assert all(not errors for errors in bringup_errors.values()), bringup_errors


def test_unknown_model_names_the_known_models():
    with pytest.raises(ToolError) as error:
        mcp.robots_definition("no_such_robot")
    assert "jackal" in error.value.hint


def test_definition_findings_flag_a_broken_robot(tmp_path):
    robot = tmp_path / "broken"
    robot.mkdir()
    params = {
        "robot_model": "other",
        "base_frame": "base_link",
        "sensors": [{"name": "sonar", "type": "sonar", "topic": "/sonar", "frame": "base_link"}],
        "control": {"mode": "ros2_control", "controllers": []},
    }
    (robot / "model_params.yaml").write_text(yaml.safe_dump(params))
    findings = mcp._definition_findings(RobotView(robot))
    by_check = {(finding["check"], finding["severity"]) for finding in findings}
    assert ("model_params", "warning") in by_check
    assert ("caps", "error") in by_check
    messages = " ".join(finding["message"] for finding in findings)
    assert "'sonar'" in messages and "${namespace}" in messages
    assert "no controllers" in messages
    assert "control.yaml not found" in messages


def test_definition_findings_are_empty_for_jackal():
    assert mcp._definition_findings(mcp._view("jackal")) == []


def test_live_sensors_resolve_the_fleet_parts_into_namespaced_topics():
    entry = {"name": "jackal", "model": "jackal", "ns": "/arena/env_0/task_generator_node/jackal", "params": {"lidar@front": "jackal_default", "imu@imu": "jackal_default"}}
    sensors = mcp._live_sensors(entry)
    assert {sensor["type"] for sensor in sensors} >= {"laserscan", "imu"}
    assert all(sensor["topic"].startswith(entry["ns"] + "/") for sensor in sensors)


def test_live_sensors_name_the_robot_when_its_parts_do_not_resolve():
    entry = {"name": "scout", "model": "jackal", "ns": "/arena/env_0/task_generator_node/scout", "params": {"lidar@front": "no_such_lidar"}}
    with pytest.raises(ToolError, match=r"sensors of scout \(jackal\) do not resolve .*lidar/no_such_lidar"):
        mcp._live_sensors(entry)


@pytest.fixture
def fake_arena(tmp_path):
    arena = Path(os.environ["ARENA_DIR"]) if "ARENA_DIR" in os.environ else None
    if arena is None or not (arena / "_meta" / "arena_cli").is_dir():
        pytest.skip("needs ARENA_DIR with the arena CLI")
    (tmp_path / "_meta").symlink_to(arena / "_meta")
    urdf = tmp_path / "arena_robots" / "arena_robots" / "robots" / "ghost" / "urdf"
    urdf.mkdir(parents=True)
    (urdf / "ghost.urdf").write_text('<robot name="ghost"><link name="base"><visual><geometry><mesh filename="package://arena_robots/robots/ghost/meshes/body.stl"/></geometry></visual></link></robot>\n')
    os.environ["ARENA_DIR"] = str(tmp_path)
    yield tmp_path
    os.environ["ARENA_DIR"] = str(arena)


def test_uri_check_turns_the_cli_failure_listing_into_findings(fake_arena):
    uri = mcp._uri_check(include_uninitialized=False)
    assert uri["missing"] == [{"owner": "ghost", "uri": "robots/ghost/meshes/body.stl", "sources": ["arena_robots/robots/ghost/urdf/ghost.urdf"]}]
    assert "feature robots check" in uri["command"]


def test_check_of_jackal_runs_the_cli_and_reports_no_definition_errors():
    if "ARENA_DIR" not in os.environ:
        pytest.skip("needs ARENA_DIR for arena feature robots check")
    report = mcp.robots_check("jackal")
    assert report["models"] == ["jackal"]
    assert [finding for finding in report["findings"] if finding["check"] != "mesh_uri"] == []
    assert all(finding["robot"] == "jackal" or "/" in finding["robot"] for finding in report["findings"])
