from __future__ import annotations

import collections
from pathlib import Path

import pytest
import yaml
from arena_robots.sensors import SENSOR_TYPES

ROBOTS_DIR = Path(__file__).resolve().parents[2] / "arena_robots" / "robots"


def _robot_dirs() -> list[Path]:
    return [p for p in ROBOTS_DIR.iterdir() if p.is_dir()]


def _model_params_files() -> list[Path]:
    return list(ROBOTS_DIR.rglob("model_params.yaml"))


def test_no_robot_named_auto() -> None:
    names = {p.name for p in _robot_dirs()}
    assert "auto" not in names, "'auto' is a reserved token and must not be used as a robot directory name"


@pytest.mark.parametrize("yaml_path", _model_params_files())
def test_sensor_types_are_canonical(yaml_path: Path) -> None:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    for entry in data.get("sensors", []):
        sensor_type = entry.get("type", "")
        assert sensor_type in SENSOR_TYPES, f"{yaml_path}: sensor type {sensor_type!r} is not in SENSOR_TYPES"


def test_priority_uniqueness_informational(capsys: pytest.CaptureFixture[str]) -> None:
    Sig = tuple[bool, frozenset[str]]
    groups: dict[Sig, list[tuple[str, int]]] = collections.defaultdict(list)

    for yaml_path in _model_params_files():
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        robot_name = yaml_path.parent.name
        is_holonomic = bool(data.get("is_holonomic", False))
        sensor_types: frozenset[str] = frozenset(str(s.get("type", "")) for s in data.get("sensors", []))
        priority = int(data.get("priority", 0))
        sig: Sig = (is_holonomic, sensor_types)
        groups[sig].append((robot_name, priority))

    for sig, entries in groups.items():
        max_p = max(p for _, p in entries)
        top = [name for name, p in entries if p == max_p]
        if len(top) > 1:
            print(f"priority tie: holonomic={sig[0]} sensors={sorted(sig[1])}, robots with max priority {max_p}: {sorted(top)}")
