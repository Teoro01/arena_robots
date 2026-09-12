"""Tests for arena_robots.bringup registry (BRINGUPS, check_caps)."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from arena_rclpy_mixins.registry import ClassRegistry


def _make_mock_robot(caps_available: frozenset[str], name: str = "test_robot") -> object:
    mock_caps = MagicMock()
    mock_caps.available = caps_available
    mock_robot = MagicMock()
    mock_robot.name = name
    mock_robot.caps = mock_caps
    return mock_robot


class TestBringupsRegistry:
    def test_duplicate_kind_raises(self):
        reg: ClassRegistry = ClassRegistry()
        reg.register("dup")(lambda: object)
        with pytest.raises((ValueError, AssertionError)):
            reg.register("dup")(lambda: object)

    def test_register_and_get(self):
        reg: ClassRegistry = ClassRegistry()

        class _Cls:
            pass

        reg.register("mykey")(lambda: _Cls)
        assert reg.get("mykey") is _Cls

    def test_unknown_kind_raises_key_error(self):
        reg: ClassRegistry = ClassRegistry()
        with pytest.raises(KeyError):
            reg.get("__no_such__")

    def test_lazy_loading(self):
        reg: ClassRegistry = ClassRegistry()

        @reg.register("bad")
        def _bad():
            raise RuntimeError("imported too eagerly")

        class _Good:
            pass

        @reg.register("good")
        def _good():
            return _Good

        assert reg.get("good") is _Good

    def test_nav2_is_registered(self):
        from arena_robots.bringup import BRINGUPS
        from arena_robots.bringup.mobile.nav2 import Nav2Bringup

        assert "nav2" in BRINGUPS["mobile"]
        assert BRINGUPS["mobile"].get("nav2") is Nav2Bringup

    def test_none_is_registered(self):
        from arena_robots.bringup import BRINGUPS
        from arena_robots.bringup.mobile.none import NoneBringup

        assert "none" in BRINGUPS["mobile"]
        assert BRINGUPS["mobile"].get("none") is NoneBringup

    def test_external_is_registered(self):
        from arena_robots.bringup import BRINGUPS
        from arena_robots.bringup.mobile.external import ExternalBringup

        assert "external" in BRINGUPS["mobile"]
        assert BRINGUPS["mobile"].get("external") is ExternalBringup

    def test_test_collision_is_registered(self):
        from arena_robots.bringup import BRINGUPS
        from arena_robots.bringup.mobile.test_collision import TestCollisionBringup

        assert "test-collision" in BRINGUPS["mobile"]
        assert BRINGUPS["mobile"].get("test-collision") is TestCollisionBringup

    def test_bringup_kind_classvar_matches_registry_key(self):
        """Convention guard: each bringup's `kind` ClassVar must equal its registry key."""
        from arena_robots.bringup import BRINGUPS

        for cap, reg in BRINGUPS.items():
            for key in reg.keys():
                cls = reg.get(key)
                assert cls.kind == key, f"{cls.__name__}.kind={cls.kind!r} != registry key {key!r} (cap={cap!r})"

    def test_bringup_meta_attached_on_every_bringup(self):
        from arena_robots.bringup import BRINGUPS, BringupMeta

        for reg in BRINGUPS.values():
            for key in reg.keys():
                cls = reg.get(key)
                assert isinstance(cls._bringup_meta, BringupMeta), f"{cls.__name__} missing _bringup_meta"


class TestAcceptsTaskKinds:
    def test_nav2_bringup_accepts_goto_pose(self):
        from arena_robots.bringup.mobile.nav2 import Nav2Bringup
        from arena_robots.task_kinds import TaskKind

        robot = _make_mock_robot(frozenset({"mobile"}))
        b = Nav2Bringup(robot=robot, namespace="/robot1")
        assert TaskKind.GOTO_POSE in b.accepts_task_kinds

    def test_none_bringup_accepts_goto_pose(self):
        from arena_robots.bringup.mobile.none import NoneBringup
        from arena_robots.task_kinds import TaskKind

        robot = _make_mock_robot(frozenset({"mobile"}))
        b = NoneBringup(robot=robot, namespace="/robot1")
        assert TaskKind.GOTO_POSE in b.accepts_task_kinds
