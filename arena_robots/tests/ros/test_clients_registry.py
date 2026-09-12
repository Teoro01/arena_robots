"""Tests for arena_robots.clients registry."""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _ros_gate():
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pytest.skip("ROS2 not available")


def test_goto_pose_is_registered():
    from arena_robots.clients import CLIENTS
    from arena_robots.clients.goto_pose import GotoPoseClient
    from arena_robots.task_kinds import TaskKind

    assert TaskKind.GOTO_POSE in CLIENTS
    assert CLIENTS.get(TaskKind.GOTO_POSE) is GotoPoseClient


def test_client_task_kind_classvar_matches_registry_key():
    """Convention guard: each client's `task_kind` ClassVar must equal its registry key."""
    from arena_robots.clients import CLIENTS

    for key in CLIENTS.keys():
        cls = CLIENTS.get(key)
        assert cls.task_kind == key, f"{cls.__name__}.task_kind={cls.task_kind!r} != registry key {key!r}"
