from __future__ import annotations

import threading
import time
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")

from arena_robots_msgs.action import GotoPose
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from arena_robots.task_server_handlers import NativeGoals, _executor_sleep


def _wait(future, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, "future did not complete"
        time.sleep(0.02)
    return future.result()


def test_cancelling_a_server_goal_cancels_the_native_goal_it_waits_on():
    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    ns = f"/native_goals_{uuid.uuid4().hex[:8]}"
    node = rclpy.create_node("native_goals", namespace=ns)
    group = ReentrantCallbackGroup()
    native_started = threading.Event()
    native_cancelled = threading.Event()
    goals = NativeGoals()
    native_client = ActionClient(node, GotoPose, f"{ns}/native", callback_group=group)

    async def native_execute(goal_handle):
        native_started.set()
        while not goal_handle.is_cancel_requested:
            await _executor_sleep(node, 0.05, wall=True)
        goal_handle.canceled()
        native_cancelled.set()
        return GotoPose.Result(status=GotoPose.Result.STATUS_CANCELED)

    async def outer_execute(goal_handle):
        native = await native_client.send_goal_async(GotoPose.Goal())
        with goals.tracking(goal_handle, native):
            wrapped = await native.get_result_async()
        goal_handle.canceled()
        return GotoPose.Result(status=wrapped.result.status)

    ActionServer(node, GotoPose, f"{ns}/native", execute_callback=native_execute, cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=group)
    ActionServer(node, GotoPose, f"{ns}/outer", execute_callback=outer_execute, cancel_callback=goals.cancel, callback_group=group)
    client = ActionClient(node, GotoPose, f"{ns}/outer", callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=10.0)
        handle = _wait(client.send_goal_async(GotoPose.Goal()))
        assert handle.accepted
        assert native_started.wait(10.0)
        _wait(handle.cancel_goal_async())
        result = _wait(handle.get_result_async())
        assert native_cancelled.wait(10.0)
        assert result.result.status == GotoPose.Result.STATUS_CANCELED
    finally:
        executor.shutdown()
        node.destroy_node()
        if owned:
            rclpy.shutdown()
