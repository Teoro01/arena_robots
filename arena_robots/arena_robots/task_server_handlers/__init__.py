"""TaskHandler ABC and shared utilities for task_server handler implementations.

Handler registration is owned by the Bringup subclass via its ``task_handlers``
ClassVar (see ``arena_robots.bringup.Bringup``); this module only exposes the
shared base class and the ``_executor_sleep`` helper.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.task import Future

if TYPE_CHECKING:
    from tf2_ros import Buffer

    from arena_robots.bringup import Bringup


class TaskHandler[GoalT, FeedbackT, ResultT](ABC):
    @abstractmethod
    def __init__(self, bringup: Bringup, *, tf_buffer: Buffer, node: Node) -> None: ...

    @abstractmethod
    async def execute(self, goal_handle: ServerGoalHandle) -> ResultT: ...

    def on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT


async def _executor_sleep(node: Node, seconds: float, *, wall: bool = False) -> None:
    """Timer-backed sleep that yields to rclpy's executor. Works inside action
    server callbacks (which run under rclpy.spin, not asyncio).

    ``wall=True`` uses a wall-clock timer so the sleep still ticks while sim
    time is paused, use it for readiness/discovery polling, not for retry
    rate-limiting where sim-time semantics are preferred.
    """
    fut: Future = Future()

    def _fire():
        if not fut.done():
            fut.set_result(None)

    timer = node.create_timer(seconds, _fire, clock=Clock() if wall else None)
    try:
        await fut
    finally:
        node.destroy_timer(timer)
