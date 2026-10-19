from __future__ import annotations

from typing import TYPE_CHECKING

from arena_robots_msgs.action import ReachPose

from arena_robots.task_server_handlers import TaskHandler

if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle
    from rclpy.node import Node
    from tf2_ros import Buffer

    from arena_robots.bringup.arm.none import NoneArmBringup


class ReachPoseHandlerNone(TaskHandler[ReachPose.Goal, ReachPose.Feedback, ReachPose.Result]):
    """Immediately succeeds without motion planning. Used by the arm/none bringup."""

    def __init__(self, bringup: NoneArmBringup, *, tf_buffer: Buffer, node: Node) -> None:
        pass

    async def execute(self, goal_handle: ServerGoalHandle) -> ReachPose.Result:
        goal_handle.succeed()
        result = ReachPose.Result()
        result.status = ReachPose.Result.STATUS_SUCCEEDED
        return result
