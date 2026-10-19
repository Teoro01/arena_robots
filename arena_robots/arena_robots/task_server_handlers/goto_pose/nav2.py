from __future__ import annotations

from typing import TYPE_CHECKING

from action_msgs.msg import GoalStatus
from arena_robots_msgs.action import GotoPose
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient

from arena_robots.lockstep_beat import LockstepBeat
from arena_robots.task_server_handlers import TaskHandler, _executor_sleep

if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle
    from rclpy.node import Node
    from tf2_ros import Buffer

    from arena_robots.bringup.mobile.nav2 import Nav2Bringup

_NAV2_RETRY_BACKOFF_SEC = 0.5
_CONTROLLER_PERIOD_FALLBACK = 0.1


def _translate_nav2_status(nav2_status: int) -> tuple[int, str]:
    if nav2_status == GoalStatus.STATUS_SUCCEEDED:
        return GotoPose.Result.STATUS_SUCCEEDED, ""
    if nav2_status == GoalStatus.STATUS_CANCELED:
        return GotoPose.Result.STATUS_CANCELED, "Nav2: canceled during execution"
    return GotoPose.Result.STATUS_ABORTED, "Nav2: planning or controller aborted"


class GotoPoseHandlerNav2(TaskHandler[GotoPose.Goal, GotoPose.Feedback, GotoPose.Result]):
    def __init__(self, bringup: Nav2Bringup, *, tf_buffer: Buffer, node: Node) -> None:
        self._bringup = bringup
        self._tf_buffer = tf_buffer
        self._node = node
        self._native_client = ActionClient(node, NavigateToPose, bringup.native_action_name)
        self._beat = LockstepBeat(node, "nav")
        self._beat_period: float | None = None
        node.create_subscription(Twist, str(bringup.namespace("cmd_vel")), lambda _msg: self._beat.pulse(), 10)

    async def _controller_period(self) -> float:
        """One controller tick in sim seconds, read once from controller_server."""
        if self._beat_period is not None:
            return self._beat_period
        client = self._node.create_client(GetParameters, str(self._bringup.namespace("controller_server", "get_parameters")))
        period = None
        if client.service_is_ready():
            response = await client.call_async(GetParameters.Request(names=["controller_frequency"]))
            assert response is not None
            values = response.values
            if values and values[0].type == ParameterType.PARAMETER_DOUBLE and values[0].double_value > 0.0:
                period = 1.0 / values[0].double_value
        self._node.destroy_client(client)
        if period is None:
            return _CONTROLLER_PERIOD_FALLBACK
        self._beat_period = period
        return period

    async def execute(self, goal_handle: ServerGoalHandle) -> GotoPose.Result:
        arena_goal: GotoPose.Goal = goal_handle.request
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = arena_goal.target

        result = GotoPose.Result()
        result.final_pose = PoseStamped()

        while not self._native_client.server_is_ready():
            if not goal_handle.is_active:
                result.status = GotoPose.Result.STATUS_CANCELED
                result.reason = "canceled before action server accepted goal"
                return result
            await _executor_sleep(self._node, 0.1, wall=True)

        await self._beat.acquire(await self._controller_period())
        try:
            return await self._dispatch(goal_handle, nav2_goal, result)
        finally:
            await self._beat.release()

    async def _dispatch(self, goal_handle: ServerGoalHandle, nav2_goal: NavigateToPose.Goal, result: GotoPose.Result) -> GotoPose.Result:
        def _on_nav2_feedback(fb_msg: NavigateToPose.Impl.FeedbackMessage) -> None:
            fb = fb_msg.feedback
            arena_fb = GotoPose.Feedback()
            arena_fb.current_pose = fb.current_pose
            arena_fb.distance_remaining = fb.distance_remaining
            eta = fb.estimated_time_remaining
            arena_fb.eta_seconds = eta.sec + eta.nanosec * 1e-9
            goal_handle.publish_feedback(arena_fb)

        while True:
            if not goal_handle.is_active:
                result.status = GotoPose.Result.STATUS_CANCELED
                result.reason = "goal preempted by new submit_task"
                return result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.status = GotoPose.Result.STATUS_CANCELED
                result.reason = "goal canceled"
                return result

            send_future = self._native_client.send_goal_async(nav2_goal, feedback_callback=_on_nav2_feedback)
            nav2_goal_handle = await send_future

            if nav2_goal_handle is None or not nav2_goal_handle.accepted:
                self._node.get_logger().info("nav2 rejected goal; retrying")
                await _executor_sleep(self._node, _NAV2_RETRY_BACKOFF_SEC)
                continue

            wrapped = await nav2_goal_handle.get_result_async()

            arena_status, arena_reason = _translate_nav2_status(wrapped.status)
            if not goal_handle.is_active:
                result.status = GotoPose.Result.STATUS_CANCELED
                result.reason = "goal preempted by new submit_task"
                return result

            if arena_status == GotoPose.Result.STATUS_SUCCEEDED:
                goal_handle.succeed()
                result.status = arena_status
                result.reason = arena_reason
                return result

            if arena_status == GotoPose.Result.STATUS_CANCELED:
                goal_handle.canceled()
                result.status = arena_status
                result.reason = arena_reason
                return result

            self._node.get_logger().info("nav2 aborted goal; replanning")
            await _executor_sleep(self._node, _NAV2_RETRY_BACKOFF_SEC)
