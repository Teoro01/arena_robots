"""Auxiliary node for the rosnav_rl local planner (the action server behind the DRLController get_command service)."""

from arena_bringup.substitutions import LaunchArgument
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld_items: list = []
    LaunchArgument.auto_append(ld_items)

    namespace = LaunchArgument('namespace')
    frame = LaunchArgument('frame')
    base_frame = LaunchArgument('base_frame')
    use_sim_time = LaunchArgument('use_sim_time')
    agent = LaunchArgument('agent')

    def _setup(context: LaunchContext, *args: object, **kwargs: object) -> list[Node]:
        ns = namespace.substitution.perform(context)
        sim_time = use_sim_time.substitution.perform(context)

        action_server = Node(
            package='rosnav_rl',
            executable='action_server.py',
            name='rosnav_action_server',
            namespace=ns,
            output='screen',
            parameters=[
                {'use_sim_time': sim_time.lower() in ('true', '1', 'yes')},
                {'agent_name': agent.substitution.perform(context)},
                {'namespace': ns},
                {'frame': frame.substitution.perform(context)},
                {'base_frame': base_frame.substitution.perform(context)},
            ],
        )

        return [action_server]

    return LaunchDescription([*ld_items, OpaqueFunction(function=_setup)])
