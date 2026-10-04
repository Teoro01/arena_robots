"""rosnav_rl adapter launch: single inference node, no nav2."""

from arena_bringup.substitutions import LaunchArgument
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld_items: list = []
    LaunchArgument.auto_append(ld_items)

    LaunchArgument("robot")
    namespace = LaunchArgument("namespace")
    use_sim_time = LaunchArgument("use_sim_time")
    frame = LaunchArgument("frame")
    base_frame = LaunchArgument("base_frame")
    task_generator_node = LaunchArgument("task_generator_node", default_value="")
    agent = LaunchArgument("agent")
    node_name = LaunchArgument("node_name", default_value="rosnav_rl_inference")
    control_rate = LaunchArgument("control_rate", default_value="10.0")
    min_lookahead_dist = LaunchArgument("min_lookahead_dist", default_value="0.5")
    max_lookahead_dist = LaunchArgument("max_lookahead_dist", default_value="2.5")
    lookahead_time = LaunchArgument("lookahead_time", default_value="1.5")
    train_mode = LaunchArgument("train_mode", default_value="false")

    inference_node = Node(
        package="rosnav_rl",
        executable="arena_inference_node.py",
        name=node_name.substitution,
        namespace=namespace.substitution,
        output="screen",
        parameters=[
            {
                **agent.str_param,
                **namespace.str_param,
                **task_generator_node.str_param,
                **frame.str_param,
                **base_frame.str_param,
                **use_sim_time.param(bool),
                **control_rate.param(float),
                **min_lookahead_dist.param(float),
                **max_lookahead_dist.param(float),
                **lookahead_time.param(float),
                **train_mode.param(bool),
            }
        ],
    )

    return LaunchDescription([*ld_items, inference_node])
