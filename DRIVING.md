# Driving a robot standalone

## Overview

`arena_robots` exposes a single launch entry point (`bringup.launch.py`) that
brings up a nav stack + a `task_server` node in a given namespace. The
`task_server` advertises arena IDL action servers, one per `TaskKind` the
chosen bringup supports. Clients send goals to those endpoints; the
`task_server` translates them to the underlying nav stack. `task_generator`
is one consumer of this surface; any ROS 2 process or CLI can be another.

## Launch a robot standalone

```bash
ros2 launch arena_robots bringup.launch.py \
    robot:=husky \
    namespace:=husky_1 \
    bringup:=nav2 \
    use_sim_time:=true
```

| Arg | Meaning |
|---|---|
| `robot` | Robot name, must match a directory under `arena_robots/robots/` |
| `namespace` | ROS namespace for all nodes and action endpoints |
| `bringup` | Nav stack to bring up: `nav2`, `none`, or `external` |
| `use_sim_time` | Pass `true` when using Gazebo sim time; `false` for a real-time clock |

## Drive via ros2 CLI

```bash
ros2 action send_goal /husky_1/goto_pose arena_robots_msgs/action/GotoPose \
    "{target: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

## Drive via Python

```python
import rclpy, tf2_ros
from arena_robots.Robot import RobotIdentifier
from arena_robots.clients.goto_pose import GotoPoseClient
from arena_robots_msgs.action import GotoPose

rclpy.init()
node = rclpy.create_node('driver')
tf = tf2_ros.Buffer()
tf2_ros.TransformListener(tf, node)

robot = RobotIdentifier('husky').resolve()
client = GotoPoseClient(robot, namespace='husky_1', node=node, tf_buffer=tf)
await client.wait_ready()

goal = GotoPose.Goal()
goal.target.header.frame_id = 'map'
goal.target.pose.position.x = 2.0
await client.send_goal(goal)
result = await client.await_result()
```

`GotoPoseClient` is in `arena_robots.clients.goto_pose`. `send_goal` returns
once the server accepts the goal; `await_result` blocks until the action
completes. Every planner-driven bringup (`nav2`, `drl`, `rosnav_rl`,
`external`) holds the goal open until the robot arrives within tolerance
and preempts it when a newer goal lands; only `none` and `manual` treat
goto_pose as a fire-and-forget goal publish.

## List published action endpoints

```bash
ros2 action list | grep /husky_1/
```

Expected output for a `nav2` bringup:

```
/husky_1/goto_pose
```

## Drive through MCP

The `robots` domain of the Arena MCP server (`arena_robots.mcp`) wraps this
surface for agents. `python3 -m arena_mcp ls robots` lists its tools.

- Offline, without ROS: the robot catalog from `RobotIdentifier`, one model's
  resolved definition (caps, limits, footprint, arm poses and gestures, default
  sensors, task endpoints and the bringups that serve them, assembly, power
  profile) and a validation that runs `arena feature robots check` plus
  definition checks as structured findings.
- Live, in a running env: the fleet from `state/robots` with the task actions
  each robot serves right now (read from the ROS graph), goto, reach and
  gesture goals, velocity commands, task pose and odometry, sensor topics,
  laser scan summaries, camera frames and power.

Live tools take `env_id` and `robot`, both optional when the runtime has one
env and the env one robot. A goal is one phase handed to the robot's episode
task through the task generator's `runtime/submit_task`, the path an rviz goal
handle takes: goto targets are in the env map frame (`frame="scenario"` takes
scenario coordinates), the robot's adapter dispatches the action, and the
episode judges the phase. Goals and timed drives run as jobs: the job follows
the phase through the `robot` semantic entity until it is met, failed or
dropped, the action feedback (distance and ETA for `goto_pose`) lands in its
progress, a later goal drops the running one, and `jobs_cancel` resubmits an
empty task. A goto target off the env map or on an occupied cell is refused
before anything is sent. Velocity commands are `geometry_msgs/Twist` on `<robot_ns>/cmd_vel`,
which the per-robot `twist_stamper` relays as `TwistStamped` to the
ros2_control controller and which flatsim reads directly. A stop zeroes that
topic and ends a running drive job, a planner holding a goal keeps commanding
until its goal is cancelled.

## Real-robot caveat

`state_publisher.launch.py` (included by `bringup.launch.py`) hardcodes
`use_sim_time=true` internally. For real-robot deployments, launch your own
`robot_state_publisher` with `use_sim_time:=false` before (or instead of)
invoking `bringup.launch.py`, or provide a compatible wrapper that does not
set `use_sim_time`.
