"""Tests for AcousticsPublisher ROS 2 Node."""

from __future__ import annotations

from pathlib import Path

from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


def test_acoustics_publisher_initialization(tmp_path: Path) -> None:
    from arena_robots.acoustics_publisher import AcousticsPublisher

    profile_content = """
L_base_0: 40.0
beta_0: 45.0
beta_1: 18.0
beta_2: 5.0
beta_3: 3.0
omega_ref: 5.0
tau_ref: 10.0
omega_deadband: 0.05
omega_active: 0.20
sigma_base: 1.5
sigma_dynamic: 2.0
sigma_no_effort: 4.0
"""
    profile_path = tmp_path / "test_profile.yaml"
    profile_path.write_text(profile_content)

    node = AcousticsPublisher(
        parameter_overrides=[
            Parameter("profile_path", value=str(profile_path)),
            Parameter("topic", value="/test_acoustics"),
        ]
    )

    try:
        assert node._L_base_0 == 40.0
        assert abs(node._P_base - 10.0**4.0) < 1e-5
    finally:
        node.destroy_node()


def test_acoustics_publisher_on_joint_state(tmp_path: Path) -> None:
    from arena_robots.acoustics_publisher import AcousticsPublisher

    profile_content = """
L_base_0: 40.0
beta_0: 45.0
beta_1: 18.0
beta_2: 5.0
beta_3: 3.0
omega_ref: 5.0
tau_ref: 10.0
omega_deadband: 0.05
omega_active: 0.20
sigma_base: 1.5
sigma_dynamic: 2.0
sigma_no_effort: 4.0
"""
    profile_path = tmp_path / "test_profile.yaml"
    profile_path.write_text(profile_content)

    node = AcousticsPublisher(
        parameter_overrides=[
            Parameter("profile_path", value=str(profile_path)),
            Parameter("topic", value="/test_acoustics"),
        ]
    )

    try:
        msg = JointState()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.name = ["front_left_wheel_joint", "front_right_wheel_joint"]
        msg.velocity = [1.0, 1.5]
        msg.effort = [2.0, 2.5]

        node._on_joint_state(msg)
        assert node._ema_p_drive > 0.0
    finally:
        node.destroy_node()


def test_acoustics_publisher_uncertainty_bounds(tmp_path: Path) -> None:
    import rclpy
    from arena_robots.acoustics_publisher import AcousticsPublisher
    from arena_robots_msgs.msg import Acoustics
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    profile_content = """
L_base_0: 42.0
beta_0: 45.0
beta_1: 18.0
beta_2: 5.0
omega_ref: 5.0
tau_ref: 10.0
omega_deadband: 0.05
omega_active: 0.20
sigma_base: 1.0
sigma_dynamic: 0.8
sigma_no_effort: 1.0
"""
    profile_path = tmp_path / "test_profile.yaml"
    profile_path.write_text(profile_content)

    node = AcousticsPublisher(
        parameter_overrides=[
            Parameter("profile_path", value=str(profile_path)),
            Parameter("topic", value="/test_acoustics"),
        ]
    )

    try:
        # High speed joint state with effort
        msg = JointState()
        msg.header.stamp.sec = 1
        msg.name = ["front_left_wheel", "front_right_wheel"]
        msg.velocity = [15.0, 15.0]
        msg.effort = [5.0, 5.0]

        captured: list[Acoustics] = []
        node.create_subscription(Acoustics, "/test_acoustics", captured.append, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        node._on_joint_state(msg)
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.05)
            if captured:
                break

        assert len(captured) == 1
        unc = captured[0].uncertainty_1sigma_dba
        assert 1.0 <= unc <= 2.5, f"Uncertainty {unc} not within [1.0, 2.5] dBA"

        # Without effort (should add bounded penalty, not explode)
        msg_no_effort = JointState()
        msg_no_effort.header.stamp.sec = 2
        msg_no_effort.name = ["front_left_wheel", "front_right_wheel"]
        msg_no_effort.velocity = [15.0, 15.0]
        msg_no_effort.effort = []

        node._on_joint_state(msg_no_effort)
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.05)
            if len(captured) == 2:
                break

        assert len(captured) == 2
        unc_no_effort = captured[1].uncertainty_1sigma_dba
        assert 1.0 <= unc_no_effort <= 2.5, f"Uncertainty without effort {unc_no_effort} exceeded 2.5 dBA"
    finally:
        node.destroy_node()


def test_acoustics_publisher_positive_flank_collision(tmp_path: Path) -> None:
    from arena_robots.acoustics_publisher import AcousticsPublisher
    from arena_robots_msgs.msg import CollisionEvent, CollisionEvents

    profile_content = """
L_base_0: 42.0
beta_0: 45.0
beta_1: 18.0
beta_2: 5.0
omega_ref: 5.0
tau_ref: 10.0
omega_deadband: 0.05
omega_active: 0.20
sigma_base: 1.0
sigma_dynamic: 0.8
sigma_no_effort: 1.0
"""
    profile_path = tmp_path / "test_profile.yaml"
    profile_path.write_text(profile_content)

    node = AcousticsPublisher(
        parameter_overrides=[
            Parameter("profile_path", value=str(profile_path)),
            Parameter("topic", value="/test_acoustics"),
        ]
    )

    try:
        captured = []
        node._acoustics_pub.publish = lambda m: captured.append(m)

        def _make_js(sec: int) -> JointState:
            js = JointState()
            js.header.stamp.sec = sec
            js.name = ["front_left_wheel", "front_right_wheel"]
            js.velocity = [2.0, 2.0]
            js.effort = [1.0, 1.0]
            return js

        # 1. Normal driving frame
        node._on_joint_state(_make_js(1))
        assert len(captured) == 1
        assert captured[0].total_level_af_dba < 60.0
        assert captured[0].operating_state == "driving"

        # 2. Collision impact onset (0 -> 1)
        col_msg = CollisionEvents()
        col_msg.events = [CollisionEvent()]
        node._on_collision_events(col_msg)

        # 3. Next tick: initial impact frame triggers 100 dBA
        node._on_joint_state(_make_js(2))
        assert len(captured) == 2
        assert captured[1].total_level_af_dba == 100.0
        assert captured[1].operating_state == "collision"

        # 4. Subsequent tick: still in same collision (1 -> 1), does NOT retrigger 100 dBA
        node._on_collision_events(col_msg)
        node._on_joint_state(_make_js(3))
        assert len(captured) == 3
        assert captured[2].total_level_af_dba < 60.0
        assert captured[2].operating_state == "driving"

        # 5. Collision clears (1 -> 0)
        empty_col_msg = CollisionEvents()
        empty_col_msg.events = []
        node._on_collision_events(empty_col_msg)
        node._on_joint_state(_make_js(4))
        assert len(captured) == 4
        assert captured[3].total_level_af_dba < 60.0

        # 6. New collision impact onset (0 -> 1)
        node._on_collision_events(col_msg)
        node._on_joint_state(_make_js(5))
        assert len(captured) == 5
        assert captured[4].total_level_af_dba == 100.0
        assert captured[4].operating_state == "collision"
    finally:
        node.destroy_node()
