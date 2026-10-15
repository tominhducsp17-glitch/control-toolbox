"""Integration test: node "trajectory" (100 Hz) + node "controller" (1 kHz, simulated arm) over ROS 2 topics.

The controller starts with bus "sim" (SimBus: ct_models OpenArm model with the friction measured on the robot),
enables, publishes joint_states; the trajectory node reads the start pose and goes approach -> track -> return
-> hold, publishing joint_commands (positions only) at about 100 Hz. Checked while running: command rate, the arm follows the
trajectory (q of joint_states against the commands), the reference q_d is published. Checked after the launch
ends (SIGINT): the controller returned to its start pose, disabled, exited 0 and wrote its log.
"""

import json
import math
import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

RIGHT = [f"openarm_right_joint{j}" for j in range(1, 8)]
HANG = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
GOAL = [0.4, 0.25, 0.0, 1.0, 0.0, 0.1, 0.0]
LOG_DIR = tempfile.mkdtemp()


def write_trajectory() -> str:
    """hold 0.5 s, 2 s quintic hang -> goal, hold 1 s; 100 Hz rows t, q1..7 (positions only)."""
    path = os.path.join(tempfile.mkdtemp(), "test_trajectory_100hz.csv")
    with open(path, "w", encoding="utf-8") as f:
        f.write("t," + ",".join(f"q{j}" for j in range(1, 8)) + "\n")
        for k in range(351):
            t = k / 100.0
            s = min(max((t - 0.5) / 2.0, 0.0), 1.0)
            b = s**3 * (10 - 15 * s + 6 * s * s)
            q = [h + b * (g - h) for h, g in zip(HANG, GOAL)]
            f.write(",".join(f"{v:.9g}" for v in [t] + q) + "\n")
    return path


@pytest.mark.launch_test
def generate_test_description():
    trajectory = launch_ros.actions.Node(
        package="openarm_trajectory", executable="trajectory_node", name="trajectory", output="screen",
        parameters=[get_package_share_directory("openarm_trajectory") + "/config/trajectory.yaml",
                    {"trajectory_file": write_trajectory(), "min_move_s": 1.0, "approach_speed": 1.0}])
    controller = launch_ros.actions.Node(
        package="openarm_controller", executable="controller_node", name="controller", output="screen",
        parameters=[get_package_share_directory("openarm_controller") + "/config/controller.yaml",
                    {"bus": "sim", "log_dir": LOG_DIR, "label": "launch-test", "min_move_s": 1.0,
                     "return_speed": 1.0}])
    return launch.LaunchDescription([
        launch.actions.SetLaunchConfiguration("sigterm_timeout", "30"),
        trajectory, controller, launch_testing.actions.ReadyToTest()]), {"controller": controller}


class TestTrajectoryController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_pipeline(self):
        node = rclpy.create_node("observer")
        commands, states, references, phases = [], [], [], []
        node.create_subscription(JointState, "joint_commands", lambda m: commands.append((time.monotonic(), m)), 50)
        node.create_subscription(JointState, "joint_states", lambda m: states.append((time.monotonic(), m)), 50)
        node.create_subscription(JointState, "controller/reference", lambda m: references.append(m), 50)
        qos = rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "trajectory/phase", lambda m: phases.append(m.data), qos)
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline and "hold" not in phases:
            rclpy.spin_once(node, timeout_sec=0.01)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()

        self.assertEqual(phases[:5], ["wait_state", "approach", "track", "return", "hold"], phases)
        stamps = [t for t, _ in commands[-100:]]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        self.assertGreater(rate, 80.0, f"joint_commands at {rate:.0f} Hz")
        self.assertLess(rate, 120.0, f"joint_commands at {rate:.0f} Hz")
        self.assertEqual(list(commands[-1][1].name), RIGHT)
        self.assertEqual(len(commands[-1][1].position), 7)
        self.assertEqual(len(commands[-1][1].velocity), 0)  # positions only: dq_d comes from the filter
        self.assertEqual(list(states[-1][1].name), RIGHT)
        self.assertTrue(references, "no controller/reference")

        # the measured q follows the reference q_d of the same cycle (both published together, same stamp)
        refs = {(r.header.stamp.sec, r.header.stamp.nanosec): r for r in references}
        errors = [max(abs(a - b) for a, b in zip(s.position, refs[k].position))
                  for _, s in states if (k := (s.header.stamp.sec, s.header.stamp.nanosec)) in refs]
        self.assertGreater(len(errors), 300)
        self.assertLess(max(errors), 0.03, f"largest |q - q_d| {max(errors):.4f} rad")
        # the reference follows the trajectory: at the end of the track phase it is at the goal
        self.assertLess(min(max(abs(a - g) for a, g in zip(r.position, GOAL)) for r in references), 0.005)
        # back at the start pose in hold
        self.assertLess(max(abs(a - b) for a, b in zip(states[-1][1].position, HANG)), 0.02)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_controller_returned_and_disabled(self, proc_info, controller):
        launch_testing.asserts.assertExitCodes(proc_info, process=controller)  # 0: status completed
        with open(os.path.join(LOG_DIR, "launch-test", "meta.json"), encoding="utf-8") as f:
            meta = json.load(f)
        self.assertEqual(meta["status"], "completed", meta["reason"])
        self.assertEqual(meta["rate_hz"], 1000.0)
        with open(os.path.join(LOG_DIR, "launch-test", "log.csv"), encoding="utf-8") as f:
            header = f.readline().strip().split(",")
            rows = [line.split(",") for line in f]
        phase = header.index("phase")
        self.assertEqual({int(r[phase]) for r in rows} >= {1, 3, 4, 5}, True)  # ramp, track, return, ramp down
        self.assertTrue(all(math.isfinite(float(v)) for r in rows[-100:] for v in r))
