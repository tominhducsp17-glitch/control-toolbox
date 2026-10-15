"""Start node "trajectory" (100 Hz target positions) and node "controller" (1 kHz loop with the motor bus inside).

Both nodes read every parameter once at start from YAML files:
  openarm_trajectory/config/trajectory.yaml and openarm_controller/config/controller.yaml
Edit them (or pass other files) and restart; no rebuild is needed.

    ros2 launch openarm_controller trajectory_controller.launch.py                      # simulated arm
    ros2 launch openarm_controller trajectory_controller.launch.py bus:=can log_dir:=/path/logs   # the robot
    ros2 launch openarm_controller trajectory_controller.launch.py trajectory_file:=/path/file.csv \\
        controller_params:=/path/my_controller.yaml

Ctrl+C once: the controller returns the arm to its start pose, ramps down and disables the motors (a few
seconds; launch waits for it), twice: disables at once.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    trajectory_file = arg("trajectory_file") or (
        get_package_share_directory("openarm_trajectory") + "/trajectories/cart_figure8_hold3_100hz.csv")
    overrides = {name: arg(name) for name in ("bus", "log_dir", "label", "can_interface") if arg(name)}
    return [
        Node(package="openarm_trajectory", executable="trajectory_node", name="trajectory", output="screen",
             parameters=[arg("trajectory_params"), {"trajectory_file": trajectory_file}]),
        Node(package="openarm_controller", executable="controller_node", name="controller", output="screen",
             parameters=[arg("controller_params"), overrides]),
    ]


def generate_launch_description():
    return LaunchDescription([
        # after Ctrl+C the controller needs a few seconds for the return and the ramp down: do not escalate to
        # SIGTERM / SIGKILL before that (SIGTERM only requests the return, SIGKILL would leave the motors enabled)
        SetLaunchConfiguration("sigterm_timeout", "60"),
        SetLaunchConfiguration("sigkill_timeout", "60"),
        DeclareLaunchArgument(
            "trajectory_params",
            default_value=get_package_share_directory("openarm_trajectory") + "/config/trajectory.yaml"),
        DeclareLaunchArgument(
            "controller_params",
            default_value=get_package_share_directory("openarm_controller") + "/config/controller.yaml"),
        DeclareLaunchArgument("trajectory_file", default_value="",
                              description="CSV t, q1..q7, dq1..dq7[, ddq...]; empty: cart_figure8_hold3_100hz.csv"),
        DeclareLaunchArgument("bus", default_value="", description="override of controller.yaml: can | sim"),
        DeclareLaunchArgument("can_interface", default_value="", description="override of controller.yaml"),
        DeclareLaunchArgument("log_dir", default_value="", description="write the run log to log_dir/<label>/"),
        DeclareLaunchArgument("label", default_value="", description="run folder name"),
        OpaqueFunction(function=nodes),
    ])
