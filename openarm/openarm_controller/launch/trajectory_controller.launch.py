"""Start node "trajectory" (q, dq reference) and node "controller" (calls openarm_control, outputs tau_ff).

Both nodes read every parameter once at start from YAML files:
  openarm_trajectory/config/trajectory.yaml and openarm_controller/config/controller.yaml
Edit them (or pass other files) and restart; no rebuild is needed.

    ros2 launch openarm_controller trajectory_controller.launch.py
    ros2 launch openarm_controller trajectory_controller.launch.py controller_params:=/path/my_controller.yaml
    ros2 launch openarm_controller trajectory_controller.launch.py trajectory_file:=/path/file.csv \
        controller_type:=gravity_compensation        # optional overrides of the YAML values
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    trajectory_file = arg("trajectory_file") or (
        get_package_share_directory("openarm_trajectory") + "/trajectories/tissue_wipe_x2.csv")
    controller_overrides = {"controller_type": arg("controller_type")} if arg("controller_type") else {}
    return [
        Node(package="openarm_trajectory", executable="trajectory_node", name="trajectory", output="screen",
             parameters=[arg("trajectory_params"), {"trajectory_file": trajectory_file}]),
        Node(package="openarm_controller", executable="controller_node", name="controller", output="screen",
             parameters=[arg("controller_params"), controller_overrides]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "trajectory_params",
            default_value=get_package_share_directory("openarm_trajectory") + "/config/trajectory.yaml"),
        DeclareLaunchArgument(
            "controller_params",
            default_value=get_package_share_directory("openarm_controller") + "/config/controller.yaml"),
        DeclareLaunchArgument("trajectory_file", default_value="",
                              description="CSV t, q1..q7, dq1..dq7[, ddq...]; empty: tissue_wipe_x2.csv"),
        DeclareLaunchArgument("controller_type", default_value="",
                              description="override of controller.yaml (ctc_feedforward | gravity_compensation)"),
        OpaqueFunction(function=nodes),
    ])
