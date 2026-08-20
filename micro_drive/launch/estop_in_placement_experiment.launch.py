import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_topic = "/vn100/data_unbiased"
    estop_topic = "/teleop/emergency_stop"
    controller_topic = "/controller/cmd_vel"

    config = os.path.join(get_package_share_directory("micro_drive"), "config", "estop_in_place_experiment_node.yaml")

    estop_exp = Node(
        package="micro_drive",
        executable="estop_in_place_experiment_node",
        output="screen",
        parameters=[config],
        remappings=[
            ("/imu/data_unbiased", imu_topic),
            ("/estop", estop_topic),
            ("/controller/cmd_vel", controller_topic),
        ],
    )

    return LaunchDescription(
        [
            estop_exp,
        ]
    )
