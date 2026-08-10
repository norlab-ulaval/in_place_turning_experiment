import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Input topics
    lock_autonomy_topic = "/teleop/lock_autonomy"
    imu_topic = "/imu/data_raw"

    # Output topics
    controller_topic = "/controller/cmd_vel"

    # Internal topics
    imu_unbiased_topic = "/imu/data_unbiased"
    imu_bias_topic = "/imu/bias"

    config = os.path.join(get_package_share_directory("micro_drive"), "config", "in_place_turning_experiment_node.yaml")

    bias_observer_node = Node(
        package="micro_drive",
        executable="imu_bias_observer",
        output="both",
        remappings=[("imu_topic_in", imu_topic), ("bias_topic_out", imu_bias_topic)],
    )

    bias_compensator_node = Node(
        package="micro_drive",
        executable="imu_bias_compensator_node",
        output="both",
        remappings=[
            ("imu_topic_in", imu_topic),
            ("bias_topic_in", imu_bias_topic),
            ("imu_topic_out", imu_unbiased_topic),
        ],
    )

    micro_drive_exp = Node(
        package="micro_drive",
        executable="in_place_turning_experiment_node",
        output="both",
        parameters=[config],
        remappings=[
            ("/imu/data_unbiased", imu_unbiased_topic),
            ("/teleop/lock_autonomy", lock_autonomy_topic),
            ("/controller/cmd_vel", controller_topic),
        ],
    )

    return LaunchDescription([micro_drive_exp, bias_observer_node, bias_compensator_node])
