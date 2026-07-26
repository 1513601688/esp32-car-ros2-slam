"""在虚拟机启动 SLAM Toolbox，消费开发板发布的 /scan、/odom 和 TF。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("esp32_car_slam"))
    slam_toolbox_share = Path(get_package_share_directory("slam_toolbox"))

    params_file = str(package_share / "config" / "slam_toolbox.yaml")
    rviz_config = str(package_share / "rviz" / "mapping.rviz")
    toolbox_launch = str(slam_toolbox_share / "launch" / "online_async_launch.py")

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Whether to start RViz2 in this launch",
        ),
        Node(
            package="esp32_car_slam",
            executable="scan_normalizer.py",
            name="ld14_scan_normalizer",
            parameters=[{
                "input_topic": "/scan",
                "output_topic": "/scan_slam",
                "bin_count": 360,
            }],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(toolbox_launch),
            launch_arguments={
                "slam_params_file": params_file,
                "use_sim_time": "false",
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="esp32_car_mapping_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
