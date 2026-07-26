"""一条命令启动静态地图、AMCL、Nav2 和 RViz。"""

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
    workspace_root = package_share.parents[3]
    localization_launch = str(package_share / "launch" / "localization.launch.py")
    navigation_launch = str(package_share / "launch" / "navigation.launch.py")
    rviz_config = str(package_share / "rviz" / "navigation.rviz")

    map_file = LaunchConfiguration("map")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=str(workspace_root / "maps" / "aaaaccc.yaml"),
            description="Absolute path to the saved map YAML file",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Whether to start the single Nav2 RViz window",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "map": map_file,
                "use_rviz": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={"use_navigation_rviz": "false"}.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="esp32_car_navigation_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(start_rviz),
            output="screen",
        ),
    ])
