"""加载已保存地图并用 AMCL 为 ESP32 麦克纳姆小车定位。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("esp32_car_slam"))
    workspace_root = package_share.parents[3]

    default_map = str(workspace_root / "maps" / "aaaaccc.yaml")
    default_params = str(package_share / "config" / "amcl.yaml")
    rviz_config = str(package_share / "rviz" / "localization.rviz")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Absolute path to the saved Nav2 map YAML file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="AMCL and map_server parameter file",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz2",
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
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[params_file, {
                "yaml_filename": map_file,
                "use_sim_time": False,
            }],
            output="screen",
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=[params_file, {"use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="esp32_car_localization_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
