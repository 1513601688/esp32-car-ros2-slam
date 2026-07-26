"""启动 Nav2 规划、全向控制、速度平滑和碰撞监控。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("esp32_car_slam"))
    default_params = str(package_share / "config" / "nav2_params.yaml")
    rviz_config = str(package_share / "rviz" / "navigation.rviz")

    params_file = LaunchConfiguration("nav2_params_file")
    use_rviz = LaunchConfiguration("use_navigation_rviz")
    managed_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]

    common = {
        "parameters": [params_file],
        "output": "screen",
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_params,
            description="Nav2 navigation parameter file",
        ),
        DeclareLaunchArgument(
            "use_navigation_rviz",
            default_value="true",
            description="Whether to start the Nav2 RViz view",
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=[("cmd_vel", "cmd_vel_nav_raw")],
            **common,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            **common,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            **common,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            remappings=[("cmd_vel", "cmd_vel_nav_raw")],
            **common,
        ),
        Node(
            package="esp32_car_slam",
            executable="cmd_vel_deadband_compensator.py",
            name="cmd_vel_deadband_compensator",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=[("cmd_vel", "cmd_vel_nav")],
            **common,
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            **common,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{
                # AMCL 必须先收到人工初始位姿。导航由 RViz 的 Startup 按钮
                # 在 map -> odom 可用后统一启动，避免全局代价地图提前超时。
                "autostart": False,
                "node_names": managed_nodes,
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="esp32_car_navigation_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
