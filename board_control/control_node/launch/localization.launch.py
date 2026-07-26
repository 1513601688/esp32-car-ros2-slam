"""Launch the ESP32 bridge, static IMU transform, and 2D EKF odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("esp32_car_control")
    serial_port = LaunchConfiguration("serial_port")
    ekf_config = LaunchConfiguration("ekf_config")

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "esp32_car.launch.py"])
        ),
        launch_arguments={"serial_port": serial_port}.items(),
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[("odometry/filtered", "/odom")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="ESP32 serial device",
        ),
        DeclareLaunchArgument(
            "ekf_config",
            default_value=PathJoinSubstitution([
                package_share,
                "config",
                "ekf.yaml",
            ]),
            description="robot_localization EKF parameter file",
        ),
        controller,
        ekf,
    ])

