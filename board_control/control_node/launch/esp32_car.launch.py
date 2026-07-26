"""Launch the ESP32 car serial bridge and chassis controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    serial_port = LaunchConfiguration("serial_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("esp32_car_control"),
                "config",
                "esp32_car.yaml",
            ]),
            description="ROS 2 parameter YAML file",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="ESP32 serial device; overrides the YAML value",
        ),
        Node(
            package="esp32_car_control",
            executable="esp32_car_node",
            name="esp32_car_node",
            output="screen",
            parameters=[config_file, {"serial_port": serial_port}],
        ),
        # base_footprint is the chassis center projected onto the floor.
        # base_link is fixed to the middle deck, measured 11 cm above the floor.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link_static_tf",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.11",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "base_link",
            ],
        ),
        # IMU axes are aligned with the chassis axes. The sensor is mounted
        # 5 cm to the right and 6 cm above base_link, on the top deck.
        # With base_link 11 cm above the floor, imu_link is 17 cm high.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_static_tf",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "-0.05",
                "--z", "0.06",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "imu_link",
            ],
        ),
    ])
