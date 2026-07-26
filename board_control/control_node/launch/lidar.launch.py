"""Launch the LDROBOT LD14 driver and its measured chassis transform."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    port_name = LaunchConfiguration("port_name")
    frame_id = LaunchConfiguration("frame_id")

    lidar = Node(
        package="ldlidar_sl_ros2",
        executable="ldlidar_sl_ros2_node",
        name="ld14_lidar",
        output="screen",
        parameters=[
            config_file,
            {
                "port_name": port_name,
                "frame_id": frame_id,
            },
        ],
    )

    # base_link is on the middle deck at 0.11 m above the floor. The LD14 scan
    # plane is 0.07 m above base_link, so its absolute height is 0.18 m.
    # Its data-sheet 180-degree direction points toward the vehicle front,
    # therefore the laser frame's 0-degree/X direction points backward and
    # must be rotated pi radians around Z relative to base_link.
    base_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_static_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.07",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "3.141592653589793",
            "--frame-id", "base_link",
            "--child-frame-id", "laser_frame",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("esp32_car_control"),
                "config",
                "ld14.yaml",
            ]),
            description="LD14 parameter file",
        ),
        DeclareLaunchArgument(
            "port_name",
            default_value=(
                "/dev/serial/by-id/"
                "usb-1a86_USB_Single_Serial_5A6C087033-if00"
            ),
            description="Stable LD14 serial device path",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="laser_frame",
            description="LaserScan frame; its static TF is added after measurement",
        ),
        lidar,
        base_to_laser,
    ])
