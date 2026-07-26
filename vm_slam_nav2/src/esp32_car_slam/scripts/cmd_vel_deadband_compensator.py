#!/usr/bin/env python3
"""Compensate the ESP32 chassis motor deadband without changing motion direction."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelDeadbandCompensator(Node):
    """Raise intentional low commands above the motor's usable speed threshold.

    Translational X/Y components are treated as one vector so compensation keeps
    its original direction. Values below the activation epsilon are considered
    optimizer noise and become zero. Angular velocity is handled independently.
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_deadband_compensator")

        self.input_topic = self.declare_parameter(
            "input_topic", "/cmd_vel_nav_raw"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/cmd_vel_nav"
        ).value
        self.min_linear_speed = float(
            self.declare_parameter("min_linear_speed", 0.05).value
        )
        self.min_angular_speed = float(
            self.declare_parameter("min_angular_speed", 0.10).value
        )
        self.linear_activation_epsilon = float(
            self.declare_parameter("linear_activation_epsilon", 0.005).value
        )
        self.angular_activation_epsilon = float(
            self.declare_parameter("angular_activation_epsilon", 0.03).value
        )

        if not (
            0.0 <= self.linear_activation_epsilon < self.min_linear_speed
            and 0.0 <= self.angular_activation_epsilon < self.min_angular_speed
        ):
            raise ValueError("activation epsilon must be non-negative and below its deadband speed")

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self.on_command, 10
        )
        self.get_logger().info(
            "速度死区补偿已启动: linear=%.3f m/s, angular=%.3f rad/s, %s -> %s"
            % (
                self.min_linear_speed,
                self.min_angular_speed,
                self.input_topic,
                self.output_topic,
            )
        )

    def on_command(self, source: Twist) -> None:
        output = Twist()

        # 将 Vx/Vy 作为一个平移向量整体缩放，保持全向轮底盘原运动方向。
        linear_norm = math.hypot(source.linear.x, source.linear.y)
        if linear_norm > self.linear_activation_epsilon:
            scale = max(1.0, self.min_linear_speed / linear_norm)
            output.linear.x = source.linear.x * scale
            output.linear.y = source.linear.y * scale

        # 极小角速度通常是 MPPI 采样噪声；只有明确旋转时才跨过电机死区。
        angular = source.angular.z
        if abs(angular) > self.angular_activation_epsilon:
            output.angular.z = math.copysign(
                max(abs(angular), self.min_angular_speed), angular
            )

        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = CmdVelDeadbandCompensator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
