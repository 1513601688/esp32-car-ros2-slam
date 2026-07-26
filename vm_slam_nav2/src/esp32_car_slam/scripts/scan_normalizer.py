#!/usr/bin/env python3
"""将 LD14 每圈变化的点数重采样为固定角度栅格，供 SLAM Toolbox 使用。"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):
    """把 0~360° 的可变长度 LaserScan 归一化为固定数量的角度单元。"""

    def __init__(self):
        super().__init__("ld14_scan_normalizer")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_slam")
        self.declare_parameter("bin_count", 360)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.bin_count = int(self.get_parameter("bin_count").value)
        if self.bin_count < 2:
            raise ValueError("bin_count 必须大于等于 2")

        # 输出使用 RELIABLE，使 SLAM 与 RViz 的不同订阅 QoS 都能兼容。
        output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(LaserScan, output_topic, output_qos)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.normalize, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"LD14 扫描归一化: {input_topic} -> {output_topic}, "
            f"固定 {self.bin_count} 点"
        )

    def normalize(self, source):
        target = LaserScan()
        target.header = source.header
        target.angle_min = 0.0
        target.angle_increment = 2.0 * math.pi / self.bin_count
        # 不重复发布 0°/360° 两个等价方向，因此最后一束小于 2π。
        target.angle_max = target.angle_increment * (self.bin_count - 1)
        target.scan_time = source.scan_time
        target.time_increment = (
            source.scan_time / self.bin_count if source.scan_time > 0.0 else 0.0
        )
        target.range_min = source.range_min
        target.range_max = source.range_max

        ranges = [math.inf] * self.bin_count
        has_intensity = len(source.intensities) == len(source.ranges)
        intensities = [0.0] * self.bin_count if has_intensity else []

        # 每个原始点落入最近的固定角度单元；同一单元保留最近障碍物。
        for index, distance in enumerate(source.ranges):
            if not math.isfinite(distance):
                continue
            if distance < source.range_min or distance > source.range_max:
                continue

            angle = source.angle_min + index * source.angle_increment
            normalized_angle = angle % (2.0 * math.pi)
            target_index = int(
                normalized_angle / target.angle_increment + 0.5
            ) % self.bin_count

            if distance < ranges[target_index]:
                ranges[target_index] = distance
                if has_intensity:
                    intensities[target_index] = source.intensities[index]

        target.ranges = ranges
        target.intensities = intensities
        self.publisher.publish(target)


def main():
    rclpy.init()
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
