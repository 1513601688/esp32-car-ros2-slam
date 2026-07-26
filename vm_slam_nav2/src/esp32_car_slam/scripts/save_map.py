#!/usr/bin/env python3
"""无需 nav2_map_server，直接把 /map 保存为 Nav2 兼容的 PGM/YAML。"""

import argparse
import math
from pathlib import Path
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class MapReceiver(Node):
    def __init__(self):
        super().__init__("esp32_car_map_saver")
        self.map = None
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.receive, qos)

    def receive(self, message):
        self.map = message


def quaternion_yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def save_map(message, output_stem):
    width = message.info.width
    height = message.info.height
    if width == 0 or height == 0 or len(message.data) != width * height:
        raise RuntimeError("收到的 /map 尺寸或数据长度无效")

    output_stem.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_stem.with_suffix(".pgm")
    yaml_path = output_stem.with_suffix(".yaml")

    pixels = bytearray()
    # OccupancyGrid 原点在左下；PGM 第一行在顶部，因此按行上下翻转。
    for row in range(height - 1, -1, -1):
        start = row * width
        for occupancy in message.data[start : start + width]:
            if occupancy < 0:
                pixels.append(205)
            elif occupancy >= 65:
                pixels.append(0)
            elif occupancy <= 25:
                pixels.append(254)
            else:
                pixels.append(205)

    header = f"P5\n# CREATOR: esp32_car_slam save_map.py\n{width} {height}\n255\n"
    pgm_path.write_bytes(header.encode("ascii") + pixels)

    origin = message.info.origin
    yaw = quaternion_yaw(origin.orientation)
    yaml_text = (
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {message.info.resolution:.9f}\n"
        f"origin: [{origin.position.x:.9f}, {origin.position.y:.9f}, {yaw:.9f}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")
    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(description="Save /map as Nav2-compatible PGM/YAML")
    parser.add_argument("--output", required=True, help="Output path without extension")
    parser.add_argument("--timeout", type=float, default=10.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MapReceiver()
    deadline = time.monotonic() + args.timeout
    while node.map is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.map is None:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"{args.timeout:.1f} 秒内没有收到 /map")

    pgm_path, yaml_path = save_map(node.map, Path(args.output).expanduser())
    print(f"saved_pgm={pgm_path}")
    print(f"saved_yaml={yaml_path}")
    print(f"size={node.map.info.width}x{node.map.info.height}")
    print(f"resolution={node.map.info.resolution:.6f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
