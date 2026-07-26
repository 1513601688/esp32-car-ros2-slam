#pragma once

#include <cstdint>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

#include "esp32_car_control/serial_protocol.hpp"

namespace esp32_car_control
{

/**
 * @brief 将 ESP32 解析后的传感器数据转换为标准 ROS 2 话题。
 *
 * 该类只负责“数据转换与发布”，不负责读取串口，也不负责控制底盘：
 * - WheelSample -> nav_msgs/msg/Odometry
 * - ImuSample   -> sensor_msgs/msg/Imu（仅角速度和线加速度可用于融合）
 *
 * 这样可以让话题发布逻辑与 esp32_car_node.cpp 中的控制逻辑保持独立。
 */
class SensorPublisher
{
public:
  /// 读取 ROS 参数并创建所有传感器相关发布器。
  explicit SensorPublisher(rclcpp::Node & node);

  /// 发布一帧轮式里程计速度。
  void publish(const WheelSample & sample);

  /// 发布一帧 IMU 角速度和线加速度；不可靠的绝对姿态被标记为不可用。
  void publish(const ImuSample & sample);

private:
  /// 把 ESP32 开机后的毫秒计时转换到当前 ROS 时钟域。
  rclcpp::Time ros_stamp_from_esp(std::uint32_t stamp_ms);

  // 借用主节点，不拥有节点生命周期。
  rclcpp::Node & node_;

  // 消息使用的坐标系名称。
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;

  // 将底盘原始前向速度转换为 ROS REP-103 的 x 向前为正。
  double wheel_vx_scale_{1.0};
  // 将底盘原始横向速度转换为 ROS REP-103 的 y 向左为正。
  double wheel_vy_scale_{1.0};

  // 传感器协方差参数。数值越小表示越信任对应测量。
  double wheel_linear_variance_{};
  double wheel_angular_variance_{};
  double imu_angular_velocity_variance_{};
  double imu_linear_acceleration_variance_{};

  // ESP32 单调时钟与 ROS 时钟之间的基准点。
  bool esp_clock_ready_{false};
  std::uint32_t esp_epoch_ms_{};
  std::uint32_t max_esp_stamp_ms_{};
  rclcpp::Time esp_epoch_ros_{0, 0, RCL_ROS_TIME};

  // 标准话题发布器。
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // 为兼容原有调试程序而保留的话题发布器。
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
};

}  // namespace esp32_car_control
