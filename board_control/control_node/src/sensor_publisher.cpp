#include "esp32_car_control/sensor_publisher.hpp"

#include <algorithm>
#include <cstdint>

namespace esp32_car_control
{

SensorPublisher::SensorPublisher(rclcpp::Node & node)
: node_(node)
{
  // --------------------------------------------------------------------------
  // 1. 声明可配置参数
  // 实际值默认从 config/esp32_car.yaml 加载，代码中的值是未加载 YAML 时的后备值。
  // --------------------------------------------------------------------------
  node_.declare_parameter("wheel_odom_topic", "/wheel/odom");
  node_.declare_parameter("imu_topic", "/imu/data");
  node_.declare_parameter("odom_frame", "odom");
  node_.declare_parameter("base_frame", "base_footprint");
  node_.declare_parameter("imu_frame", "imu_link");
  node_.declare_parameter("wheel_vx_scale", 1.0);
  node_.declare_parameter("wheel_vy_scale", 1.0);
  node_.declare_parameter("wheel_linear_variance", 0.0025);
  node_.declare_parameter("wheel_angular_variance", 0.01);
  node_.declare_parameter("imu_angular_velocity_variance", 0.0025);
  node_.declare_parameter("imu_linear_acceleration_variance", 0.04);

  // --------------------------------------------------------------------------
  // 2. 缓存配置
  // 发布时直接使用成员变量，避免每收到一帧数据就重新查询 ROS 参数。
  // --------------------------------------------------------------------------
  odom_frame_ = node_.get_parameter("odom_frame").as_string();
  base_frame_ = node_.get_parameter("base_frame").as_string();
  imu_frame_ = node_.get_parameter("imu_frame").as_string();
  wheel_vx_scale_ = node_.get_parameter("wheel_vx_scale").as_double();
  wheel_vy_scale_ = node_.get_parameter("wheel_vy_scale").as_double();
  wheel_linear_variance_ = node_.get_parameter("wheel_linear_variance").as_double();
  wheel_angular_variance_ = node_.get_parameter("wheel_angular_variance").as_double();
  imu_angular_velocity_variance_ = node_.get_parameter("imu_angular_velocity_variance").as_double();
  imu_linear_acceleration_variance_ = node_.get_parameter("imu_linear_acceleration_variance").as_double();

  // --------------------------------------------------------------------------
  // 3. 创建发布器
  // 传感器话题使用 SensorDataQoS：优先保证数据实时性，允许丢弃过期数据。
  // --------------------------------------------------------------------------
  wheel_odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(node_.get_parameter("wheel_odom_topic").as_string(), rclcpp::SensorDataQoS());
  imu_pub_ = node_.create_publisher<sensor_msgs::msg::Imu>(node_.get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS());
  velocity_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>("/esp32_car/velocity", 10);
  yaw_pub_ = node_.create_publisher<std_msgs::msg::Float32>("/esp32_car/yaw", 10);
}

rclcpp::Time SensorPublisher::ros_stamp_from_esp(std::uint32_t stamp_ms)
{
  // ESP32 上报的是“自开机起经过的毫秒数”，它不能直接作为 ROS 时间戳。
  // 第一次收到数据时，记录 ESP32 时间和 ROS 时间的对应关系；后续只计算增量。
  const auto now = node_.now();
  if (!esp_clock_ready_ || (max_esp_stamp_ms_ > stamp_ms && max_esp_stamp_ms_ - stamp_ms > 5000U))
  {
    // 时间戳倒退超过 5 秒通常表示 ESP32 重启，重新建立时钟基准。
    esp_clock_ready_ = true;
    esp_epoch_ms_ = stamp_ms;
    max_esp_stamp_ms_ = stamp_ms;
    esp_epoch_ros_ = now;
    return now;
  }

  // WHEEL 与 IMU 独立发送，较晚到达的帧可能略旧，因此只保存最大时间戳。
  max_esp_stamp_ms_ = std::max(max_esp_stamp_ms_, stamp_ms);
  const auto delta_ms = static_cast<std::int64_t>(stamp_ms) - static_cast<std::int64_t>(esp_epoch_ms_);
  return esp_epoch_ros_ + rclcpp::Duration::from_seconds(delta_ms * 0.001);
}

void SensorPublisher::publish(const WheelSample & sample)
{
  // nav_msgs/Odometry 同时包含 pose（位置）和 twist（速度）。
  // 当前下位机只提供已校准的 vx、vy、wz，所以这里只填写 twist。
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = ros_stamp_from_esp(sample.stamp_ms);
  // 速度描述的是 base_footprint 相对于 odom 坐标系的二维运动。
  msg.header.frame_id = odom_frame_;
  msg.child_frame_id = base_frame_;

  // 当前没有轮式积分位置，因此 pose 不能作为有效测量参与融合。
  // 对 pose 六个维度设置很大的方差，告诉融合器不要信任这些字段。
  // 本节点也不发布 odom -> base_footprint TF；该 TF 由 robot_localization 统一发布。
  msg.pose.pose.orientation.w = 1.0;
  msg.pose.covariance.fill(0.0);
  for (std::size_t i = 0; i < 6; ++i)
  {
    msg.pose.covariance[i * 6 + i] = 1.0e6;
  }

  // ROS REP-103 约定：x 向前、y 向左、绕 z 轴逆时针为正。
  // 实车直行标定表明底盘上报的 vx 与 REP-103 的“x 向前为正”相反。
  // 通过参数修正坐标符号，避免修改串口解析和底盘控制方向。
  msg.twist.twist.linear.x = sample.vx * wheel_vx_scale_;
  // 左移实测用于同时校正 vy 的坐标符号和尺度。
  msg.twist.twist.linear.y = sample.vy * wheel_vy_scale_;
  msg.twist.twist.angular.z = sample.wz;
  msg.twist.covariance.fill(0.0);
  // 6x6 协方差顺序为 [vx, vy, vz, wx, wy, wz]。
  // 对角线索引 0、7、35 分别对应 vx、vy、wz 的方差。
  msg.twist.covariance[0] = wheel_linear_variance_;
  msg.twist.covariance[7] = wheel_linear_variance_;
  msg.twist.covariance[35] = wheel_angular_variance_;
  wheel_odom_pub_->publish(msg);

  // 兼容旧调试工具：复用相同数据发布简化的速度话题。
  geometry_msgs::msg::TwistStamped velocity;
  velocity.header = msg.header;
  velocity.header.frame_id = msg.child_frame_id;
  velocity.twist = msg.twist.twist;
  velocity_pub_->publish(velocity);
}

void SensorPublisher::publish(const ImuSample & sample)
{
  // sensor_msgs/Imu 包含姿态、角速度和线加速度，三者使用同一个采样时间戳。
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = ros_stamp_from_esp(sample.stamp_ms);
  msg.header.frame_id = imu_frame_;

  // 实测表明 JY901 的地磁融合 Yaw 在车体安装环境中存在明显非线性误差：
  // 实际旋转约 90° 时，融合 Yaw 只变化约 63°，而 gz 积分约为 91°。
  // 按 sensor_msgs/Imu 约定，将 orientation_covariance[0] 设为 -1，明确
  // 表示“不提供姿态估计”，防止 EKF 或其他节点误用不可靠的绝对 Yaw。
  // orientation 保持单位四元数占位；诊断用原始 Yaw 仍发布到 /esp32_car/yaw。
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1.0;
  msg.orientation_covariance.fill(0.0);
  msg.orientation_covariance[0] = -1.0;

  // 角速度单位 rad/s，线加速度单位 m/s^2。
  msg.angular_velocity.x = sample.gx;
  msg.angular_velocity.y = sample.gy;
  msg.angular_velocity.z = sample.gz;
  msg.linear_acceleration.x = sample.ax;
  msg.linear_acceleration.y = sample.ay;
  msg.linear_acceleration.z = sample.az;

  // 角速度和加速度协方差都是 3x3 矩阵，这里暂按各轴互不相关，
  // 只填写对角线。后续可根据静止采样和实车数据更新 YAML 参数。
  msg.angular_velocity_covariance.fill(0.0);
  msg.linear_acceleration_covariance.fill(0.0);
  for (std::size_t i : {0U, 4U, 8U})
  {
    msg.angular_velocity_covariance[i] = imu_angular_velocity_variance_;
    msg.linear_acceleration_covariance[i] = imu_linear_acceleration_variance_;
  }
  imu_pub_->publish(msg);

  // 原始地磁融合 Yaw 不参与 /imu/data 融合，但继续以角度值发布，供诊断和校准。
  constexpr double radians_to_degrees = 57.29577951308232;
  std_msgs::msg::Float32 yaw;
  yaw.data = static_cast<float>(sample.yaw * radians_to_degrees);
  yaw_pub_->publish(yaw);
}

}  // namespace esp32_car_control
