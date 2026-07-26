/**
 * @file mapping_teleop_node.cpp
 * @brief 开发板本地建图键盘遥控节点。
 *
 * 本节点只负责读取开发板 SSH 终端的按键并发布 /cmd_vel，不直接访问
 * ESP32 串口。esp32_car_node 仍是唯一的串口控制节点，并提供 0.5 秒
 * /cmd_vel 看门狗保护。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

/** 在不阻塞 ROS 定时器的情况下从当前 SSH 终端读取单个按键。 */
class TerminalInput
{
public:
  TerminalInput()
  {
    // ros2 run 的 stdin 可能被重定向，因此显式打开进程所属的控制终端。
    fd_ = ::open("/dev/tty", O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(
              std::string("无法打开 /dev/tty，请在开发板交互式 SSH 终端运行: ") +
              std::strerror(errno));
    }

    if (::tcgetattr(fd_, &original_) != 0) {
      const std::string reason = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("读取终端配置失败: " + reason);
    }

    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &raw) != 0) {
      const std::string reason = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("设置终端原始输入模式失败: " + reason);
    }
  }

  TerminalInput(const TerminalInput &) = delete;
  TerminalInput & operator=(const TerminalInput &) = delete;

  ~TerminalInput()
  {
    if (fd_ >= 0) {
      ::tcsetattr(fd_, TCSANOW, &original_);
      ::close(fd_);
    }
  }

  int read_key() const
  {
    char key = 0;
    const ssize_t count = ::read(fd_, &key, 1);
    return count == 1 ? static_cast<unsigned char>(key) : -1;
  }

private:
  int fd_{-1};
  termios original_{};
};

class MappingTeleopNode : public rclcpp::Node
{
public:
  MappingTeleopNode()
  : Node("esp32_car_mapping_teleop")
  {
    linear_speed_ = declare_parameter<double>("linear_speed", 0.20);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.40);
    linear_speed_step_ = declare_parameter<double>("linear_speed_step", 0.05);
    angular_speed_step_ = declare_parameter<double>("angular_speed_step", 0.10);
    max_teleop_linear_speed_ = declare_parameter<double>("max_teleop_linear_speed", 0.60);
    max_teleop_angular_speed_ = declare_parameter<double>("max_teleop_angular_speed", 1.20);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    key_timeout_sec_ = declare_parameter<double>("key_timeout_sec", 0.70);
    repeat_timeout_sec_ = declare_parameter<double>("repeat_timeout_sec", 0.12);
    repeat_detection_max_sec_ =
      declare_parameter<double>("repeat_detection_max_sec", 0.25);

    if (linear_speed_ <= 0.0 || angular_speed_ <= 0.0 ||
      linear_speed_step_ <= 0.0 || angular_speed_step_ <= 0.0 ||
      max_teleop_linear_speed_ < linear_speed_ ||
      max_teleop_angular_speed_ < angular_speed_ ||
      publish_rate_hz_ <= 2.0 || key_timeout_sec_ <= 0.0 ||
      repeat_timeout_sec_ <= 0.0 || repeat_timeout_sec_ >= key_timeout_sec_ ||
      repeat_detection_max_sec_ <= repeat_timeout_sec_)
    {
      throw std::invalid_argument(
              "遥控速度和超时参数无效：repeat_timeout_sec 必须小于 "
              "key_timeout_sec，publish_rate_hz 必须大于 2 Hz");
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {update();});

    print_help();
    RCLCPP_INFO(
      get_logger(),
      "开发板本地遥控已启动：线速度 %.2f m/s，角速度 %.2f rad/s，"
      "连续按键松开后约 %.2f 秒停车",
      linear_speed_, angular_speed_, repeat_timeout_sec_);
  }

  ~MappingTeleopNode() override
  {
    publish_stop();
  }

private:
  void print_help() const
  {
    RCLCPP_INFO(
      get_logger(),
      "\n"
      "========== ESP32 Car 开发板本地建图遥控 ==========\n"
      " W / S       前进 / 后退\n"
      " A / D       左移 / 右移\n"
      " Q / E       逆时针 / 顺时针旋转\n"
      " U / O       左前 / 右前斜向平移\n"
      " M / .       左后 / 右后斜向平移\n"
      " 1 / 2 / 3   低速 / 中速 / 高速\n"
      " + / -       逐级加速 / 减速\n"
      " K / X / 空格 立即停车\n"
      " Ctrl-C      停车并退出\n"
      "按住运动键持续行驶；松开后自动停车。\n"
      "==================================================");
  }

  void update()
  {
    // 一次处理终端缓冲区中的全部按键，减少 SSH 输入突发时的响应延迟。
    for (int key = terminal_.read_key(); key >= 0; key = terminal_.read_key()) {
      handle_key(key);
      if (!rclcpp::ok()) {
        return;
      }
    }

    if (!motion_active_) {
      return;
    }

    const double idle_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_motion_key_time_).count();

    // SSH 终端没有 key-up 事件。首次按键使用较长超时跨过操作系统的首次
    // 键盘重复延迟；检测到连续重复后切换到短超时，松手即可快速停车。
    const double stop_timeout = repeat_detected_ ? repeat_timeout_sec_ : key_timeout_sec_;
    if (idle_seconds >= stop_timeout) {
      stop_motion("松键超时，已停车");
      return;
    }

    // 运动期间持续发布，频率高于底盘节点的看门狗要求。
    cmd_vel_pub_->publish(command_);
  }

  void handle_key(int key)
  {
    const char normalized = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    const char * action = nullptr;
    const auto now = std::chrono::steady_clock::now();

    // 每个按键都从全零 Twist 构造完整命令，方向切换时不会残留上一条命令的
    // vx、vy 或 wz。SSH 无法可靠提供多键状态，斜移由独立单键完成。
    geometry_msgs::msg::Twist next;

    switch (normalized) {
      case 'w':
        next.linear.x = linear_speed_;
        action = "前进";
        break;
      case 's':
        next.linear.x = -linear_speed_;
        action = "后退";
        break;
      case 'a':
        next.linear.y = linear_speed_;
        action = "左移";
        break;
      case 'd':
        next.linear.y = -linear_speed_;
        action = "右移";
        break;
      case 'q':
        next.angular.z = angular_speed_;
        action = "逆时针旋转";
        break;
      case 'e':
        next.angular.z = -angular_speed_;
        action = "顺时针旋转";
        break;
      case 'u':
        next = geometry_msgs::msg::Twist();
        next.linear.x = linear_speed_;
        next.linear.y = linear_speed_;
        action = "左前斜移";
        break;
      case 'o':
        next = geometry_msgs::msg::Twist();
        next.linear.x = linear_speed_;
        next.linear.y = -linear_speed_;
        action = "右前斜移";
        break;
      case 'm':
        next = geometry_msgs::msg::Twist();
        next.linear.x = -linear_speed_;
        next.linear.y = linear_speed_;
        action = "左后斜移";
        break;
      case '.':
        next = geometry_msgs::msg::Twist();
        next.linear.x = -linear_speed_;
        next.linear.y = -linear_speed_;
        action = "右后斜移";
        break;
      case '1':
        set_speed(0.10, 0.20, "低速档");
        return;
      case '2':
        set_speed(0.20, 0.40, "中速档");
        return;
      case '3':
        set_speed(0.35, 0.70, "高速档");
        return;
      case '=':
      case '+':
        adjust_speed(1.0);
        return;
      case '-':
      case '_':
        adjust_speed(-1.0);
        return;
      case 'k':
      case 'x':
      case ' ':
        stop_motion("手动停车");
        return;
      case 0x03:  // Ctrl-C；终端仍保留 ISIG 时通常由 ROS 信号处理器接管。
        publish_stop();
        rclcpp::shutdown();
        return;
      default:
        return;
    }

    const bool command_changed =
      !motion_active_ || next.linear.x != command_.linear.x ||
      next.linear.y != command_.linear.y || next.angular.z != command_.angular.z;

    if (motion_active_ && normalized == active_motion_key_) {
      const double repeat_interval = std::chrono::duration<double>(
        now - last_motion_key_time_).count();
      if (repeat_interval <= repeat_detection_max_sec_) {
        repeat_detected_ = true;
      }
    } else {
      repeat_detected_ = false;
    }

    command_ = next;
    active_motion_key_ = normalized;
    last_motion_key_time_ = now;
    motion_active_ = true;

    // 按下后立即发布一帧，不必等待下一个定时器周期。
    cmd_vel_pub_->publish(command_);
    if (command_changed) {
      RCLCPP_INFO(
        get_logger(), "%s  vx=%.2f  vy=%.2f  wz=%.2f",
        action, command_.linear.x, command_.linear.y, command_.angular.z);
    }
  }

  void stop_motion(const char * reason)
  {
    const bool was_moving = motion_active_;
    motion_active_ = false;
    repeat_detected_ = false;
    active_motion_key_ = 0;
    command_ = geometry_msgs::msg::Twist();
    publish_stop();
    if (was_moving) {
      RCLCPP_INFO(get_logger(), "%s", reason);
    }
  }

  void adjust_speed(double direction)
  {
    set_speed(
      linear_speed_ + direction * linear_speed_step_,
      angular_speed_ + direction * angular_speed_step_,
      direction > 0.0 ? "加速" : "减速");
  }

  void set_speed(double linear, double angular, const char * label)
  {
    stop_motion("切换速度档位前已停车");
    linear_speed_ = std::clamp(linear, min_linear_speed_, max_teleop_linear_speed_);
    angular_speed_ = std::clamp(angular, min_angular_speed_, max_teleop_angular_speed_);

    // 同步 ROS 参数，便于使用 ros2 param get 查看当前实时速度设置。
    set_parameters({
        rclcpp::Parameter("linear_speed", linear_speed_),
        rclcpp::Parameter("angular_speed", angular_speed_)
    });

    RCLCPP_INFO(
      get_logger(), "%s：线速度 %.2f m/s，角速度 %.2f rad/s",
      label, linear_speed_, angular_speed_);
  }

  void publish_stop()
  {
    if (cmd_vel_pub_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  TerminalInput terminal_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist command_;
  std::chrono::steady_clock::time_point last_motion_key_time_{};
  double linear_speed_{0.20};
  double angular_speed_{0.40};
  double linear_speed_step_{0.05};
  double angular_speed_step_{0.10};
  double max_teleop_linear_speed_{0.60};
  double max_teleop_angular_speed_{1.20};
  const double min_linear_speed_{0.05};
  const double min_angular_speed_{0.10};
  double publish_rate_hz_{50.0};
  double key_timeout_sec_{0.70};
  double repeat_timeout_sec_{0.12};
  double repeat_detection_max_sec_{0.25};
  bool motion_active_{false};
  bool repeat_detected_{false};
  char active_motion_key_{0};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MappingTeleopNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mapping_teleop"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
