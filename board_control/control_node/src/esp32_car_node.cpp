/**
 * esp32_car_node.cpp — ESP32-Car 底盘串口控制节点 (ROS 2)
 *
 * 功能:
 *   1. 订阅 /cmd_vel, 通过串口发送速度指令到 ESP32
 *   2. 独立接收轮速和 IMU 数据
 *   3. 发布标准 /wheel/odom 和 /imu/data 话题
 *   4. 提供定时定速服务和 /cmd_vel 超时停车保护
 *
 * 下行协议: 14字节帧, 0xAA + 3×f32 LE + 0x55
 * 上行协议: WHEEL/IMU ASCII 行, 详见 serial_protocol.hpp
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "esp32_car_control/sensor_publisher.hpp"
#include "esp32_car_control/serial_protocol.hpp"
#include "esp32_car_control/srv/timed_velocity.hpp"

using namespace std::chrono_literals;

// ============================================================================
// 协议常量
// ============================================================================
static constexpr uint8_t  FRAME_HDR = 0xAA;
static constexpr uint8_t  FRAME_TAIL = 0x55;
static constexpr int      FRAME_LEN = 14;             // 1 + 4*3 + 1
static constexpr int      PAYLOAD_LEN = 12;           // 3 × f32

// 运动学限制
static constexpr double   MAX_LINEAR_MPS = 1.5;       // 最大线速度  m/s
static constexpr double   MAX_ANGULAR_RPS = 5.0;      // 最大角速度  rad/s

// ============================================================================
// 串口封装 (Linux termios)
// ============================================================================
class SerialPort
{
public:
  SerialPort()
  : fd_(-1) {}

  ~SerialPort() {close();}

  bool open(const std::string & device, int baud = 115200)
  {
    // 阻塞模式 + VMIN=0/VTIME=0 (轮询模式, read 立即返回, 不阻塞)
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("SerialPort"),
                   "无法打开串口 %s: %s", device.c_str(), strerror(errno));
      return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "tcgetattr 失败");
      close();
      return false;
    }

    // 波特率
    speed_t speed = B115200;
    switch (baud) {
      case 9600:    speed = B9600;    break;
      case 57600:   speed = B57600;   break;
      case 115200:  speed = B115200;  break;
      case 230400:  speed = B230400;  break;
      case 460800:  speed = B460800;  break;
      case 921600:  speed = B921600;  break;
      default:      speed = B115200;  break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;   // 无硬件流控

    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | ISTRIP);
    tty.c_oflag &= ~OPOST;

    // 轮询模式: VMIN=0, VTIME=0 — read() 立即返回已有数据, 不等待
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "tcsetattr 失败");
      close();
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("SerialPort"),
                "串口 %s @ %d 波特 已打开", device.c_str(), baud);
    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const {return fd_ >= 0;}

  /** 发送原始字节 */
  int write_bytes(const uint8_t * data, int len)
  {
    if (fd_ < 0) {return -1;}
    return ::write(fd_, data, len);
  }

  /** 非阻塞读取可用字节数 */
  int available()
  {
    if (fd_ < 0) {return 0;}
    int bytes = 0;
    ioctl(fd_, FIONREAD, &bytes);
    return bytes;
  }

  /** 读取指定字节数 (阻塞最多约 100ms) */
  int read_bytes(uint8_t * buf, int len)
  {
    if (fd_ < 0) {return -1;}
    int total = 0;
    while (total < len) {
      int n = ::read(fd_, buf + total, len - total);
      if (n > 0) {total += n;} else if (n == 0) {
        break;
      } else if (errno != EAGAIN && errno != EWOULDBLOCK) {break;}
    }
    return total;
  }

  /** 读取单个字节 */
  int read_byte()
  {
    uint8_t b;
    int n = ::read(fd_, &b, 1);
    return (n == 1) ? b : -1;
  }

  /** 一次性读取当前可用数据 (类似 pyserial read) */
  int read_all(uint8_t * buf, int max_len)
  {
    return ::read(fd_, buf, max_len);
  }

  int get_fd() const {return fd_;}

private:
  int fd_;
};

// ============================================================================
// ESP32CarNode
// ============================================================================
class ESP32CarNode : public rclcpp::Node
{
public:
  ESP32CarNode()
  : Node("esp32_car_node"),
    target_vx_(0.0), target_vy_(0.0), target_wz_(0.0)
  {
    // --- 参数声明 ---
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("pub_rate_hz", 20.0);
    this->declare_parameter("max_linear_mps", 1.5);
    this->declare_parameter("max_angular_rps", 5.0);
    this->declare_parameter("cmd_vel_timeout_sec", 0.5);

    // /cmd_vel 需要持续刷新；发布端掉线或网络中断时自动停车。
    cmd_vel_timeout_sec_ = this->get_parameter("cmd_vel_timeout_sec").as_double();
    max_linear_ = this->get_parameter("max_linear_mps").as_double();
    max_angular_ = this->get_parameter("max_angular_rps").as_double();

    // --- 打开串口 ---
    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    if (!serial_.open(port, baud)) {
      RCLCPP_FATAL(this->get_logger(), "串口打开失败, 节点即将退出");
      rclcpp::shutdown();
      return;
    }

    // 发送零速度指令初始化
    send_cmd_vel(0.0, 0.0, 0.0);

    // --- 订阅 /cmd_vel ---
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_callback(msg);
      });

    // 传感器消息转换与发布由独立模块负责。
    sensor_publisher_ = std::make_unique<esp32_car_control::SensorPublisher>(*this);

    // --- 发布实际下发的指令速度 (/cmd_vel 或定时定速服务 → 串口) ---
    cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/esp32_car/command", 10);

    // --- 主循环定时器: 50Hz ---
    timer_ = this->create_wall_timer(20ms, [this]() {main_loop();});

    // --- 定时定速 Service ---
    timed_vel_srv_ = this->create_service<esp32_car_control::srv::TimedVelocity>(
      "/esp32_car/timed_velocity",
      [this](const std::shared_ptr<esp32_car_control::srv::TimedVelocity::Request> req,
      std::shared_ptr<esp32_car_control::srv::TimedVelocity::Response> res) {
        timed_velocity_service(req, res);
      });

    RCLCPP_INFO(this->get_logger(), "ESP32-Car 控制节点已启动");
  }

  ~ESP32CarNode() override
  {
    // 停止底盘
    send_cmd_vel(0.0, 0.0, 0.0);
    std::this_thread::sleep_for(100ms);
    serial_.close();
    RCLCPP_INFO(this->get_logger(), "ESP32-Car 控制节点已退出");
  }

private:
  // ----- 串口帧发送 ------------------------------------------------
  void send_cmd_vel(float vx, float vy, float wz)
  {
    static int send_count = 0;
    uint8_t frame[FRAME_LEN];
    frame[0] = FRAME_HDR;
    memcpy(&frame[1], &vx, 4);
    memcpy(&frame[5], &vy, 4);
    memcpy(&frame[9], &wz, 4);
    frame[13] = FRAME_TAIL;
    serial_.write_bytes(frame, FRAME_LEN);

    // 每 50 帧打印一次发送数据
    if (++send_count % 50 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "📤 发送#%d hex: aa %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x 55  → Vx=%.3f Vy=%.3f Wz=%.3f",
        send_count,
        frame[1], frame[2], frame[3], frame[4],
        frame[5], frame[6], frame[7], frame[8],
        frame[9], frame[10], frame[11], frame[12],
        vx, vy, wz);
    }
  }

  // ----- 串口 ASCII 行接收 (WHEEL 与 IMU 独立帧) -----------------
  void recv_sensor_data()
  {
    // 1. 一次性读空内核缓冲区
    uint8_t chunk[256];
    int n = serial_.read_all(chunk, sizeof(chunk));
    while (n > 0) {
      rx_buf_.insert(rx_buf_.end(), chunk, chunk + n);
      n = serial_.read_all(chunk, sizeof(chunk));
    }

    // 限制缓冲区最大 4KB, 防止异常时无限增长
    if (rx_buf_.size() > 4096) {
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 2048);
    }

    // 2. 逐行解析
    while (true) {
      auto nl = std::find(rx_buf_.begin(), rx_buf_.end(), '\n');
      if (nl == rx_buf_.end()) {break;}

      // 构造一行字符串 (不含 \n)
      std::string line(rx_buf_.begin(), nl);
      rx_buf_.erase(rx_buf_.begin(), nl + 1);  // 消费此行

      esp32_car_control::SensorSample sample;
      if (!esp32_car_control::parse_sensor_line(line, sample)) {
        continue;
      }

      if (const auto * wheel = std::get_if<esp32_car_control::WheelSample>(&sample)) {
        sensor_publisher_->publish(*wheel);
        const int count = wheel_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (count % 10 == 0) {
          RCLCPP_INFO(this->get_logger(),
            "WHEEL #%d stamp=%u vx=%.3f vy=%.3f wz=%.4f",
            count, wheel->stamp_ms, wheel->vx, wheel->vy, wheel->wz);
        }
        continue;
      }

      if (const auto * imu = std::get_if<esp32_car_control::ImuSample>(&sample)) {
        sensor_publisher_->publish(*imu);
        const int count = imu_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (count % 50 == 0) {
          RCLCPP_INFO(this->get_logger(),
            "IMU #%d stamp=%u gz=%.4f yaw=%.3f",
            count, imu->stamp_ms, imu->gz, imu->yaw);
        }
      }
    }
  }

  // ----- cmd_vel 回调 (ROS 话题 → 目标速度) -----------------------
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cancel_timed_velocity();
    target_vx_ = clamp(msg->linear.x, -max_linear_, max_linear_);
    target_vy_ = clamp(msg->linear.y, -max_linear_, max_linear_);
    target_wz_ = clamp(msg->angular.z, -max_angular_, max_angular_);
    last_cmd_vel_time_ = this->now();
    cmd_vel_active_ = true;
  }

  // ----- /cmd_vel 超时保护 -----------------------------------------
  void check_cmd_vel_timeout()
  {
    // 定时定速由自身的 duration 管理，不受 /cmd_vel 看门狗影响。
    if (!cmd_vel_active_ || timed_active_ || cmd_vel_timeout_sec_ <= 0.0) {
      return;
    }

    const double elapsed = (this->now() - last_cmd_vel_time_).seconds();
    if (elapsed < cmd_vel_timeout_sec_) {
      return;
    }

    target_vx_ = 0.0;
    target_vy_ = 0.0;
    target_wz_ = 0.0;
    cmd_vel_active_ = false;
    RCLCPP_WARN(
      this->get_logger(),
      "/cmd_vel 超过 %.2f 秒未更新，已自动停车",
      cmd_vel_timeout_sec_);
  }

  // ----- 发布实际下发到串口的指令速度 -------------------------------
  void publish_command()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = snap_to_zero(target_vx_);
    msg.linear.y = snap_to_zero(target_vy_);
    msg.angular.z = snap_to_zero(target_wz_);
    cmd_vel_out_pub_->publish(msg);
  }

  // ----- 定时定速 --------------------------------------------------
  void check_timed_velocity()
  {
    if (!timed_active_) {return;}

    auto elapsed = (this->now() - timed_start_).seconds();
    if (elapsed >= timed_duration_) {
      // 定时结束: 自动停车
      timed_active_ = false;
      target_vx_ = 0.0;
      target_vy_ = 0.0;
      target_wz_ = 0.0;
      RCLCPP_INFO(this->get_logger(),
        "定时定速结束 (%.1f 秒)", timed_duration_);
    } else {
      // 每帧覆盖目标速度，防止其他控制源的残留值影响定时定速。
      target_vx_ = timed_vx_;
      target_vy_ = timed_vy_;
      target_wz_ = timed_wz_;
    }
  }

  void cancel_timed_velocity()
  {
    if (timed_active_) {
      timed_active_ = false;
      // 停止小车。
      target_vx_ = 0.0;
      target_vy_ = 0.0;
      target_wz_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "定时定速已取消, 小车停止");
    }
  }

  void timed_velocity_service(
    const std::shared_ptr<esp32_car_control::srv::TimedVelocity::Request> req,
    std::shared_ptr<esp32_car_control::srv::TimedVelocity::Response> res)
  {
    if (req->duration <= 0.0) {
      res->success = false;
      res->message = "持续时间必须大于 0";
      return;
    }

    double vx = clamp(req->vx, -max_linear_, max_linear_);
    double vy = clamp(req->vy, -max_linear_, max_linear_);
    double wz = clamp(req->wz, -max_angular_, max_angular_);

    timed_vx_ = vx;
    timed_vy_ = vy;
    timed_wz_ = wz;
    timed_duration_ = req->duration;
    timed_start_ = this->now();
    timed_active_ = true;
    cmd_vel_active_ = false;

    // 立即覆盖目标速度
    target_vx_ = vx;
    target_vy_ = vy;
    target_wz_ = wz;

    RCLCPP_INFO(this->get_logger(),
      "Service 定时定速: Vx=%.2f Vy=%.2f Wz=%.2f 持续 %.1f 秒",
      vx, vy, wz, req->duration);

    res->success = true;
    res->message = "定时定速已启动";
  }

  // ----- 主循环 (50Hz) ---------------------------------------------
  void main_loop()
  {
    check_timed_velocity();
    check_cmd_vel_timeout();
    recv_sensor_data();
    send_cmd_vel(static_cast<float>(target_vx_),
                 static_cast<float>(target_vy_),
                 static_cast<float>(target_wz_));
    publish_command();
  }

  // ----- 工具函数 --------------------------------------------------
  static double clamp(double val, double lo, double hi)
  {
    return (val < lo) ? lo : (val > hi) ? hi : val;
  }

  /// 绝对值小于 epsilon 的值直接归零, 消除 IEEE 754 舍入噪声
  static double snap_to_zero(double val, double eps = 1e-9)
  {
    return (std::abs(val) < eps) ? 0.0 : val;
  }

  // ----- 成员变量 --------------------------------------------------
  // 串口
  SerialPort serial_;
  std::vector<uint8_t> rx_buf_;                   // 串口接收累积缓冲区

  // 目标速度 (实际下发到小车, 由定时定速或 cmd_vel 控制)
  double target_vx_, target_vy_, target_wz_;

  double max_linear_ = MAX_LINEAR_MPS;
  double max_angular_ = MAX_ANGULAR_RPS;
  bool   cmd_vel_active_ = false;
  double cmd_vel_timeout_sec_ = 0.5;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};

  // 定时定速状态
  bool timed_active_ = false;
  double timed_vx_ = 0.0, timed_vy_ = 0.0, timed_wz_ = 0.0;
  rclcpp::Time timed_start_{0, 0, RCL_ROS_TIME};
  double timed_duration_ = 0.0;
  std::atomic<int> wheel_frame_count_{0};
  std::atomic<int> imu_frame_count_{0};

  // ROS 2 接口
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<esp32_car_control::SensorPublisher> sensor_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Service 服务器 (定时定速)
  rclcpp::Service<esp32_car_control::srv::TimedVelocity>::SharedPtr timed_vel_srv_;
};

// ============================================================================
// main
// ============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ESP32CarNode>();
  if (!rclcpp::ok()) {return 0;}

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
