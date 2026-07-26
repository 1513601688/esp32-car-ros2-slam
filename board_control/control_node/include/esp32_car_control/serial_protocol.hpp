#pragma once

#include <cstdint>
#include <string_view>
#include <variant>

namespace esp32_car_control
{

struct WheelSample
{
  std::uint32_t stamp_ms{};
  double vx{};
  double vy{};
  double wz{};
};

struct ImuSample
{
  std::uint32_t stamp_ms{};
  double ax{};
  double ay{};
  double az{};
  double gx{};
  double gy{};
  double gz{};
  double roll{};
  double pitch{};
  double yaw{};
};

using SensorSample = std::variant<WheelSample, ImuSample>;

// Parses one complete ASCII line without the trailing newline.
// Invalid, incomplete, or overlong lines return false.
bool parse_sensor_line(std::string_view line, SensorSample & sample);

}  // namespace esp32_car_control
