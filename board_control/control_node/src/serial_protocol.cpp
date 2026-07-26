#include "esp32_car_control/serial_protocol.hpp"

#include <cstdio>
#include <limits>
#include <string>

namespace esp32_car_control
{

bool parse_sensor_line(std::string_view line, SensorSample & sample)
{
  // Keep parsing bounded and provide a NUL terminator for sscanf.
  if (line.empty() || line.size() > 255U) 
  {
    return false;
  }
  const std::string text(line);
  unsigned long stamp = 0;
  int consumed = 0;

  WheelSample wheel;
  if (std::sscanf(
      text.c_str(), "WHEEL,%lu,%lf,%lf,%lf%n", &stamp,
      &wheel.vx, &wheel.vy, &wheel.wz, &consumed) == 4 &&
    consumed == static_cast<int>(text.size()) &&
    stamp <= std::numeric_limits<std::uint32_t>::max())
  {
    wheel.stamp_ms = static_cast<std::uint32_t>(stamp);
    sample = wheel;
    return true;
  }

  ImuSample imu;
  consumed = 0;
  if (std::sscanf(
      text.c_str(), "IMU,%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf%n", &stamp,
      &imu.ax, &imu.ay, &imu.az, &imu.gx, &imu.gy, &imu.gz,
      &imu.roll, &imu.pitch, &imu.yaw, &consumed) == 10 &&
    consumed == static_cast<int>(text.size()) &&
    stamp <= std::numeric_limits<std::uint32_t>::max())
  {
    imu.stamp_ms = static_cast<std::uint32_t>(stamp);
    sample = imu;
    return true;
  }
  return false;
}

}  // namespace esp32_car_control
