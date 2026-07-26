#include "esp32_car_control/serial_protocol.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace
{
bool near(double lhs, double rhs) {return std::abs(lhs - rhs) < 1.0e-9;}
}

int main()
{
  using esp32_car_control::ImuSample;
  using esp32_car_control::SensorSample;
  using esp32_car_control::WheelSample;

  SensorSample sample;
  if (!esp32_car_control::parse_sensor_line("WHEEL,123,0.5,-0.2,1.25", sample)) {
    std::cerr << "valid WHEEL frame rejected\n";
    return 1;
  }
  const auto * wheel = std::get_if<WheelSample>(&sample);
  if (!wheel || wheel->stamp_ms != 123U || !near(wheel->vx, 0.5) ||
    !near(wheel->vy, -0.2) || !near(wheel->wz, 1.25))
  {
    std::cerr << "WHEEL fields parsed incorrectly\n";
    return 1;
  }

  if (!esp32_car_control::parse_sensor_line(
      "IMU,456,0.1,0.2,9.8,0.01,0.02,0.03,0.4,0.5,0.6", sample))
  {
    std::cerr << "valid IMU frame rejected\n";
    return 1;
  }
  const auto * imu = std::get_if<ImuSample>(&sample);
  if (!imu || imu->stamp_ms != 456U || !near(imu->az, 9.8) || !near(imu->yaw, 0.6)) {
    std::cerr << "IMU fields parsed incorrectly\n";
    return 1;
  }

  const std::string invalid[] = {
    "", "ODOM,1,2,3,4", "WHEEL,1,2,3", "WHEEL,1,2,3,4,5",
    "IMU,1,2,3", "IMU,1,2,3,4,5,6,7,8,9 trailing"
  };
  for (const auto & line : invalid) {
    if (esp32_car_control::parse_sensor_line(line, sample)) {
      std::cerr << "invalid frame accepted: " << line << '\n';
      return 1;
    }
  }
  return 0;
}
