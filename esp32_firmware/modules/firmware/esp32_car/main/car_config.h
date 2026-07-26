#pragma once

/*
 * Shared vehicle and firmware constants.
 * Keep hardware-facing values here so modules do not silently diverge.
 */

#define CAR_MOTOR_COUNT             4

#define CAR_MOTOR_UART_TX_PIN       17
#define CAR_MOTOR_UART_RX_PIN       18
#define CAR_MOTOR_UART_BAUDRATE     512000

#define CAR_HTTP_PORT               80
#define CAR_TELEMETRY_PERIOD_US     100000

#define CAR_WHEEL_DIAMETER_M        0.035f
#define CAR_WHEEL_CENTER_RADIUS_M   0.170f
#define CAR_ENCODER_PULSES_PER_REV  3200.0f

/*
 * Command-side velocity calibration.
 * These scales compensate the current cmd_vel -> motor mapping so the chassis
 * tracks commanded body velocity more closely.
 *
 * Current VX scale comes from measured forward tests:
 * 0.10 / 0.20 / 0.30 m/s command produced about 1.60x actual speed.
 * 1 / 1.61 ~= 0.62, so we shrink forward command before joystick mapping.
 *
 * VY / WZ remain 1.0 until separate calibration data is collected.
 */
#define CAR_CMD_SCALE_VX            0.62f
#define CAR_CMD_SCALE_VY            0.62f
#define CAR_CMD_SCALE_WZ            0.47f

/*
 * Odom-side velocity calibration.
 * These scales compensate encoder-to-body-velocity estimation so ROS/web
 * feedback better matches measured chassis motion.
 *
 * Current VX scale comes from measured forward tests:
 * actual_speed / reported_speed ~= 1.90.
 *
 * VY / WZ remain 1.0 until separate calibration data is collected.
 */
#define CAR_ODOM_SCALE_VX           1.90f
#define CAR_ODOM_SCALE_VY           1.85f
#define CAR_ODOM_SCALE_WZ           1.00f

/*
 * Low-speed command shaping.
 * Keep a small noise deadzone so idle jitter does not move the chassis, then
 * guarantee a small but effective wheel RPM once a real command is accepted.
 */
#define CAR_CMD_LINEAR_DEADZONE     0.010f
#define CAR_CMD_ROT_DEADZONE        0.006f
#define CAR_MOTOR_MIN_EFFECTIVE_RPM 10U

/*
 * Encoder sign calibration from hardware direction to logical omni wheel speed.
 * Logical wheel order is FL, FR, RL, RR. For +Vx forward, the expected signs are
 * [+,-,-,+]; for +Vy left, expected signs are [+,+,-,-].
 */
#define CAR_ENCODER_DIR_FL          1
#define CAR_ENCODER_DIR_FR          1
#define CAR_ENCODER_DIR_RL         -1
#define CAR_ENCODER_DIR_RR         -1

/*
 * IMU yaw integration sign. Change to -1 if rotating the chassis clockwise makes
 * the displayed/ROS yaw move in the opposite direction expected by your ROS node.
 */
#define CAR_IMU_YAW_SIGN            1.0f
