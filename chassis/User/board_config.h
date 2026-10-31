#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#define BOARD_SYSTEM_CLOCK_HZ              (72000000UL)
#define BOARD_MOTOR_PWM_HZ                 (80000UL)
#define BOARD_CONTROL_PERIOD_MS            (10U)
#define BOARD_CONTROL_DT_MIN_SEC            (0.005f)
#define BOARD_CONTROL_DT_MAX_SEC            (0.030f)
#define BOARD_COMMAND_TIMEOUT_MS            (500U)
#define BOARD_TELEMETRY_PERIOD_MS           (20U)

#define BOARD_MOTOR_LEFT_INVERT            (0U)
#define BOARD_MOTOR_RIGHT_INVERT           (0U)
#define BOARD_MAX_LINEAR_SPEED_M_S          (0.50f)
#define BOARD_MAX_ANGULAR_SPEED_RAD_S       (3.00f)
#define BOARD_MAX_WHEEL_SPEED_M_S           (0.70f)

#define BOARD_ENCODER_MOTOR_PULSES_PER_REV (13U)
#define BOARD_ENCODER_GEAR_RATIO           (30U)
#define BOARD_ENCODER_QUADRATURE_FACTOR    (4U)
#define BOARD_ENCODER_COUNTS_PER_WHEEL_REV \
    (BOARD_ENCODER_MOTOR_PULSES_PER_REV * BOARD_ENCODER_GEAR_RATIO * BOARD_ENCODER_QUADRATURE_FACTOR)
/* Measured 2026-10-18: 2 m straight push x3, 1560 counts/rev.
 * Left averaged 13789 counts, right 13474; the 2.3% gap repeated every run. */
#define BOARD_WHEEL_CIRCUMFERENCE_LEFT_M    (0.22627f)
#define BOARD_WHEEL_CIRCUMFERENCE_RIGHT_M   (0.23155f)
#define BOARD_WHEEL_CIRCUMFERENCE_M \
    ((BOARD_WHEEL_CIRCUMFERENCE_LEFT_M + BOARD_WHEEL_CIRCUMFERENCE_RIGHT_M) * 0.5f)
#define BOARD_METERS_PER_COUNT_LEFT \
    (BOARD_WHEEL_CIRCUMFERENCE_LEFT_M / (float)BOARD_ENCODER_COUNTS_PER_WHEEL_REV)
#define BOARD_METERS_PER_COUNT_RIGHT \
    (BOARD_WHEEL_CIRCUMFERENCE_RIGHT_M / (float)BOARD_ENCODER_COUNTS_PER_WHEEL_REV)
#define BOARD_WHEEL_SEPARATION_M            (0.115f) /* Measured contact-centre distance. */
#define BOARD_ENCODER_LEFT_SIGN             (1)
#define BOARD_ENCODER_RIGHT_SIGN            (-1)

#define BOARD_UART_BAUDRATE                 (115200UL)

#define BOARD_IMU_I2C_ADDRESS_7BIT          (0x68U)
#define BOARD_IMU_CALIBRATION_SAMPLES       (200U)
#define BOARD_IMU_CALIBRATION_DELAY_MS      (10U)
#define BOARD_IMU_GYRO_FUSION_WEIGHT        (0.70f)
#define BOARD_ODOM_STATIONARY_CONFIRM_SEC   (0.20f)
#define BOARD_ODOM_STATIONARY_GYRO_MAX_RAD_S (0.05f)
#define BOARD_ODOM_GYRO_BIAS_TAU_SEC        (1.00f)

/* IMU axis source: 0=X, 1=Y, 2=Z. Change these after mounting is confirmed. */
#define BOARD_IMU_AXIS_X_SOURCE             (1U)
#define BOARD_IMU_AXIS_Y_SOURCE             (0U)
#define BOARD_IMU_AXIS_Z_SOURCE             (2U)
#define BOARD_IMU_AXIS_X_SIGN               (1.0f)
#define BOARD_IMU_AXIS_Y_SIGN               (-1.0f)
#define BOARD_IMU_AXIS_Z_SIGN               (1.0f)

#endif
