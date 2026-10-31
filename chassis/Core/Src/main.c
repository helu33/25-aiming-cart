/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board_config.h"
#include "chassis_config.h"
#include "app_chassis.h"
#include "app_motor.h"
#include "app_odometry.h"
#include "app_uart.h"
#include "bsp_encoder.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HARDWARE_TEST_MODE_NORMAL          (0U)
#define HARDWARE_TEST_MODE_ENCODER_UART    (1U)
#define HARDWARE_TEST_MODE_IMU_UART        (2U)
#define HARDWARE_TEST_MODE_MOTOR_OPEN_LOOP (3U)
#define HARDWARE_TEST_MODE_MOTOR_CLOSED_LOOP (4U)
#define HARDWARE_TEST_MODE_CHASSIS_ROTATION  (5U)
#define HARDWARE_TEST_MODE                 HARDWARE_TEST_MODE_NORMAL
#define HARDWARE_TEST_UART_PERIOD_MS       (100U)
#define MOTOR_OPEN_LOOP_TEST_DUTY_PERCENT  (20)
#define MOTOR_OPEN_LOOP_TEST_ARM_MS        (5000U)
#define MOTOR_OPEN_LOOP_TEST_RUN_MS        (3000U)
#define MOTOR_CLOSED_LOOP_TEST_TARGET_M_S  (0.15f)
#define MOTOR_CLOSED_LOOP_TEST_ARM_MS      (5000U)
#define MOTOR_CLOSED_LOOP_TEST_RUN_MS      (10000U)
#define ROTATION_TEST_ANGULAR_RAD_S        (1.50f)
#define ROTATION_TEST_ARM_MS               (2000U)
#define ROTATION_TEST_RUN_MS               (4000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#if (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_IMU_UART)
static uint8_t imu_uart_test_ready;
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_CHASSIS_ROTATION)
static uint8_t rotation_test_imu_ready;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_ENCODER_UART)
static void encoder_uart_test_publish(int32_t left_count,
                                      int32_t right_count,
                                      int32_t left_delta,
                                      int32_t right_delta)
{
  char line[72];
  const int length = snprintf(line, sizeof(line),
                              "ENC L=%ld R=%ld DL=%ld DR=%ld\r\n",
                              (long)left_count, (long)right_count,
                              (long)left_delta, (long)right_delta);

  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    (void)uart_bsp_write((const uint8_t *)line, (uint16_t)length);
  }
}
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_IMU_UART)
static int32_t imu_uart_test_round(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void imu_uart_test_publish(const imu_sample_t *sample)
{
  char line[128];
  const int32_t accel_x_mg = imu_uart_test_round((sample->accel_m_s2[0] / 9.80665f) * 1000.0f);
  const int32_t accel_y_mg = imu_uart_test_round((sample->accel_m_s2[1] / 9.80665f) * 1000.0f);
  const int32_t accel_z_mg = imu_uart_test_round((sample->accel_m_s2[2] / 9.80665f) * 1000.0f);
  const int32_t gyro_x_mrad_s = imu_uart_test_round(sample->gyro_rad_s[0] * 1000.0f);
  const int32_t gyro_y_mrad_s = imu_uart_test_round(sample->gyro_rad_s[1] * 1000.0f);
  const int32_t gyro_z_mrad_s = imu_uart_test_round(sample->gyro_rad_s[2] * 1000.0f);
  const int32_t temperature_centi_c = imu_uart_test_round(sample->temperature_c * 100.0f);
  const int length = snprintf(line, sizeof(line),
                              "IMU AX=%ld AY=%ld AZ=%ld GX=%ld GY=%ld GZ=%ld T=%ld\r\n",
                              (long)accel_x_mg, (long)accel_y_mg, (long)accel_z_mg,
                              (long)gyro_x_mrad_s, (long)gyro_y_mrad_s,
                              (long)gyro_z_mrad_s, (long)temperature_centi_c);

  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    (void)uart_bsp_write((const uint8_t *)line, (uint16_t)length);
  }
}
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_OPEN_LOOP)
static int32_t motor_uart_test_round(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void motor_uart_test_publish(uint32_t elapsed_ms,
                                    uint8_t state,
                                    int32_t left_count,
                                    int32_t right_count,
                                    float left_velocity_m_s,
                                    float right_velocity_m_s)
{
  char line[112];
  const int32_t left_velocity_mm_s = motor_uart_test_round(left_velocity_m_s * 1000.0f);
  const int32_t right_velocity_mm_s = motor_uart_test_round(right_velocity_m_s * 1000.0f);
  const int length = snprintf(line, sizeof(line),
                              "MOTOR T=%lu STATE=%u DUTY=%d L=%ld R=%ld VL=%ld VR=%ld\r\n",
                              (unsigned long)elapsed_ms, (unsigned int)state,
                              (state == 1U) ? (int)MOTOR_OPEN_LOOP_TEST_DUTY_PERCENT : 0,
                              (long)left_count, (long)right_count,
                              (long)left_velocity_mm_s, (long)right_velocity_mm_s);

  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    (void)uart_bsp_write((const uint8_t *)line, (uint16_t)length);
  }
}
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_CLOSED_LOOP)
static int32_t closed_loop_uart_test_round(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void closed_loop_uart_test_publish(uint32_t elapsed_ms, uint8_t state)
{
  char line[128];
  const int32_t target_mm_s = (state == 1U) ? closed_loop_uart_test_round(MOTOR_CLOSED_LOOP_TEST_TARGET_M_S * 1000.0f) : 0;
  const int32_t left_velocity_mm_s = closed_loop_uart_test_round(
      app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_LEFT) * 1000.0f);
  const int32_t right_velocity_mm_s = closed_loop_uart_test_round(
      app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_RIGHT) * 1000.0f);
  const int32_t left_error_mm_s = closed_loop_uart_test_round(
      app_motor_last_speed_err_left_get() * 1000.0f);
  const int32_t right_error_mm_s = closed_loop_uart_test_round(
      app_motor_last_speed_err_right_get() * 1000.0f);
  const int32_t left_duty_tenths = closed_loop_uart_test_round(
      app_motor_last_duty_percent_get(ENCODER_SIDE_LEFT) * 10.0f);
  const int32_t right_duty_tenths = closed_loop_uart_test_round(
      app_motor_last_duty_percent_get(ENCODER_SIDE_RIGHT) * 10.0f);
  const int length = snprintf(line, sizeof(line),
                              "CLOSED T=%lu STATE=%u TARGET=%ld VL=%ld VR=%ld EL=%ld ER=%ld UL=%ld UR=%ld\r\n",
                              (unsigned long)elapsed_ms, (unsigned int)state,
                              (long)target_mm_s, (long)left_velocity_mm_s,
                              (long)right_velocity_mm_s, (long)left_error_mm_s,
                              (long)right_error_mm_s, (long)left_duty_tenths,
                              (long)right_duty_tenths);

  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    (void)uart_bsp_write((const uint8_t *)line, (uint16_t)length);
  }
}
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_CHASSIS_ROTATION)
static int32_t rotation_uart_test_round(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void rotation_uart_test_publish(uint32_t elapsed_ms,
                                       uint8_t state,
                                       float encoder_yaw_rad,
                                       float gyro_yaw_rad,
                                       const imu_sample_t *imu_sample,
                                       uint8_t imu_valid)
{
  char line[176];
  const odometry_context_t *odom = app_odometry_state_get();
  const int32_t left_velocity_mm_s = rotation_uart_test_round(
      app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_LEFT) * 1000.0f);
  const int32_t right_velocity_mm_s = rotation_uart_test_round(
      app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_RIGHT) * 1000.0f);
  const int32_t gyro_z_mrad_s = ((imu_valid != 0U) && (imu_sample != NULL)) ?
      rotation_uart_test_round(imu_sample->gyro_rad_s[2] * 1000.0f) : 0;
  const int32_t left_duty_tenths = rotation_uart_test_round(
      app_motor_last_duty_percent_get(ENCODER_SIDE_LEFT) * 10.0f);
  const int32_t right_duty_tenths = rotation_uart_test_round(
      app_motor_last_duty_percent_get(ENCODER_SIDE_RIGHT) * 10.0f);
  const int length = snprintf(
      line, sizeof(line),
      "ROT T=%lu STATE=%u TARGET=%ld VL=%ld VR=%ld ENC=%ld GYRO=%ld FUSED=%ld GZ=%ld UL=%ld UR=%ld IMU=%u\r\n",
      (unsigned long)elapsed_ms, (unsigned int)state,
      (long)rotation_uart_test_round(ROTATION_TEST_ANGULAR_RAD_S * 1000.0f),
      (long)left_velocity_mm_s, (long)right_velocity_mm_s,
      (long)rotation_uart_test_round(encoder_yaw_rad * 1000.0f),
      (long)rotation_uart_test_round(gyro_yaw_rad * 1000.0f),
      (long)rotation_uart_test_round(odom->yaw_rad * 1000.0f),
      (long)gyro_z_mrad_s, (long)left_duty_tenths,
      (long)right_duty_tenths, (unsigned int)imu_valid);

  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    (void)uart_bsp_write((const uint8_t *)line, (uint16_t)length);
  }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();

  /* USER CODE BEGIN 2 */
  encoder_bsp_init();
  app_motor_init();
#if (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_ENCODER_UART)
  /* Leave both motor outputs in coast mode so the wheels can be turned by hand. */
  motor_bsp_stop(MOTOR_LEFT, MOTOR_DIR_COAST);
  motor_bsp_stop(MOTOR_RIGHT, MOTOR_DIR_COAST);
  uart_bsp_init();
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_IMU_UART)
  /* Keep the chassis still and level while the IMU performs its startup calibration. */
  motor_bsp_stop(MOTOR_LEFT, MOTOR_DIR_COAST);
  motor_bsp_stop(MOTOR_RIGHT, MOTOR_DIR_COAST);
  uart_bsp_init();
  imu_uart_test_ready = imu_bsp_init();
  if (imu_uart_test_ready != 0U)
  {
    imu_uart_test_ready = imu_bsp_calibrate(BOARD_IMU_CALIBRATION_SAMPLES,
                                            BOARD_IMU_CALIBRATION_DELAY_MS);
  }
  if (imu_uart_test_ready != 0U)
  {
    (void)uart_bsp_write_str("IMU READY\r\n");
  }
  else
  {
    (void)uart_bsp_write_str("IMU ERROR INIT\r\n");
  }
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_OPEN_LOOP)
  /* The open-loop test starts only after the arming delay in the main loop. */
  app_motor_stop();
  uart_bsp_init();
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_CLOSED_LOOP)
  /* The closed-loop test starts only after the arming delay in the main loop. */
  app_motor_stop();
  app_motor_pid_reset();
  uart_bsp_init();
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_CHASSIS_ROTATION)
  app_motor_stop();
  app_motor_pid_reset();
  app_odometry_init();
  uart_bsp_init();
  rotation_test_imu_ready = imu_bsp_init();
  if (rotation_test_imu_ready != 0U)
  {
    rotation_test_imu_ready = imu_bsp_calibrate(BOARD_IMU_CALIBRATION_SAMPLES,
                                                BOARD_IMU_CALIBRATION_DELAY_MS);
  }
  if (rotation_test_imu_ready != 0U)
  {
    (void)uart_bsp_write_str("ROT READY\r\n");
  }
  else
  {
    (void)uart_bsp_write_str("ROT ERROR IMU\r\n");
  }
#else
  app_motor_stop();
  app_odometry_init();
  {
    const uint8_t imu_initialized = imu_bsp_init();
    const uint8_t imu_calibrated = (imu_initialized != 0U) ?
        imu_bsp_calibrate(BOARD_IMU_CALIBRATION_SAMPLES, BOARD_IMU_CALIBRATION_DELAY_MS) : 0U;
    car_uart_fault_set(CAR_FAULT_IMU, (imu_calibrated == 0U) ? 1U : 0U);
    uart_bsp_init();
#if CHASSIS_AUTONOMOUS_ENABLE
    app_chassis_init(imu_calibrated);
#endif
  }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  {
    //记录下对应的时间戳，计算时间间隔
#if (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_NORMAL)
    uint32_t last_control_tick = HAL_GetTick();
    uint32_t next_control_tick = last_control_tick + BOARD_CONTROL_PERIOD_MS;
#if !CHASSIS_AUTONOMOUS_ENABLE
    uint32_t next_telemetry_tick = HAL_GetTick() + BOARD_TELEMETRY_PERIOD_MS;
    uint8_t was_halted = 1U;
#endif
    imu_sample_t imu_sample = {0};
    uint8_t imu_sample_valid = 0U;
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_ENCODER_UART)
    uint32_t next_encoder_uart_tick = HAL_GetTick();
    int32_t previous_left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    int32_t previous_right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_IMU_UART)
    uint32_t next_imu_uart_tick = HAL_GetTick();
    imu_sample_t imu_uart_sample = {0};
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_OPEN_LOOP)
    const uint32_t motor_test_start_tick = HAL_GetTick();
    uint32_t next_motor_uart_tick = motor_test_start_tick;
    uint32_t previous_motor_sample_tick = motor_test_start_tick;
    uint8_t motor_test_stopped = 0U;
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_CLOSED_LOOP)
    const uint32_t closed_loop_test_start_tick = HAL_GetTick();
    uint32_t closed_loop_last_control_tick = closed_loop_test_start_tick;
    uint32_t closed_loop_next_control_tick = closed_loop_test_start_tick + BOARD_CONTROL_PERIOD_MS;
    uint32_t closed_loop_next_uart_tick = closed_loop_test_start_tick;
    uint8_t closed_loop_test_started = 0U;
    uint8_t closed_loop_test_stopped = 0U;
#else
    const uint32_t rotation_test_start_tick = HAL_GetTick();
    uint32_t rotation_last_control_tick = rotation_test_start_tick;
    uint32_t rotation_next_control_tick = rotation_test_start_tick + BOARD_CONTROL_PERIOD_MS;
    uint32_t rotation_next_uart_tick = rotation_test_start_tick;
    int32_t rotation_previous_left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    int32_t rotation_previous_right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
    float rotation_encoder_yaw_rad = 0.0f;
    float rotation_gyro_yaw_rad = 0.0f;
    imu_sample_t rotation_imu_sample = {0};
    uint8_t rotation_imu_valid = 0U;
    uint8_t rotation_test_started = 0U;
    uint8_t rotation_test_stopped = 0U;
#endif

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
#if (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_NORMAL)
#if CHASSIS_AUTONOMOUS_ENABLE
    app_chassis_poll();
#else
    car_uart_command_poll();
#endif

    if ((int32_t)(HAL_GetTick() - next_control_tick) >= 0)
    {
      const uint32_t now = HAL_GetTick();
      const uint32_t elapsed_ms = now - last_control_tick;
      const float measurement_dt_sec = (elapsed_ms != 0U) ?
          ((float)elapsed_ms / 1000.0f) : BOARD_CONTROL_DT_MIN_SEC;
      float pid_dt_sec = measurement_dt_sec;
      last_control_tick = now;
      if (pid_dt_sec < BOARD_CONTROL_DT_MIN_SEC) pid_dt_sec = BOARD_CONTROL_DT_MIN_SEC;
      if (pid_dt_sec > BOARD_CONTROL_DT_MAX_SEC) pid_dt_sec = BOARD_CONTROL_DT_MAX_SEC;
      if ((uint32_t)(now - next_control_tick) >= BOARD_CONTROL_PERIOD_MS)
      {
        car_uart_fault_set(CAR_FAULT_CONTROL_LATE, 1U);
        next_control_tick = now + BOARD_CONTROL_PERIOD_MS;
      }
      else
      {
        next_control_tick += BOARD_CONTROL_PERIOD_MS;
      }
      encoder_bsp_velocity_sample(measurement_dt_sec);
      imu_sample_valid = imu_bsp_read(&imu_sample);
      car_uart_fault_set(CAR_FAULT_IMU, (imu_sample_valid == 0U) ? 1U : 0U);
      app_odometry_update(imu_sample.gyro_rad_s[2], imu_sample_valid, measurement_dt_sec);

#if CHASSIS_AUTONOMOUS_ENABLE
      app_chassis_control(measurement_dt_sec, pid_dt_sec, &imu_sample, imu_sample_valid);
#else
      if (car_uart_halted_get() != 0U)
      {
        if (was_halted == 0U)
        {
          app_motor_stop();
        }
        was_halted = 1U;
      }
      else
      {
        if (was_halted != 0U)
        {
          app_motor_pid_reset();
        }
        was_halted = 0U;
        app_motor_speed_control_update(pid_dt_sec);
      }
#endif
      /* Refresh only after the complete control iteration has run successfully. */
      (void)HAL_IWDG_Refresh(&hiwdg);
    }

#if !CHASSIS_AUTONOMOUS_ENABLE
    if ((int32_t)(HAL_GetTick() - next_telemetry_tick) >= 0)
    {
      next_telemetry_tick += BOARD_TELEMETRY_PERIOD_MS;
      car_uart_publish_telemetry(&imu_sample, imu_sample_valid);
    }
#endif

    __WFI();
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_ENCODER_UART)
    const uint32_t now = HAL_GetTick();

    if ((int32_t)(now - next_encoder_uart_tick) >= 0)
    {
      const int32_t left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
      const int32_t right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
      encoder_uart_test_publish(left_count, right_count,
                                left_count - previous_left_count,
                                right_count - previous_right_count);
      previous_left_count = left_count;
      previous_right_count = right_count;
      next_encoder_uart_tick += HARDWARE_TEST_UART_PERIOD_MS;
      if ((int32_t)(now - next_encoder_uart_tick) >= 0)
      {
        next_encoder_uart_tick = now + HARDWARE_TEST_UART_PERIOD_MS;
      }
    }

    /* The encoder-only test still keeps the independent watchdog alive. */
    (void)HAL_IWDG_Refresh(&hiwdg);
    __WFI();
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_IMU_UART)
    const uint32_t now = HAL_GetTick();

    if ((int32_t)(now - next_imu_uart_tick) >= 0)
    {
      if ((imu_uart_test_ready != 0U) && (imu_bsp_read(&imu_uart_sample) != 0U))
      {
        imu_uart_test_publish(&imu_uart_sample);
      }
      else
      {
        (void)uart_bsp_write_str("IMU ERROR READ\r\n");
      }
      next_imu_uart_tick += HARDWARE_TEST_UART_PERIOD_MS;
      if ((int32_t)(now - next_imu_uart_tick) >= 0)
      {
        next_imu_uart_tick = now + HARDWARE_TEST_UART_PERIOD_MS;
      }
    }

    /* The IMU-only test still keeps the independent watchdog alive. */
    (void)HAL_IWDG_Refresh(&hiwdg);
    __WFI();
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_OPEN_LOOP)
    const uint32_t now = HAL_GetTick();
    const uint32_t motor_test_elapsed_ms = now - motor_test_start_tick;
    uint8_t motor_test_state;

    if (motor_test_elapsed_ms < MOTOR_OPEN_LOOP_TEST_ARM_MS)
    {
      motor_test_state = 0U;
    }
    else if (motor_test_elapsed_ms <
             (MOTOR_OPEN_LOOP_TEST_ARM_MS + MOTOR_OPEN_LOOP_TEST_RUN_MS))
    {
      motor_test_state = 1U;
      app_motor_set_differential_manual((float)MOTOR_OPEN_LOOP_TEST_DUTY_PERCENT,
                                        (float)MOTOR_OPEN_LOOP_TEST_DUTY_PERCENT);
    }
    else
    {
      motor_test_state = 2U;
      if (motor_test_stopped == 0U)
      {
        app_motor_stop();
        motor_test_stopped = 1U;
      }
    }

    if ((int32_t)(now - next_motor_uart_tick) >= 0)
    {
      const uint32_t sample_elapsed_ms = now - previous_motor_sample_tick;
      const float sample_dt_sec = (sample_elapsed_ms != 0U) ?
          ((float)sample_elapsed_ms / 1000.0f) :
          ((float)HARDWARE_TEST_UART_PERIOD_MS / 1000.0f);
      previous_motor_sample_tick = now;
      encoder_bsp_velocity_sample(sample_dt_sec);
      motor_uart_test_publish(motor_test_elapsed_ms, motor_test_state,
                              encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT),
                              encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT),
                              encoder_bsp_velocity_m_s_get(ENCODER_SIDE_LEFT),
                              encoder_bsp_velocity_m_s_get(ENCODER_SIDE_RIGHT));
      next_motor_uart_tick += HARDWARE_TEST_UART_PERIOD_MS;
      if ((int32_t)(now - next_motor_uart_tick) >= 0)
      {
        next_motor_uart_tick = now + HARDWARE_TEST_UART_PERIOD_MS;
      }
    }

    /* The one-shot open-loop test still keeps the independent watchdog alive. */
    (void)HAL_IWDG_Refresh(&hiwdg);
    __WFI();
#elif (HARDWARE_TEST_MODE == HARDWARE_TEST_MODE_MOTOR_CLOSED_LOOP)
    const uint32_t now = HAL_GetTick();
    const uint32_t closed_loop_elapsed_ms = now - closed_loop_test_start_tick;
    uint8_t closed_loop_state;

    if (closed_loop_elapsed_ms < MOTOR_CLOSED_LOOP_TEST_ARM_MS)
    {
      closed_loop_state = 0U;
    }
    else if (closed_loop_elapsed_ms <
             (MOTOR_CLOSED_LOOP_TEST_ARM_MS + MOTOR_CLOSED_LOOP_TEST_RUN_MS))
    {
      closed_loop_state = 1U;
      if (closed_loop_test_started == 0U)
      {
        app_motor_pid_reset();
        app_motor_set_wheel_velocity_targets(MOTOR_CLOSED_LOOP_TEST_TARGET_M_S,
                                             MOTOR_CLOSED_LOOP_TEST_TARGET_M_S);
        closed_loop_test_started = 1U;
      }
    }
    else
    {
      closed_loop_state = 2U;
      if (closed_loop_test_stopped == 0U)
      {
        app_motor_stop();
        closed_loop_test_stopped = 1U;
      }
    }

    if ((int32_t)(now - closed_loop_next_control_tick) >= 0)
    {
      const uint32_t control_elapsed_ms = now - closed_loop_last_control_tick;
      const float measurement_dt_sec = (control_elapsed_ms != 0U) ?
          ((float)control_elapsed_ms / 1000.0f) : BOARD_CONTROL_DT_MIN_SEC;
      float pid_dt_sec = measurement_dt_sec;
      closed_loop_last_control_tick = now;
      if (pid_dt_sec < BOARD_CONTROL_DT_MIN_SEC) pid_dt_sec = BOARD_CONTROL_DT_MIN_SEC;
      if (pid_dt_sec > BOARD_CONTROL_DT_MAX_SEC) pid_dt_sec = BOARD_CONTROL_DT_MAX_SEC;
      closed_loop_next_control_tick += BOARD_CONTROL_PERIOD_MS;
      if ((int32_t)(now - closed_loop_next_control_tick) >= 0)
      {
        closed_loop_next_control_tick = now + BOARD_CONTROL_PERIOD_MS;
      }
      encoder_bsp_velocity_sample(measurement_dt_sec);
      if (closed_loop_state == 1U)
      {
        app_motor_speed_control_update(pid_dt_sec);
      }
    }

    if ((int32_t)(now - closed_loop_next_uart_tick) >= 0)
    {
      closed_loop_uart_test_publish(closed_loop_elapsed_ms, closed_loop_state);
      closed_loop_next_uart_tick += HARDWARE_TEST_UART_PERIOD_MS;
      if ((int32_t)(now - closed_loop_next_uart_tick) >= 0)
      {
        closed_loop_next_uart_tick = now + HARDWARE_TEST_UART_PERIOD_MS;
      }
    }

    /* The one-shot closed-loop test still keeps the independent watchdog alive. */
    (void)HAL_IWDG_Refresh(&hiwdg);
    __WFI();
#else
    const uint32_t now = HAL_GetTick();
    const uint32_t rotation_test_elapsed_ms = now - rotation_test_start_tick;
    uint8_t rotation_test_state;

    if (rotation_test_elapsed_ms < ROTATION_TEST_ARM_MS)
    {
      rotation_test_state = 0U;
    }
    else if (rotation_test_elapsed_ms < (ROTATION_TEST_ARM_MS + ROTATION_TEST_RUN_MS))
    {
      rotation_test_state = 1U;
      if (rotation_test_started == 0U)
      {
        app_motor_pid_reset();
        app_odometry_reset(0.0f, 0.0f, 0.0f);
        rotation_previous_left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
        rotation_previous_right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
        rotation_encoder_yaw_rad = 0.0f;
        rotation_gyro_yaw_rad = 0.0f;
        app_motor_set_chassis_velocity(0.0f, ROTATION_TEST_ANGULAR_RAD_S);
        rotation_test_started = 1U;
      }
    }
    else
    {
      rotation_test_state = 2U;
      if (rotation_test_stopped == 0U)
      {
        app_motor_stop();
        rotation_test_stopped = 1U;
      }
    }

    if ((int32_t)(now - rotation_next_control_tick) >= 0)
    {
      const uint32_t control_elapsed_ms = now - rotation_last_control_tick;
      const float measurement_dt_sec = (control_elapsed_ms != 0U) ?
          ((float)control_elapsed_ms / 1000.0f) : BOARD_CONTROL_DT_MIN_SEC;
      float pid_dt_sec = measurement_dt_sec;
      rotation_last_control_tick = now;
      if (pid_dt_sec < BOARD_CONTROL_DT_MIN_SEC) pid_dt_sec = BOARD_CONTROL_DT_MIN_SEC;
      if (pid_dt_sec > BOARD_CONTROL_DT_MAX_SEC) pid_dt_sec = BOARD_CONTROL_DT_MAX_SEC;
      rotation_next_control_tick += BOARD_CONTROL_PERIOD_MS;
      if ((int32_t)(now - rotation_next_control_tick) >= 0)
      {
        rotation_next_control_tick = now + BOARD_CONTROL_PERIOD_MS;
      }

      encoder_bsp_velocity_sample(measurement_dt_sec);
      rotation_imu_valid = (rotation_test_imu_ready != 0U) ?
          imu_bsp_read(&rotation_imu_sample) : 0U;

      if (rotation_test_state == 1U)
      {
        const int32_t left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
        const int32_t right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
        const int32_t left_delta_count = left_count - rotation_previous_left_count;
        const int32_t right_delta_count = right_count - rotation_previous_right_count;
        rotation_previous_left_count = left_count;
        rotation_previous_right_count = right_count;
        rotation_encoder_yaw_rad +=
            ((float)right_delta_count * BOARD_METERS_PER_COUNT_RIGHT -
             (float)left_delta_count * BOARD_METERS_PER_COUNT_LEFT) /
            BOARD_WHEEL_SEPARATION_M;
        if (rotation_imu_valid != 0U)
        {
          rotation_gyro_yaw_rad += rotation_imu_sample.gyro_rad_s[2] * measurement_dt_sec;
        }
        app_odometry_update(rotation_imu_sample.gyro_rad_s[2],
                            rotation_imu_valid, measurement_dt_sec);
        app_motor_speed_control_update(pid_dt_sec);
      }
      else
      {
        rotation_previous_left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
        rotation_previous_right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
      }
    }

    if ((int32_t)(now - rotation_next_uart_tick) >= 0)
    {
      rotation_uart_test_publish(rotation_test_elapsed_ms, rotation_test_state,
                                 rotation_encoder_yaw_rad, rotation_gyro_yaw_rad,
                                 &rotation_imu_sample, rotation_imu_valid);
      rotation_next_uart_tick += HARDWARE_TEST_UART_PERIOD_MS;
      if ((int32_t)(now - rotation_next_uart_tick) >= 0)
      {
        rotation_next_uart_tick = now + HARDWARE_TEST_UART_PERIOD_MS;
      }
    }

    /* The one-shot rotation test still keeps the independent watchdog alive. */
    (void)HAL_IWDG_Refresh(&hiwdg);
    __WFI();
#endif
  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
