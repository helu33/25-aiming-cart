#include "imu_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "gyro_bias.h"

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

#define IMU_TASK_PERIOD_TICKS  (1U)   //1是1kHz 2是500Hz
//陀螺仪零偏校准：只接受静止窗口（见 gyro_bias.h），超时后放宽
#define GYRO_BIAS_TIMEOUT_MS (10000U)
//上电先等加热稳定再采零偏，最长等这么久（加热功率小时靠"温度不再变化"判定）
#define IMU_WARMUP_TIMEOUT_MS (20000U)
//每次采样之间的延迟，单位为毫秒，确保足够的时间让陀螺仪数据稳定
#define GYRO_BIAS_SAMPLE_DELAY_MS (2U)
//陀螺仪偏置校准函数

float gyro[3] = {0.0f};
//加速度计数据
float acc[3] = {0.0f};
//温度数据？
static float temp = 0.0f;
static float gyro_bias[3] = {0.0f};
/* Keil Watch: start-up bias result. restarts > 0 means the board was moved during it. */
volatile uint32_t dbg_imu_bias_restarts;
volatile uint8_t dbg_imu_bias_relaxed;
volatile float dbg_imu_bias_z_deg_s;
volatile float dbg_imu_temp_c;          /* BMI088 temperature, heater target DES_TEMP */
volatile uint32_t dbg_imu_warmup_ms;    /* time spent waiting for the temperature to settle */
static imu_snapshot_t imu_published;

void IMU_GetSnapshot(imu_snapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    taskENTER_CRITICAL();
    *snapshot = imu_published;
    taskEXIT_CRITICAL();
}

float imuQuat[4] = {0.0f};//四元数数据
float imuAngle[3] = {0.0f};//欧拉角数据，顺序为yaw、pitch、roll

float out = 0;//PID输出
float err = 0;//当前误差
float err_l = 0;//上一次误差
float err_ll = 0;//上上次误差

static void imu_heater_step(float temp_c)//BMI088 恒温加热 PID（TIM3 CH4）
{
    err_ll = err_l;
    err_l = err;
    err = DES_TEMP - temp_c;
    out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
    if (out > MAX_OUT) out = MAX_OUT;
    if (out < 0) out = 0.f;
    htim3.Instance->CCR4 = (uint16_t)out;
}

void AHRS_init(float quat[4])//四元数初始化，初始姿态为水平向前
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])//使用Mahony滤波算法更新四元数，输入为陀螺仪和加速度计数据
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)//从四元数计算欧拉角，输出为yaw、pitch、roll，单位为弧度。
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
void ImuTask_Entry(void const * argument)//IMU任务入口函数，负责读取BMI088传感器数据，更新姿态估计，并进行PID控制以调整输出。
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10);//等待系统稳定
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);//启动定时器3的4通道。
    while(BMI088_init())//等待bm1088初始化完成。
    {
        osDelay(100);
    }
    
    AHRS_init(imuQuat);
    {
        /* Let the heater bring the chip to temperature first; see gyro_temp_settled. */
        gyro_temp_settle_t settle = {0};
        uint32_t warm_start = HAL_GetTick();
        for (;;)
        {
            BMI088_read(gyro, acc, &temp);
            imu_heater_step(temp);
            dbg_imu_temp_c = temp;
            if (gyro_temp_settled(&settle, temp, DES_TEMP, HAL_GetTick()) ||
                HAL_GetTick() - warm_start > IMU_WARMUP_TIMEOUT_MS) break;
            osDelay(GYRO_BIAS_SAMPLE_DELAY_MS);
        }
        dbg_imu_warmup_ms = HAL_GetTick() - warm_start;
    }
    {
        /* Average only a window in which the board was still; touching it at power-on
         * restarts the window. After GYRO_BIAS_TIMEOUT_MS accept any window so the
         * gimbal still starts on a vibrating base. */
        gyro_bias_t bias;
        gyro_bias_reset(&bias);
        uint32_t start_ms = HAL_GetTick();
        for (;;)
        {
            BMI088_read(gyro, acc, &temp);
            imu_heater_step(temp);
            if (gyro_bias_feed(&bias, gyro)) break;
            if (!bias.relaxed && HAL_GetTick() - start_ms > GYRO_BIAS_TIMEOUT_MS) bias.relaxed = 1U;
            dbg_imu_bias_restarts = bias.restarts;
            osDelay(GYRO_BIAS_SAMPLE_DELAY_MS);
        }
        for (unsigned axis = 0; axis < 3; ++axis) gyro_bias[axis] = bias.bias[axis];
        dbg_imu_bias_restarts = bias.restarts;
        dbg_imu_bias_relaxed = bias.relaxed;
        dbg_imu_bias_z_deg_s = gyro_bias[2] * (180.0f / 3.14159265f);
    }
    TickType_t last_wake = xTaskGetTickCount();
    //记录上次唤醒时间，确保任务以固定周期执行。。
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &temp);
        gyro[0] -= gyro_bias[0];
        gyro[1] -= gyro_bias[1];
        gyro[2] -= gyro_bias[2];
        
        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);
        taskENTER_CRITICAL();
        imu_published.ready = 1;
        for (unsigned axis = 0; axis < 3; ++axis) {
            imu_published.angle[axis] = imuAngle[axis];
            imu_published.gyro[axis] = gyro[axis];
            if (!isfinite(imuAngle[axis]) || !isfinite(gyro[axis])) imu_published.ready = 0;
        }
        imu_published.updated_ms = HAL_GetTick();
        taskEXIT_CRITICAL();
        
        imu_heater_step(temp);
        dbg_imu_temp_c = temp;

        vTaskDelayUntil(&last_wake, IMU_TASK_PERIOD_TICKS);
    }
    /* USER CODE END ImuTask_Entry */
}
