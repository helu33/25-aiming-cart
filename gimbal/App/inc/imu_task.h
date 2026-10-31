#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "cmsis_os.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

/* IMU 姿态、陀螺仪数据由 imu_task.c 中更新，云台控制直接读取 */
extern float imuAngle[3];
extern float gyro[3];

typedef struct {
    float angle[3];
    float gyro[3];
    uint32_t updated_ms;
    uint8_t ready;
} imu_snapshot_t;

/* Coherent sample; ready is set only after calibration and a finite AHRS update. */
void IMU_GetSnapshot(imu_snapshot_t *snapshot);

#endif
