#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "cmsis_os.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_PITCH,
    GIMBAL_AXIS_COUNT
} gimbal_axis_t;

#ifndef GIMBAL_YAW_CAN_HANDLE
#define GIMBAL_YAW_CAN_HANDLE       (&hfdcan1)   /* 水平：CAN1 */
#endif

#ifndef GIMBAL_CAN_TX_ENABLE
#define GIMBAL_CAN_TX_ENABLE        1   /* 1: 允许任务发送CAN */
#endif

#ifndef GIMBAL_YAW_CAN_ID
#define GIMBAL_YAW_CAN_ID           (0x601U)      /* yaw（水平）：CAN1，驱动器 ID 1 */
#endif

#ifndef GIMBAL_YAW_CAN_RX_ID
#define GIMBAL_YAW_CAN_RX_ID        (0x580U + 0x1U) /* 对应 0x601 */
#endif

#ifndef GIMBAL_PITCH_CAN_HANDLE
#define GIMBAL_PITCH_CAN_HANDLE     (&hfdcan2)   /* 竖直：CAN2 */
#endif

#ifndef GIMBAL_PITCH_CAN_ID
#define GIMBAL_PITCH_CAN_ID         (0x601U)      /* pitch（竖直）：CAN2，驱动器 ID 1 */
#endif

#ifndef GIMBAL_PITCH_CAN_RX_ID
#define GIMBAL_PITCH_CAN_RX_ID      (0x580U + 0x1U)
#endif

#ifndef GIMBAL_SAFE_MAX_SPEED_RPM
#define GIMBAL_SAFE_MAX_SPEED_RPM       (50.0f) /* 安全最大角速度限制，单位 rpm */
#endif

/* 发热排查：1 = pitch 照常使能，但始终发 0 rpm（只靠驱动器自身保持），用来区分
 * 发热来自我们的控制指令还是驱动器/电机本身。正常使用必须为 0。 */
#ifndef GIMBAL_PITCH_ZERO_CMD_TEST
#define GIMBAL_PITCH_ZERO_CMD_TEST      0
#endif

#ifndef GIMBAL_CONTROL_PERIOD_MS
#define GIMBAL_CONTROL_PERIOD_MS    (2U)
#endif

/* USART10 (PE2 RX / PE3 TX, 921600): vision. USART1 (PA10 RX, the 3-pin "USART1" header,
 * 921600): chassis - that is where the chassis wire is plugged (2026-10-21; it was set to
 * UART7 before, so the chassis link was never received). UART7 (4-pin header with VCC):
 * Bluetooth telemetry, see telemetry.h. Sensor/control equations remain local. */
#ifndef GIMBAL_VISION_UART
#define GIMBAL_VISION_UART          (&huart10)
#endif
#ifndef GIMBAL_CHASSIS_UART
#define GIMBAL_CHASSIS_UART         (&huart1)
#endif
#define GIMBAL_VISION_TIMEOUT_MS    300U
#define GIMBAL_CHASSIS_TIMEOUT_MS   300U
#define GIMBAL_FEEDBACK_TIMEOUT_MS  100U
#define GIMBAL_IMU_TIMEOUT_MS       20U
/* Yaw has a slip ring, so continuous rotation is safe mechanically. The +/-60 deg limit
 * guarded the first bring-up against an inverted vision sign; the sign was confirmed on
 * the bench (2026-10-20), and a lap turns the chassis 360 deg under a fixed target, so it
 * is off. Set to 1 again when testing a changed VISION_YAW_DIR. */
#ifndef GIMBAL_YAW_LIMIT_ENABLE
#define GIMBAL_YAW_LIMIT_ENABLE     0
#endif
#ifndef GIMBAL_YAW_LIMIT_MIN_DEG
#define GIMBAL_YAW_LIMIT_MIN_DEG    (-60.0f)
#endif
#ifndef GIMBAL_YAW_LIMIT_MAX_DEG
#define GIMBAL_YAW_LIMIT_MAX_DEG    (60.0f)
#endif
/* 0 preserves standalone vision operation until the first chassis frame. */
#ifndef GIMBAL_REQUIRE_CHASSIS
#define GIMBAL_REQUIRE_CHASSIS      0
#endif

typedef struct {
    float yaw_deg, pitch_deg, roll_deg;
    float gx_deg_s, gy_deg_s, gz_deg_s;
    uint32_t received_ms;
    uint8_t online, imu_valid, turning;
    uint8_t has_speed;      /* $C version 2 */
    float speed_m_s;        /* forward speed from the chassis encoders */
} gimbal_chassis_state_t;

typedef struct {
    uint32_t invalid_frames, dropped_lines, uart_errors, can_tx_errors;
    uint8_t control_ready, vision_online, chassis_online, vision_suspended;
    uint8_t waiting_for_vision;
} gimbal_status_t;

/* Task-context diagnostic getters. Chassis IMU is received but not fused yet. */
void Gimbal_GetChassisState(gimbal_chassis_state_t *state);
void Gimbal_GetStatus(gimbal_status_t *status);
/* Live tuning by name (Bluetooth "S <name> <value>", see telemetry_link.h): the tune_*
 * variables of gimbal_task.c. 0 = set, -1 = unknown name, -2 = value out of range. */
int Gimbal_SetTune(const char *name, float value);
/* "latency=80 alpha=0.50 ..." into out (always terminated); returns the length. */
int Gimbal_FormatTunes(char *out, unsigned capacity);
/* Vehicle-speed feed-forward: the current yaw motor angle becomes "laser along the chassis
 * forward axis" (Bluetooth "F"). Point the laser straight ahead first. */
void Gimbal_CaptureForward(void);

void GimbalTask_Entry(void const *argument);
float Gimbal_GetCurrentAngle(gimbal_axis_t axis);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_TASK_H__ */
