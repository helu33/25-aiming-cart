/**
 * 云台控制任务：速度控制逻辑，核心是 yaw 的动态基准校准、前馈和多段积分控制；
 * 后来加入了视觉跟踪、车速前馈和底盘协议。
 * 最初版本由队友 kian 编写（https://github.com/GGbond-dot/TIcar），本项目在此基础上修改。
 */

#include "gimbal_task.h"
#include "gimbal_tune.h"
#include "gimbal_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can_bsp.h"
#include "main.h"
#include "bsp_buzzer.h"
#include "imu_task.h"
#include "track_filter.h"
#include "rate_integral.h"
#include "telemetry_link.h"
#include "usart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

static imu_snapshot_t g_imu;

/* ===================== 底盘转向通知配置 ===================== */
/* 底盘急转弯时通过串口发送命令，云台开启IMU增稳 */
// 转向命令将在 commUARTCallback 函数中检测

/* ===================== PITCH轴角度限位 ===================== */
/* 相对上电（控制初始化）时的电机角度，往下为正，往上为负。电机角度是驱动器的多圈绝对值，
 * 只复位/重新调试主控时不会归零（实测上电读到 -162.7 度），所以不能直接和绝对角度比较。 */
#define PITCH_LIMIT_DOWN            80.0f    // 最大往下80度
#define PITCH_LIMIT_UP              (-80.0f) // 最大往上80度（负值）
/* PITCH 方向修正：-1.0f 代表反向 */
#define PITCH_DIR                   (1.0f)

/* ===================== YAW 轴角度限位 (新增) ===================== */
/* 以底盘正前方为0度，左正右负（或者反过来，看你电机定义） */
/* 建议先给小一点，比如左右各 60 度，测试没问题再放开到 90 或 120 */
#define YAW_LIMIT_MAX               GIMBAL_YAW_LIMIT_MAX_DEG
#define YAW_LIMIT_MIN               GIMBAL_YAW_LIMIT_MIN_DEG

/* ===================== 视觉闭环参数 ===================== */
/* K230 发来的是"拍照那一刻"靶相对激光的角度误差（度）。 */
/* 方向控制：如果跟踪方向反了，改为 -1.0f */
#define VISION_YAW_DIR          (-1.0f)  // Yaw 方向：1.0f 或 -1.0f
#define VISION_PITCH_DIR        (1.0f)   // Pitch 方向：1.0f 或 -1.0f

#define YAW_GZ_LEAD_MAX_DEG_S 80.0f        /* |lead x dgz/dt| bound */
static uint32_t g_turning_seen_ms;         /* last control cycle with the chassis turning */
static bool g_turning_seen;
static float g_chassis_gz_rate;            /* deg/s^2 from consecutive $C frames, low-passed */
#define YAW_RATE_I_ZONE_DEG 3.0f
static float g_yaw_rate_integral;          /* deg/s, added inside the inner-loop bracket */
static int g_yaw_out_saturated;            /* +1 / -1: last yaw command hit out_limit that way */
volatile float dbg_yaw_rate_integral;
volatile float dbg_yaw_gz_lead;            /* deg/s added to the chassis gz feed-forward */
volatile uint8_t dbg_chassis_link_lost;   /* 1 = chassis seen, then silent (buzzer on) */
volatile float dbg_chassis_gz_deg_s, dbg_yaw_motor_rate_deg_s;
#define VEH_SPEED_LEAD_MAX_M_S 0.2f
static float g_chassis_accel;              /* m/s^2 from consecutive $C speeds, low-passed */
static float g_veh_speed_used;             /* m/s, speed in the feed-forward incl. the lead */
volatile float dbg_veh_rate_deg_s, dbg_vision_range_m, dbg_chassis_speed_m_s, dbg_veh_phi_deg;
volatile float dbg_veh_pitch_rate_deg_s;
#define VEH_RANGE_MAX_AGE_MS    1000U   /* a vision range older than this is not used */
#define VEH_RANGE_MIN_M         0.15f
#define ANGLE_HISTORY_LEN       128U    /* one sample per 2 ms control cycle -> 256 ms */

typedef struct
{
	FDCAN_HandleTypeDef *hcan;
	uint16_t tx_id;
	uint16_t rx_id;
} gimbal_axis_config_t;

typedef struct
{
	gimbal_axis_config_t cfg;
	float current_angle_deg;
	uint8_t initialized;
	uint8_t feedback_valid;
	uint32_t last_feedback_tick;
} gimbal_axis_runtime_t;

typedef struct {
    float angle;
    uint32_t received_ms;
    uint8_t valid;
} motor_feedback_t;
static volatile motor_feedback_t g_feedback[GIMBAL_AXIS_COUNT];

static gimbal_axis_runtime_t g_axis_table[GIMBAL_AXIS_COUNT] =
{
	[ GIMBAL_AXIS_YAW ] =
	{
		.cfg =
		{
			.hcan = GIMBAL_YAW_CAN_HANDLE,
			.tx_id = GIMBAL_YAW_CAN_ID,
			.rx_id = GIMBAL_YAW_CAN_RX_ID,
		},
		.current_angle_deg = 0.0f,
	},
	[ GIMBAL_AXIS_PITCH ] =
	{
		.cfg =
		{
			.hcan = GIMBAL_PITCH_CAN_HANDLE,
			.tx_id = GIMBAL_PITCH_CAN_ID,
			.rx_id = GIMBAL_PITCH_CAN_RX_ID,
		},
		.current_angle_deg = 0.0f,
	},
};

/* ===================== 全程融合控制变量 ===================== */
static float g_world_target_yaw = 0.0f;     // 世界坐标系下的目标 Yaw
/* Vision pitch target as a motor angle plus the IMU pitch when it was set (base-tilt
 * compensation). The IMU rides on the yaw stage and does not see pitch motion, so the old
 * "IMU pitch + error" target only moved pitch a fraction of the way (field: stuck 3.5 deg
 * below the bullseye). Target = current motor angle + error closes the loop like yaw. */
static float g_vision_pitch_motor = 0.0f, g_vision_pitch_imu = 0.0f;
static bool g_target_initialized = false;   // 初始化标志
static float g_pitch_base_imu = 0.0f;       // 记录IMU基准pitch
static float g_pitch_base_motor = 0.0f;     // 记录电机基准pitch
static float g_yaw_limit_origin;
static bool g_yaw_limit_origin_valid;
static bool g_pitch_base_valid;
static float g_hold_pitch_imu, g_hold_pitch_motor;

/* ===================== 方案D：静止时同步基准（消除零漂） ===================== */
#define YAW_DRIFT_GYRO_THRESHOLD    0.5f    // 角速度阈值（deg/s），小于此值认为静止
#define YAW_DRIFT_STABLE_TIME_MS    2000    // 静止持续时间（ms），超过此时间才同步
#define YAW_DRIFT_SYNC_INTERVAL_MS  5000    // 同步间隔（ms），防止频繁同步
static uint32_t g_yaw_stable_start_tick = 0;    // 开始静止的时刻
static uint32_t g_yaw_last_sync_tick = 0;       // 上次同步的时刻
static bool g_yaw_is_stable = false;            // 当前是否处于静止状态

float dbg_target_pitch = 0.0f;
float dbg_pitch_error = 0.0f;
float dbg_pitch_speed_rpm = 0.0f;
float dbg_imu_pitch_deg = 0.0f;
float dbg_motor_pitch_deg = 0.0f;
float dbg_yaw_speed_rpm = 0.0f;
float dbg_yaw_error = 0.0f;
float dbg_yaw_ff_rpm = 0.0f;
float dbg_yaw_relative_deg = 0.0f;
/* Bring-up watch (Keil Watch window): link and control state without static symbols. */
volatile uint8_t dbg_control_ready, dbg_vision_online, dbg_vision_valid, dbg_vision_suspended;
volatile uint32_t dbg_vision_frames;       /* accepted vision frames since power-on */
/* Chassis link (USART1) bring-up: raw bytes seen, frames accepted, UART errors. Bytes 0 ->
 * nothing arrives (wire, PA10, ground, chassis not sending); bytes but no frames -> baud or
 * garbled; frames rising -> link OK. */
volatile uint32_t dbg_chassis_rx_bytes, dbg_chassis_frames, dbg_chassis_uart_errors;
volatile float dbg_vision_yaw_err, dbg_vision_pitch_err;   /* last error from the K230, deg */
/* $V version 3: why the last frame was not valid (0 = valid, vision/protocol.py CODE_*) and
 * the tape rectangle's area in % of the image; -1 before any version-3 frame. Telemetry 11/12. */
volatile float dbg_vision_code = -1.0f, dbg_vision_area_pct;
/* Extremes of the error of valid frames since last cleared: write 0 to all four in Keil
 * Watch before a test run, read after it (deg; 1 deg is 1.4 cm at 80 cm). Max and min are
 * kept apart: lagging behind a moving target and overshooting it when it stops have
 * opposite signs, and a single |peak| cannot tell them apart. */
volatile float dbg_vision_yaw_err_max, dbg_vision_yaw_err_min;
volatile float dbg_vision_pitch_err_max, dbg_vision_pitch_err_min;
/* Pitch heat diagnosis, statistics of the command actually sent over the last full second. */
volatile uint32_t dbg_pitch_flips_per_s;      /* sign reversals of the sent command */
volatile float dbg_pitch_abs_max_rpm, dbg_pitch_abs_mean_rpm;
volatile float dbg_pitch_ff_rpm;              /* gyro feed-forward part of the command */

typedef struct
{
	float kp;
	float ki;
	float kd;
	float target;
	float integral;
	float prev;
	float out_limit;
	float integral_limit;
} pid_ctrl_t;

static void gimbal_axis_enable(gimbal_axis_runtime_t *axis);
static void gimbal_request_angle(gimbal_axis_runtime_t *axis);
static bool gimbal_send_speed(gimbal_axis_runtime_t *axis, int32_t rpm_value);
static bool gimbal_send_frame(gimbal_axis_runtime_t *axis, const uint8_t *frame);
static float pid_calc(pid_ctrl_t *pid, float input);
static float yaw_deg_from_imu(void);
static void vision_target_update(void);
static void vehicle_rate_update(float motor_yaw_deg);
static void gimbal_step_imu_execute(void);
static void gimbal_poll_inputs(void);
static void gimbal_stop(void);
static void gimbal_reset_pid(void);
static void gimbal_control_cycle(void);
static void telemetry_sample(void);
static int32_t rpm_to_raw(float rpm);
static void pitch_command_stats(int32_t raw);
static bool axis_stalled(unsigned axis, float rpm, float angle_deg, uint32_t now);
static float wrap_deg(float deg);
static float wrap_pm180_deg(float deg);
static float pitch_motor_unwrapped(void);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static pid_ctrl_t pid_yaw_speed = {
    .kp = 2.5f, .ki = 0.0f, .kd = 0.0f, .target = 0.0f, .integral = 0.0f, .prev = 0.0f,
    .out_limit = GIMBAL_SAFE_MAX_SPEED_RPM, .integral_limit = NAN};

static pid_ctrl_t pid_pitch_speed = {
    .kp = 2.5f, .ki = 0.0f, .kd = 0.0f, .target = 0.0f, .integral = 0.0f, .prev = 0.0f,
    .out_limit = GIMBAL_SAFE_MAX_SPEED_RPM, .integral_limit = NAN};

static float g_yaw_kp = 2.8f;
static float g_yaw_ki = 0.05f;
static float g_yaw_kd = 0.10f;
static float g_yaw_integral_limit = 600.0f;
static float g_yaw_ff_gain = 7.0f;
static float g_yaw_ff_sign = -1.0f;
static float g_yaw_deg_s_to_rpm = (1.0f / 6.0f);
static float g_yaw_integral_decay = 0.90f;
static float g_yaw_integral_decay_threshold = 1.0f;

static float g_pitch_ki = 0.02f;
static float g_pitch_kd = 0.10f;
static float g_pitch_integral_limit = 300.0f;
static float g_pitch_ff_gain = 1.5f;
static float g_pitch_ff_sign = -1.0f;
static float g_pitch_deg_s_to_rpm = (1.0f / 6.0f);
/* Hold error below this (IMU/encoder noise) is treated as zero, so the speed loop does not
 * chase noise back and forth while standing still. (The field pitch over-heating itself was
 * the JC4010 driver config: pole pairs 7 instead of the factory 14; fixed by restoring defaults
 * and recalibrating.) */
static float g_pitch_hold_deadzone_deg = 0.10f;
static float g_pitch_integral_decay = 0.90f;
static float g_pitch_integral_decay_threshold = 0.50f;

/* UART ISRs only assemble/enqueue complete lines. Parsing and all control state
 * updates run in GimbalTask, so a yaw/pitch pair cannot be torn by an ISR. */
#define RX_QUEUE_DEPTH 8U
typedef struct {
    char text[GIMBAL_LINE_CAPACITY];
    uint32_t received_ms;
} rx_line_t;
typedef struct {
    UART_HandleTypeDef *uart;
    /* A whole line per receive-to-idle event. With 64 bytes every $C v3 frame (65-70 bytes, the
     * longest in turns) filled the buffer mid-line; with the UART FIFO off, the next bytes overran
     * while the callback re-armed, the frame failed its CRC, and > 300 ms of lost frames in a turn
     * sounded the chassis-link buzzer (2026-10-31). */
    uint8_t rx[GIMBAL_LINE_CAPACITY];
    gimbal_line_t assembling;
    rx_line_t queue[RX_QUEUE_DEPTH];
    volatile uint8_t head, tail;
    volatile uint32_t dropped, errors;
    uint32_t last_byte_ms;
    gimbal_sequence_t vision_seq, chassis_seq;
} gimbal_uart_t;
static gimbal_uart_t g_uarts[2] = {{.uart = GIMBAL_VISION_UART}, {.uart = GIMBAL_CHASSIS_UART}};
static float vision_yaw_deg, vision_pitch_deg;
static bool vision_has_target, vision_data_new, vision_seen;
static bool vision_warmup_done;
static uint32_t vision_received_ms, vision_resume_after_ms;
static bool vision_suspended;
static bool waiting_for_vision;
static bool chassis_seen;
static gimbal_chassis_state_t g_chassis;
static gimbal_status_t g_status;
static gimbal_chassis_state_t g_chassis_published;
static gimbal_status_t g_status_published;
static bool g_can_tx_fault;
static uint32_t g_motor_init_retry_ms;

void GimbalTask_Entry(void const *argument)
{
    UNUSED(argument);
    can_bsp_init();
    BSP_Buzzer_Init();
    for (unsigned i = 0; i < 2; ++i)
        HAL_UARTEx_ReceiveToIdle_IT(g_uarts[i].uart, g_uarts[i].rx, sizeof(g_uarts[i].rx));
    Telemetry_Init();
    osDelay(500);
    for (unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) gimbal_axis_enable(&g_axis_table[i]);
    osDelay(2000);

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t loop_tick = 0;
    for (;;) {
        IMU_GetSnapshot(&g_imu);
        taskENTER_CRITICAL();
        for (unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
            g_axis_table[i].current_angle_deg = g_feedback[i].angle;
            g_axis_table[i].feedback_valid = g_feedback[i].valid;
            g_axis_table[i].last_feedback_tick = g_feedback[i].received_ms;
        }
        taskEXIT_CRITICAL();
        gimbal_poll_inputs();
        if (HAL_GetTick() - g_motor_init_retry_ms >= 100U) {
            g_motor_init_retry_ms = HAL_GetTick();
            for (unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i)
                if (!g_axis_table[i].initialized) gimbal_axis_enable(&g_axis_table[i]);
        }
        if (loop_tick % GIMBAL_CONTROL_PERIOD_MS == 0U) {
            gimbal_control_cycle();
        }
        if (loop_tick % (1000U / TELEM_RATE_HZ) == 0U) telemetry_sample();
        taskENTER_CRITICAL();
        g_chassis_published = g_chassis;
        g_status_published = g_status;
        taskEXIT_CRITICAL();
        ++loop_tick;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

/* ===================== 就绪回报（$G -> 底盘，上电自启动） ===================== */
/* 每 GIMBAL_READY_PERIOD_MS 在底盘串口 TX 上发一帧 $G（gimbal_protocol.h）：CONTROL = IMU 预热、
 * 零偏完成且两个电机都有反馈（g_status.control_ready）；VISION = 视觉帧有效、|误差| <
 * GIMBAL_READY_ERR_DEG 已连续 GIMBAL_READY_LOCK_MS（误差变大或断帧超过 GIMBAL_READY_GAP_MS 就重新计时）；
 * READY = 两者都有。底盘收到 READY、自己也就绪后自动跑圈（底盘 CHASSIS_AUTOSTART_LAPS）。 */
#define GIMBAL_READY_PERIOD_MS 100U
#define GIMBAL_READY_LOCK_MS   2000U
#define GIMBAL_READY_ERR_DEG   1.0f
#define GIMBAL_READY_GAP_MS    300U
static uint32_t g_ready_tx_ms, g_lock_since_ms, g_lock_seen_ms;
static bool g_ready_tx_started, g_locked;
static uint16_t g_ready_seq;
static char g_ready_tx[24];                 /* owned by the UART until the frame is sent */
volatile uint8_t dbg_ready_flags;           /* last $G flags (GIMBAL_STATUS_*) */

static uint8_t gimbal_ready_flags(uint32_t now)
{
    const bool good = vision_has_target &&
        (uint32_t)(now - vision_received_ms) <= GIMBAL_READY_GAP_MS &&
        fabsf(dbg_vision_yaw_err) < GIMBAL_READY_ERR_DEG &&
        fabsf(dbg_vision_pitch_err) < GIMBAL_READY_ERR_DEG;
    uint8_t flags = 0U;
    if (!good) {
        g_locked = false;
    } else {                                  /* (re)start the clock after a gap in the checks too */
        if (!g_locked || (uint32_t)(now - g_lock_seen_ms) > GIMBAL_READY_GAP_MS) g_lock_since_ms = now;
        g_locked = true;
        g_lock_seen_ms = now;
    }
    if (g_status.control_ready) flags |= GIMBAL_STATUS_CONTROL;
    if (g_locked && (uint32_t)(now - g_lock_since_ms) >= GIMBAL_READY_LOCK_MS) flags |= GIMBAL_STATUS_VISION;
    if (flags == (GIMBAL_STATUS_CONTROL | GIMBAL_STATUS_VISION)) flags |= GIMBAL_STATUS_READY;
    return flags;
}

static void gimbal_ready_report(uint32_t now)
{
    int n;
    if (g_ready_tx_started && (uint32_t)(now - g_ready_tx_ms) < GIMBAL_READY_PERIOD_MS) return;
    g_ready_tx_started = true;
    g_ready_tx_ms = now;
    dbg_ready_flags = gimbal_ready_flags(now);
    if (GIMBAL_CHASSIS_UART->gState != HAL_UART_STATE_READY) return;   /* last frame still going */
    n = GimbalProtocol_EncodeStatus(g_ready_tx, sizeof(g_ready_tx), g_ready_seq++, dbg_ready_flags);
    if (n > 0) (void)HAL_UART_Transmit_IT(GIMBAL_CHASSIS_UART, (uint8_t *)g_ready_tx, (uint16_t)n);
}

/* ===================== 画圆任务（$C v3 -> $K -> K230） ===================== */
/* 底盘在 $C v3 里报画圆模式和本次跑圈进度（1/1000 圈），云台每 GIMBAL_TASK_PERIOD_MS 原样转发给
 * K230（$K，视觉串口 TX）；K230 按进度把瞄准点沿靶上半径 60 mm 的圆移动，一圈画一圈。
 * 收到 v1/v2 帧时保持上次的值（上电前为 0、0 = 打靶心）。 */
#define GIMBAL_TASK_PERIOD_MS 50U
static uint8_t g_task;
static uint16_t g_task_progress;
static uint32_t g_task_tx_ms;
static bool g_task_tx_started;
static uint16_t g_task_seq;
static char g_task_tx[28];                  /* owned by the UART until the frame is sent */

static void gimbal_task_report(uint32_t now)
{
    int n;
    if (g_task_tx_started && (uint32_t)(now - g_task_tx_ms) < GIMBAL_TASK_PERIOD_MS) return;
    g_task_tx_started = true;
    g_task_tx_ms = now;
    if (GIMBAL_VISION_UART->gState != HAL_UART_STATE_READY) return;   /* last frame still going */
    n = GimbalProtocol_EncodeTask(g_task_tx, sizeof(g_task_tx), g_task_seq++, g_task, g_task_progress);
    if (n > 0) (void)HAL_UART_Transmit_IT(GIMBAL_VISION_UART, (uint8_t *)g_task_tx, (uint16_t)n);
}

static void gimbal_control_cycle(void)
{
    uint32_t now = HAL_GetTick();
    bool ready = GIMBAL_CAN_TX_ENABLE && g_imu.ready &&
                 (uint32_t)(now - g_imu.updated_ms) <= GIMBAL_IMU_TIMEOUT_MS;
    for (unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        gimbal_request_angle(&g_axis_table[i]);
        ready = ready && g_axis_table[i].initialized && g_axis_table[i].feedback_valid &&
            (uint32_t)(now - g_axis_table[i].last_feedback_tick) <= GIMBAL_FEEDBACK_TIMEOUT_MS;
    }
    g_status.control_ready = ready && !g_can_tx_fault;
    if (!g_status.control_ready) {
        g_target_initialized = false;
        vision_data_new = false;
        gimbal_stop();
    } else {
        if (g_target_initialized) vision_target_update();
        gimbal_step_imu_execute();
    }
    gimbal_ready_report(now);
    gimbal_task_report(now);
}


float Gimbal_GetCurrentAngle(gimbal_axis_t axis)
{
	if ((unsigned)axis >= GIMBAL_AXIS_COUNT)
	{
		return 0.0f;
	}
	return g_axis_table[axis].current_angle_deg;
}

static bool gimbal_send_frame(gimbal_axis_runtime_t *axis, const uint8_t *frame)
{
#if GIMBAL_CAN_TX_ENABLE
    if (fdcanx_send_data(axis->cfg.hcan, axis->cfg.tx_id, (uint8_t *)frame, 8U) == 0U)
        return true;
    ++g_status.can_tx_errors;
    g_can_tx_fault = true;
#else
    UNUSED(axis);
    UNUSED(frame);
#endif
    return false;
}

static void gimbal_axis_enable(gimbal_axis_runtime_t *axis)
{
    static const uint8_t enable_cmd[8] = {0x2B, 0x00, 0xA2, 0x00, 0x00, 0x01, 0x00, 0x00};
    static const uint8_t speed_mode_cmd[8] = {0x2B, 0x00, 0x60, 0x00, 0x00, 0x01, 0x00, 0x00};
    axis->initialized = 0;
    if (!gimbal_send_frame(axis, enable_cmd)) return;
    osDelay(2);
    if (!gimbal_send_frame(axis, speed_mode_cmd)) return;
    osDelay(2);
    if (!gimbal_send_speed(axis, 0)) return;
    osDelay(2);
    axis->initialized = 1; /* Local enqueue succeeded; angle feedback is required separately. */
}

static void gimbal_request_angle(gimbal_axis_runtime_t *axis)
{
    static const uint8_t frame[8] = {0x43, 0x00, 0x08, 0x00, 0, 0, 0, 0};
    (void)gimbal_send_frame(axis, frame);
}

static bool gimbal_send_speed(gimbal_axis_runtime_t *axis, int32_t rpm_value)
{
    uint32_t raw = (uint32_t)rpm_value;
    uint8_t frame[8] = {0x23, 0x00, 0x21, 0x00, 0, 0, 0, 0};
    frame[4] = (uint8_t)(raw >> 24);
    frame[5] = (uint8_t)(raw >> 16);
    frame[6] = (uint8_t)(raw >> 8);
    frame[7] = (uint8_t)raw;
    return gimbal_send_frame(axis, frame);
}

static void gimbal_reset_pid(void)
{
    pid_yaw_speed.integral = pid_yaw_speed.prev = 0.0f;
    pid_pitch_speed.integral = pid_pitch_speed.prev = 0.0f;
    g_yaw_rate_integral = 0.0f;
    g_yaw_out_saturated = 0;
    g_yaw_is_stable = false;
}

static void gimbal_stop(void)
{
    bool yaw_ok = gimbal_send_speed(&g_axis_table[GIMBAL_AXIS_YAW], 0);
    bool pitch_ok = gimbal_send_speed(&g_axis_table[GIMBAL_AXIS_PITCH], 0);
    gimbal_reset_pid();
    /* Retry zero commands each cycle until both are accepted by the controller. */
    if (yaw_ok && pitch_ok) g_can_tx_fault = false;
}


static float pid_calc(pid_ctrl_t *pid, float input)
{
	const float err = pid->target - input;
	pid->integral += err;
	if (!isnan(pid->integral_limit))
	{
		if (pid->integral > pid->integral_limit)
			pid->integral = pid->integral_limit;
		else if (pid->integral < -pid->integral_limit)
			pid->integral = -pid->integral_limit;
	}
	float out = pid->kp * err + pid->ki * pid->integral + pid->kd * (err - pid->prev);
	pid->prev = err;
	if (!isnan(pid->out_limit))
	{
		if (out > pid->out_limit)
			out = pid->out_limit;
		else if (out < -pid->out_limit)
			out = -pid->out_limit;
	}
	return out;
}

static float wrap_pm180_deg(float deg)
{
    float wrapped = wrap_deg(deg);
    return wrapped > 180.0f ? wrapped - 360.0f : wrapped;
}

/* Pitch motor angle on the same side of the +/-180 wrap as the start angle. */
static float pitch_motor_unwrapped(void)
{
    float motor = g_axis_table[GIMBAL_AXIS_PITCH].current_angle_deg;
    return g_pitch_base_motor + wrap_pm180_deg(motor - g_pitch_base_motor);
}

static float wrap_deg(float deg)
{
    if (!isfinite(deg)) return 0.0f;
    deg = fmodf(deg, 360.0f);
    return deg < 0.0f ? deg + 360.0f : deg;
}

static float yaw_deg_from_imu(void)
{
	float yaw_rad = g_imu.angle[INS_YAW_ADDRESS_OFFSET];
	float yaw_deg = yaw_rad * (180.0f / (float)M_PI);
	if (yaw_deg < 0.0f)
	{
		yaw_deg += 360.0f;
	}
	return wrap_deg(yaw_deg);
}

/* Stall / limp watchdog. A JC4010 whose temperature protection trips drops out of closed
 * loop ("可恢复" protection needs a new enable) while still answering angle requests, so the
 * axis goes limp unnoticed; a mechanical stall looks the same from here. If an axis is
 * commanded >= AXIS_STALL_MIN_RPM for AXIS_STALL_MS without moving AXIS_STALL_MOVE_DEG,
 * it is re-enabled (the 100 ms retry in the task loop resends enable + speed mode). */
#define AXIS_STALL_MIN_RPM   5.0f     /* 30 deg/s expected: 0.5 s should move ~15 deg */
#define AXIS_STALL_MOVE_DEG  1.0f
#define AXIS_STALL_MS        500U
volatile uint32_t dbg_axis_stall_count[GIMBAL_AXIS_COUNT];

static bool axis_stalled(unsigned axis, float rpm, float angle_deg, uint32_t now)
{
    static float start_angle[GIMBAL_AXIS_COUNT];
    static uint32_t start_ms[GIMBAL_AXIS_COUNT];
    static bool watching[GIMBAL_AXIS_COUNT];
    bool moved = watching[axis] &&
                 fabsf(wrap_pm180_deg(angle_deg - start_angle[axis])) >= AXIS_STALL_MOVE_DEG;
    if (fabsf(rpm) < AXIS_STALL_MIN_RPM || !watching[axis] || moved) {
        watching[axis] = fabsf(rpm) >= AXIS_STALL_MIN_RPM;
        start_angle[axis] = angle_deg;
        start_ms[axis] = now;
        return false;
    }
    if ((uint32_t)(now - start_ms[axis]) < AXIS_STALL_MS) return false;
    watching[axis] = false;
    ++dbg_axis_stall_count[axis];
    return true;
}

typedef struct {
    uint32_t ms;
    float yaw_imu, pitch_motor, pitch_imu;   /* deg; pitch_motor unwrapped */
    float veh_yaw, veh_pitch;                /* integrated vehicle-explained direction change */
} angle_sample_t;
static angle_sample_t g_angle_history[ANGLE_HISTORY_LEN];
static uint32_t g_angle_history_head, g_angle_history_count;
volatile uint32_t dbg_vision_lookback_ms;    /* how far back the last frame was applied */
static track_filter_t g_track_yaw = {.wrap360 = 1U}, g_track_pitch;
static uint32_t g_yaw_settle_until, g_pitch_settle_until;   /* capture times: no rate learning before */
static float g_veh_yaw_cum, g_veh_rate;              /* vehicle feed-forward input, deg / deg/s */
static float g_veh_pitch_cum, g_veh_pitch_rate;      /* the same for the elevation */
static float g_vision_range_m;                       /* last valid K230 range */
static uint32_t g_vision_range_ms;
static uint16_t g_vision_age_ms;
static bool g_vision_has_age;
volatile float dbg_track_yaw_rate, dbg_track_pitch_rate;   /* deg/s, feed-forward in use */
volatile uint32_t dbg_track_resets;                         /* yaw track (re)starts */

static void angle_history_push(const angle_sample_t *sample)
{
    g_angle_history[g_angle_history_head] = *sample;
    g_angle_history_head = (g_angle_history_head + 1U) % ANGLE_HISTORY_LEN;
    if (g_angle_history_count < ANGLE_HISTORY_LEN) ++g_angle_history_count;
}

/* Newest sample taken at or before ms; the oldest one if the history is shorter; the
 * current pose if there is no history yet (just initialised). */
static angle_sample_t angle_history_at(uint32_t ms, const angle_sample_t *current)
{
    angle_sample_t found = *current;
    for (uint32_t k = 1; k <= g_angle_history_count; ++k) {
        found = g_angle_history[(g_angle_history_head + ANGLE_HISTORY_LEN - k) % ANGLE_HISTORY_LEN];
        if ((int32_t)(found.ms - ms) <= 0) break;
    }
    return found;
}

static void pitch_command_stats(int32_t raw)
{
    static int32_t last_sign;
    static uint32_t window_start, flips, samples;
    static float abs_sum, abs_max;
    int32_t sign = (raw > 0) - (raw < 0);
    if (sign != 0 && last_sign != 0 && sign != last_sign) ++flips;
    if (sign != 0) last_sign = sign;
    float abs_rpm = fabsf((float)raw / 100.0f);
    abs_sum += abs_rpm;
    if (abs_rpm > abs_max) abs_max = abs_rpm;
    ++samples;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - window_start) >= 1000U) {
        dbg_pitch_flips_per_s = flips;
        dbg_pitch_abs_max_rpm = abs_max;
        dbg_pitch_abs_mean_rpm = abs_sum / (float)samples;
        window_start = now;
        flips = samples = 0U;
        abs_sum = abs_max = 0.0f;
    }
}

static int32_t rpm_to_raw(float rpm)
{
	return (int32_t)lroundf(rpm * 100.0f); // JC 协议: rpm*100
}


/* ===================== 视觉目标更新 ===================== */
/* 职责：根据视觉角度误差，计算世界坐标系目标角度 */
/* 注意：视觉端 opencv_v2.py 已经将像素转为角度，这里直接用 */
static void vision_target_update(void)
{
    if (vision_suspended || !vision_data_new) return;
    vision_data_new = false;
    if (!vision_has_target) return;   // 无目标帧：滤波器按预测继续（最多 TRACK_COAST_MS）

    // 拍照那一刻的角度（延迟补偿）；历史不够时用当前值
    uint32_t latency = (tune_vision_use_age && g_vision_has_age) ?
        (uint32_t)g_vision_age_ms + tune_vision_extra_ms : tune_vision_latency_ms;
    if (latency > VISION_LATENCY_MAX_MS) latency = VISION_LATENCY_MAX_MS;
    uint32_t capture_ms = vision_received_ms - latency;
    angle_sample_t current = {HAL_GetTick(), yaw_deg_from_imu(), pitch_motor_unwrapped(),
                              g_imu.angle[INS_PITCH_ADDRESS_OFFSET] * (180.0f / (float)M_PI),
                              g_veh_yaw_cum, g_veh_pitch_cum};
    angle_sample_t then = angle_history_at(capture_ms, &current);
    dbg_vision_lookback_ms = current.ms - then.ms;

    // 靶的方向：yaw 世界方位；pitch 对地俯仰（电机角 + IMU 俯仰，与底座补偿关系一致）
    float yaw_bearing = wrap_deg(then.yaw_imu + VISION_YAW_DIR * vision_yaw_deg);
    float pitch_elevation = then.pitch_motor + VISION_PITCH_DIR * vision_pitch_deg + then.pitch_imu;
    float alpha = tune_track_alpha, beta = tune_track_beta;
    g_track_yaw.rate_clip = g_track_pitch.rate_clip = tune_track_rate_clip_deg;
    if (track_update(&g_track_yaw, yaw_bearing, capture_ms, alpha, beta, then.veh_yaw) == TRACK_RESET) {
        ++dbg_track_resets;
        g_yaw_settle_until = capture_ms + tune_track_settle_ms;
    }
    if (track_update(&g_track_pitch, pitch_elevation, capture_ms, alpha, beta, then.veh_pitch) == TRACK_RESET)
        g_pitch_settle_until = capture_ms + tune_track_settle_ms;
    /* While the chassis turns (vision on, tune_turn_hold 0) learn the position only, until
     * tune_track_settle_ms after the turn: the body rotation makes the frames jittery. */
    if (g_chassis.online && g_chassis.turning)
        g_yaw_settle_until = g_pitch_settle_until = capture_ms + tune_track_settle_ms;
    /* Frames right after a (re)start come while the gimbal is still slewing onto the target:
     * learn the position only (the residual rate would be noise, see tune_track_settle_ms). */
    if ((int32_t)(capture_ms - g_yaw_settle_until) < 0) g_track_yaw.v = 0.0f;
    if ((int32_t)(capture_ms - g_pitch_settle_until) < 0) g_track_pitch.v = 0.0f;
}

/* Targets from the trackers each control cycle; returns the feed-forward rates (deg/s,
 * 0 when not tracking). A track idle for TRACK_LOST_MS is stopped: the targets then stay
 * where the last prediction left them (yaw world bearing, pitch elevation over the base). */
static void vision_track_apply(uint32_t now, float *yaw_rate, float *pitch_rate)
{
    *yaw_rate = *pitch_rate = 0.0f;
    dbg_track_yaw_rate = dbg_track_pitch_rate = 0.0f;   /* stays 0 while not tracking */
    if (g_track_yaw.active && (int32_t)(now - g_track_yaw.t_ms) > (int32_t)TRACK_LOST_MS) {
        track_stop(&g_track_yaw);
        track_stop(&g_track_pitch);
    }
    if (!g_track_yaw.active || vision_suspended || waiting_for_vision) return;
    g_world_target_yaw = track_predict(&g_track_yaw, now, g_veh_yaw_cum, yaw_rate);
    *yaw_rate += g_veh_rate;             // residual (learnt) + vehicle-explained rate
    g_vision_pitch_motor = track_predict(&g_track_pitch, now, g_veh_pitch_cum, pitch_rate);
    *pitch_rate += g_veh_pitch_rate;     // residual (learnt) + vehicle-explained rate
    g_vision_pitch_imu = 0.0f;           // pitch target = elevation - current IMU pitch
    dbg_track_yaw_rate = *yaw_rate;
    dbg_track_pitch_rate = *pitch_rate;
}

/* Vehicle-explained bearing rate (deg/s, gimbal yaw sense) and its integral. 0 unless
 * tune_veh_ff != 0, the chassis sends its speed and a recent vision range exists. */
static void vehicle_rate_update(float motor_yaw_deg)
{
    const uint32_t now = HAL_GetTick();
    float rate = 0.0f, pitch_rate = 0.0f;
    const bool range_ok = g_vision_range_m >= VEH_RANGE_MIN_M &&
                          (uint32_t)(now - g_vision_range_ms) <= VEH_RANGE_MAX_AGE_MS;
    dbg_veh_phi_deg = isnan(tune_yaw_fwd_deg) ? 0.0f : wrap_pm180_deg(motor_yaw_deg - tune_yaw_fwd_deg);
    if (g_chassis.online && g_chassis.has_speed && range_ok && !isnan(tune_yaw_fwd_deg)) {
        const float phi = dbg_veh_phi_deg * ((float)M_PI / 180.0f);
        float lead = tune_veh_speed_lead_s * g_chassis_accel;
        if (lead > VEH_SPEED_LEAD_MAX_M_S) lead = VEH_SPEED_LEAD_MAX_M_S;
        if (lead < -VEH_SPEED_LEAD_MAX_M_S) lead = -VEH_SPEED_LEAD_MAX_M_S;
        g_veh_speed_used = g_chassis.speed_m_s + lead;
        const float k = g_veh_speed_used / g_vision_range_m * (180.0f / (float)M_PI);   /* deg/s */
        rate = tune_veh_ff * k * sinf(phi);
        pitch_rate = tune_veh_pitch_h * k * cosf(phi) / g_vision_range_m;
        if (!isfinite(rate)) rate = 0.0f;
        if (!isfinite(pitch_rate)) pitch_rate = 0.0f;
    }
    g_veh_rate = rate;
    g_veh_pitch_rate = pitch_rate;
    g_veh_yaw_cum += rate * ((float)GIMBAL_CONTROL_PERIOD_MS * 0.001f);
    g_veh_pitch_cum += pitch_rate * ((float)GIMBAL_CONTROL_PERIOD_MS * 0.001f);
    dbg_veh_rate_deg_s = rate;
    dbg_veh_pitch_rate_deg_s = pitch_rate;
}

void Gimbal_CaptureForward(void)
{
    tune_yaw_fwd_deg = g_axis_table[GIMBAL_AXIS_YAW].current_angle_deg;
}

/* Yaw motor rate (rpm, relative to the chassis) from its encoder angle between two
 * feedback frames; 0 until two frames have arrived. Shows whether the motor reaches the
 * commanded speed (channel 6). */
static float yaw_motor_rpm_measured(void)
{
    static float last_angle, rpm;
    static uint32_t last_ms;
    static bool have_last;
    const gimbal_axis_runtime_t *yaw = &g_axis_table[GIMBAL_AXIS_YAW];
    if (!yaw->feedback_valid) return rpm;
    const uint32_t dt_ms = yaw->last_feedback_tick - last_ms;
    if (have_last && dt_ms == 0U) return rpm;
    if (have_last && dt_ms < 100U)
        rpm = wrap_pm180_deg(yaw->current_angle_deg - last_angle) * 1000.0f / (float)dt_ms
              * g_yaw_deg_s_to_rpm;
    last_angle = yaw->current_angle_deg;
    last_ms = yaw->last_feedback_tick;
    have_last = true;
    return rpm;
}

/* One telemetry sample (channels: see telemetry.h). */
static void telemetry_sample(void)
{
    const float ch[TELEM_CHANNELS] = {
        dbg_vision_yaw_err, dbg_vision_pitch_err,
        (float)((vision_has_target ? 1 : 0) | (vision_suspended ? 2 : 0) |
                (waiting_for_vision ? 4 : 0) | (g_track_yaw.active ? 8 : 0) |
                ((g_chassis.online && g_chassis.turning) ? 16 : 0)),
        dbg_track_yaw_rate, dbg_yaw_error, dbg_yaw_speed_rpm,
        g_imu.gyro[2] * (180.0f / (float)M_PI), dbg_pitch_error,
        yaw_motor_rpm_measured(), dbg_chassis_gz_deg_s, dbg_vision_code, dbg_vision_area_pct};
    float out[TELEM_CHANNELS];
    memcpy(out, ch, sizeof(out));
    switch (tune_telem_select) {      /* channels 9/10 for diagnostics, see tune_telem_select */
    case 1:
        out[8] = g_imu.angle[INS_PITCH_ADDRESS_OFFSET] * (180.0f / (float)M_PI);
        out[9] = dbg_track_pitch_rate;
        break;
    case 2:
        out[8] = g_veh_pitch_rate;
        out[9] = g_chassis.speed_m_s * 100.0f;
        break;
    case 3:
        out[8] = dbg_yaw_gz_lead;
        out[9] = g_chassis_gz_rate;
        break;
    case 4:
        out[8] = g_veh_rate;
        out[9] = g_veh_speed_used * 100.0f;
        break;
    default:
        break;
    }
    Telemetry_Sample(out);
}

/* ===================== [任务B] IMU增稳执行 (500Hz) ===================== */
/* 职责：读取IMU -> 计算与目标的误差 -> 发送电机指令 */
static void gimbal_step_imu_execute(void)
{
    // 1. 获取当前传感器数据
    float imu_yaw_current = yaw_deg_from_imu();
    float motor_yaw_current = g_axis_table[GIMBAL_AXIS_YAW].current_angle_deg;
    float imu_pitch_current = g_imu.angle[INS_PITCH_ADDRESS_OFFSET] * (180.0f / (float)M_PI);
    float motor_pitch_current = g_axis_table[GIMBAL_AXIS_PITCH].current_angle_deg;
    dbg_imu_pitch_deg = imu_pitch_current;
    dbg_motor_pitch_deg = motor_pitch_current;

    // 2. 初始化检查 (防止刚上电IMU没稳就乱转)
    if (!g_target_initialized)
    {
        if (g_imu.ready)
        {
            /* Pitch limit origin, like yaw's, is captured once: a control dropout must not
             * move the +/-80 deg window to wherever pitch happens to be at re-init. */
            if (!g_pitch_base_valid) {
                g_pitch_base_imu = imu_pitch_current;
                g_pitch_base_motor = motor_pitch_current;
                g_pitch_base_valid = true;
            }
            if (!g_yaw_limit_origin_valid) {
                g_yaw_limit_origin = motor_yaw_current;
                g_yaw_limit_origin_valid = true;
            }
            /* Hold the current pose (unwrapped against the fixed origin). */
            g_angle_history_count = 0U;   /* angles from before a dropout are not this pose */
            track_stop(&g_track_yaw);
            track_stop(&g_track_pitch);
            g_world_target_yaw = imu_yaw_current;
            g_vision_pitch_motor = g_hold_pitch_motor = pitch_motor_unwrapped();
            g_vision_pitch_imu = g_hold_pitch_imu = imu_pitch_current;
            g_target_initialized = true;
        }
        return; // 未初始化时不执行控制
    }

    vehicle_rate_update(motor_yaw_current);
    {
        angle_sample_t sample = {HAL_GetTick(), imu_yaw_current, pitch_motor_unwrapped(), imu_pitch_current,
                                 g_veh_yaw_cum, g_veh_pitch_cum};
        angle_history_push(&sample);
    }
    float track_yaw_rate, track_pitch_rate;
    vision_track_apply(HAL_GetTick(), &track_yaw_rate, &track_pitch_rate);

    /* ---------------- YAW 轴控制 (IMU 闭环) ---------------- */

    // 计算误差：目标世界角度 - 当前世界角度
    float yaw_world_error = g_world_target_yaw - imu_yaw_current;

    // 关键：处理过零点 (例如目标是1度，当前是359度，误差应为+2度，而不是-358度)
    if (yaw_world_error > 180.0f) yaw_world_error -= 360.0f;
    if (yaw_world_error < -180.0f) yaw_world_error += 360.0f;
    dbg_yaw_error = yaw_world_error;

    if ((yaw_world_error > 0.0f && pid_yaw_speed.prev < 0.0f) ||
        (yaw_world_error < 0.0f && pid_yaw_speed.prev > 0.0f))
    {
        pid_yaw_speed.integral = 0.0f;
    }

    pid_yaw_speed.kp = g_yaw_kp * tune_yaw_kp_scale;
    pid_yaw_speed.ki = g_yaw_ki;
    pid_yaw_speed.kd = g_yaw_kd;
    pid_yaw_speed.integral_limit = g_yaw_integral_limit;

    float yaw_speed_rpm = pid_calc(&pid_yaw_speed, -yaw_world_error);
    float yaw_gyro_deg_s = g_imu.gyro[2] * (180.0f / (float)M_PI);
    float yaw_ff_rpm = g_yaw_ff_sign * yaw_gyro_deg_s * g_yaw_deg_s_to_rpm * g_yaw_ff_gain;
    yaw_speed_rpm += yaw_ff_rpm;
    dbg_yaw_ff_rpm = yaw_ff_rpm;
    /* Target moving in the world (car passing it): command that rate directly instead of
     * waiting for a position error to build up (Kp 2.8 rpm/deg -> 1.5 deg at 25 deg/s). */
    /* x (1+K): the stage gyro feed-forward above damps the commanded rate by that factor;
     * without it only 1/8 of the target rate came through (track_sim.py; field: no gain). */
    yaw_speed_rpm += tune_track_ff * (1.0f + g_yaw_ff_gain) * track_yaw_rate * g_yaw_deg_s_to_rpm;

    if (fabsf(yaw_world_error) < g_yaw_integral_decay_threshold)
    {
        pid_yaw_speed.integral *= g_yaw_integral_decay;
    }

    {
        /* Chassis yaw rate from the $C frames, and the yaw motor's own rate (encoder) to check
         * its sign against: while the gimbal holds a heading they turn opposite ways. */
        static float last_motor_yaw;
        static bool have_last_motor_yaw;
        const float chassis_gz = (g_chassis.online && g_chassis.imu_valid) ? g_chassis.gz_deg_s : 0.0f;
        const float motor_rate = have_last_motor_yaw ?
            wrap_pm180_deg(motor_yaw_current - last_motor_yaw) / ((float)GIMBAL_CONTROL_PERIOD_MS * 0.001f) : 0.0f;
        last_motor_yaw = motor_yaw_current;
        have_last_motor_yaw = true;
        dbg_yaw_motor_rate_deg_s += 0.05f * (motor_rate - dbg_yaw_motor_rate_deg_s);
        dbg_chassis_gz_deg_s = chassis_gz;
        if (tune_yaw_two_loop) {
            float pos_rate = tune_yaw_kpos * yaw_world_error;
            if (pos_rate > tune_yaw_pos_rate_max) pos_rate = tune_yaw_pos_rate_max;
            if (pos_rate < -tune_yaw_pos_rate_max) pos_rate = -tune_yaw_pos_rate_max;
            const float w_star = tune_track_ff * track_yaw_rate + pos_rate;
            const float rate_error = w_star - yaw_gyro_deg_s;
            g_yaw_rate_integral = rate_integral_step(g_yaw_rate_integral, tune_yaw_rate_ki, rate_error,
                (float)GIMBAL_CONTROL_PERIOD_MS * 0.001f, tune_yaw_rate_i_max,
                fabsf(yaw_world_error) < YAW_RATE_I_ZONE_DEG, g_yaw_out_saturated);
            dbg_yaw_rate_integral = g_yaw_rate_integral;
            float lead = 0.0f;
            const uint32_t lead_now = HAL_GetTick();
            const bool gz_ok = g_chassis.online && g_chassis.imu_valid;
            if (gz_ok && g_chassis.turning) {
                g_turning_seen = true;
                g_turning_seen_ms = lead_now;
            }
            const bool turn_tail = g_turning_seen && tune_yaw_gz_tail_ms > 0U &&
                                   (uint32_t)(lead_now - g_turning_seen_ms) <= tune_yaw_gz_tail_ms;
            if (gz_ok && (g_chassis.turning || turn_tail)) {
                lead = tune_yaw_gz_lead_s * g_chassis_gz_rate;
                if (lead > YAW_GZ_LEAD_MAX_DEG_S) lead = YAW_GZ_LEAD_MAX_DEG_S;
                if (lead < -YAW_GZ_LEAD_MAX_DEG_S) lead = -YAW_GZ_LEAD_MAX_DEG_S;
            }
            dbg_yaw_gz_lead = lead;
            yaw_speed_rpm = (w_star - tune_yaw_chassis_ff * (chassis_gz + lead) +
                             tune_yaw_krate * rate_error + g_yaw_rate_integral) * g_yaw_deg_s_to_rpm;
            pid_yaw_speed.integral = 0.0f;           /* the old PID is not in this loop */
        }
    }

    g_yaw_out_saturated = 0;
    if (!isnan(pid_yaw_speed.out_limit))
    {
        if (yaw_speed_rpm >= pid_yaw_speed.out_limit) {
            yaw_speed_rpm = pid_yaw_speed.out_limit;
            g_yaw_out_saturated = 1;
        } else if (yaw_speed_rpm <= -pid_yaw_speed.out_limit) {
            yaw_speed_rpm = -pid_yaw_speed.out_limit;
            g_yaw_out_saturated = -1;
        }
    }
    dbg_yaw_speed_rpm = yaw_speed_rpm;

    /* ---------------- 静止时同步基准，消除零漂 ---------------- */
    {
        uint32_t now = HAL_GetTick();

        // yaw_gyro_deg_s 已在上面计算，直接使用
        if (!vision_suspended && !waiting_for_vision && !vision_has_target && fabsf(yaw_world_error) < 1.0f &&
            fabsf(yaw_gyro_deg_s) < YAW_DRIFT_GYRO_THRESHOLD) {
            // 角速度小于阈值，认为静止
            if (!g_yaw_is_stable) {
                g_yaw_is_stable = true;
                g_yaw_stable_start_tick = now;
            }

            // 静止超过阈值时间，且距离上次同步足够久
            if ((now - g_yaw_stable_start_tick > YAW_DRIFT_STABLE_TIME_MS) &&
                (now - g_yaw_last_sync_tick > YAW_DRIFT_SYNC_INTERVAL_MS)) {

                // 仅在空闲、小误差时缓慢释放原目标；转弯/恢复等待期间不执行。
                g_world_target_yaw = imu_yaw_current;

                g_yaw_last_sync_tick = now;
            }
        } else {
            // 有运动，重置静止状态
            g_yaw_is_stable = false;
        }
    }

    /* ---------------- PITCH 轴控制 (IMU 闭环) ---------------- */
    float target_motor_pitch;

    if (vision_suspended || waiting_for_vision) {
        /* Same compensation relation as the original no-target branch, captured
         * once on entry into turn/link-loss hold. */
        target_motor_pitch = g_hold_pitch_motor - (imu_pitch_current - g_hold_pitch_imu);
    } else {
        /* 有/无视觉目标都跟最近一次的视觉目标（上电时等于上电姿态），再补偿此后的底座俯仰。
         * 以前无目标就回到上电姿态：K230 每隔几帧报一次 pose reprojection error (valid=0)，
         * pitch 就在上电姿态和靶之间来回跳（实测约 0.2 s 一次、上下 2 cm）。 */
        target_motor_pitch = g_vision_pitch_motor - (imu_pitch_current - g_vision_pitch_imu);
    }

    /* Work relative to the start angle and unwrap the motor reading: the driver angle may
     * wrap at +/-180 (field start readings -162.7 and 173.7), and a 360 deg jump must not
     * look like a huge error. Pitch never legitimately moves 180 deg from its start. */
    float pitch_target_relative = target_motor_pitch - g_pitch_base_motor;
    if (pitch_target_relative > PITCH_LIMIT_DOWN) pitch_target_relative = PITCH_LIMIT_DOWN;
    if (pitch_target_relative < PITCH_LIMIT_UP) pitch_target_relative = PITCH_LIMIT_UP;
    target_motor_pitch = g_pitch_base_motor + pitch_target_relative;
    float pitch_relative = wrap_pm180_deg(motor_pitch_current - g_pitch_base_motor);
    float pitch_world_error = (pitch_target_relative - pitch_relative) * PITCH_DIR;
    dbg_target_pitch = target_motor_pitch;
    dbg_pitch_error = pitch_world_error;
    pid_pitch_speed.kp = g_pitch_kp;
    pid_pitch_speed.ki = g_pitch_ki;
    pid_pitch_speed.kd = g_pitch_kd;
    pid_pitch_speed.integral_limit = g_pitch_integral_limit;

    /* Shrink by the deadzone (no step at its edge) and bleed the integral near target,
     * as yaw does, so the hold settles at exactly zero command instead of a limit cycle. */
    float pitch_loop_error = 0.0f;
    if (fabsf(pitch_world_error) > g_pitch_hold_deadzone_deg) {
        pitch_loop_error = pitch_world_error - copysignf(g_pitch_hold_deadzone_deg, pitch_world_error);
    }
    float pitch_speed_rpm = pid_calc(&pid_pitch_speed, -pitch_loop_error);
    if (fabsf(pitch_world_error) < g_pitch_integral_decay_threshold) {
        pid_pitch_speed.integral *= g_pitch_integral_decay;
    }
    float pitch_gyro_deg_s = g_imu.gyro[1] * (180.0f / (float)M_PI);
    float pitch_ff_rpm = g_pitch_ff_sign * pitch_gyro_deg_s * g_pitch_deg_s_to_rpm * g_pitch_ff_gain;
    pitch_speed_rpm += pitch_ff_rpm;
    dbg_pitch_ff_rpm = pitch_ff_rpm;
    pitch_speed_rpm += tune_track_ff * PITCH_DIR * track_pitch_rate * g_pitch_deg_s_to_rpm;

    if (!isnan(pid_pitch_speed.out_limit))
    {
        if (pitch_speed_rpm > pid_pitch_speed.out_limit)
            pitch_speed_rpm = pid_pitch_speed.out_limit;
        else if (pitch_speed_rpm < -pid_pitch_speed.out_limit)
            pitch_speed_rpm = -pid_pitch_speed.out_limit;
    }
    dbg_pitch_speed_rpm = pitch_speed_rpm;

    if ((pitch_relative >= PITCH_LIMIT_DOWN && pitch_speed_rpm > 0.0f) ||
        (pitch_relative <= PITCH_LIMIT_UP && pitch_speed_rpm < 0.0f))
    {
        pitch_speed_rpm = 0.0f;
        pid_pitch_speed.integral = 0.0f;
    }

    float yaw_relative = motor_yaw_current - g_yaw_limit_origin;
    dbg_yaw_relative_deg = yaw_relative;   /* multi-turn motor yaw since first init */
#if GIMBAL_YAW_LIMIT_ENABLE
    if ((yaw_relative >= YAW_LIMIT_MAX && yaw_speed_rpm > 0.0f) ||
        (yaw_relative <= YAW_LIMIT_MIN && yaw_speed_rpm < 0.0f)) {
        yaw_speed_rpm = 0.0f;
        pid_yaw_speed.integral = 0.0f;
    }
#endif
    if (!isfinite(yaw_speed_rpm) || !isfinite(pitch_speed_rpm)) {
        g_status.control_ready = 0;
        gimbal_stop();
        return;
    }
    dbg_yaw_speed_rpm = yaw_speed_rpm;
    dbg_pitch_speed_rpm = pitch_speed_rpm;
    {
        uint32_t now = HAL_GetTick();
        bool yaw_stuck = axis_stalled(GIMBAL_AXIS_YAW, yaw_speed_rpm, motor_yaw_current, now);
        bool pitch_stuck = axis_stalled(GIMBAL_AXIS_PITCH, pitch_speed_rpm, motor_pitch_current, now);
        if (yaw_stuck || pitch_stuck) {
            if (yaw_stuck) g_axis_table[GIMBAL_AXIS_YAW].initialized = 0;
            if (pitch_stuck) g_axis_table[GIMBAL_AXIS_PITCH].initialized = 0;
            g_status.control_ready = 0;
            gimbal_stop();
            return;
        }
    }
    /* ---------------- 发送指令 ---------------- */
    bool yaw_ok = gimbal_send_speed(&g_axis_table[GIMBAL_AXIS_YAW], rpm_to_raw(yaw_speed_rpm));
    int32_t pitch_raw = rpm_to_raw(pitch_speed_rpm);
#if GIMBAL_PITCH_ZERO_CMD_TEST
    pitch_raw = 0;
#endif
    pitch_command_stats(pitch_raw);
    bool pitch_ok = gimbal_send_speed(&g_axis_table[GIMBAL_AXIS_PITCH], yaw_ok ? pitch_raw : 0);
    if (!yaw_ok || !pitch_ok) {
        g_status.control_ready = 0;
        gimbal_stop();
    }
}

void CAN_UserRxCallback(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    if (!header || !data || header->IdType != FDCAN_STANDARD_ID ||
        header->RxFrameType != FDCAN_DATA_FRAME || header->FDFormat != FDCAN_CLASSIC_CAN ||
        header->DataLength != FDCAN_DLC_BYTES_8) return;
    if (data[0] != 0x43 || data[1] != 0 || data[2] != 0x08 || data[3] != 0) return;
    for (unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        if (g_axis_table[i].cfg.hcan != hfdcan || g_axis_table[i].cfg.rx_id != header->Identifier) continue;
        uint32_t bits = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                        ((uint32_t)data[6] << 8) | data[7];
        int32_t raw;
        memcpy(&raw, &bits, sizeof(raw));
        g_feedback[i].angle = (float)raw / 100.0f;
        g_feedback[i].received_ms = HAL_GetTick();
        g_feedback[i].valid = 1;
    }
}

static bool pop_line(gimbal_uart_t *port, rx_line_t *line)
{
    bool present = false;
    taskENTER_CRITICAL();
    if (port->tail != port->head) {
        *line = port->queue[port->tail];
        port->tail = (uint8_t)((port->tail + 1U) % RX_QUEUE_DEPTH);
        present = true;
    }
    taskEXIT_CRITICAL();
    return present;
}

static void apply_packet(gimbal_uart_t *port, const rx_line_t *line)
{
    gimbal_packet_t packet;
    uint32_t now = HAL_GetTick();
    if (!GimbalProtocol_Parse(line->text, &packet)) {
        ++g_status.invalid_frames;
        return;
    }
    if (packet.kind == GIMBAL_PACKET_CHASSIS) {
        if ((packet.versioned && port->uart != GIMBAL_CHASSIS_UART) ||
            (uint32_t)(now - line->received_ms) > GIMBAL_CHASSIS_TIMEOUT_MS) return;
        if (chassis_seen && (int32_t)(line->received_ms - g_chassis.received_ms) < 0) return;
        if (packet.versioned && !GimbalProtocol_AcceptSequence(&port->chassis_seq, packet.seq,
                                      line->received_ms, GIMBAL_CHASSIS_TIMEOUT_MS)) return;
        g_chassis.yaw_deg = packet.yaw;
        g_chassis.pitch_deg = packet.pitch;
        g_chassis.roll_deg = packet.roll;
        g_chassis.gx_deg_s = packet.gx;
        g_chassis.gy_deg_s = packet.gy;
        if (chassis_seen && packet.valid && g_chassis.imu_valid) {
            const int32_t dt_ms = (int32_t)(line->received_ms - g_chassis.received_ms);
            if (dt_ms > 0 && dt_ms <= 100) {
                const float rate = (packet.gz - g_chassis.gz_deg_s) * 1000.0f / (float)dt_ms;
                g_chassis_gz_rate += 0.5f * (rate - g_chassis_gz_rate);
            } else {
                g_chassis_gz_rate = 0.0f;
            }
        }
        g_chassis.gz_deg_s = packet.gz;
        if (chassis_seen && packet.has_speed && g_chassis.has_speed) {
            const int32_t dt_ms = (int32_t)(line->received_ms - g_chassis.received_ms);
            if (dt_ms > 0 && dt_ms <= 100) {
                const float accel = (packet.speed_m_s - g_chassis.speed_m_s) * 1000.0f / (float)dt_ms;
                g_chassis_accel += 0.5f * (accel - g_chassis_accel);
            } else {
                g_chassis_accel = 0.0f;
            }
        } else {
            g_chassis_accel = 0.0f;
        }
        g_chassis.has_speed = packet.has_speed;
        if (packet.has_task) {
            g_task = packet.task;
            g_task_progress = packet.progress_permille;
        }
        g_chassis.speed_m_s = packet.has_speed ? packet.speed_m_s : 0.0f;
        dbg_chassis_speed_m_s = g_chassis.speed_m_s;
        g_chassis.received_ms = line->received_ms;
        g_chassis.imu_valid = packet.valid;
        g_chassis.turning = packet.turning;
        chassis_seen = true;
        ++dbg_chassis_frames;
    } else {
        if (now >= 5000U) vision_warmup_done = true;
        if ((packet.versioned && port->uart != GIMBAL_VISION_UART) ||
            (port->uart == GIMBAL_VISION_UART && !vision_warmup_done) ||
            (uint32_t)(now - line->received_ms) > GIMBAL_VISION_TIMEOUT_MS) return;
        if (vision_seen && (int32_t)(line->received_ms - vision_received_ms) < 0) return;
        if (packet.versioned && !GimbalProtocol_AcceptSequence(&port->vision_seq, packet.seq,
                                      line->received_ms, GIMBAL_VISION_TIMEOUT_MS)) return;
        vision_seen = true;
        vision_received_ms = line->received_ms;
        if (packet.has_diag) {          /* every frame, also while vision is held */
            dbg_vision_code = (float)packet.vision_code;
            dbg_vision_area_pct = (float)packet.area_x100 * 0.01f;
        }
        if (vision_suspended || (waiting_for_vision &&
            (int32_t)(line->received_ms - vision_resume_after_ms) <= 0)) return;
        vision_yaw_deg = packet.yaw;
        vision_pitch_deg = packet.pitch;
        vision_has_target = packet.valid;
        g_vision_has_age = packet.has_range;
        g_vision_age_ms = packet.age_ms;
        if (packet.valid && packet.has_range && packet.range_m > 0.0f) {
            g_vision_range_m = packet.range_m;
            g_vision_range_ms = line->received_ms;
            dbg_vision_range_m = packet.range_m;
        }
        vision_data_new = true;
        ++dbg_vision_frames;
        dbg_vision_yaw_err = packet.yaw;
        dbg_vision_pitch_err = packet.pitch;
        if (packet.valid) {
            if (packet.yaw > dbg_vision_yaw_err_max) dbg_vision_yaw_err_max = packet.yaw;
            if (packet.yaw < dbg_vision_yaw_err_min) dbg_vision_yaw_err_min = packet.yaw;
            if (packet.pitch > dbg_vision_pitch_err_max) dbg_vision_pitch_err_max = packet.pitch;
            if (packet.pitch < dbg_vision_pitch_err_min) dbg_vision_pitch_err_min = packet.pitch;
        }
        if (packet.valid) waiting_for_vision = false;
    }
}

static void gimbal_poll_inputs(void)
{
    rx_line_t line;
    /* Chassis first: final mode is evaluated after draining both bounded queues. */
    for (int i = 1; i >= 0; --i) {
        for (unsigned n = 0; n < RX_QUEUE_DEPTH - 1U && pop_line(&g_uarts[i], &line); ++n)
            apply_packet(&g_uarts[i], &line);
        taskENTER_CRITICAL();
        if (g_uarts[i].uart->RxState == HAL_UART_STATE_READY)
            HAL_UARTEx_ReceiveToIdle_IT(g_uarts[i].uart, g_uarts[i].rx, sizeof(g_uarts[i].rx));
        taskEXIT_CRITICAL();
    }
    uint32_t now = HAL_GetTick();
    bool chassis_online = chassis_seen && (uint32_t)(now - g_chassis.received_ms) <= GIMBAL_CHASSIS_TIMEOUT_MS;
    bool suspend = (chassis_online && g_chassis.turning && tune_turn_hold) ||
                   (!chassis_online && GIMBAL_REQUIRE_CHASSIS);
    const uint8_t link_lost = (chassis_seen && !chassis_online) ? 1U : 0U;
    if (link_lost != dbg_chassis_link_lost) {      /* alarm only: vision keeps tracking */
        dbg_chassis_link_lost = link_lost;
        if (link_lost) BSP_Buzzer_On(); else BSP_Buzzer_Off();
    }
    if (suspend != vision_suspended) {
        vision_suspended = suspend;
        waiting_for_vision = !suspend;
        vision_has_target = false;
        vision_data_new = false;
        vision_resume_after_ms = now;
        gimbal_reset_pid();
        track_stop(&g_track_yaw);       /* turn / link loss: hold, restart on the next target */
        track_stop(&g_track_pitch);
        if (suspend && g_target_initialized && g_imu.ready) {
            g_world_target_yaw = yaw_deg_from_imu();
            g_hold_pitch_imu = g_imu.angle[INS_PITCH_ADDRESS_OFFSET] * (180.0f / (float)M_PI);
            g_hold_pitch_motor = pitch_motor_unwrapped();
            /* After the hold, keep this pose until vision gives a new pitch target. */
            g_vision_pitch_motor = g_hold_pitch_motor;
            g_vision_pitch_imu = g_hold_pitch_imu;
        }
    }
    g_chassis.online = chassis_online;
    if (!chassis_online) g_chassis.imu_valid = 0;
    g_status.vision_online = vision_seen && (uint32_t)(now - vision_received_ms) <= GIMBAL_VISION_TIMEOUT_MS;
    if (!g_status.vision_online || vision_suspended) {
        vision_has_target = false;
        vision_data_new = false;
    }
    g_status.chassis_online = chassis_online;
    g_status.vision_suspended = vision_suspended;
    g_status.waiting_for_vision = waiting_for_vision;
    dbg_control_ready = g_status.control_ready;
    dbg_vision_online = g_status.vision_online;
    dbg_vision_valid = vision_has_target;
    dbg_vision_suspended = vision_suspended;
}

void Gimbal_GetChassisState(gimbal_chassis_state_t *state)
{
    if (!state) return;
    taskENTER_CRITICAL();
    *state = g_chassis_published;
    taskEXIT_CRITICAL();
}

void Gimbal_GetStatus(gimbal_status_t *status)
{
    if (!status) return;
    taskENTER_CRITICAL();
    *status = g_status_published;
    status->dropped_lines = g_uarts[0].dropped + g_uarts[1].dropped;
    status->uart_errors = g_uarts[0].errors + g_uarts[1].errors;
    taskEXIT_CRITICAL();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == TELEMETRY_UART) { Telemetry_OnError(); return; }
    if (huart == GIMBAL_CHASSIS_UART) ++dbg_chassis_uart_errors;
    for (unsigned i = 0; i < 2; ++i) {
        gimbal_uart_t *port = &g_uarts[i];
        if (port->uart != huart) continue;
        ++port->errors;
        GimbalProtocol_Discard(&port->assembling);
        HAL_UART_AbortReceive(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_IT(huart, port->rx, sizeof(port->rx));
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == TELEMETRY_UART) { Telemetry_OnRxEvent(size); return; }
    for (unsigned i = 0; i < 2; ++i) {
        gimbal_uart_t *port = &g_uarts[i];
        if (port->uart != huart) continue;
        uint32_t now = HAL_GetTick();
        if (port->assembling.length && (uint32_t)(now - port->last_byte_ms) > 100U)
            GimbalProtocol_Discard(&port->assembling);
        port->last_byte_ms = now;
        if (size > sizeof(port->rx)) size = sizeof(port->rx);
        if (huart == GIMBAL_CHASSIS_UART) dbg_chassis_rx_bytes += size;
        for (uint16_t n = 0; n < size; ++n) {
            int result = GimbalProtocol_Push(&port->assembling, port->rx[n]);
            if (result < 0) ++port->dropped;
            if (result == 1) {
                uint8_t next = (uint8_t)((port->head + 1U) % RX_QUEUE_DEPTH);
                if (next == port->tail) { ++port->dropped; continue; }
                rx_line_t *line = &port->queue[port->head];
                strcpy(line->text, port->assembling.data);
                line->received_ms = now;
                port->head = next;
            }
        }
        HAL_UARTEx_ReceiveToIdle_IT(huart, port->rx, sizeof(port->rx));
    }
}
