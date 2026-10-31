/* Include the production task to exercise its internal transitions with fake IO. */
#include "../gimbal/App/gimbal_task.c"
#include <assert.h>

FDCAN_HandleTypeDef hfdcan1 = {1}, hfdcan2 = {2}, hfdcan3 = {3};
UART_HandleTypeDef huart1 = {0, 1, 0}, huart2 = {0, 2, 0}, huart7 = {0, 7, 0}, huart10 = {0, 10, 0};
static uint32_t now_ms;
static int fail_send;
static int32_t last_speed[2];
static unsigned tx_count;
uint32_t HAL_GetTick(void) { return now_ms; }
void osDelay(uint32_t ms) { now_ms += ms; }
TickType_t xTaskGetTickCount(void) { return now_ms; }
void vTaskDelayUntil(TickType_t *last, TickType_t delta) { *last += delta; }
void can_bsp_init(void) {}
void BSP_Buzzer_Init(void) {}
void BSP_Buzzer_On(void) {}
void BSP_Buzzer_Off(void) {}
void IMU_GetSnapshot(imu_snapshot_t *sample) { *sample = g_imu; }
/* Telemetry glue (telemetry.c) is board code; record what the task hands to it. */
static unsigned telem_rx_events, telem_errors, telem_samples;
static float telem_last[TELEM_CHANNELS];
void Telemetry_Init(void) {}
void Telemetry_Sample(const float ch[TELEM_CHANNELS])
{ ++telem_samples; memcpy(telem_last, ch, sizeof(telem_last)); }
void Telemetry_OnRxEvent(uint16_t size) { (void)size; ++telem_rx_events; }
void Telemetry_OnError(void) { ++telem_errors; }
int HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ (void)b; (void)n; u->RxState = 1; return 0; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *u) { u->RxState = 0; return 0; }
/* $G status frames to the chassis (huart1 TX) and $K task frames to the K230 (huart10 TX);
 * gState stays READY (sent at once). */
static char uart1_tx[64], uart10_tx[64];
static unsigned uart1_tx_count, uart10_tx_count;
int HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    char *out = u == &huart1 ? uart1_tx : uart10_tx;
    assert((u == &huart1 || u == &huart10) && n < 64);
    memcpy(out, b, n); out[n] = '\0';
    if (u == &huart1) ++uart1_tx_count; else ++uart10_tx_count;
    return 0;
}
static void last_task(unsigned *task, unsigned *progress)
{
    assert(sscanf(uart10_tx, "$K,1,%*u,%u,%u*", task, progress) == 2);
}
static unsigned last_status_flags(void)
{
    unsigned flags = 0xFFU;
    assert(sscanf(uart1_tx, "$G,1,%*u,%u*", &flags) == 1);
    return flags;
}
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *can, uint16_t id, uint8_t *data, uint32_t len)
{
    (void)id; assert(len == 8); ++tx_count;
    if (fail_send) return 1;
    if (data[2] == 0x21) {
        uint32_t raw = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                       ((uint32_t)data[6] << 8) | data[7];
        memcpy(&last_speed[can == &hfdcan1 ? 0 : 1], &raw, 4);
    }
    return 0;
}

static void receive(unsigned port, const char *text)
{
    while (*text) {
        size_t length = strlen(text);
        if (length > 64) length = 64;
        memcpy(g_uarts[port].rx, text, length);
        HAL_UARTEx_RxEventCallback(g_uarts[port].uart, (uint16_t)length);
        text += length;
    }
}

static void fresh(void)
{
    g_imu.ready = 1;
    g_imu.updated_ms = now_ms;
    for (unsigned i = 0; i < 2; ++i) {
        g_axis_table[i].initialized = 1;
        g_axis_table[i].feedback_valid = 1;
        g_axis_table[i].last_feedback_tick = now_ms;
    }
}

/* Run the control loop for ms with the motors where they are (fills the angle history). */
static void hold_still(unsigned ms)
{
    for (unsigned t = 0; t < ms; t += 2U) { now_ms += 2U; fresh(); gimbal_step_imu_execute(); }
}

static void receive_body(unsigned port, const char *body);
/* Keep the chassis link fresh (otherwise > 300 ms of silence suspends vision). */
static unsigned g_test_chassis_mode;   /* 1 = chassis_alive() reports a turn */

static void chassis_alive(void)
{
    static unsigned seq = 200;
    char body[64];
    snprintf(body, sizeof(body), "C,1,%u,%u,0,0,0,0,0,0,0", seq++, g_test_chassis_mode);
    receive_body(1, body);
    gimbal_poll_inputs();
}

static void receive_body(unsigned port, const char *body)
{
    char frame[192];
    snprintf(frame, sizeof(frame), "$%s*%04X\n", body,
             (unsigned)GimbalProtocol_Crc16(body, strlen(body)));
    receive(port, frame);
}

int main(void)
{
    now_ms = 6000;
    /* Default yaw loop is the two-loop one; the tests up to the two-loop block below exercise
     * the old structure (PID + stage gyro feed-forward), so switch it off for them. */
    assert(tune_yaw_two_loop == 1U);
    tune_yaw_two_loop = 0U;
    /* Vision stays on in corners by default; the hold tests below switch the old hold on. */
    assert(tune_turn_hold == 0U);
    tune_turn_hold = 1U;
#if !GIMBAL_CAN_TX_ENABLE
    fresh();
    gimbal_axis_enable(&g_axis_table[0]);
    gimbal_control_cycle();
    assert(tx_count == 0);
    puts("CAN disabled test passed: no command path transmits.");
    return 0;
#endif
    gimbal_control_cycle();
    assert(!g_target_initialized && !g_status.control_ready);
    assert(last_speed[0] == 0 && last_speed[1] == 0);
    fresh();
    gimbal_control_cycle();
    assert(g_target_initialized);

    receive(0, "10,2,0,0\n");
    gimbal_poll_inputs();
    gimbal_control_cycle();
    assert(g_world_target_yaw == 350.0f && fabsf(g_vision_pitch_motor - 2.0f) < 1e-4f);
    /* Vision is wired to USART10 (PE2 RX), the chassis to USART1 (PA10 RX); bytes arriving
     * on any other UART (here USART2, RS485) are neither. */
    assert(g_uarts[0].uart == &huart10 && g_uarts[1].uart == &huart1);
    memcpy(g_uarts[0].rx, "30,20\n", 6);
    HAL_UARTEx_RxEventCallback(&huart2, 6);
    gimbal_poll_inputs();
    gimbal_control_cycle();
    assert(g_world_target_yaw == 350.0f && fabsf(g_vision_pitch_motor - 2.0f) < 1e-4f);
    float before = g_world_target_yaw;
    receive(0, "inf,0\n10,abc\n");
    gimbal_poll_inputs();
    assert(g_status.invalid_frames == 2 && g_world_target_yaw == before);

    ++now_ms;
    receive(1, "T,T,T,T\n");
    gimbal_poll_inputs();
    assert(vision_suspended && !vision_has_target && g_world_target_yaw == 0);
    g_imu.angle[0] = 0.1f;
    receive(1, "T,T,T,T\n");
    receive(0, "30,20\n");
    gimbal_poll_inputs();
    assert(g_world_target_yaw == 0 && !vision_data_new); /* No repeated relock. */
    gimbal_step_imu_execute();
    assert(g_world_target_yaw == 0); /* Drift sync cannot override turn lock. */

    ++now_ms;
    receive(1, "N,N,N,N\n");
    gimbal_poll_inputs();
    assert(!vision_suspended && waiting_for_vision && !vision_has_target);
    receive(0, "10,2\n"); /* Same millisecond: do not reuse a pre-transition image. */
    gimbal_poll_inputs();
    assert(!vision_has_target);
    ++now_ms;
    receive(0, "10,2\n");
    gimbal_poll_inputs();
    assert(vision_has_target && !waiting_for_vision);
    now_ms += 301;
    gimbal_poll_inputs();
    /* Chassis link lost after it was seen: alarm only, vision keeps tracking. */
    assert(!vision_suspended && !g_status.chassis_online && dbg_chassis_link_lost);
    ++now_ms;
    receive(0, "0,0\n");
    gimbal_poll_inputs();
    assert(vision_has_target && !vision_suspended);
    gimbal_control_cycle();
    assert(!g_status.control_ready && last_speed[0] == 0 && last_speed[1] == 0);

    /* Long line recovery and binary/error resynchronization through the real ISR queue. */
    char long_line[260]; memset(long_line, '1', sizeof(long_line)-2);
    long_line[sizeof(long_line)-2] = '\n'; long_line[sizeof(long_line)-1] = 0;
    receive(1, long_line);
    receive(1, "N,N,N,N\n");
    gimbal_poll_inputs();
    assert(!vision_suspended && g_uarts[1].dropped == 1 && !dbg_chassis_link_lost);
    receive(1, "T,T,");
    HAL_UART_ErrorCallback(GIMBAL_CHASSIS_UART);
    receive(1, "T,T\nN,N,N,N\n");
    gimbal_poll_inputs();
    assert(!vision_suspended && g_uarts[1].errors == 1);

    /* CAN callback rejects short, remote, extended and FD frames. */
    uint8_t angle[8] = {0x43, 0, 8, 0, 0, 0, 0x04, 0xD2};
    FDCAN_RxHeaderTypeDef h = {0x581, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_CLASSIC_CAN, 4};
    CAN_UserRxCallback(&hfdcan1, &h, angle);
    assert(!g_feedback[0].valid);
    h.DataLength = FDCAN_DLC_BYTES_8; h.IdType = 1;
    CAN_UserRxCallback(&hfdcan1, &h, angle); assert(!g_feedback[0].valid);
    h.IdType = FDCAN_STANDARD_ID; h.RxFrameType = 1;
    CAN_UserRxCallback(&hfdcan1, &h, angle); assert(!g_feedback[0].valid);
    h.RxFrameType = FDCAN_DATA_FRAME; h.FDFormat = 1;
    CAN_UserRxCallback(&hfdcan1, &h, angle); assert(!g_feedback[0].valid);
    h.FDFormat = FDCAN_CLASSIC_CAN;
    CAN_UserRxCallback(&hfdcan1, &h, angle);
    assert(g_feedback[0].valid && fabsf(g_feedback[0].angle - 12.34f) < 0.001f);

    fresh(); gimbal_control_cycle();
    fail_send = 1;
    gimbal_control_cycle();
    assert(g_can_tx_fault && !g_status.control_ready && g_status.can_tx_errors);
    fail_send = 0;
    gimbal_control_cycle();
    assert(!g_can_tx_fault && last_speed[0] == 0 && last_speed[1] == 0);
    gimbal_control_cycle();
    assert(g_status.control_ready);
    g_imu.ready = 0;
    gimbal_control_cycle();
    assert(!g_status.control_ready && !g_target_initialized);
    fresh(); g_axis_table[1].last_feedback_tick = now_ms - 101;
    gimbal_control_cycle();
    assert(!g_status.control_ready);

    fresh(); g_imu.angle[0] = 0; g_imu.gyro[2] = 0;
    gimbal_control_cycle();
#if GIMBAL_YAW_LIMIT_ENABLE
    /* Relative yaw limits block outward motion while allowing recovery inward. */
    g_axis_table[0].current_angle_deg = g_yaw_limit_origin + 60;
    g_world_target_yaw = 10;
    gimbal_step_imu_execute(); assert(last_speed[0] == 0);
    g_world_target_yaw = 350;
    gimbal_step_imu_execute(); assert(last_speed[0] < 0);
#else
    /* Slip ring: continuous yaw. Past 60 deg and past a full turn it still drives
     * outward (multi-turn motor angles must not trip anything). */
    g_world_target_yaw = 10;
    g_axis_table[0].current_angle_deg = g_yaw_limit_origin + 60;
    gimbal_step_imu_execute(); assert(last_speed[0] > 0);
    g_axis_table[0].current_angle_deg = g_yaw_limit_origin + 400;
    gimbal_step_imu_execute(); assert(last_speed[0] > 0);
    g_axis_table[0].current_angle_deg = g_yaw_limit_origin - 400;
    g_world_target_yaw = 350;
    gimbal_step_imu_execute(); assert(last_speed[0] < 0);
#endif

    /* Pitch hold: sub-deadzone error (IMU noise) commands exactly zero and lets the
     * integral bleed off, so the motor does not chatter in place and heat up. */
    g_imu.gyro[1] = 0;
    gimbal_step_imu_execute();
    pid_pitch_speed.integral = 50.0f;
    g_axis_table[1].current_angle_deg = dbg_target_pitch - 0.05f * PITCH_DIR;
    for (int i = 0; i < 200; ++i) gimbal_step_imu_execute();
    assert(last_speed[1] == 0 && fabsf(pid_pitch_speed.integral) < 1.0f);
    g_axis_table[1].current_angle_deg = dbg_target_pitch - 2.0f;
    gimbal_step_imu_execute(); assert(last_speed[1] != 0);

    /* Field: the driver's multi-turn angle read -162.7 deg at start (MCU reset only).
     * Pitch limits are relative to that start angle, so the hold must not be clamped
     * to the absolute -80 deg and driven flat out at 50 rpm into a stall. */
    g_target_initialized = false; g_pitch_base_valid = false;   /* fresh power-on */
    g_axis_table[1].current_angle_deg = -162.68f;
    gimbal_step_imu_execute();
    for (int i = 0; i < 10; ++i) gimbal_step_imu_execute();
    /* |speed| <= 0.05 rpm: only the decaying integral left by the step above. */
    assert(fabsf(dbg_target_pitch + 162.68f) < 0.5f && abs(last_speed[1]) <= 5);
    g_axis_table[1].current_angle_deg = -162.68f + PITCH_LIMIT_DOWN + 1.0f;
    gimbal_step_imu_execute(); assert(last_speed[1] <= 0);

    /* Field start reading 173.67 deg: moving 7 deg down wraps the driver angle to -179.
     * That is a 7 deg error, not a 353 deg one. */
    g_target_initialized = false; g_pitch_base_valid = false;   /* fresh power-on */
    g_axis_table[1].current_angle_deg = 173.67f;
    gimbal_step_imu_execute();
    g_axis_table[1].current_angle_deg = -179.0f;
    gimbal_step_imu_execute();
    assert(fabsf(dbg_pitch_error + 7.33f * PITCH_DIR) < 0.5f && abs(last_speed[1]) < 5000);

    /* A control dropout (IMU/feedback timeout) re-initialises the targets but must keep
     * the pitch limit origin: re-init 70 deg down, the window stays at start +/-80. */
    g_axis_table[1].current_angle_deg = 173.67f + 70.0f - 360.0f;
    g_target_initialized = false;
    gimbal_step_imu_execute();
    assert(fabsf(g_pitch_base_motor - 173.67f) < 0.01f);
    gimbal_step_imu_execute();
    assert(fabsf(dbg_target_pitch - (173.67f + 70.0f)) < 0.5f);   /* holds where it is */
    g_axis_table[1].current_angle_deg = 173.67f + PITCH_LIMIT_DOWN + 1.0f - 360.0f;
    gimbal_step_imu_execute(); assert(last_speed[1] <= 0);         /* old window still applies */

    /* Limp/stall watchdog: pitch commanded flat out but not moving (driver protection
     * dropped it out of closed loop) -> re-enable it instead of pushing for ever. A pitch
     * that does move is left alone. */
    g_axis_table[1].current_angle_deg = 173.67f;
    g_vision_pitch_motor = 173.67f + 20.0f;
    for (int i = 0; i < 150; ++i) {                                 /* 0.3 s, moving */
        now_ms += 2; fresh(); g_axis_table[1].current_angle_deg += 0.05f;
        gimbal_step_imu_execute();
    }
    assert(g_axis_table[1].initialized && last_speed[1] > 0);
    uint32_t stalls = dbg_axis_stall_count[1];
    for (int i = 0; i < 300 && g_axis_table[1].initialized; ++i) {  /* now stuck */
        now_ms += 2; fresh(); gimbal_step_imu_execute();
    }
    assert(!g_axis_table[1].initialized && last_speed[1] == 0 && dbg_axis_stall_count[1] == stalls + 1);
    gimbal_axis_enable(&g_axis_table[1]);                            /* task-loop retry */
    assert(g_axis_table[1].initialized);
    g_vision_pitch_motor = g_axis_table[1].current_angle_deg;

    /* Exercise new-protocol routing, telemetry and replay rejection in the task. */
    ++now_ms;
    receive_body(1, "C,1,100,1,1,90,-3,2,0,0,45");
    gimbal_poll_inputs();
    assert(vision_suspended && g_chassis.imu_valid && g_chassis.gz_deg_s == 45);
    uint32_t received_at = g_chassis.received_ms;
    ++now_ms;
    receive_body(1, "C,1,100,0,1,90,-3,2,0,0,45"); /* Same seq cannot change mode. */
    gimbal_poll_inputs();
    assert(vision_suspended && g_chassis.received_ms == received_at);
    receive_body(0, "C,1,101,0,0,0,0,0,0,0,0"); /* Wrong port. */
    gimbal_poll_inputs(); assert(vision_suspended);
    receive_body(1, "C,1,101,0,0,0,0,0,0,0,0");
    gimbal_poll_inputs(); assert(!vision_suspended && waiting_for_vision);
    /* tune_turn_hold 0: a turning chassis no longer holds vision; telemetry flags it (16). */
    assert(tune_turn_hold == 1U);
    tune_turn_hold = 0U;
    ++now_ms; receive_body(1, "C,1,500,1,1,90,-3,2,0,0,45"); gimbal_poll_inputs();
    assert(!vision_suspended && g_chassis.turning);
    telemetry_sample(); assert((int)telem_last[2] & 16); telem_samples = 0;
    tune_turn_hold = 1U;
    ++now_ms; receive_body(1, "C,1,501,1,1,90,-3,2,0,0,45"); gimbal_poll_inputs();
    assert(vision_suspended);
    ++now_ms; receive_body(1, "C,1,502,0,0,0,0,0,0,0,0"); gimbal_poll_inputs();
    assert(!vision_suspended && waiting_for_vision);
    g_uarts[1].chassis_seq.initialized = false;                /* later frames reuse lower numbers */
    ++now_ms;
    receive_body(1, "V,1,1,1,3,4"); /* Wrong port. */
    gimbal_poll_inputs(); assert(!vision_has_target);
    receive_body(0, "V,1,1,1,3,4");
    gimbal_poll_inputs(); assert(vision_has_target && vision_yaw_deg == 3 && vision_pitch_deg == 4);
    receive_body(0, "V,1,1,1,10,20");
    gimbal_poll_inputs(); assert(vision_yaw_deg == 3);
    /* Visual timeout alone leaves chassis tracking mode intact. */
    now_ms += 301;
    receive_body(1, "C,1,102,0,0,0,0,0,0,0,0");
    gimbal_poll_inputs(); assert(!vision_suspended && !vision_has_target && !g_status.vision_online);

    /* Queue overload drops complete new lines and recovers after consumption. */
    uint32_t drops_before = g_uarts[0].dropped;
    for (unsigned i = 0; i < RX_QUEUE_DEPTH + 2; ++i) receive(0, "1,2\n");
    assert(g_uarts[0].dropped == drops_before + 3);
    gimbal_poll_inputs();
    ++now_ms; receive(0, "3,4\n"); gimbal_poll_inputs();
    assert(vision_yaw_deg == 3 && vision_pitch_deg == 4);
    /* Field: pitch sat 3.5 deg off the bullseye. The IMU is on the yaw stage, so a pitch
     * target of "IMU pitch + error" never accumulated. The target is the elevation the frame
     * saw: motor angle + error (+ IMU pitch, 0 here): 10 + 3 -> 13. */
    ++now_ms; receive_body(1, "C,1,103,0,0,0,0,0,0,0,0"); gimbal_poll_inputs();
    fresh(); g_imu.angle[0] = 0; g_imu.angle[1] = 0; g_imu.gyro[1] = 0; g_imu.gyro[2] = 0;
    g_axis_table[1].current_angle_deg = g_pitch_base_motor + 10.0f;
    hold_still(150);                         /* history now covers the latency window */
    ++now_ms; fresh(); receive_body(0, "V,1,2,1,6,3"); gimbal_poll_inputs(); gimbal_control_cycle();
    assert(fabsf(dbg_target_pitch - (g_pitch_base_motor + 13.0f)) < 0.01f);

    /* Latency compensation: the error describes the pose when the frame was shot. Pitch was
     * at +20 until 40 ms ago and has since moved to +23; a frame saying "3 deg low" means
     * 20 + 3, not 23 + 3 (that double-count overshot on acquisition). */
    track_stop(&g_track_yaw); track_stop(&g_track_pitch);
    g_axis_table[1].current_angle_deg = g_pitch_base_motor + 20.0f;
    hold_still(150);
    g_axis_table[1].current_angle_deg = g_pitch_base_motor + 23.0f;
    hold_still(40);
    chassis_alive();
    ++now_ms; fresh(); receive_body(0, "V,1,3,1,6,3"); gimbal_poll_inputs(); gimbal_control_cycle();
    assert(fabsf(dbg_target_pitch - (g_pitch_base_motor + 23.0f)) < 0.01f);
    assert(dbg_vision_lookback_ms >= tune_vision_latency_ms && dbg_vision_lookback_ms <= tune_vision_latency_ms + 4U);

    /* Field: the K230 drops single frames (valid=0, pose reprojection error). Pitch used to
     * fall back to its power-on pose on each one and bobbed 2 cm; it must hold the target. */
    float held = dbg_target_pitch;
    chassis_alive();
    ++now_ms; fresh(); receive_body(0, "V,1,4,0,0,0"); gimbal_poll_inputs(); gimbal_control_cycle();
    assert(!vision_has_target && fabsf(dbg_target_pitch - held) < 0.01f);

    /* Error extremes for field runs: cleared from Keil, then signed max/min of valid frames. */
    dbg_vision_yaw_err_max = dbg_vision_yaw_err_min = 0.0f;
    dbg_vision_pitch_err_max = dbg_vision_pitch_err_min = 0.0f;
    chassis_alive();
    ++now_ms; receive_body(0, "V,1,5,1,-1.5,0.7"); gimbal_poll_inputs();
    ++now_ms; receive_body(0, "V,1,6,1,0.2,-0.1"); gimbal_poll_inputs();
    ++now_ms; receive_body(0, "V,1,7,0,0,0"); gimbal_poll_inputs();
    assert(fabsf(dbg_vision_yaw_err_min + 1.5f) < 1e-4f && fabsf(dbg_vision_yaw_err_max - 0.2f) < 1e-4f);
    assert(fabsf(dbg_vision_pitch_err_max - 0.7f) < 1e-4f && fabsf(dbg_vision_pitch_err_min + 0.1f) < 1e-4f);

    /* Car passing the target: its world bearing moves 20 deg/s. The tracker learns the rate
     * and feeds it to the yaw motor, instead of waiting for a position error to build up. */
    track_stop(&g_track_yaw); track_stop(&g_track_pitch);
    unsigned seq = 10;
    for (int k = 0; k < 30; ++k) {
        char body[64];
        float bearing = 20.0f * 0.044f * (float)k;              /* deg, world */
        snprintf(body, sizeof(body), "V,1,%u,1,%.3f,0", seq++, -bearing);   /* yaw dir -1 */
        chassis_alive();
        ++now_ms; fresh(); receive_body(0, body); gimbal_poll_inputs(); gimbal_control_cycle();
        hold_still(42);
    }
    assert(fabsf(dbg_track_yaw_rate - 20.0f) < 1.5f);
    g_imu.angle[0] = g_world_target_yaw * (float)M_PI / 180.0f;   /* gimbal on the target */
    g_axis_table[0].current_angle_deg = g_yaw_limit_origin;          /* clear of any yaw limit */
    tune_track_ff = 0.0f; hold_still(2); int32_t without_ff = last_speed[0];
    tune_track_ff = 1.0f; hold_still(2); int32_t with_ff = last_speed[0];
    /* 20 deg/s at 6 deg/s per rpm, x (1+K) for the stage gyro damping: ~26.7 rpm. */
    assert(with_ff - without_ff > (int32_t)(20.0f / 6.0f * (1.0f + g_yaw_ff_gain) * 100.0f * 0.8f));

    /* No frames for TRACK_LOST_MS: the track stops and the feed-forward with it. */
    hold_still(TRACK_LOST_MS + 10U);
    assert(!g_track_yaw.active && dbg_track_yaw_rate == 0.0f);
    /* Reacquiring (field: oscillation after corners): for tune_track_settle_ms after a (re)start
     * the tracker follows the position only, then learns the rate as before. */
    for (int k = 0; k < 6; ++k) {                               /* 0..220 ms of the new track */
        char body[64];
        snprintf(body, sizeof(body), "V,1,%u,1,%.3f,0", seq++, -(20.0f * 0.044f * (float)k));
        chassis_alive();
        ++now_ms; fresh(); receive_body(0, body); gimbal_poll_inputs(); gimbal_control_cycle();
        assert(g_track_yaw.active && g_track_yaw.v == 0.0f);
        hold_still(42);
    }
    for (int k = 6; k < 30; ++k) {
        char body[64];
        snprintf(body, sizeof(body), "V,1,%u,1,%.3f,0", seq++, -(20.0f * 0.044f * (float)k));
        chassis_alive();
        ++now_ms; fresh(); receive_body(0, body); gimbal_poll_inputs(); gimbal_control_cycle();
        hold_still(42);
    }
    assert(fabsf(dbg_track_yaw_rate - 20.0f) < 2.5f);
    /* Corner with vision on (tune_turn_hold 0): while the chassis turns the tracker keeps
     * following the position but learns no rate; tune_track_settle_ms after it, it does again. */
    tune_turn_hold = 0U; g_test_chassis_mode = 1U;
    for (int k = 30; k < 36; ++k) {
        char body[64];
        snprintf(body, sizeof(body), "V,1,%u,1,%.3f,0", seq++, -(20.0f * 0.044f * (float)k));
        chassis_alive();
        ++now_ms; fresh(); receive_body(0, body); gimbal_poll_inputs(); gimbal_control_cycle();
        assert(!vision_suspended && g_track_yaw.active && g_track_yaw.v == 0.0f);
        hold_still(42);
    }
    g_test_chassis_mode = 0U;
    for (int k = 36; k < 70; ++k) {
        char body[64];
        snprintf(body, sizeof(body), "V,1,%u,1,%.3f,0", seq++, -(20.0f * 0.044f * (float)k));
        chassis_alive();
        ++now_ms; fresh(); receive_body(0, body); gimbal_poll_inputs(); gimbal_control_cycle();
        hold_still(42);
    }
    assert(fabsf(dbg_track_yaw_rate - 20.0f) < 2.5f);
    tune_turn_hold = 1U;
    track_stop(&g_track_yaw); track_stop(&g_track_pitch);
    g_uarts[0].vision_seq.initialized = false;                 /* later frames reuse lower numbers */
    /* UART7 now carries the Bluetooth telemetry: its events go to the telemetry glue and
     * never reach the vision/chassis line parsers. */
    uint32_t chassis_errors = g_uarts[1].errors;
    HAL_UARTEx_RxEventCallback(&huart7, 2);
    HAL_UART_ErrorCallback(&huart7);
    assert(telem_rx_events == 1 && telem_errors == 1 && g_uarts[1].errors == chassis_errors);
    vision_has_target = true; vision_suspended = false; waiting_for_vision = false;
    dbg_vision_yaw_err = 1.25f; dbg_yaw_speed_rpm = -3.5f;
    telemetry_sample();
    assert(telem_samples == 1 && telem_last[0] == 1.25f && telem_last[5] == -3.5f);
    assert(((int)telem_last[2] & 1) && !((int)telem_last[2] & 2));
    /* Two-loop yaw (tune_yaw_two_loop): the chassis turns at 60 deg/s, the gimbal is on its
     * heading and the stage still -> counter-rotate at 60 deg/s = -10 rpm from gz alone. */
    track_stop(&g_track_yaw); track_stop(&g_track_pitch);
    vision_has_target = false;
    ++now_ms; receive_body(1, "C,1,900,0,1,0,0,0,0,0,60"); gimbal_poll_inputs();
    fresh(); g_imu.gyro[2] = 0.0f; g_axis_table[0].current_angle_deg = g_yaw_limit_origin;
    g_world_target_yaw = yaw_deg_from_imu();
    tune_yaw_two_loop = 1;
    const float rate_ki = tune_yaw_rate_ki;
    tune_yaw_rate_ki = 0.0f;                                   /* P structure first */
    gimbal_step_imu_execute();
    assert(dbg_chassis_gz_deg_s == 60.0f && abs(last_speed[0] + 1000) <= 2);
    tune_yaw_chassis_ff = 0.0f;
    gimbal_step_imu_execute(); assert(abs(last_speed[0]) <= 2);
    /* 1 deg behind, no chassis motion: w* = kpos * 1 = 8 deg/s, rpm = (8 + 7 * 8) / 6. */
    tune_yaw_chassis_ff = 1.0f;
    ++now_ms; receive_body(1, "C,1,901,0,1,0,0,0,0,0,0"); gimbal_poll_inputs();
    fresh(); g_world_target_yaw = wrap_deg(yaw_deg_from_imu() + 1.0f);
    gimbal_step_imu_execute();
    assert(abs(last_speed[0] - (int32_t)lroundf((8.0f + 7.0f * 8.0f) / 6.0f * 100.0f)) <= 3);
    /* 5 deg behind: kpos * 5 = 40 deg/s, capped by tune_yaw_pos_rate_max (10 here). */
    tune_yaw_pos_rate_max = 10.0f;
    fresh(); g_world_target_yaw = wrap_deg(yaw_deg_from_imu() + 5.0f);
    gimbal_step_imu_execute();
    assert(abs(last_speed[0] - (int32_t)lroundf((10.0f + 7.0f * 10.0f) / 6.0f * 100.0f)) <= 3);
    tune_yaw_pos_rate_max = 40.0f;
    /* Inner-loop integral: 1 deg behind with the stage still, the rate error stays 8 deg/s,
     * so 50 cycles add ki * 8 * 0.1 s = 32 deg/s (5.3 rpm); outside the 3 deg zone it holds,
     * and gimbal_reset_pid clears it. */
    tune_yaw_rate_ki = 40.0f;
    fresh(); g_world_target_yaw = wrap_deg(yaw_deg_from_imu() + 1.0f);
    for (int i = 0; i < 50; ++i) gimbal_step_imu_execute();
    assert(fabsf(g_yaw_rate_integral - 32.0f) < 0.5f);
    assert(abs(last_speed[0] - (int32_t)lroundf((8.0f + 7.0f * 8.0f + 32.0f) / 6.0f * 100.0f)) <= 3);
    g_world_target_yaw = wrap_deg(yaw_deg_from_imu() + 5.0f);
    gimbal_step_imu_execute();
    assert(fabsf(g_yaw_rate_integral - 32.0f) < 0.5f);
    gimbal_reset_pid();
    assert(g_yaw_rate_integral == 0.0f);
    /* gz lead, only while the chassis turns: gz 20 -> 40 deg/s in 20 ms = 1000 deg/s^2, half of
     * it after the low-pass -> 0.1 s lead adds 50 deg/s to the chassis feed-forward. */
    tune_yaw_rate_ki = 0.0f;
    tune_yaw_gz_lead_s = 0.1f;
    ++now_ms; receive_body(1, "C,1,904,1,1,0,0,0,0,0,20"); gimbal_poll_inputs();
    g_chassis_gz_rate = 0.0f;
    now_ms += 20; receive_body(1, "C,1,905,1,1,0,0,0,0,0,40"); gimbal_poll_inputs();
    assert(fabsf(g_chassis_gz_rate - 500.0f) < 1.0f);
    fresh(); g_imu.gyro[2] = 0.0f; g_world_target_yaw = yaw_deg_from_imu();
    gimbal_step_imu_execute();
    assert(fabsf(dbg_yaw_gz_lead - 50.0f) < 0.1f);
    assert(abs(last_speed[0] - (int32_t)lroundf(-(40.0f + 50.0f) / 6.0f * 100.0f)) <= 3);
    ++now_ms; receive_body(1, "C,1,906,0,1,0,0,0,0,0,40"); gimbal_poll_inputs();   /* straight */
    tune_yaw_gz_tail_ms = 0U;                                        /* no tail: off at once */
    fresh(); g_world_target_yaw = yaw_deg_from_imu();
    gimbal_step_imu_execute();
    assert(dbg_yaw_gz_lead == 0.0f && abs(last_speed[0] - (int32_t)lroundf(-40.0f / 6.0f * 100.0f)) <= 3);
    /* gztail: the lead stays on this long after the turn flag clears (gz is still falling
     * then); dgz/dt 1000 -> 0 deg/s^2 low-passed to 250 -> 25 deg/s. Off after the tail. */
    tune_yaw_gz_tail_ms = 30U;
    fresh(); g_world_target_yaw = yaw_deg_from_imu();
    gimbal_step_imu_execute();
    assert(fabsf(g_chassis_gz_rate - 250.0f) < 1.0f && fabsf(dbg_yaw_gz_lead - 25.0f) < 0.1f);
    /* (a control step every ms: the angle history stays dense for the lookback check below) */
    for (int i = 0; i < 20; ++i) { ++now_ms; fresh(); gimbal_step_imu_execute(); }
    receive_body(1, "C,1,907,0,1,0,0,0,0,0,40"); gimbal_poll_inputs();
    fresh(); g_world_target_yaw = yaw_deg_from_imu();
    gimbal_step_imu_execute();                                       /* 21 ms: still on */
    assert(fabsf(dbg_yaw_gz_lead - 0.1f * g_chassis_gz_rate) < 0.01f && dbg_yaw_gz_lead > 1.0f);
    for (int i = 0; i < 20; ++i) { ++now_ms; fresh(); gimbal_step_imu_execute(); }
    receive_body(1, "C,1,908,0,1,0,0,0,0,0,40"); gimbal_poll_inputs();
    fresh(); g_world_target_yaw = yaw_deg_from_imu();
    gimbal_step_imu_execute();                                       /* 41 ms: off */
    assert(g_chassis.online && g_chassis_gz_rate > 10.0f && dbg_yaw_gz_lead == 0.0f);
    assert(Gimbal_SetTune("gztail", 300.0f) == 0 && tune_yaw_gz_tail_ms == 300U);
    tune_yaw_gz_tail_ms = 150U;                                      /* default */
    tune_yaw_gz_lead_s = 0.0f;
    /* tsel 3 puts the lead and dgz/dt on channels 9/10. */
    tune_telem_select = 3U;
    telemetry_sample();
    assert(telem_last[8] == dbg_yaw_gz_lead && telem_last[9] == g_chassis_gz_rate);
    tune_telem_select = 0U;
    tune_yaw_rate_ki = rate_ki;
    tune_yaw_two_loop = 0;
    /* Vehicle-speed feed-forward (on by default): chassis 0.2 m/s from $C v2, target 0.48 m away
     * per $V v2, laser 90 deg left of the chassis axis -> 0.2/0.48 rad/s = 23.9 deg/s.
     * Defaults: feed-forward +1, Kp x2, forward = the calibrated motor angle (absolute encoder). */
    assert(tune_veh_ff == 1.0f && tune_yaw_kp_scale == 2.0f);
    assert(tune_yaw_fwd_deg == VEH_CHASSIS_FORWARD_DEG);
    track_stop(&g_track_yaw); track_stop(&g_track_pitch);
    ++now_ms; receive_body(1, "C,2,950,0,1,0,0,0,0,0,0,200"); gimbal_poll_inputs();
    assert(g_chassis.has_speed && fabsf(g_chassis.speed_m_s - 0.2f) < 1e-6f);
    ++now_ms; receive_body(0, "V,2,60,1,0,0,480,40"); gimbal_poll_inputs();
    assert(fabsf(g_vision_range_m - 0.48f) < 1e-6f && g_vision_has_age && g_vision_age_ms == 40);
    fresh(); g_imu.gyro[2] = 0.0f;
    g_axis_table[0].current_angle_deg = g_yaw_limit_origin;
    tune_yaw_fwd_deg = g_yaw_limit_origin - 90.0f;
    tune_veh_ff = 0.0f;
    gimbal_step_imu_execute();
    assert(dbg_veh_rate_deg_s == 0.0f);                                 /* switched off */
    tune_veh_ff = 1.0f;
    gimbal_step_imu_execute();
    assert(fabsf(dbg_veh_rate_deg_s - 0.2f / 0.48f * 57.29578f) < 0.05f);
    assert(fabsf(dbg_veh_phi_deg - 90.0f) < 1e-3f);
    /* vlead: 0.2 -> 0.25 m/s in 20 ms = 2.5 m/s^2, half after the low-pass; 0.08 s lead ->
     * +0.1 m/s in the feed-forward (0.3 m/s); bounded to +-0.2 m/s; tsel 4 shows rate and speed. */
    ++now_ms; receive_body(1, "C,2,951,0,1,0,0,0,0,0,0,200"); gimbal_poll_inputs();
    g_chassis_accel = 0.0f;
    now_ms += 20; receive_body(1, "C,2,952,0,1,0,0,0,0,0,0,250"); gimbal_poll_inputs();
    assert(fabsf(g_chassis_accel - 1.25f) < 1e-3f);
    tune_veh_speed_lead_s = 0.08f;
    fresh(); gimbal_step_imu_execute();
    assert(fabsf(g_veh_speed_used - 0.35f) < 1e-3f);
    assert(fabsf(dbg_veh_rate_deg_s - 0.35f / 0.48f * 57.29578f) < 0.05f);
    tune_telem_select = 4U; telemetry_sample();
    assert(telem_last[8] == g_veh_rate && fabsf(telem_last[9] - 35.0f) < 1e-3f);
    tune_telem_select = 0U;
    g_chassis_accel = 10.0f; fresh(); gimbal_step_imu_execute();
    assert(fabsf(g_veh_speed_used - (0.25f + VEH_SPEED_LEAD_MAX_M_S)) < 1e-3f);
    tune_veh_speed_lead_s = 0.05f; g_chassis_accel = 0.0f;               /* default */
    ++now_ms; receive_body(1, "C,2,953,0,1,0,0,0,0,0,0,200"); gimbal_poll_inputs();
    g_chassis_accel = 0.0f;
    tune_yaw_fwd_deg = g_yaw_limit_origin;                              /* looking ahead: none */
    gimbal_step_imu_execute(); assert(fabsf(dbg_veh_rate_deg_s) < 1e-3f);
    /* Elevation feed-forward (tune_veh_pitch_h, 0.182 m by default): driving toward the target
     * (phi 0), h 0.1 m: 0.1 * 0.2 / 0.48^2 rad/s = 4.97 deg/s; side-on (phi 90) none. */
    assert(tune_veh_pitch_h == -0.182f);
    tune_veh_pitch_h = 0.1f;
    gimbal_step_imu_execute();
    assert(fabsf(dbg_veh_pitch_rate_deg_s - 0.1f * 0.2f / (0.48f * 0.48f) * 57.29578f) < 0.05f);
    tune_yaw_fwd_deg = g_yaw_limit_origin - 90.0f;
    gimbal_step_imu_execute(); assert(fabsf(dbg_veh_pitch_rate_deg_s) < 1e-3f);
    tune_veh_pitch_h = -0.182f;
    /* No speed (version-1 chassis frame) -> no feed-forward. */
    tune_yaw_fwd_deg = g_yaw_limit_origin - 90.0f;
    ++now_ms; receive_body(1, "C,1,954,0,1,0,0,0,0,0,0"); gimbal_poll_inputs();
    gimbal_step_imu_execute(); assert(dbg_veh_rate_deg_s == 0.0f);
    /* With a track, the yaw command gets the vehicle rate through the feed-forward. */
    ++now_ms; receive_body(1, "C,2,955,0,1,0,0,0,0,0,0,200"); gimbal_poll_inputs();
    ++now_ms; fresh(); receive_body(0, "V,2,61,1,0,0,480,40"); gimbal_poll_inputs(); gimbal_control_cycle();
    g_imu.angle[0] = g_world_target_yaw * (float)M_PI / 180.0f;
    tune_track_ff = 0.0f; hold_still(2); int32_t no_ff = last_speed[0];
    tune_track_ff = 1.0f; hold_still(2); int32_t veh_ff = last_speed[0];
    assert(veh_ff - no_ff > (int32_t)(23.0f / 6.0f * (1.0f + g_yaw_ff_gain) * 100.0f * 0.8f));
    /* ... and the pitch command the elevation rate (toward the target, h 0.1 m: ~5 deg/s). */
    tune_yaw_fwd_deg = g_yaw_limit_origin;
    tune_veh_pitch_h = 0.0f; hold_still(2); int32_t no_pff = last_speed[1];
    tune_veh_pitch_h = 0.1f; hold_still(2); int32_t pff = last_speed[1];
    assert(abs(pff - no_pff) > (int32_t)(4.97f / 6.0f * 100.0f * 0.8f));
    tune_veh_pitch_h = -0.182f; tune_yaw_fwd_deg = g_yaw_limit_origin - 90.0f;
    /* Frame age replaces the fixed latency when enabled: 40 ms age + 25 ms extra = 65. */
    tune_vision_use_age = 1U; tune_vision_extra_ms = 25U;
    ++now_ms; fresh(); receive_body(0, "V,2,62,1,0,0,480,40"); gimbal_poll_inputs(); gimbal_control_cycle();
    assert(dbg_vision_lookback_ms >= 65U && dbg_vision_lookback_ms <= 69U);
    tune_vision_use_age = 0U; tune_vision_extra_ms = 40U; tune_veh_ff = 1.0f;
    tune_yaw_fwd_deg = VEH_CHASSIS_FORWARD_DEG;
    track_stop(&g_track_yaw); track_stop(&g_track_pitch);

    /* Live tuning over Bluetooth: by name, any case, range-checked; integers and switches. */
    assert(Gimbal_SetTune("beta", 0.3f) == 0 && fabsf(tune_track_beta - 0.3f) < 1e-6f);
    assert(Gimbal_SetTune("KPSCALE", 2.0f) == 0 && tune_yaw_kp_scale == 2.0f);
    assert(Gimbal_SetTune("latency", 95.4f) == 0 && tune_vision_latency_ms == 95U);
    assert(Gimbal_SetTune("twoloop", 1.0f) == 0 && tune_yaw_two_loop == 1U);
    assert(Gimbal_SetTune("beta", 1.5f) == -2 && fabsf(tune_track_beta - 0.3f) < 1e-6f);
    assert(Gimbal_SetTune("nosuch", 1.0f) == -1);
    assert(Gimbal_SetTune("alpha", NAN) == -2);
    assert(Gimbal_SetTune("vehff", -1.0f) == 0 && tune_veh_ff == -1.0f);
    assert(Gimbal_SetTune("useage", 1.0f) == 0 && tune_vision_use_age == 1U);
    assert(Gimbal_SetTune("settle", 250.0f) == 0 && tune_track_settle_ms == 250U);
    assert(Gimbal_SetTune("turnhold", 0.0f) == 0 && tune_turn_hold == 0U);
    assert(Gimbal_SetTune("pitchh", -0.2f) == 0 && tune_veh_pitch_h == -0.2f);
    tune_veh_pitch_h = -0.182f;
    tune_turn_hold = 1U;
    assert(Gimbal_SetTune("posmax", 2.0f) == -2 && tune_yaw_pos_rate_max == 40.0f);
    assert(Gimbal_SetTune("tsel", 4.0f) == 0 && tune_telem_select == 4U);   /* a select, not an on/off */
    assert(Gimbal_SetTune("tsel", 5.0f) == -2 && tune_telem_select == 4U);
    tune_telem_select = 0U;
    tune_track_settle_ms = 300U;
    g_axis_table[0].current_angle_deg = 123.5f;
    Gimbal_CaptureForward(); assert(tune_yaw_fwd_deg == 123.5f);
    tune_veh_ff = 1.0f; tune_vision_use_age = 0U; tune_yaw_fwd_deg = VEH_CHASSIS_FORWARD_DEG;
    {
        char text[160];
        const int n = Gimbal_FormatTunes(text, sizeof(text));
        assert(n > 0 && n < (int)sizeof(text) && strstr(text, "beta=0.3") && strstr(text, "twoloop=1"));
        assert(Gimbal_FormatTunes(text, 8) == 7 && strlen(text) == 7);     /* truncated, terminated */
    }
    {   /* The whole list fits the "#P" reply (telemetry.c REPLY_CAPACITY 384 - "#P " - newlines),
         * with room for a few more: the last entry must not be cut off. */
        char text[384 - 4 - 3];
        const int n = Gimbal_FormatTunes(text, sizeof(text));
        assert(n > 0 && n < 330 && strstr(text, " pkp=") && strstr(text, " gztail=150"));
    }
    tune_track_beta = 0.17f; tune_yaw_kp_scale = 2.0f; tune_vision_latency_ms = 80U; tune_yaw_two_loop = 1U;
    tune_turn_hold = 0U;
    {   /* $G to the chassis (power-up auto-start), one frame per 100 ms on the chassis UART:
         * CONTROL with the IMU and both motors up, VISION after GIMBAL_READY_LOCK_MS of valid frames
         * with |error| < GIMBAL_READY_ERR_DEG, READY = both. A large error or a gap drops VISION. */
        const unsigned control = GIMBAL_CAN_TX_ENABLE ? GIMBAL_STATUS_CONTROL : 0U;
        const unsigned all = GIMBAL_CAN_TX_ENABLE ?
            (GIMBAL_STATUS_READY | GIMBAL_STATUS_CONTROL | GIMBAL_STATUS_VISION) : GIMBAL_STATUS_VISION;
        unsigned vseq = 700, cseq = 960, sent;
        char body[64];
        now_ms += 1000;                         /* older sequence numbers time out */
        uart1_tx_count = 0;
        for (unsigned t = 0; t < 2400; t += 10) {
            now_ms += 10; fresh();
            if (t % 20 == 0) {
                snprintf(body, sizeof(body), "C,1,%u,0,1,0,0,0,0,0,0", cseq++); receive_body(1, body);
                snprintf(body, sizeof(body), "V,2,%u,1,0.200,-0.300,480,40", vseq++); receive_body(0, body);
            }
            gimbal_poll_inputs(); gimbal_control_cycle();
            if (t == 1500) assert(last_status_flags() == control);              /* locking */
        }
        assert(last_status_flags() == all);
        sent = uart1_tx_count;
        assert(sent >= 23 && sent <= 25);                                      /* 2.4 s / 100 ms */
        assert(!strncmp(uart1_tx, "$G,1,", 5) && uart1_tx[strlen(uart1_tx) - 1] == '\n');
        for (unsigned t = 0; t < 100; t += 10) {                               /* 1.5 deg off */
            now_ms += 10; fresh();
            if (t % 20 == 0) {
                snprintf(body, sizeof(body), "C,1,%u,0,1,0,0,0,0,0,0", cseq++); receive_body(1, body);
                snprintf(body, sizeof(body), "V,2,%u,1,1.500,0,480,40", vseq++); receive_body(0, body);
            }
            gimbal_poll_inputs(); gimbal_control_cycle();
        }
        assert(last_status_flags() == control && uart1_tx_count == sent + 1);
        for (unsigned t = 0; t < 2200; t += 10) {                              /* locked again */
            now_ms += 10; fresh();
            if (t % 20 == 0) {
                snprintf(body, sizeof(body), "C,1,%u,0,1,0,0,0,0,0,0", cseq++); receive_body(1, body);
                snprintf(body, sizeof(body), "V,2,%u,1,0,0,480,40", vseq++); receive_body(0, body);
            }
            gimbal_poll_inputs(); gimbal_control_cycle();
        }
        assert(last_status_flags() == all);
        for (unsigned t = 0; t < 400; t += 10) {                               /* no frames */
            now_ms += 10; fresh();
            if (t % 20 == 0) {
                snprintf(body, sizeof(body), "C,1,%u,0,1,0,0,0,0,0,0", cseq++); receive_body(1, body);
            }
            gimbal_poll_inputs(); gimbal_control_cycle();
        }
        assert(last_status_flags() == control);
    }
    {   /* $K to the K230 every 50 ms: the chassis task and run progress from $C v3 (0, 0 before
         * any); a version-2 frame leaves them as they were. */
        unsigned task = 9, progress = 9, sent, seq = 5000;
        char body[64];
        for (unsigned t = 0; t < 60; t += 10) { now_ms += 10; fresh(); gimbal_control_cycle(); }
        last_task(&task, &progress); assert(task == 0 && progress == 0);
        snprintf(body, sizeof(body), "C,3,%u,0,1,0,0,0,0,0,0,200,1,375", seq++); receive_body(1, body);
        gimbal_poll_inputs();
        sent = uart10_tx_count;
        for (unsigned t = 0; t < 100; t += 10) { now_ms += 10; fresh(); gimbal_control_cycle(); }
        assert(uart10_tx_count == sent + 2);
        last_task(&task, &progress); assert(task == 1 && progress == 375);
        assert(!strncmp(uart10_tx, "$K,1,", 5) && uart10_tx[strlen(uart10_tx) - 1] == '\n');
        snprintf(body, sizeof(body), "C,2,%u,0,1,0,0,0,0,0,0,200", seq++); receive_body(1, body);
        gimbal_poll_inputs();
        for (unsigned t = 0; t < 60; t += 10) { now_ms += 10; fresh(); gimbal_control_cycle(); }
        last_task(&task, &progress); assert(task == 1 && progress == 375);
    }
    {   /* A $C v3 frame in a turn is ~70 bytes: it must arrive in one receive event (the old 64-byte
         * buffer split every frame mid-line, bytes overran and whole frames were lost in turns). */
        const char *body = "C,3,6000,1,1,0.000,0.000,0.000,0.000,0.000,-104.567,140,1,1000";
        char frame[192];
        const int n = snprintf(frame, sizeof(frame), "$%s*%04X\n", body,
                               (unsigned)GimbalProtocol_Crc16(body, strlen(body)));
        assert(n > 64 && (size_t)n <= sizeof(g_uarts[1].rx));      /* 69 bytes */
        memcpy(g_uarts[1].rx, frame, (size_t)n);
        ++now_ms;
        HAL_UARTEx_RxEventCallback(g_uarts[1].uart, (uint16_t)n);
        gimbal_poll_inputs();
        assert(fabsf(g_chassis.gz_deg_s + 104.567f) < 1e-3f && g_task == 1 && g_task_progress == 1000);
    }
    puts("Gimbal integration tests passed: UART ISR, modes, timeouts, IMU readiness, CAN validation/failure, limits.");
    return 0;
}
