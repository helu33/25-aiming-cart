#include "app_chassis.h"
#include "command_line.h"
#include "chassis_config.h"
#include "board_config.h"
#include "bsp_line.h"
#include "bsp_gimbal_link.h"
#include "bsp_uart.h"
#include "app_motor.h"
#include "drive_test.h"
#include "autostart.h"
#include "mode_key.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static chassis_mission_t mission;
static uint8_t selected_laps=1, calibration_ok, last_imu_valid, motor_running;
static uint32_t last_scan, previous_left, previous_right;
static uint32_t last_imu_ms, last_status_ms;
static uint32_t last_rx_overflow, last_rx_byte_ms;
static command_line_t command_input;
/* DRIVE <cm/s> <cm>: straight test run (see drive_test.h); result printed from the poll loop. */
static drive_test_t drive;
static volatile uint8_t drive_finished;
/* Power-up auto-start (CHASSIS_AUTOSTART_LAPS); the body must be still (latest control cycle). */
static autostart_t autostart;
static float still_gz_rad_s=1e9f, still_speed_m_s=1e9f;
/* Mode key (CHASSIS_MODE_KEY_PIN): bullseye <-> circle before the power-up run, buzzer beeps. */
static mode_key_t mode_key;
static beeper_t beeper;
static uint8_t mode_key_down(void) {
    return (uint8_t)(HAL_GPIO_ReadPin(CHASSIS_MODE_KEY_PORT,CHASSIS_MODE_KEY_PIN)==CHASSIS_MODE_KEY_PRESSED);
}
/* Fresh scans whose state or mask changed while running, with the time since RUN in
 * centiseconds; frozen when the mission faults, kept after DONE. Printed by TRACE.
 * 32 changes cover a corner plus the settling on the next edge. */
#define TRACE_LEN 32U
#define TRACE_PER_LINE 8U
static uint8_t trace_state[TRACE_LEN], trace_mask[TRACE_LEN], trace_count, trace_next;
static uint16_t trace_cs[TRACE_LEN];
static int trace_turn_deg;

static void trace_reset(void) { trace_count=0; trace_next=0; trace_turn_deg=0; }
static void trace_push(uint8_t state, uint8_t mask, float elapsed_s) {
    if (trace_count) {
        const unsigned last=(trace_next+TRACE_LEN-1U)%TRACE_LEN;
        if (trace_state[last]==state && trace_mask[last]==mask) return;
    }
    trace_state[trace_next]=state; trace_mask[trace_next]=mask;
    trace_cs[trace_next]=(uint16_t)(elapsed_s*100.0f);
    trace_next=(uint8_t)((trace_next+1U)%TRACE_LEN);
    if (trace_count<TRACE_LEN) trace_count++;
}
/* Header plus lines of "state/mask@seconds", oldest first (~430 bytes for 32 entries,
 * within the 512-byte TX ring). */
static void trace_print(void) {
    char text[112]; int n; unsigned i;
    /* elapsed stops counting once the mission stops: after DONE it is the lap time. */
    const unsigned elapsed_cs=(unsigned)(mission.elapsed*100.0f+0.5f);
    n=snprintf(text,sizeof(text),"TRACE state=%u fault=%u corners=%u turn_deg=%d entries=%u run_s=%u.%02u\r\n",
               (unsigned)mission.state,(unsigned)mission.fault,(unsigned)mission.corners,
               trace_turn_deg,(unsigned)trace_count,elapsed_cs/100U,elapsed_cs%100U);
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
    for (i=0;i<trace_count;i+=TRACE_PER_LINE) {
        unsigned j;
        n=0;
        for (j=i;j<trace_count && j<i+TRACE_PER_LINE;j++) {
            const unsigned k=(trace_next+TRACE_LEN-trace_count+j)%TRACE_LEN;
            n+=snprintf(text+n,sizeof(text)-(size_t)n," %u/%02X@%u.%02u",(unsigned)trace_state[k],
                        (unsigned)trace_mask[k],(unsigned)(trace_cs[k]/100U),(unsigned)(trace_cs[k]%100U));
        }
        if (n>0 && n<(int)sizeof(text)-2) { text[n++]='\r'; text[n++]='\n'; (void)uart_bsp_write((uint8_t *)text,(uint16_t)n); }
    }
}

const chassis_mission_t *app_chassis_state(void) { return &mission; }
static void status(void) {
    uint8_t mask; uint32_t seq;
    uint8_t line_ok=line_bsp_snapshot(&mask,&seq);
    char text[256]; int n=snprintf(text,sizeof(text),
        "AUTO armed=%u state=%u fault=%u laps=%u corners=%u mask=%02X fresh=%u scan=%lu tx_fail=%lu"
        " gimbal_rx=%lu gimbal_flags=%u gimbal_ready=%u autostart=%u circle=%u key=%u\r\n",
        (unsigned)CHASSIS_HARDWARE_CONFIRMED,(unsigned)mission.state,(unsigned)mission.fault,
        (unsigned)selected_laps,(unsigned)mission.corners,(unsigned)mask,(unsigned)line_ok,(unsigned long)seq,
        (unsigned long)gimbal_link_failures(),(unsigned long)gimbal_link_status_frames(),
        (unsigned)gimbal_link_status_flags(),(unsigned)gimbal_link_ready(),(unsigned)autostart_armed(&autostart),
        (unsigned)(mission.cfg.circle>=0.5f),(unsigned)mode_key_down());
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
    /* Hardware health: IMU found (WHO_AM_I 38), boot calibration, latest read, encoder counts. */
    n=snprintf(text,sizeof(text),
        "HW imu_found=%u who_am_i=%02X calib=%u imu_read=%u link_ok=%u enc_l=%ld enc_r=%ld rx_bytes=%lu rx_err=%lu"
        " cruise_mm_s=%d turn_mm_s=%d fw=%s %s\r\n",
        (unsigned)imu_bsp_ready_get(),(unsigned)imu_bsp_who_am_i_get(),(unsigned)calibration_ok,
        (unsigned)last_imu_valid,(unsigned)gimbal_link_healthy(),
        (long)encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT),(long)encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT),
        (unsigned long)uart_bsp_rx_total_get(),(unsigned long)uart_bsp_rx_error_count_get(),
        (int)(CHASSIS_CRUISE_M_S*1000.0f+0.5f),(int)(mission.cfg.turn_m_s*1000.0f+0.5f),
        __DATE__,__TIME__);  /* which build is flashed */
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
    if (mission.state==MISSION_FAULT && trace_count) (void)uart_bsp_write_str("FAULT: send TRACE for the last changes\r\n");
}
void app_chassis_stop(void) { mission_stop(&mission); drive_test_stop(&drive); app_motor_stop(); motor_running=0; }
/* LOG ON: one integer CSV line every LOG_EVERY control cycles (10 ms each) while the
 * mission runs, plus the cycle where it stops. ~80 bytes at 50 Hz on the 115200 debug
 * UART (Bluetooth module must match). Parsed by host/plot_run_log.py. */
#define LOG_EVERY 2U
#define LOG_NO_ERROR (-9999)
static uint8_t log_enabled;
static unsigned log_divider;
static int log_round(float value) { return (int)(value + (value >= 0.0f ? 0.5f : -0.5f)); }
static void log_header(void) {
    (void)uart_bsp_write_str("H,t_ms,state,mask,err_raw_dmm,err_f_dmm,v_mm_s,vmeas_mm_s,w_mrad_s,"
                             "gz_mrad_s,hdg_mrad,l_tgt,l_meas,r_tgt,r_meas,duty_l,duty_r\r\n");
}
static void log_frame(const mission_input_t *in) {
    char text[128]; float raw; int n;
    const int err_raw = mission_line_error(in->line_mask,CHASSIS_PROBE_SPACING_MM,&raw) ?
        log_round(raw*10.0f) : LOG_NO_ERROR;
    const float heading_error = mission_heading_error(&mission,mission.corners);
    n=snprintf(text,sizeof(text),"D,%ld,%u,%02X,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
        (long)log_round(mission.elapsed*1000.0f),(unsigned)mission.state,(unsigned)in->line_mask,
        err_raw,log_round(mission.line_error*10.0f),
        log_round(mission.v*1000.0f),log_round(mission.v_meas*1000.0f),
        log_round(mission.w*1000.0f),log_round(in->gyro_z_rad_s*1000.0f),log_round(heading_error*1000.0f),
        log_round(app_motor_speed_target_m_s_get(ENCODER_SIDE_LEFT)*1000.0f),
        log_round(app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_LEFT)*1000.0f),
        log_round(app_motor_speed_target_m_s_get(ENCODER_SIDE_RIGHT)*1000.0f),
        log_round(app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_RIGHT)*1000.0f),
        log_round(app_motor_last_duty_percent_get(ENCODER_SIDE_LEFT)),
        log_round(app_motor_last_duty_percent_get(ENCODER_SIDE_RIGHT)));
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
}

/* NULL when RUN may start; otherwise the first failed condition, printed on rejection. */
static const char *run_blocker(uint8_t *mask, uint32_t *seq) {
    float error;
    if (!CHASSIS_HARDWARE_CONFIRMED) return "not_armed(CHASSIS_HARDWARE_CONFIRMED=0)";
    if (mission.state==MISSION_FAULT) return "fault_latched(send STOP first)";
    if (mission_running(&mission)) return "already_running";
    if (!imu_bsp_ready_get()) return "imu_not_found(check I2C2 PB10/PB11 wiring or IMU)";
    if (!calibration_ok) return "imu_calibration_failed(power-cycle and keep still 2 s)";
    if (!last_imu_valid) return "imu_read_failed(I2C error after boot)";
    if ((uint32_t)(HAL_GetTick()-last_imu_ms)>30) return "imu_stale";
    if (!line_bsp_snapshot(mask,seq)) return "line_sensor_stale";
    if (!mission_line_error(*mask,CHASSIS_PROBE_SPACING_MM,&error)) return "line_not_normal(need 1-3 adjacent probes)";
    if (!gimbal_link_healthy()) return "gimbal_uart4_tx_not_completing";
    return NULL;
}
/* NULL when DRIVE may start: like RUN, but no line or gimbal link needed. */
static const char *drive_blocker(void) {
    if (!CHASSIS_HARDWARE_CONFIRMED) return "not_armed(CHASSIS_HARDWARE_CONFIRMED=0)";
    if (mission.state==MISSION_FAULT) return "fault_latched(send STOP first)";
    if (mission_running(&mission) || drive.active) return "already_running";
    if (!imu_bsp_ready_get() || !calibration_ok || !last_imu_valid) return "imu_not_ready";
    if ((uint32_t)(HAL_GetTick()-last_imu_ms)>30) return "imu_stale";
    return NULL;
}
/* "DRIVE <cm/s> <cm>", e.g. DRIVE 20 100 = 0.20 m/s for 1.00 m, then stop. */
static void drive_command(const char *args) {
    char *end; char text[96]; int n;
    const long speed_cm_s=strtol(args,&end,10);
    const long distance_cm=strtol(end,&end,10);
    const char *blocker=drive_blocker();
    if (*end) blocker="usage(DRIVE <cm/s> <cm>)";
    if (!blocker && !drive_test_start(&drive,(float)speed_cm_s*0.01f,(float)distance_cm*0.01f,
                                      CHASSIS_ACCEL_M_S2,CHASSIS_DECEL_M_S2,CHASSIS_LINE_KH))
        blocker="limits(1..50 cm/s, 1..300 cm)";
    if (blocker) {
        n=snprintf(text,sizeof(text),"AUTO DRIVE REJECTED reason=%s\r\n",blocker);
    } else {
        drive_finished=0; app_motor_pid_reset(); motor_running=1;
        n=snprintf(text,sizeof(text),"AUTO DRIVE START v_mm_s=%ld d_mm=%ld\r\n",speed_cm_s*10L,distance_cm*10L);
    }
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
}
/* TURN <cm/s>: corner speed for the next runs (RAM only; power-up uses CHASSIS_TURN_M_S).
 * The turn rate is speed / CHASSIS_RADIUS_M, which the gimbal yaw motor has to cancel. */
static void turn_command(const char *args) {
    char *end; char text[80]; int n;
    const long speed_cm_s=strtol(args,&end,10);
    if (*end || !mission_set_turn_speed(&mission,(float)speed_cm_s*0.01f))
        n=snprintf(text,sizeof(text),"AUTO TURN REJECTED reason=%s\r\n",
                   mission_running(&mission) ? "running" : "limits(6 cm/s..cruise)");
    else
        n=snprintf(text,sizeof(text),"AUTO TURN turn_mm_s=%ld rate_mrad_s=%ld\r\n",speed_cm_s*10L,
                   (long)(mission.cfg.turn_m_s/mission.cfg.radius_m*1000.0f+0.5f));
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
}
/* CIRCLE <s>: static circle test while stopped - the car stays, the circle task progress
 * sweeps 0 -> 1 lap in <s> seconds (1..120), so the laser draws the circle on the paper.
 * CIRCLE 0 ends it; RUN or DRIVE also do. */
static uint32_t circle_test_start_ms, circle_test_ms;
static void circle_command(const char *args) {
    char *end; char text[80]; int n;
    const long seconds=strtol(args,&end,10);
    if (*end || seconds<0 || seconds>120 || mission_running(&mission) || drive.active) {
        n=snprintf(text,sizeof(text),"AUTO CIRCLE REJECTED reason=%s\r\n",
                   *end || seconds<0 || seconds>120 ? "limits(0..120 s)" : "running");
    } else {
        circle_test_ms=(uint32_t)seconds*1000U; circle_test_start_ms=HAL_GetTick();
        n=snprintf(text,sizeof(text),"AUTO CIRCLE s=%ld\r\n",seconds);
    }
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
}
/* SET <name> <value>: tune the mission while stopped (RAM only, see mission_set_param);
 * PARAMS: print the current values. Both answer one "AUTO ..." line. */
static void params_report(const char *head) {
    char text[200]; int n=snprintf(text,sizeof(text),"AUTO %s ",head);
    if (n<0 || n>=(int)sizeof(text)-2) return;
    n+=mission_format_params(&mission,text+n,(unsigned)(sizeof(text)-(unsigned)n-2));
    text[n++]='\r'; text[n++]='\n';
    (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
}
static void set_command(const char *args) {
    char name[12]; char *end; unsigned k=0; char text[80]; int n;
    while (*args && *args!=' ' && k<sizeof(name)-1) name[k++]=*args++;
    name[k]=0;
    const float value=(float)strtod(args,&end);
    if (end==args || *end || !mission_set_param(&mission,name,value)) {
        n=snprintf(text,sizeof(text),"AUTO SET REJECTED %s reason=%s\r\n",name,
                   mission_running(&mission) ? "running" : "name/range");
        if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
        return;
    }
    params_report("SET");
}
static void drive_report(void) {
    char text[96];
    const int n=snprintf(text,sizeof(text),"AUTO DRIVE DONE d_mm=%d t_ms=%d heading_mdeg=%d timeout=%u\r\n",
        (int)(drive.travelled*1000.0f),(int)(drive.elapsed*1000.0f),
        (int)(drive.heading*57295.78f),(unsigned)drive.timed_out);
    if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
}
uint8_t app_chassis_start(uint8_t laps) {
    uint8_t mask=0; uint32_t seq=0;
    const char *blocker=run_blocker(&mask,&seq);
    if (blocker) {
        char text[128];
        const int n=snprintf(text,sizeof(text),"AUTO RUN REJECTED reason=%s mask=%02X\r\n",blocker,(unsigned)mask);
        if (n>0 && n<(int)sizeof(text)) (void)uart_bsp_write((uint8_t *)text,(uint16_t)n);
        return 0;
    }
    if (drive.active) { (void)uart_bsp_write_str("AUTO RUN REJECTED reason=drive_running\r\n"); return 0; }
    if (!mission_start(&mission,laps)) {
        (void)uart_bsp_write_str("AUTO RUN REJECTED reason=bad_config(check chassis_config.h limits)\r\n");
        return 0;
    }
    previous_left=(uint32_t)encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    previous_right=(uint32_t)encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
    last_scan=seq; trace_reset(); app_motor_pid_reset(); motor_running=1;
    if (log_enabled) { log_divider=0; log_header(); } /* each run starts with its own header */
    return 1;
}
void app_chassis_init(uint8_t calibrated) {
    const mission_config_t cfg={BOARD_WHEEL_SEPARATION_M,CHASSIS_SENSOR_AHEAD_M,
        CHASSIS_PROBE_SPACING_MM,CHASSIS_RADIUS_M,CHASSIS_CRUISE_M_S,CHASSIS_TURN_M_S,
        CHASSIS_ACCEL_M_S2,CHASSIS_DECEL_M_S2,CHASSIS_ANGULAR_ACCEL_RAD_S2,
        CHASSIS_LINE_KP,CHASSIS_LINE_KD,0.45f,1.5f,CHASSIS_LINE_KH,CHASSIS_EDGE_M,
        CHASSIS_FIRST_EDGE_M_S,CHASSIS_LINE_ANGULAR_ACCEL_RAD_S2,CHASSIS_YAW_RATE_KG,CHASSIS_CIRCLE};
    GPIO_InitTypeDef key_pin={0};
    mission_init(&mission,&cfg); calibration_ok=calibrated; last_imu_valid=0;
    autostart_init(&autostart,(uint8_t)(CHASSIS_AUTOSTART_LAPS>0U && calibrated),CHASSIS_AUTOSTART_HOLD_MS,
                   CHASSIS_AUTOSTART_KEY_HOLD_MS);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    key_pin.Pin=CHASSIS_MODE_KEY_PIN; key_pin.Mode=GPIO_MODE_INPUT; key_pin.Pull=CHASSIS_MODE_KEY_PULL;
    HAL_GPIO_Init(CHASSIS_MODE_KEY_PORT,&key_pin);
    mode_key_init(&mode_key,mode_key_down(),HAL_GetTick()); beeper_init(&beeper);
    motor_running=0; selected_laps=1; command_line_init(&command_input);
    last_rx_overflow=uart_bsp_rx_overflow_count_get();
    line_bsp_init(); gimbal_link_init(); app_motor_stop();
    previous_left=(uint32_t)encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    previous_right=(uint32_t)encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
    (void)uart_bsp_write_str("AUTO: STOP | RUN | LAPS 1..5 | DRIVE <cm/s> <cm> | TURN <cm/s> | SET <name> <value> | PARAMS | STATUS | TRACE | LOG ON/OFF. Wiring/geometry must be confirmed.\r\n");
}
/* Commands that take manual control (cancel the power-up run). Bytes that are not a command do
 * not count: at power-up the Bluetooth bridge sent 23 bytes of noise, which cancelled it. */
static uint8_t known_command(const char *c) {
    return !strcmp(c,"STOP") || !strcmp(c,"RUN") || !strncmp(c,"LAPS ",5) || !strcmp(c,"LOG ON") ||
           !strcmp(c,"LOG OFF") || !strncmp(c,"DRIVE ",6) || !strncmp(c,"SET ",4) || !strcmp(c,"PARAMS") ||
           !strncmp(c,"TURN ",5) || !strcmp(c,"TRACE") || !strcmp(c,"STATUS") || !strncmp(c,"CIRCLE ",7);
}
static void execute(void) {
    const char *command=command_input.text;
    if (known_command(command)) autostart_cancel(&autostart);
    if (!strcmp(command,"STOP")) app_chassis_stop();
    else if (!strcmp(command,"RUN")) {
        (void)app_chassis_start(selected_laps); /* prints its own rejection reason */
    } else if (!strncmp(command,"LAPS ",5) && command[5]>='1' && command[5]<='5' && !command[6]) {
        if (!mission_running(&mission)) selected_laps=(uint8_t)(command[5]-'0');
    } else if (!strcmp(command,"LOG ON")) {
        log_enabled=1; log_divider=0;
        (void)uart_bsp_write_str("AUTO LOG ON (50 Hz while running)\r\n");
        log_header();
        return;
    } else if (!strcmp(command,"LOG OFF")) {
        log_enabled=0;
        (void)uart_bsp_write_str("AUTO LOG OFF\r\n");
        return;
    } else if (!strncmp(command,"DRIVE ",6)) {
        drive_command(command+6);
        return;
    } else if (!strncmp(command,"SET ",4)) {
        set_command(command+4);
        return;
    } else if (!strcmp(command,"PARAMS")) {
        params_report("PARAMS");
        return;
    } else if (!strncmp(command,"TURN ",5)) {
        turn_command(command+5);
        return;
    } else if (!strncmp(command,"CIRCLE ",7)) {
        circle_command(command+7);
        return;
    } else if (!strcmp(command,"TRACE")) {
        trace_print(); /* no STATUS after it: keep the TX ring for the trace lines */
        return;
    } else if (strcmp(command,"STATUS")) (void)uart_bsp_write_str("AUTO UNKNOWN COMMAND\r\n");
    status();
}
void app_chassis_poll(void) {
    uint8_t byte, keys; unsigned budget=32;
    line_bsp_poll();
    if (last_rx_overflow!=uart_bsp_rx_overflow_count_get()) {
        last_rx_overflow=uart_bsp_rx_overflow_count_get();
        uart_bsp_rx_flush(); command_line_init(&command_input); command_input.overflow=1;
        mission_fail(&mission,MISSION_COMMAND); app_motor_stop(); motor_running=0;
    }
    keys=line_bsp_key_events();
    if (keys&4) app_chassis_stop();
    else {
        if ((keys&1) && !mission_running(&mission)) selected_laps=(selected_laps%5)+1;
        if (keys&2) (void)app_chassis_start(selected_laps);
    }
    if (mode_key_step(&mode_key,mode_key_down(),HAL_GetTick()) && autostart_press(&autostart,HAL_GetTick())) {
        /* Before the power-up run (armed, so nothing runs): switch, 1 beep bullseye, 2 circle. */
        const uint8_t circle=(uint8_t)(mission.cfg.circle<0.5f);
        (void)mission_set_param(&mission,"circle",circle?1.0f:0.0f);
        beeper_start(&beeper,circle?2U:1U,HAL_GetTick());
        (void)uart_bsp_write_str(circle?"AUTO MODE circle (key)\r\n":"AUTO MODE bullseye (key)\r\n");
    }
    /* Bounded parsing: serial input must not starve scanning/control. */
    while (budget-- && uart_bsp_read_byte(&byte)) {
        const command_event_t event=command_line_feed(&command_input,byte);
        last_rx_byte_ms=HAL_GetTick();
        if (event==COMMAND_READY) execute();
        else if (event==COMMAND_OVERFLOW) (void)uart_bsp_write_str("AUTO LINE TOO LONG (ignored)\r\n");
    }
    {   /* Apps that send no CR/LF: a 150 ms gap ends the pending command. */
        const command_event_t event=command_line_idle(&command_input,HAL_GetTick()-last_rx_byte_ms);
        if (event==COMMAND_READY) execute();
        else if (event==COMMAND_OVERFLOW) (void)uart_bsp_write_str("AUTO LINE TOO LONG (ignored)\r\n");
    }
    gimbal_link_rx_poll();
    {   /* Power-up run: once the gimbal reports ready ($G) and RUN could start, standing still,
         * both for CHASSIS_AUTOSTART_HOLD_MS. A command before that cancels it (execute). */
        uint8_t mask=0; uint32_t seq=0;
        const uint8_t chassis_ready=autostart_armed(&autostart) && !run_blocker(&mask,&seq) && !drive.active &&
            fabsf(still_gz_rad_s)<CHASSIS_AUTOSTART_STILL_RAD_S && fabsf(still_speed_m_s)<CHASSIS_AUTOSTART_STILL_M_S;
        if (autostart_step(&autostart,HAL_GetTick(),gimbal_link_ready(),chassis_ready)) {
            (void)uart_bsp_write_str(mission.cfg.circle>=0.5f ? "AUTO START (power-up, gimbal ready, circle)\r\n" :
                                     "AUTO START (power-up, gimbal ready, bullseye)\r\n");
            (void)app_chassis_start(CHASSIS_AUTOSTART_LAPS);   /* prints its own rejection reason */
        }
    }
    gimbal_link_poll(mission_hold_vision(&mission));
    if (drive_finished) { drive_finished=0; drive_report(); }
    if ((uint32_t)(HAL_GetTick()-last_status_ms)>=1000) { last_status_ms=HAL_GetTick(); status(); }
    /* Existing RGB outputs: red fault, green running, blue idle/done. */
    HAL_GPIO_WritePin(RGB_R_GPIO_Port,RGB_R_Pin,mission.state==MISSION_FAULT?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_G_GPIO_Port,RGB_G_Pin,mission_running(&mission)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_B_GPIO_Port,RGB_B_Pin,!mission_running(&mission)&&mission.state!=MISSION_FAULT?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,beeper_on(&beeper,HAL_GetTick())?GPIO_PIN_SET:GPIO_PIN_RESET);
}
void app_chassis_control(float dt, float pid_dt, const imu_sample_t *imu, uint8_t valid) {
    mission_input_t in={0}; uint32_t seq;
    uint32_t left=(uint32_t)encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    uint32_t right=(uint32_t)encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
    in.left_delta_m=(float)(int32_t)(left-previous_left)*BOARD_METERS_PER_COUNT_LEFT;
    in.right_delta_m=(float)(int32_t)(right-previous_right)*BOARD_METERS_PER_COUNT_RIGHT;
    previous_left=left; previous_right=right;
    in.dt=dt; in.imu_valid=valid && calibration_ok; in.gyro_z_rad_s=imu->gyro_rad_s[2];
    last_imu_valid=in.imu_valid; last_imu_ms=HAL_GetTick();
    gimbal_link_set_gyro(in.imu_valid,in.gyro_z_rad_s*57.29578f);   /* for the gimbal yaw */
    if (dt>0.0f) {   /* encoder forward speed, 30 ms low-pass: gimbal vehicle-speed feed-forward */
        static float speed_m_s;
        const float raw=0.5f*(in.left_delta_m+in.right_delta_m)/dt;
        speed_m_s+=dt/(0.03f+dt)*(raw-speed_m_s);
        gimbal_link_set_speed(speed_m_s);
        still_speed_m_s=speed_m_s;
    }
    still_gz_rad_s=in.imu_valid?in.gyro_z_rad_s:1e9f;
    in.line_valid=line_bsp_snapshot(&in.line_mask,&seq); in.new_line=seq!=last_scan; last_scan=seq;
    {
        const uint8_t was_running=mission_running(&mission);
        const mission_state_t before=mission.state;
        if (was_running && !gimbal_link_healthy()) mission_fail(&mission,MISSION_LINK);
        mission_step(&mission,&in);
        if (was_running && in.new_line) trace_push((uint8_t)before,in.line_mask,mission.elapsed);
        if (was_running && mission.state==MISSION_FAULT)
            trace_turn_deg=(int)(mission.turn_angle*57.29578f);
    }
    /* Circle task: the gimbal forwards these to the K230, which moves its aim point around the
     * circle by the run progress (one circle per lap). */
    if (circle_test_ms && (mission_running(&mission) || drive.active)) circle_test_ms=0;
    if (circle_test_ms) {                       /* CIRCLE <s>: sweep while standing */
        const uint32_t elapsed=HAL_GetTick()-circle_test_start_ms;
        gimbal_link_set_task(1U,elapsed>=circle_test_ms ? 1000U :
                             (uint32_t)((uint64_t)elapsed*1000U/circle_test_ms));
    } else {
        gimbal_link_set_task((uint8_t)(mission.cfg.circle>=0.5f),
                             (uint32_t)(mission_run_progress(&mission)*1000.0f+0.5f));
    }
    if (mission_running(&mission)) {
        app_motor_set_chassis_velocity(mission.v,
            mission_wheel_w(&mission,in.gyro_z_rad_s,in.imu_valid,mission.cfg.yaw_rate_kg));
        app_motor_speed_control_update(pid_dt); motor_running=1;
        if (log_enabled && (++log_divider % LOG_EVERY)==0U) log_frame(&in);
    } else if (drive.active) {
        drive_test_step(&drive,dt,in.left_delta_m,in.right_delta_m,in.gyro_z_rad_s,in.imu_valid);
        app_motor_set_chassis_velocity(drive.v,drive.w);
        app_motor_speed_control_update(pid_dt); motor_running=1;
        if (!drive.active) drive_finished=1;
    } else if (motor_running) {
        if (log_enabled) log_frame(&in); /* the stop cycle: DONE or FAULT */
        app_motor_stop(); motor_running=0;
    }
    /* State transition frame is submitted immediately, in addition to 50 Hz heartbeat. */
    gimbal_link_poll(mission_hold_vision(&mission));
}
