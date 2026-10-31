#ifndef CHASSIS_MISSION_H
#define CHASSIS_MISSION_H
#include <stdint.h>

/* Pure C: metres, radians, seconds. Sensor bit 0 is the leftmost probe. */
typedef enum { MISSION_IDLE, MISSION_LINE, MISSION_PRE_TURN, MISSION_TURN,
               MISSION_REACQUIRE, MISSION_FINISH, MISSION_DONE, MISSION_FAULT } mission_state_t;
typedef enum { MISSION_OK, MISSION_BAD_CONFIG, MISSION_SENSOR, MISSION_LATE,
               MISSION_LOST_LINE, MISSION_TURN_TIMEOUT, MISSION_STALL,
               MISSION_ROUTE, MISSION_LINK, MISSION_COMMAND } mission_fault_t;
typedef struct {
    float wheelbase_m, sensor_ahead_m, probe_spacing_mm, radius_m;
    float cruise_m_s, turn_m_s, acceleration, deceleration, angular_acceleration;
    float line_kp, line_kd, min_edge_m, max_edge_m;
    float line_kh; /* rad/s per rad of heading error against the current edge */
    float edge_m;  /* nominal corner-trigger spacing; 0 disables early slow-down */
    float first_edge_m_s; /* speed cap until corner 1 (its position is unknown); 0 = off */
    /* Appended (older initialisers leave them 0): */
    float line_angular_acceleration; /* rad/s^2 for w on the straight states; 0 = angular_acceleration */
    float yaw_rate_kg;    /* gyro yaw-rate loop gain for mission_wheel_w (the app passes it) */
    float circle;         /* 1: the laser draws one circle per lap ($C v3 task), 0: bullseye */
} mission_config_t;
typedef struct {
    float dt, left_delta_m, right_delta_m, gyro_z_rad_s;
    uint8_t line_mask, line_valid, imu_valid, new_line;
} mission_input_t;
typedef struct {
    mission_config_t cfg;
    mission_state_t state;
    mission_fault_t fault;
    uint8_t laps, corners, corner_confirm, center_confirm;
    float v, w, elapsed, state_time, distance, state_distance, edge_distance;
    float first_trigger, last_trigger, edge_sum, finish_distance, turn_angle;
    float line_error, derivative, lost_time, left_stall, right_stall, imu_invalid_time;
    float heading; /* integrated yaw since start; edge k runs at k*90 deg */
    float heading_trim; /* reference correction learnt while the line is centred */
    float v_meas;  /* low-passed encoder forward speed */
    uint8_t error_initialized;
} chassis_mission_t;

void mission_init(chassis_mission_t *m, const mission_config_t *cfg);
uint8_t mission_start(chassis_mission_t *m, uint8_t laps);
void mission_stop(chassis_mission_t *m);
void mission_fail(chassis_mission_t *m, mission_fault_t fault);
/* Corner speed (m/s) for the next runs; the turn rate is turn_m_s / radius_m. Only while
 * stopped, within MISSION_TURN_MIN_M_S..cruise and the pre-turn geometry check of
 * mission_start. Returns 0 (and changes nothing) otherwise. */
#define MISSION_TURN_MIN_M_S 0.06f
uint8_t mission_set_turn_speed(chassis_mission_t *m, float turn_m_s);
/* Runtime tuning while stopped (SET <name> <value> on the command port, RAM only):
 * kg, kp, kd, kh, accel, decel, wacc (straights), tacc (turns), turn, cruise, edge (nominal
 * trigger-to-trigger spacing until two corners are measured), circle. Returns 0 for
 * an unknown name, a value out of range, a broken cross-check (turn <= cruise, ...) or while
 * running; nothing changes then. */
uint8_t mission_set_param(chassis_mission_t *m, const char *name, float value);
/* Prints "name=value ..." of all tunables into text (cap bytes); returns the length. */
int mission_format_params(const chassis_mission_t *m, char *text, unsigned cap);
void mission_step(chassis_mission_t *m, const mission_input_t *in);
/* Run progress in laps: distance over 4 x the trigger-to-trigger edge (nominal until two
 * corners are measured), exact once the finish distance is known, `laps` at DONE; 0 when
 * idle. The circle task draws one circle per lap from it. */
float mission_run_progress(const chassis_mission_t *m);
uint8_t mission_running(const chassis_mission_t *m);
uint8_t mission_hold_vision(const chassis_mission_t *m);
/* Body yaw rate for the wheel differential. On the straight states (LINE, PRE_TURN, FINISH)
 * it adds kg * (w - gyro), clamped, so the body turns as commanded even while the wheel loops
 * lag or the wheels differ (Tests/exit_yaw_sim.py); turns keep the rate they were tuned with.
 * Returns m->w when the gyro is not usable. */
float mission_wheel_w(const chassis_mission_t *m, float gyro_z_rad_s, uint8_t gyro_valid, float kg);
/* 1: contiguous normal line (1..3 probes), 0: ambiguous/no line. */
uint8_t mission_line_error(uint8_t mask, float spacing_mm, float *error_mm);
/* Heading minus the direction of edge `edge` (edge*90 deg + learnt trim), radians. */
float mission_heading_error(const chassis_mission_t *m, unsigned edge);
#endif
