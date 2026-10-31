#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 1: standalone mission; 0: retain the original USART2 velocity-command mode. */
#ifndef CHASSIS_AUTONOMOUS_ENABLE
#define CHASSIS_AUTONOMOUS_ENABLE 1
#endif
/* Autonomous debug/commands: 5 = H4 ESP32 (PC12 TX / PD2 RX), 2 = PA2/PA3.
 * Original external-command mode always keeps USART2. */
#ifndef CHASSIS_DEBUG_UART
#define CHASSIS_DEBUG_UART 5
#endif
#if CHASSIS_DEBUG_UART != 2 && CHASSIS_DEBUG_UART != 5
#error CHASSIS_DEBUG_UART must be 2 or 5
#endif
/* Gimbal link ($C frames, 921600): 2 = USART2 PA2 TX, where the wire to the gimbal is
 * (found 2026-10-22: the firmware sent on UART4 PC10 and the gimbal never received a byte);
 * 4 = UART4 PC10 TX. USART2 is only free in the mission with the debug port on UART5. */
#ifndef CHASSIS_GIMBAL_UART
#if CHASSIS_AUTONOMOUS_ENABLE && CHASSIS_DEBUG_UART == 5
#define CHASSIS_GIMBAL_UART 2
#else
#define CHASSIS_GIMBAL_UART 4
#endif
#endif
#if CHASSIS_GIMBAL_UART != 2 && CHASSIS_GIMBAL_UART != 4
#error CHASSIS_GIMBAL_UART must be 2 or 4
#endif
#if CHASSIS_GIMBAL_UART == 2 && !(CHASSIS_AUTONOMOUS_ENABLE && CHASSIS_DEBUG_UART == 5)
#error USART2 is the debug/command port in this configuration: put the gimbal link on UART4
#endif
/* Set to 1 only after wiring, voltage, direction and geometry are checked. */
#ifndef CHASSIS_HARDWARE_CONFIRMED
#define CHASSIS_HARDWARE_CONFIRMED 1
#endif
/* Power-up run: after boot, once the gimbal reports ready ($G: IMU warmed up, motors up, vision
 * locked 2 s) and RUN's own checks pass (IMU calibrated, line under the probes, link), with the
 * car still, for CHASSIS_AUTOSTART_HOLD_MS, run this many laps once, aiming at the bullseye or
 * drawing the circle (mode key below). A command on the debug port before that cancels it.
 * 0 = off (RUN only). */
#define CHASSIS_AUTOSTART_LAPS 1U
/* Task 3: while the car runs, the laser draws a 60 mm circle around the bullseye, one circle
 * per lap, synchronised by the run progress ($C v3 -> gimbal -> K230). 0 = aim at the
 * bullseye. Power-up default; the mode key or SET circle 0/1 changes it in RAM. */
#define CHASSIS_CIRCLE 0.0f
/* Mode key (PB5, the key on the chassis board): before the power-up run each press switches
 * bullseye <-> circle and beeps on the chassis buzzer, once = bullseye, twice = circle; the
 * start then waits CHASSIS_AUTOSTART_KEY_HOLD_MS (hand off the car). No effect once the run
 * has started or a debug command cancelled the power-up run. Wiring: key to GND, internal
 * pull-up (STATUS key=1 while pressed). */
#define CHASSIS_MODE_KEY_PORT GPIOB
#define CHASSIS_MODE_KEY_PIN GPIO_PIN_5
#define CHASSIS_MODE_KEY_PULL GPIO_PULLUP
#define CHASSIS_MODE_KEY_PRESSED GPIO_PIN_RESET
#define CHASSIS_AUTOSTART_KEY_HOLD_MS 2000U
#define CHASSIS_AUTOSTART_HOLD_MS 1000U
#define CHASSIS_AUTOSTART_STILL_RAD_S 0.05f   /* |gyro z| (~3 deg/s) */
#define CHASSIS_AUTOSTART_STILL_M_S 0.02f     /* |encoder speed| */

/* User-confirmed wiring: PC0=AD0, PC1=AD1, PC2=AD2; PB0=OUT. */
#define CHASSIS_LINE_ADDRESS_PORT GPIOC
#define CHASSIS_LINE_AD0 GPIO_PIN_0
#define CHASSIS_LINE_AD1 GPIO_PIN_1
#define CHASSIS_LINE_AD2 GPIO_PIN_2
#define CHASSIS_LINE_OUT_PORT GPIOB
#define CHASSIS_LINE_OUT GPIO_PIN_0
#define CHASSIS_LINE_BLACK_LEVEL GPIO_PIN_SET
/* Keys are not assigned. Define all four CHASSIS_KEY_* macros when enabling. */
#ifndef CHASSIS_KEYS_ENABLE
#define CHASSIS_KEYS_ENABLE 0
#endif
#if CHASSIS_KEYS_ENABLE
#if !defined(CHASSIS_KEY_PORT) || !defined(CHASSIS_KEY_LAPS) || !defined(CHASSIS_KEY_RUN) || !defined(CHASSIS_KEY_STOP)
#error Define the real key pins before enabling keys (PC0..PC2 are line addresses).
#endif
#endif
#define CHASSIS_MUX_SETTLE_US 500U
#define CHASSIS_LINE_MAX_AGE_MS 30U
#define CHASSIS_LINK_PERIOD_MS 20U
#define CHASSIS_LINK_TIMEOUT_MS 300U
/* Gimbal status ($G, 10 Hz on the gimbal link RX): a READY older than this does not count. */
#define CHASSIS_GIMBAL_STATUS_TIMEOUT_MS 500U
#define CHASSIS_SENSOR_AHEAD_M 0.095f /* Measured: probe row to drive-wheel axle. */
#define CHASSIS_PROBE_SPACING_MM 11.14f /* Measured: 78 mm across ch0..ch7 / 7. */
/* Must satisfy wheelbase/2 < radius <= SENSOR_AHEAD - TURN_M_S*0.06 (start check).
 * Pre-turn travel is SENSOR_AHEAD - RADIUS minus the body-lag lead; tune on the track.
 * 0.10 m/s / 0.062 m lands within ~8 mm for 0.1-0.3 s body lag when the car is already
 * slow at the corner (Tests/corner_sim.py). */
#define CHASSIS_RADIUS_M 0.062f
/* Speed-up: 0.20/0.10 m/s -> 26.7 s lap; step 1 0.30/0.10 -> 21.7 s (field, stable,
 * max line error 11 mm). Step 2 (2026-10-19): 0.35 cruise, 0.12 turn (model lands the
 * slowed corners 2-12 mm outside; turn rate 1.94 rad/s, inner wheel ~0.009 m/s). */
#define CHASSIS_CRUISE_M_S 0.350f
/* Field exits at 0.10/0.12 m/s all landed slightly INSIDE (line +2..+22 mm right of
 * centre after left turns): the model's lag is pessimistic. 0.14 moves the landing
 * ~11 mm outward (model) toward the centre; turn rate 2.26 rad/s, inner wheel ~0.010. */
#define CHASSIS_TURN_M_S 0.140f
#define CHASSIS_ACCEL_M_S2 0.8f
/* 1.5 felt like a hard stop before each corner; 1.0 starts braking ~2 cm earlier and
 * costs ~0.03 s per corner, with less body pitch for the gimbal/camera and less slip.
 * 0.6 (2026-10-28, with LINE_ANGULAR_ACCEL 3): at 1.0 one wheel's duty fell to 0/-1 while
 * braking and the body was kicked 18-23 deg/s just before some corners. Also DRIVE. */
#define CHASSIS_DECEL_M_S2 0.6f
/* 3 rad/s^2 needed 0.5 s to reach turn rate and overshot the new line by ~28 mm
 * (kinematic model); 8 brings the landing error under ~10 mm. Also limits line-following w. */
#define CHASSIS_ANGULAR_ACCEL_RAD_S2 8.0f
/* Gyro yaw-rate loop on the wheel differential, straight states only (mission_wheel_w):
 * 3-lap log 2026-10-24: after every corner exit the body kept turning +0.3 rad/s with the
 * command at 0, then the left wheel ran ~3 % fast (-0.2 rad/s against +0.1 commanded); the
 * line swung +11 / -22 mm and needed ~2 s to centre, and twice a corner met an off-centre
 * line and was missed. Tests/exit_yaw_sim.py: 11 -> 5 mm, settled at once; 0 = off. */
#define CHASSIS_YAW_RATE_KG 2.0f
/* w slew on the straight states (LINE, PRE_TURN, FINISH); turns keep ANGULAR_ACCEL.
 * 3-lap runs 2026-10-28 (gimbal on, SET over Bluetooth), straights:
 *   8 / decel 1.0 / kg 2: body yaw rms 5.4, max 31 deg/s, 75 kicks > 12 deg/s per minute,
 *                         gimbal yaw rms 0.32, max 2.05 deg (kicks > 12 deg/s doubled it)
 *   3 / decel 0.6 / kg 2: 3.2, 22 deg/s, 11 kicks/min, gimbal 0.25, max 1.33 deg; exits unchanged
 *   3 / decel 0.6 / kg 1: 3.6, 16 deg/s, 6 kicks/min, gimbal 0.29, max 1.63 deg, but the
 *                         exit swing grew 22 -> 33 mm (so YAW_RATE_KG stays 2).
 * The line error steps one probe (11 mm) at a time; a fast w slew turned each step into a
 * yaw kick. */
#define CHASSIS_LINE_ANGULAR_ACCEL_RAD_S2 3.0f
#define CHASSIS_LINE_KP 0.020f
#define CHASSIS_LINE_KD 0.002f
/* Gyro heading hold against the current edge (edges are 90 deg apart). Chosen with
 * Tests/line_follow_sim.py: recovers a 22 deg post-turn heading error for 0.2-0.4 s body lag. */
#define CHASSIS_LINE_KH 1.5f
/* Nominal corner-trigger spacing on the 100 cm track (SET edge): used to slow down before
 * corner 2 and for the run progress until two corners are measured, later the measured
 * average. 0 disables. Measured 2026-10-30 over 4 runs: 1.063 m average, 1.014-1.110 (the
 * triggers include the arc); the old 0.95 braked early on edge 1-2. 1 lap 0.95 / 1.00:
 * 18.76 / 18.36 s (edge 1-2 5.14 / 4.90 s), tracking unchanged (corner p99 0.54 / 0.52 deg). */
#define CHASSIS_EDGE_M 1.00f
/* Corner 1 is entered without warning (start position unknown); at 0.30 m/s the model
 * lands it 20-42 mm outside, at 0.20 m/s 3-18 mm. 0 disables the cap. */
#define CHASSIS_FIRST_EDGE_M_S 0.200f
#endif
