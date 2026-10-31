#ifndef CHASSIS_PROTOCOL_EXAMPLE_H
#define CHASSIS_PROTOCOL_EXAMPLE_H
#include <stddef.h>
#include <stdint.h>

/* Version-3 $C frame ($C,3,...,gz,speed_mm_s,task,progress). Returns byte length including LF,
 * or -1 for invalid values/insufficient buffer. mode: 0=line following, 1=right-angle turning.
 * imu_valid: 0/1 (attitude/rates zeroed when 0). speed: chassis forward speed, m/s
 * (|speed| <= 3 m/s), sent whatever imu_valid is. task: 0 = aim at the bullseye, 1 = draw the
 * circle; progress: run progress in 1/1000 lap (0..9999).
 * Caller owns the buffer until UART transmission (including DMA) completes. */
int chassis_encode(char *out, size_t capacity, uint16_t seq, unsigned mode,
                   unsigned imu_valid, float yaw, float pitch, float roll,
                   float gx, float gy, float gz, float speed_m_s,
                   unsigned task, unsigned progress_permille);
#endif
