/* Copy this encoder/header into the chassis project; no STM32 dependency. */
#include "chassis_protocol.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint16_t crc16(const char *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)(uint8_t)data[i] << 8);
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}

int chassis_encode(char *out, size_t capacity, uint16_t seq, unsigned mode,
                   unsigned imu_valid, float yaw, float pitch, float roll,
                   float gx, float gy, float gz, float speed_m_s,
                   unsigned task, unsigned progress_permille)
{
    if (!out || capacity < 8 || mode > 1 || imu_valid > 1 || task > 1 || progress_permille > 9999U)
        return -1;
    out[0] = '\0';
    if (!imu_valid) yaw = pitch = roll = gx = gy = gz = 0;
    if (!isfinite(yaw) || fabsf(yaw) > 360 || !isfinite(pitch) || fabsf(pitch) > 180 ||
        !isfinite(roll) || fabsf(roll) > 180 || !isfinite(gx) || fabsf(gx) > 2000 ||
        !isfinite(gy) || fabsf(gy) > 2000 || !isfinite(gz) || fabsf(gz) > 2000 ||
        !isfinite(speed_m_s) || fabsf(speed_m_s) > 3.0f) return -1;
    int length = snprintf(out, capacity, "$C,3,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%u,%u",
                          (unsigned)seq, mode, imu_valid, yaw, pitch, roll, gx, gy, gz,
                          (double)(speed_m_s * 1000.0f), task, progress_permille);
    if (length < 0 || (size_t)length + 7U > capacity || length + 5 > 191) {
        out[0] = '\0'; return -1;
    }
    uint16_t crc = crc16(out + 1, (size_t)length - 1U);
    int suffix = snprintf(out + length, capacity - (size_t)length, "*%04X\n", (unsigned)crc);
    if (suffix != 6) { out[0] = '\0'; return -1; }
    return length + suffix;
}

static int read_decimal(const char **p, unsigned max, unsigned *value) {
    unsigned v = 0; int digits = 0;
    while (**p >= '0' && **p <= '9') {
        v = v * 10U + (unsigned)(**p - '0');
        if (v > max || ++digits > 5) return 0;
        (*p)++;
    }
    if (!digits) return 0;
    *value = v;
    return 1;
}
static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
int chassis_decode_gimbal_status(const char *line, unsigned *flags) {
    const char *p, *star;
    unsigned version, seq, value, crc = 0;
    if (!line || !flags || line[0] != '$' || line[1] != 'G' || line[2] != ',') return 0;
    star = strchr(line, '*');
    if (!star || strlen(star) != 5U) return 0;
    for (int i = 1; i <= 4; i++) {
        const int h = hex_value(star[i]);
        if (h < 0) return 0;
        crc = (crc << 4) | (unsigned)h;
    }
    if (crc16(line + 1, (size_t)(star - line - 1)) != crc) return 0;
    p = line + 3;
    if (!read_decimal(&p, 9U, &version) || version != 1U || *p++ != ',') return 0;
    if (!read_decimal(&p, 65535U, &seq) || *p++ != ',') return 0;
    if (!read_decimal(&p, 255U, &value) || p != star) return 0;
    (void)seq;
    *flags = value;
    return 1;
}
