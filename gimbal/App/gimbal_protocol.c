#include "gimbal_protocol.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint16_t GimbalProtocol_Crc16(const char *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)(uint8_t)data[i] << 8);
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool read_uint(const char *text, uint32_t max, uint32_t *value)
{
    uint32_t result = 0;
    if (!*text) return false;
    for (; *text; ++text) {
        if (*text < '0' || *text > '9') return false;
        result = result * 10U + (uint32_t)(*text - '0');
        if (result > max) return false; /* max <= 65535: multiplication cannot overflow. */
    }
    *value = result;
    return true;
}

static bool read_float(const char *text, float limit, float *value)
{
    char *end;
    /* Restrict to decimal syntax; reject whitespace, hex, NaN, infinity. */
    if (!*text) return false;
    for (const char *p = text; *p; ++p) {
        if (!((*p >= '0' && *p <= '9') || *p == '+' || *p == '-' ||
              *p == '.' || *p == 'e' || *p == 'E')) return false;
    }
    *value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(*value) && fabsf(*value) <= limit;
}

bool GimbalProtocol_Parse(const char *line, gimbal_packet_t *packet)
{
    char buffer[GIMBAL_LINE_CAPACITY];
    char *field[14];                    /* C version 3 has 14 */
    size_t length, count = 0;
    uint32_t seq, valid, mode;
    gimbal_packet_t parsed;
    memset(&parsed, 0, sizeof(parsed));
    if (!line || !packet) return false;
    length = strlen(line);
    if (!length || length >= sizeof(buffer)) return false;

    if (!strcmp(line, "T,T,T,T") || !strcmp(line, "N,N,N,N")) {
        parsed.kind = GIMBAL_PACKET_CHASSIS;
        parsed.turning = line[0] == 'T';
        *packet = parsed;
        return true;
    }

    if (line[0] == '$') {
        const char *star = strchr(line, '*');
        uint16_t received_crc = 0;
        if (!star || strlen(star + 1) != 4U) return false;
        for (unsigned i = 1; i <= 4; ++i) {
            int digit = hex_digit(star[i]);
            if (digit < 0) return false;
            received_crc = (uint16_t)((received_crc << 4) | (unsigned)digit);
        }
        length = (size_t)(star - line - 1);
        if (GimbalProtocol_Crc16(line + 1, length) != received_crc) return false;
        memcpy(buffer, line + 1, length);
        parsed.versioned = true;
    } else {
        memcpy(buffer, line, length);
    }
    buffer[length] = '\0';
    field[count++] = buffer;
    for (size_t i = 0; i < length; ++i) {
        if (buffer[i] == ',') {
            if (count == sizeof(field) / sizeof(field[0])) return false;
            buffer[i] = '\0';
            field[count++] = &buffer[i + 1];
        }
    }

    if (parsed.versioned) {
        /* Version 2 appends fields: V range_mm,age_ms; C speed_mm_s. Version 3 then appends
         * V code,area_x100 (telemetry); C task,progress (circle task). Versions 1, 2 stay valid. */
        bool v2, v3 = false;
        if (count < 3 || !read_uint(field[2], 65535U, &seq)) return false;
        if (!strcmp(field[1], "1")) v2 = false;
        else if (!strcmp(field[1], "2")) v2 = true;
        else if (!strcmp(field[1], "3")) v2 = v3 = true;
        else return false;
        parsed.seq = (uint16_t)seq;
        if (!strcmp(field[0], "V")) {
            if (count != (v3 ? 10U : v2 ? 8U : 6U) || !read_uint(field[3], 1, &valid) ||
                !read_float(field[4], GIMBAL_VISION_MAX_ERROR_DEG, &parsed.yaw) ||
                !read_float(field[5], GIMBAL_VISION_MAX_ERROR_DEG, &parsed.pitch)) return false;
            if (v2) {
                uint32_t range_mm, age_ms;
                if (!read_uint(field[6], 20000U, &range_mm) || !read_uint(field[7], 1000U, &age_ms))
                    return false;
                parsed.has_range = true;
                parsed.range_m = (float)range_mm * 0.001f;
                parsed.age_ms = (uint16_t)age_ms;
            }
            if (v3) {
                uint32_t code, area_x100;
                if (!read_uint(field[8], 99U, &code) || !read_uint(field[9], 10000U, &area_x100))
                    return false;
                parsed.has_diag = true;
                parsed.vision_code = (uint8_t)code;
                parsed.area_x100 = (uint16_t)area_x100;
            }
            parsed.kind = GIMBAL_PACKET_VISION;
            parsed.valid = valid != 0;
        } else if (!strcmp(field[0], "C")) {
            if (count != (v3 ? 14U : v2 ? 12U : 11U) || !read_uint(field[3], 1, &mode) ||
                !read_uint(field[4], 1, &valid) ||
                !read_float(field[5], 360.0f, &parsed.yaw) ||
                !read_float(field[6], 180.0f, &parsed.pitch) ||
                !read_float(field[7], 180.0f, &parsed.roll) ||
                !read_float(field[8], 2000.0f, &parsed.gx) ||
                !read_float(field[9], 2000.0f, &parsed.gy) ||
                !read_float(field[10], 2000.0f, &parsed.gz)) return false;
            if (v2) {
                float speed_mm_s;
                if (!read_float(field[11], 3000.0f, &speed_mm_s)) return false;
                parsed.has_speed = true;
                parsed.speed_m_s = speed_mm_s * 0.001f;
            }
            if (v3) {
                uint32_t task, progress;
                if (!read_uint(field[12], 1U, &task) || !read_uint(field[13], 9999U, &progress))
                    return false;
                parsed.has_task = true;
                parsed.task = (uint8_t)task;
                parsed.progress_permille = (uint16_t)progress;
            }
            parsed.kind = GIMBAL_PACKET_CHASSIS;
            parsed.turning = mode != 0;
            parsed.valid = valid != 0;
        } else return false;
    } else {
        float ignored;
        /* Preserve deployed two/four-number frames, but validate the whole frame. */
        if ((count != 2 && count != 4) ||
            !read_float(field[0], 404.0f, &parsed.yaw) ||
            !read_float(field[1], 404.0f, &parsed.pitch)) return false;
        for (size_t i = 2; i < count; ++i)
            if (!read_float(field[i], 1000000.0f, &ignored)) return false;
        parsed.kind = GIMBAL_PACKET_VISION;
        parsed.valid = parsed.yaw != 404.0f && parsed.pitch != 404.0f;
        if (parsed.valid && (fabsf(parsed.yaw) > GIMBAL_VISION_MAX_ERROR_DEG ||
                             fabsf(parsed.pitch) > GIMBAL_VISION_MAX_ERROR_DEG)) return false;
    }
    *packet = parsed;
    return true;
}

int GimbalProtocol_Push(gimbal_line_t *line, uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        bool complete = !line->dropping && line->length != 0;
        line->data[line->length] = '\0';
        line->length = 0;
        line->dropping = false;
        return complete ? 1 : 0;
    }
    if (line->dropping) return 0;
    if (byte < 0x20U || byte > 0x7EU || line->length >= sizeof(line->data) - 1U) {
        GimbalProtocol_Discard(line);
        return -1;
    }
    line->data[line->length++] = (char)byte;
    return 0;
}

void GimbalProtocol_Discard(gimbal_line_t *line)
{
    line->length = 0;
    line->dropping = true;
}

bool GimbalProtocol_AcceptSequence(gimbal_sequence_t *state, uint16_t seq,
                                   uint32_t received_ms, uint32_t timeout_ms)
{
    uint16_t delta = (uint16_t)(seq - state->last);
    if (state->initialized && (delta == 0U ||
        ((uint32_t)(received_ms - state->received_ms) <= timeout_ms && delta >= 0x8000U))) return false;
    state->initialized = true;
    state->last = seq;
    state->received_ms = received_ms;
    return true;
}

int GimbalProtocol_EncodeStatus(char *out, size_t capacity, uint16_t seq, unsigned flags)
{
    if (!out || capacity == 0U) return -1;
    const int body = snprintf(out, capacity, "$G,1,%u,%u", (unsigned)seq, flags & 0xFFU);
    if (body < 0 || (size_t)body + 7U > capacity) {           /* "*XXXX" + LF + NUL */
        out[0] = '\0';
        return -1;
    }
    const uint16_t crc = GimbalProtocol_Crc16(out + 1, (size_t)body - 1U);
    return body + snprintf(out + body, capacity - (size_t)body, "*%04X\n", (unsigned)crc);
}

int GimbalProtocol_EncodeTask(char *out, size_t capacity, uint16_t seq, unsigned task,
                              unsigned progress_permille)
{
    if (!out || capacity == 0U) return -1;
    out[0] = '\0';
    if (task > 1U || progress_permille > 9999U) return -1;
    const int body = snprintf(out, capacity, "$K,1,%u,%u,%u", (unsigned)seq, task, progress_permille);
    if (body < 0 || (size_t)body + 7U > capacity) {           /* "*XXXX" + LF + NUL */
        out[0] = '\0';
        return -1;
    }
    const uint16_t crc = GimbalProtocol_Crc16(out + 1, (size_t)body - 1U);
    return body + snprintf(out + body, capacity - (size_t)body, "*%04X\n", (unsigned)crc);
}
