#ifndef GIMBAL_PROTOCOL_H
#define GIMBAL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GIMBAL_LINE_CAPACITY 192U /* Includes terminating NUL; excludes CR/LF. */
#define GIMBAL_VISION_MAX_ERROR_DEG 180.0f

typedef enum {
    GIMBAL_PACKET_VISION,
    GIMBAL_PACKET_CHASSIS
} gimbal_packet_kind_t;

typedef struct {
    gimbal_packet_kind_t kind;
    bool versioned;
    uint16_t seq;
    bool valid;             /* Target valid for V; chassis IMU valid for C. */
    bool turning;
    float yaw, pitch, roll; /* Degrees. Vision yaw/pitch are errors. */
    float gx, gy, gz;       /* Chassis rates, degrees/second. */
    /* Version 2 only (has_* false for version 1 and legacy frames). */
    bool has_range;         /* V: range_m, age_ms */
    float range_m;          /* camera to target point, metres (0 when no target) */
    uint16_t age_ms;        /* snapshot to send on the K230, ms */
    bool has_speed;         /* C: speed_m_s */
    float speed_m_s;        /* chassis forward speed (encoders), m/s */
    /* V version 3 only: why the frame is not valid (vision/protocol.py CODE_*, 0 = valid)
     * and the tape rectangle's area in % of the image x 100; telemetry only. */
    bool has_diag;
    uint8_t vision_code;
    uint16_t area_x100;
    /* C version 3: 1 = the laser draws the circle task, 0 = bullseye; run progress, 1/1000 lap. */
    bool has_task;
    uint8_t task;
    uint16_t progress_permille;
} gimbal_packet_t;

typedef struct {
    char data[GIMBAL_LINE_CAPACITY];
    uint16_t length;
    bool dropping;
} gimbal_line_t;

typedef struct {
    bool initialized;
    uint16_t last;
    uint32_t received_ms;
} gimbal_sequence_t;

uint16_t GimbalProtocol_Crc16(const char *data, size_t length);
/* Called in task context; input must be a NUL-terminated line without CR/LF. */
bool GimbalProtocol_Parse(const char *line, gimbal_packet_t *packet);
/* 1: complete nonempty line in data; 0: incomplete; -1: rejected character/overflow. */
int GimbalProtocol_Push(gimbal_line_t *line, uint8_t byte);
/* After a transport error, discard through the next delimiter. */
void GimbalProtocol_Discard(gimbal_line_t *line);
bool GimbalProtocol_AcceptSequence(gimbal_sequence_t *state, uint16_t seq,
                                   uint32_t received_ms, uint32_t timeout_ms);

/* $G,1,<seq>,<flags>*CRC16 + LF: gimbal -> chassis status, every 100 ms on the chassis UART TX.
 * The chassis starts its power-up run once READY holds (chassis_decode_gimbal_status). */
#define GIMBAL_STATUS_READY   0x01U   /* CONTROL and VISION */
#define GIMBAL_STATUS_CONTROL 0x02U   /* IMU warmed up with its bias, both motors reporting */
#define GIMBAL_STATUS_VISION  0x04U   /* locked on the target (see gimbal_task.c) */
/* Returns the length (LF included) or -1 when it does not fit; out is NUL-terminated. */
int GimbalProtocol_EncodeStatus(char *out, size_t capacity, uint16_t seq, unsigned flags);

/* $K,1,<seq>,<task>,<progress>*CRC16 + LF: gimbal -> K230, the chassis task (0 bullseye, 1 circle)
 * and run progress (1/1000 lap, 0..9999) from $C v3, every 50 ms on the vision UART TX. The K230
 * moves its aim point around the circle by it (vision/protocol.py decode_task). Returns the
 * length (LF included) or -1 for a bad value or a too small buffer. */
int GimbalProtocol_EncodeTask(char *out, size_t capacity, uint16_t seq, unsigned task,
                              unsigned progress_permille);

#endif
