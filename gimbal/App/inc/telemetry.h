#ifndef TELEMETRY_H
#define TELEMETRY_H
/* Bluetooth telemetry on UART7 (4-pin header with VCC) for VOFA+ "JustFloat":
 * each frame = time (s) + TELEM_CHANNELS floats, little-endian, then 00 00 80 7F.
 * Samples are taken every 10 ms; they are streamed live when the link has room and are
 * always kept in an on-board recorder (last TELEM_RECORD_S seconds, int16-packed), which
 * can be replayed over the same link after a run in case Bluetooth dropped out.
 *
 * Channels (VOFA+ I0 is the time):
 *   1 vision yaw error, deg        5 yaw world error (target - IMU), deg
 *   2 vision pitch error, deg      6 yaw motor command, rpm
 *   3 flags: 1 valid, 2 suspended  7 gyro z (stage world rate), deg/s
 *            4 waiting, 8 tracking 8 pitch loop error, deg
 *            16 chassis turning    9 yaw motor rate from its encoder (relative to the
 *   4 tracker yaw rate, deg/s        chassis), rpm
 *                                 10 chassis gz as the gimbal uses it, deg/s
 *                                 11 K230 code of the last frame (0 valid, $V version 3)
 *                                 12 its tape rectangle area, % of the image
 * The framing and the recorder below are pure C (host-tested); the UART glue is in
 * telemetry.c. */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define TELEM_CHANNELS      12U
#define TELEM_RATE_HZ       100U
#define TELEM_RECORD_S      60U
#define TELEM_RECORD_LEN    (TELEM_RATE_HZ * TELEM_RECORD_S)          /* 6000 x 24 B = 144 kB */
#define TELEM_FRAME_BYTES   ((TELEM_CHANNELS + 1U) * 4U + 4U)          /* 56 */
/* Share of the raw baud rate the live stream may use. The HC-05 forwards only ~3.5 kB/s at
 * 115200 (field: 73-75 of 100 frames/s of 48 B arrived), and 56-byte frames at 100 Hz
 * (5.6 kB/s) stopped it forwarding anything, replies included (2026-10-28, old 48-byte build
 * fine on the same module). 0.4 -> every 2nd sample live (50 Hz, 2.8 kB/s); the on-board
 * recorder keeps all 100 Hz for replay. */
#define TELEM_LINK_USE      0.4f

/* Resolution of the int16 recorder per channel: value * scale is stored. */
static inline float telem_scale(unsigned channel)
{
    static const float scale[TELEM_CHANNELS] = {100.0f, 100.0f, 1.0f, 100.0f,
                                                100.0f, 100.0f, 10.0f, 100.0f,
                                                100.0f, 10.0f, 1.0f, 100.0f};
    return channel < TELEM_CHANNELS ? scale[channel] : 1.0f;
}

typedef struct { int16_t v[TELEM_CHANNELS]; } telem_packed_t;
typedef struct {
    telem_packed_t buf[TELEM_RECORD_LEN];
    uint32_t head, count, total;     /* total: samples pushed since reset (gives the time) */
} telem_ring_t;

static inline uint32_t telem_frame(uint8_t *out, float t, const float *ch)
{
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(out, &t, 4);
    memcpy(out + 4, ch, 4U * TELEM_CHANNELS);
    memcpy(out + 4U * (TELEM_CHANNELS + 1U), tail, 4);
    return TELEM_FRAME_BYTES;
}

static inline telem_packed_t telem_pack(const float *ch)
{
    telem_packed_t p;
    for (unsigned i = 0; i < TELEM_CHANNELS; ++i) {
        float x = ch[i] * telem_scale(i);
        if (!(x == x)) x = 0.0f;                         /* NaN */
        if (x > 32767.0f) x = 32767.0f;
        if (x < -32767.0f) x = -32767.0f;
        p.v[i] = (int16_t)lrintf(x);
    }
    return p;
}

static inline void telem_unpack(const telem_packed_t *p, float *ch)
{
    for (unsigned i = 0; i < TELEM_CHANNELS; ++i) ch[i] = (float)p->v[i] / telem_scale(i);
}

static inline void telem_ring_reset(telem_ring_t *r)
{
    r->head = r->count = r->total = 0U;
}

static inline void telem_ring_push(telem_ring_t *r, const float *ch)
{
    r->buf[r->head] = telem_pack(ch);
    r->head = (r->head + 1U) % TELEM_RECORD_LEN;
    if (r->count < TELEM_RECORD_LEN) ++r->count;
    ++r->total;
}

/* i-th oldest kept sample (0 .. count-1) into ch; returns its time in seconds. */
static inline float telem_ring_get(const telem_ring_t *r, uint32_t i, float *ch)
{
    const uint32_t k = (r->head + TELEM_RECORD_LEN - r->count + i) % TELEM_RECORD_LEN;
    telem_unpack(&r->buf[k], ch);
    return (float)(r->total - r->count + i) / (float)TELEM_RATE_HZ;
}

/* One command line from the PC: a single letter (D/P/L/C/G) or "S <name> <value>".
 * letter gets the upper-case command letter; for SET also name (truncated to name_cap - 1)
 * and value. Anything else is BAD; an empty line is NONE. */
typedef enum { TELEM_LINE_NONE, TELEM_LINE_LETTER, TELEM_LINE_SET, TELEM_LINE_BAD } telem_line_t;

static inline telem_line_t telem_parse_line(const char *line, char *letter, char *name,
                                            unsigned name_cap, float *value)
{
    while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') ++line;
    if (!*line) return TELEM_LINE_NONE;
    char c = *line++;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c < 'A' || c > 'Z') return TELEM_LINE_BAD;
    *letter = c;
    while (*line == ' ' || *line == '\t') ++line;
    if (!*line || *line == '\r' || *line == '\n') return c == 'S' ? TELEM_LINE_BAD : TELEM_LINE_LETTER;
    if (c != 'S' || name_cap < 2U) return TELEM_LINE_BAD;
    unsigned n = 0;
    while ((*line >= 'a' && *line <= 'z') || (*line >= 'A' && *line <= 'Z') ||
           (*line >= '0' && *line <= '9') || *line == '_') {
        if (n + 1U < name_cap) name[n++] = *line;
        ++line;
    }
    name[n] = '\0';
    if (!n || (*line != ' ' && *line != '\t')) return TELEM_LINE_BAD;
    char *end;
    *value = strtof(line, &end);
    if (end == line) return TELEM_LINE_BAD;
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') ++end;
    return *end ? TELEM_LINE_BAD : TELEM_LINE_SET;
}

/* Send every n-th sample live so the stream fits TELEM_LINK_USE of the baud rate. */
/* Next piece of a text reply: copies up to max bytes from reply + *off into out, advances
 * *off and returns the count (0 once all is sent). A reply goes out one frame-sized piece per
 * sample: the HC-05 dropped the middle of a 149-byte burst while the live stream ran
 * (2026-10-24), frame-sized writes at the frame rate always got through. */
static inline uint32_t telem_reply_chunk(const char *reply, uint32_t len, uint32_t *off,
                                         uint8_t *out, uint32_t max)
{
    if (*off >= len) return 0U;
    uint32_t n = len - *off;
    if (n > max) n = max;
    memcpy(out, reply + *off, n);
    *off += n;
    return n;
}

static inline uint32_t telem_live_divider(uint32_t baud)
{
    const float frames_per_s = (float)baud / 10.0f * TELEM_LINK_USE / (float)TELEM_FRAME_BYTES;
    uint32_t d = (uint32_t)ceilf((float)TELEM_RATE_HZ / frames_per_s);
    return d < 1U ? 1U : d;
}
#endif
