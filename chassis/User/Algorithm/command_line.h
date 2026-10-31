#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H
#include <stdint.h>

/* Assembles debug-UART bytes into commands. Pure C, no HAL. */
#define COMMAND_LINE_CAPACITY 32U
/* A pending line with no new byte for this long counts as finished (apps without CR/LF). */
#define COMMAND_IDLE_MS 150U

typedef enum { COMMAND_NONE, COMMAND_READY, COMMAND_OVERFLOW } command_event_t;
typedef struct {
    char text[COMMAND_LINE_CAPACITY]; /* valid after COMMAND_READY until the next byte */
    uint8_t length;
    uint8_t overflow;
} command_line_t;

void command_line_init(command_line_t *c);
/* CR or LF ends a line. Non-printable bytes (e.g. Bluetooth connect noise) are dropped
 * on their own instead of discarding the whole line; text is upper-cased and trimmed.
 * COMMAND_OVERFLOW: the finished line was longer than CAPACITY-1 and was dropped. */
command_event_t command_line_feed(command_line_t *c, uint8_t byte);
/* Call with the time since the last received byte; ends a pending line once it reaches
 * COMMAND_IDLE_MS. Returns what a line ending would have returned, else COMMAND_NONE. */
command_event_t command_line_idle(command_line_t *c, uint32_t idle_ms);
#endif
