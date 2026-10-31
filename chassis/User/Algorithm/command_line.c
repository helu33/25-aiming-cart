#include "command_line.h"

void command_line_init(command_line_t *c) {
    c->text[0] = '\0';
    c->length = 0U;
    c->overflow = 0U;
}

command_event_t command_line_feed(command_line_t *c, uint8_t byte) {
    if (byte == '\r' || byte == '\n') {
        const uint8_t overflow = c->overflow;
        while (c->length && c->text[c->length - 1U] == ' ') c->length--;
        c->text[c->length] = '\0';
        c->overflow = 0U;
        if (overflow) { c->length = 0U; return COMMAND_OVERFLOW; }
        if (!c->length) return COMMAND_NONE;
        c->length = 0U;           /* text stays readable until the next byte */
        return COMMAND_READY;
    }
    if (byte < 32U || byte > 126U) return COMMAND_NONE;     /* drop just this byte */
    if (byte == ' ' && c->length == 0U) return COMMAND_NONE; /* leading spaces */
    if (c->overflow) return COMMAND_NONE;
    if (c->length >= COMMAND_LINE_CAPACITY - 1U) { c->overflow = 1U; return COMMAND_NONE; }
    if (byte >= 'a' && byte <= 'z') byte = (uint8_t)(byte - 'a' + 'A');
    c->text[c->length++] = (char)byte;
    return COMMAND_NONE;
}

command_event_t command_line_idle(command_line_t *c, uint32_t idle_ms) {
    if ((c->length == 0U && !c->overflow) || idle_ms < COMMAND_IDLE_MS) return COMMAND_NONE;
    return command_line_feed(c, (uint8_t)'\n');
}
