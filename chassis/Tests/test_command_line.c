#include "command_line.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Feed a byte string; return the last READY command (or "" if none) and count events. */
static unsigned ready_count, overflow_count;
static char last[COMMAND_LINE_CAPACITY];
static void feed(command_line_t *c, const char *bytes, size_t length) {
    size_t i;
    for (i = 0; i < length; i++) {
        const command_event_t event = command_line_feed(c, (uint8_t)bytes[i]);
        if (event == COMMAND_READY) { ready_count++; strcpy(last, c->text); }
        if (event == COMMAND_OVERFLOW) overflow_count++;
    }
}
static void reset(command_line_t *c) { command_line_init(c); ready_count = overflow_count = 0; last[0] = 0; }
#define FEED(c, literal) feed((c), (literal), sizeof(literal) - 1U)

int main(void) {
    command_line_t c;
    reset(&c); FEED(&c, "RUN\r\n");
    assert(ready_count == 1 && !strcmp(last, "RUN"));
    /* Bluetooth connect noise before the command must not swallow it. */
    reset(&c); FEED(&c, "\x00\xff\x1b" "RUN\r\n");
    assert(ready_count == 1 && !strcmp(last, "RUN"));
    reset(&c); FEED(&c, "R\xffU\x01N\n");
    assert(ready_count == 1 && !strcmp(last, "RUN"));
    /* Case and surrounding spaces from serial tools. */
    reset(&c); FEED(&c, "  laps 2  \r");
    assert(ready_count == 1 && !strcmp(last, "LAPS 2"));
    reset(&c); FEED(&c, "log on\n");
    assert(!strcmp(last, "LOG ON"));
    /* Empty lines and bare CR/LF pairs produce nothing. */
    reset(&c); FEED(&c, "\r\n\r\n   \n");
    assert(ready_count == 0 && overflow_count == 0);
    /* An overlong line is reported once and the next command still works. */
    reset(&c); FEED(&c, "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\nSTOP\n");
    assert(overflow_count == 1 && ready_count == 1 && !strcmp(last, "STOP"));
    /* Exactly capacity-1 characters still fit. */
    reset(&c); FEED(&c, "0123456789012345678901234567890\n");
    assert(ready_count == 1 && strlen(last) == COMMAND_LINE_CAPACITY - 1U);
    /* Phone Bluetooth apps often send no line ending: an idle gap ends the command. */
    reset(&c); FEED(&c, "RUN");
    assert(ready_count == 0);
    assert(command_line_idle(&c, COMMAND_IDLE_MS - 1U) == COMMAND_NONE);
    assert(command_line_idle(&c, COMMAND_IDLE_MS) == COMMAND_READY && !strcmp(c.text, "RUN"));
    assert(command_line_idle(&c, COMMAND_IDLE_MS + 500U) == COMMAND_NONE);   /* only once */
    reset(&c); FEED(&c, "RUN\r\n");
    assert(ready_count == 1 && command_line_idle(&c, 1000U) == COMMAND_NONE); /* no double RUN */
    puts("Command line tests passed: noise bytes skipped, case/space normalised, overflow recovery.");
    return 0;
}
