#ifndef CHASSIS_AUTOSTART_H
#define CHASSIS_AUTOSTART_H
#include <stdint.h>

/* Power-up auto-start (pure C; the app passes the conditions): start the run once, when the
 * gimbal has reported ready ($G) and the chassis could start by itself (RUN preconditions,
 * standing still), both without a break for hold_ms. A command on the debug port before that
 * cancels it: manual control from then on. The mode key (bullseye <-> circle) does not cancel
 * it; after a press the start waits key_hold_ms (the hand is still on the car). */
typedef struct {
    uint8_t armed, holding, pressed;
    uint32_t since_ms, hold_ms, press_ms, key_hold_ms;
} autostart_t;

void autostart_init(autostart_t *a, uint8_t enabled, uint32_t hold_ms, uint32_t key_hold_ms);
void autostart_cancel(autostart_t *a);
uint8_t autostart_armed(const autostart_t *a);
/* Mode key pressed: 1 if taken (still armed, before the start; the app switches the mode),
 * 0 otherwise (off, cancelled, already started). */
uint8_t autostart_press(autostart_t *a, uint32_t now_ms);
/* 1 exactly once: both ready, continuously, for hold_ms, and key_hold_ms after the last press. */
uint8_t autostart_step(autostart_t *a, uint32_t now_ms, uint8_t gimbal_ready, uint8_t chassis_ready);

#endif
