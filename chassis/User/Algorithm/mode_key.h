#ifndef CHASSIS_MODE_KEY_H
#define CHASSIS_MODE_KEY_H
#include <stdint.h>

/* Power-up mode key (bullseye <-> circle before the power-up run) and its buzzer feedback.
 * Pure C: the app reads the pin and drives the buzzer. */
#define MODE_KEY_DEBOUNCE_MS 20U
typedef struct {
    uint8_t raw, stable;
    uint32_t changed_ms;
} mode_key_t;

/* pressed_now: the key right now (1 = pressed). A key held at power-up (or stuck) counts only
 * after it has been released. */
void mode_key_init(mode_key_t *k, uint8_t pressed_now, uint32_t now_ms);
/* 1 once per press: when the key has been pressed, without a bounce, for MODE_KEY_DEBOUNCE_MS. */
uint8_t mode_key_step(mode_key_t *k, uint8_t pressed_now, uint32_t now_ms);

/* count short beeps: 1 = bullseye, 2 = circle. */
#define BEEPER_ON_MS 100U
#define BEEPER_OFF_MS 150U
typedef struct {
    uint8_t count;
    uint32_t start_ms;
} beeper_t;

void beeper_init(beeper_t *b);
/* Starts count beeps now (replaces a pattern still sounding). */
void beeper_start(beeper_t *b, uint8_t count, uint32_t now_ms);
/* 1 while a beep sounds; silent again after the last one. */
uint8_t beeper_on(beeper_t *b, uint32_t now_ms);

#endif
