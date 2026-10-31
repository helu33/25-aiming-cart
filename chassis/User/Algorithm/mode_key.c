#include "mode_key.h"

void mode_key_init(mode_key_t *k, uint8_t pressed_now, uint32_t now_ms) {
    k->raw=k->stable=pressed_now?1U:0U; k->changed_ms=now_ms;
}
uint8_t mode_key_step(mode_key_t *k, uint8_t pressed_now, uint32_t now_ms) {
    const uint8_t raw=pressed_now?1U:0U;
    if (raw!=k->raw) { k->raw=raw; k->changed_ms=now_ms; return 0; }
    if (raw==k->stable || (uint32_t)(now_ms-k->changed_ms)<MODE_KEY_DEBOUNCE_MS) return 0;
    k->stable=raw;
    return raw;                                     /* a release is no event */
}

void beeper_init(beeper_t *b) { b->count=0; b->start_ms=0; }
void beeper_start(beeper_t *b, uint8_t count, uint32_t now_ms) { b->count=count; b->start_ms=now_ms; }
uint8_t beeper_on(beeper_t *b, uint32_t now_ms) {
    const uint32_t period=BEEPER_ON_MS+BEEPER_OFF_MS, t=now_ms-b->start_ms;
    if (!b->count) return 0;
    if (t/period>=b->count) { b->count=0; return 0; }
    return (uint8_t)(t%period<BEEPER_ON_MS);
}
