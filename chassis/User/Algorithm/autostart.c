#include "autostart.h"

void autostart_init(autostart_t *a, uint8_t enabled, uint32_t hold_ms, uint32_t key_hold_ms) {
    a->armed=enabled?1U:0U; a->holding=a->pressed=0; a->since_ms=a->press_ms=0;
    a->hold_ms=hold_ms; a->key_hold_ms=key_hold_ms;
}
void autostart_cancel(autostart_t *a) { a->armed=0; a->holding=0; }
uint8_t autostart_armed(const autostart_t *a) { return a->armed; }
uint8_t autostart_press(autostart_t *a, uint32_t now_ms) {
    if (!a->armed) return 0;
    a->pressed=1; a->press_ms=now_ms;
    return 1;
}
uint8_t autostart_step(autostart_t *a, uint32_t now_ms, uint8_t gimbal_ready, uint8_t chassis_ready) {
    if (!a->armed) return 0;
    if (!gimbal_ready || !chassis_ready) { a->holding=0; return 0; }
    if (!a->holding) { a->holding=1; a->since_ms=now_ms; }
    if ((uint32_t)(now_ms-a->since_ms)<a->hold_ms) return 0;
    if (a->pressed && (uint32_t)(now_ms-a->press_ms)<a->key_hold_ms) return 0;
    a->armed=0; a->holding=0;
    return 1;
}
