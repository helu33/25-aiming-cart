#include "mode_key.h"
#include <assert.h>
#include <stdio.h>

static void test_one_event_per_press(void) {
    mode_key_t k;
    mode_key_init(&k, 0, 0);
    assert(!mode_key_step(&k, 0, 5));
    assert(!mode_key_step(&k, 1, 10));                       /* pressed, not yet stable */
    assert(!mode_key_step(&k, 1, 10 + MODE_KEY_DEBOUNCE_MS - 1));
    assert(mode_key_step(&k, 1, 10 + MODE_KEY_DEBOUNCE_MS));
    assert(!mode_key_step(&k, 1, 2000));                     /* held: no repeat */
    assert(!mode_key_step(&k, 0, 2010));
    assert(!mode_key_step(&k, 0, 2010 + MODE_KEY_DEBOUNCE_MS)); /* release is no event */
    assert(!mode_key_step(&k, 1, 2100));
    assert(mode_key_step(&k, 1, 2100 + MODE_KEY_DEBOUNCE_MS)); /* second press */
}
static void test_bounce_is_ignored(void) {
    mode_key_t k;
    mode_key_init(&k, 0, 0);
    for (uint32_t t = 100; t < 160; t += 4)                  /* contact bounce, 4 ms flips, ends up */
        assert(!mode_key_step(&k, (uint8_t)(((t / 4) & 1U) ^ 1U), t));
    assert(!mode_key_step(&k, 1, 160));
    assert(!mode_key_step(&k, 1, 160 + MODE_KEY_DEBOUNCE_MS - 1));
    assert(mode_key_step(&k, 1, 160 + MODE_KEY_DEBOUNCE_MS));
}
static void test_held_at_power_up_needs_a_release(void) {
    mode_key_t k;
    mode_key_init(&k, 1, 0);                                 /* held (or stuck) at boot */
    assert(!mode_key_step(&k, 1, 500));
    assert(!mode_key_step(&k, 0, 600));
    assert(!mode_key_step(&k, 0, 600 + MODE_KEY_DEBOUNCE_MS));
    assert(!mode_key_step(&k, 1, 700));
    assert(mode_key_step(&k, 1, 700 + MODE_KEY_DEBOUNCE_MS));
}
static void test_key_tick_wrap(void) {
    mode_key_t k;
    mode_key_init(&k, 0, 0xFFFFFFF0U);
    assert(!mode_key_step(&k, 1, 0xFFFFFFF8U));
    assert(!mode_key_step(&k, 1, 0xFFFFFFF8U + MODE_KEY_DEBOUNCE_MS - 1U));
    assert(mode_key_step(&k, 1, 0xFFFFFFF8U + MODE_KEY_DEBOUNCE_MS));
}
static void test_beeps(void) {
    const uint32_t period = BEEPER_ON_MS + BEEPER_OFF_MS;
    beeper_t b;
    beeper_init(&b);
    assert(!beeper_on(&b, 0));                                /* silent until started */
    beeper_start(&b, 1, 1000);
    assert(beeper_on(&b, 1000) && beeper_on(&b, 1000 + BEEPER_ON_MS - 1));
    assert(!beeper_on(&b, 1000 + BEEPER_ON_MS));
    assert(!beeper_on(&b, 1000 + period) && !beeper_on(&b, 1000 + 5 * period)); /* one only */
    beeper_start(&b, 2, 5000);                                /* two beeps */
    assert(beeper_on(&b, 5000) && !beeper_on(&b, 5000 + BEEPER_ON_MS));
    assert(beeper_on(&b, 5000 + period) && beeper_on(&b, 5000 + period + BEEPER_ON_MS - 1));
    assert(!beeper_on(&b, 5000 + period + BEEPER_ON_MS));
    assert(!beeper_on(&b, 5000 + 2 * period) && !beeper_on(&b, 5000 + 3 * period));
    beeper_start(&b, 2, 9000);                                /* a new press restarts it */
    assert(beeper_on(&b, 9000));
    beeper_start(&b, 1, 9000 + period);
    assert(beeper_on(&b, 9000 + period) && !beeper_on(&b, 9000 + 2 * period));
    beeper_start(&b, 1, 0xFFFFFFF0U);                          /* tick wrap */
    assert(beeper_on(&b, 0xFFFFFFF0U + BEEPER_ON_MS - 1U) && !beeper_on(&b, 0xFFFFFFF0U + BEEPER_ON_MS));
}
int main(void) {
    test_one_event_per_press(); test_bounce_is_ignored(); test_held_at_power_up_needs_a_release();
    test_key_tick_wrap(); test_beeps();
    puts("Mode key tests passed: one event per press, bounce, held at boot, tick wrap, beep patterns.");
    return 0;
}
