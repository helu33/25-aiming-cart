#include "autostart.h"
#include <assert.h>
#include <stdio.h>

static void test_fires_once_after_the_hold(void) {
    autostart_t a;
    autostart_init(&a, 1, 1000, 2000);
    assert(autostart_armed(&a));
    assert(!autostart_step(&a, 0, 1, 1));
    assert(!autostart_step(&a, 999, 1, 1));
    assert(autostart_step(&a, 1000, 1, 1));
    assert(!autostart_armed(&a));
    assert(!autostart_step(&a, 5000, 1, 1));              /* once per power-up */
}
static void test_a_break_restarts_the_hold(void) {
    autostart_t a;
    autostart_init(&a, 1, 1000, 2000);
    assert(!autostart_step(&a, 0, 1, 1));
    assert(!autostart_step(&a, 800, 0, 1));                /* gimbal not ready */
    assert(!autostart_step(&a, 900, 1, 1));
    assert(!autostart_step(&a, 1800, 1, 0));               /* chassis not ready (moved) */
    assert(!autostart_step(&a, 1850, 1, 1));
    assert(!autostart_step(&a, 2849, 1, 1));
    assert(autostart_step(&a, 2850, 1, 1));
}
static void test_cancel_and_disabled(void) {
    autostart_t a;
    autostart_init(&a, 1, 1000, 2000);
    assert(!autostart_step(&a, 0, 1, 1));
    autostart_cancel(&a);                                  /* a command came first */
    assert(!autostart_armed(&a) && !autostart_step(&a, 2000, 1, 1));
    autostart_init(&a, 0, 1000, 2000);
    assert(!autostart_armed(&a) && !autostart_step(&a, 0, 1, 1) && !autostart_step(&a, 9000, 1, 1));
}
static void test_tick_wrap(void) {
    autostart_t a;
    autostart_init(&a, 1, 1000, 2000);
    assert(!autostart_step(&a, 0xFFFFFF00U, 1, 1));
    assert(!autostart_step(&a, 0x000002E7U, 1, 1));        /* 999 ms later */
    assert(autostart_step(&a, 0x000002E8U, 1, 1));
}
/* The mode key (bullseye <-> circle) before the start: taken only while armed, and the car
 * waits key_hold_ms after the last press (the hand is still on the car). */
static void test_key_press_delays_the_start(void) {
    autostart_t a;
    autostart_init(&a, 1, 1000, 2000);
    assert(autostart_press(&a, 0));                         /* long before ready */
    assert(!autostart_step(&a, 500, 1, 1));
    assert(!autostart_step(&a, 1500, 1, 1));                /* hold done, press only 1.5 s ago */
    assert(!autostart_step(&a, 1999, 1, 1));
    assert(autostart_step(&a, 2000, 1, 1));
    assert(!autostart_press(&a, 2100));                     /* running: the key does nothing */

    autostart_init(&a, 1, 1000, 2000);
    assert(!autostart_step(&a, 0, 1, 1));
    assert(autostart_press(&a, 900));                       /* just before the start */
    assert(!autostart_step(&a, 1000, 1, 1));
    assert(autostart_press(&a, 2500));                      /* pressed again: wait again */
    assert(!autostart_step(&a, 2900, 1, 1) && !autostart_step(&a, 4499, 1, 1));
    assert(autostart_step(&a, 4500, 1, 1));
}
static void test_key_needs_armed(void) {
    autostart_t a;
    autostart_init(&a, 0, 1000, 2000);                      /* autostart off */
    assert(!autostart_press(&a, 0));
    autostart_init(&a, 1, 1000, 2000);
    autostart_cancel(&a);                                   /* Bluetooth command came first */
    assert(!autostart_press(&a, 0) && !autostart_step(&a, 5000, 1, 1));
}
static void test_key_tick_wrap(void) {
    autostart_t a;
    autostart_init(&a, 1, 1000, 2000);
    assert(autostart_press(&a, 0xFFFFFF00U));
    assert(!autostart_step(&a, 0xFFFFFF00U, 1, 1));
    assert(!autostart_step(&a, 0x000006CFU, 1, 1));         /* 1999 ms after the press */
    assert(autostart_step(&a, 0x000006D0U, 1, 1));
}
int main(void) {
    test_fires_once_after_the_hold(); test_a_break_restarts_the_hold();
    test_cancel_and_disabled(); test_tick_wrap();
    test_key_press_delays_the_start(); test_key_needs_armed(); test_key_tick_wrap();
    puts("Autostart tests passed: hold, once, restart on a break, cancel, disabled, tick wrap, mode key.");
    return 0;
}
