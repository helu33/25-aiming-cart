#ifndef BSP_LINE_H
#define BSP_LINE_H
#include <stdint.h>
void line_bsp_init(void);
void line_bsp_poll(void);
uint8_t line_bsp_snapshot(uint8_t *mask, uint32_t *sequence);
/* Debounced press edges: bit0 laps, bit1 run, bit2 stop. */
uint8_t line_bsp_key_events(void);
#endif
