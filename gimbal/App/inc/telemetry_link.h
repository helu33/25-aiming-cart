#ifndef TELEMETRY_LINK_H
#define TELEMETRY_LINK_H
/* Bluetooth telemetry glue on UART7 (see telemetry.h for the frame and channels).
 *
 * PC side: pair the module, open its COM port in VOFA+ with the "JustFloat" protocol at
 * TELEMETRY_BAUD. Command lines (any case, end with CR/LF or a pause):
 *   D  replay the on-board recording (last 60 s, oldest first), then resume live
 *   P  pause the live stream        L  resume the live stream
 *   C  clear the recording          G  report the tune_* values ("#P beta=0.17 ...")
 *   F  the laser points straight ahead now: record the yaw motor angle as chassis forward
 *      (vehicle-speed feed-forward, tune_veh_ff)
 *   S <name> <value>  set a tune_* value live, e.g. "S beta 0.3" -> "#OK beta=0.3"
 *      names: latency alpha beta ff kpscale twoloop kpos krate chassisff vehff yawfwd
 *      useage latencyx settle posmax turnhold pitchh rateclip (gimbal_task.c);
 *      they are not stored: a reset brings back the defaults.
 * Text replies start with '#' on their own line between the binary frames.
 * Recording continues while live streaming; it is frozen during a replay. */
#include <stdint.h>
#include "usart.h"
#include "telemetry.h"

#define TELEMETRY_UART      (&huart7)
/* Must match the Bluetooth module's UART setting. */
#ifndef TELEMETRY_BAUD
#define TELEMETRY_BAUD      115200U
#endif

void Telemetry_Init(void);
/* Every 10 ms from the gimbal task (not from an ISR). */
void Telemetry_Sample(const float ch[TELEM_CHANNELS]);
/* UART7 hooks, called from the shared HAL callbacks in gimbal_task.c. */
void Telemetry_OnRxEvent(uint16_t size);
void Telemetry_OnError(void);
#endif
