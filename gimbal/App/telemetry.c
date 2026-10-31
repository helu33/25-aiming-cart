#include "telemetry_link.h"
#include "gimbal_task.h"
#include "gimbal_tune.h"                               /* tune_yaw_fwd_deg, for the F reply */
#include <ctype.h>
#include <stdio.h>

/* Frames per transmission while replaying: 4 x 40 B = 14 ms at 115200 baud. */
#define DUMP_FRAMES_PER_SEND 4U

static telem_ring_t g_ring;                          /* 96 kB, AXI SRAM (DMA-free anyway) */
static uint8_t g_tx[TELEM_FRAME_BYTES * DUMP_FRAMES_PER_SEND];
static uint8_t g_rx[16];
static volatile uint8_t g_tx_busy;
/* Command line assembled in the RX ISR; handled in the task (Telemetry_Sample). */
#define LINE_CAPACITY 48U
static char g_line[LINE_CAPACITY], g_pending[LINE_CAPACITY];
static unsigned g_line_len;
static volatile uint8_t g_pending_ready;
/* Text reply ("#OK ...", "#ERR ...", "#P ..."), sent between frames before the next live one. */
#define REPLY_CAPACITY 384U   /* "#P ..." lists every tune_* value (~250 chars in 2026-10) */
static char g_reply[REPLY_CAPACITY];
static uint32_t g_reply_len, g_reply_off;   /* sent in frame-sized pieces (telem_reply_chunk) */
static uint32_t g_divider = 1U, g_tick;
static uint8_t g_live = 1U, g_dumping;
static uint32_t g_dump_next, g_dump_count;
/* Keil Watch: frames sent, samples whose live frame was skipped (link busy), replays. */
volatile uint32_t dbg_telem_frames, dbg_telem_skipped, dbg_telem_dumps, dbg_telem_init_ok;

void Telemetry_Init(void)
{
    TELEMETRY_UART->Init.BaudRate = TELEMETRY_BAUD;   /* CubeMX sets 921600 for the old link */
    dbg_telem_init_ok = HAL_UART_Init(TELEMETRY_UART) == HAL_OK;
    telem_ring_reset(&g_ring);
    g_divider = telem_live_divider(TELEMETRY_BAUD);
    (void)HAL_UARTEx_ReceiveToIdle_IT(TELEMETRY_UART, g_rx, sizeof(g_rx));
}

static void reply(const char *text)
{
    const int n = snprintf(g_reply, sizeof(g_reply), "\n%s\n", text);
    g_reply_len = n < 0 ? 0U : (uint32_t)n < sizeof(g_reply) ? (uint32_t)n : sizeof(g_reply) - 1U;
    g_reply_off = 0U;
}

static void handle_command(void)
{
    static char text[REPLY_CAPACITY - 4U];   /* "\n" + text + "\n" fits g_reply; static: off the stack */
    char line[LINE_CAPACITY], letter = 0, name[16];
    float value = 0.0f;
    memcpy(line, g_pending, sizeof(line));
    g_pending_ready = 0U;
    switch (telem_parse_line(line, &letter, name, sizeof(name), &value)) {
    case TELEM_LINE_SET: {
        const int r = Gimbal_SetTune(name, value);
        if (r == 0) snprintf(text, sizeof(text), "#OK %s=%.4g", name, (double)value);
        else snprintf(text, sizeof(text), "#ERR %s %s", name, r == -1 ? "unknown" : "out_of_range");
        reply(text);
        return;
    }
    case TELEM_LINE_LETTER: break;
    case TELEM_LINE_BAD: reply("#ERR syntax: D P L C G F | S <name> <value>"); return;
    default: return;
    }
    switch (letter) {
    case 'F':
        Gimbal_CaptureForward();
        snprintf(text, sizeof(text), "#OK yawfwd=%.2f", (double)tune_yaw_fwd_deg);
        reply(text);
        break;
    case 'G':
        memcpy(text, "#P ", 3);
        Gimbal_FormatTunes(text + 3, sizeof(text) - 3U);
        reply(text);
        break;
    case 'D':
        g_dumping = 1U; g_dump_next = 0U; g_dump_count = g_ring.count; ++dbg_telem_dumps;
        break;
    case 'P': g_live = 0U; break;
    case 'L': g_live = 1U; break;
    case 'C': if (!g_dumping) telem_ring_reset(&g_ring); break;
    default: break;
    }
}

static void send(uint32_t length)
{
    g_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(TELEMETRY_UART, g_tx, (uint16_t)length) != HAL_OK) g_tx_busy = 0U;
    else dbg_telem_frames += length / TELEM_FRAME_BYTES;
}

void Telemetry_Sample(const float ch[TELEM_CHANNELS])
{
    if (g_pending_ready) handle_command();
    if (!g_dumping) telem_ring_push(&g_ring, ch);
    if (g_tx_busy) { ++dbg_telem_skipped; return; }
    if (g_reply_len) {          /* replies go first, one frame-sized piece per sample, no frames */
        const uint32_t length = telem_reply_chunk(g_reply, g_reply_len, &g_reply_off, g_tx,
                                                  TELEM_FRAME_BYTES);
        if (g_reply_off >= g_reply_len) g_reply_len = g_reply_off = 0U;
        if (length) {
            g_tx_busy = 1U;
            if (HAL_UART_Transmit_IT(TELEMETRY_UART, g_tx, (uint16_t)length) != HAL_OK) g_tx_busy = 0U;
        }
        return;
    }

    if (g_dumping) {
        uint32_t length = 0U;
        float sample[TELEM_CHANNELS];
        while (g_dump_next < g_dump_count && length < sizeof(g_tx)) {
            const float t = telem_ring_get(&g_ring, g_dump_next++, sample);
            length += telem_frame(g_tx + length, t, sample);
        }
        if (length) send(length);
        else g_dumping = 0U;                          /* replay finished: back to live */
        return;
    }
    if (g_live && (++g_tick % g_divider) == 0U) {
        const float t = (float)(g_ring.total - 1U) / (float)TELEM_RATE_HZ;
        send(telem_frame(g_tx, t, ch));
    }
}

/* A line ends at CR/LF, or at an idle gap (terminals that send no line end): a full RX
 * buffer without a line end is the middle of a longer line. */
static void finish_line(void)
{
    if (!g_line_len) return;
    if (!g_pending_ready) {                           /* an unhandled line is not overwritten */
        memcpy(g_pending, g_line, g_line_len);
        g_pending[g_line_len] = '\0';
        g_pending_ready = 1U;
    }
    g_line_len = 0U;
}

void Telemetry_OnRxEvent(uint16_t size)
{
    uint8_t ended = 0U;
    for (uint16_t i = 0; i < size && i < sizeof(g_rx); ++i) {
        const char c = (char)g_rx[i];
        if (c == '\r' || c == '\n') { finish_line(); ended = 1U; continue; }
        if (g_line_len + 1U < LINE_CAPACITY) g_line[g_line_len++] = c;
    }
    if (!ended && size < sizeof(g_rx)) finish_line();
    (void)HAL_UARTEx_ReceiveToIdle_IT(TELEMETRY_UART, g_rx, sizeof(g_rx));
}

void Telemetry_OnError(void)
{
    (void)HAL_UART_AbortReceive(TELEMETRY_UART);
    __HAL_UART_CLEAR_OREFLAG(TELEMETRY_UART);
    __HAL_UART_CLEAR_NEFLAG(TELEMETRY_UART);
    __HAL_UART_CLEAR_FEFLAG(TELEMETRY_UART);
    __HAL_UART_CLEAR_PEFLAG(TELEMETRY_UART);
    if (TELEMETRY_UART->gState == HAL_UART_STATE_READY) g_tx_busy = 0U;   /* TX aborted too */
    (void)HAL_UARTEx_ReceiveToIdle_IT(TELEMETRY_UART, g_rx, sizeof(g_rx));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == TELEMETRY_UART) g_tx_busy = 0U;
}
