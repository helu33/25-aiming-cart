/* Telemetry framing (VOFA+ JustFloat) and the on-board recorder. */
#include "telemetry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static telem_ring_t ring;   /* large: keep off the stack */

static void test_justfloat_frame_layout(void)
{
    float ch[TELEM_CHANNELS] = {1.5f, -2.0f, 3.0f, 0.0f, 4.25f, -5.5f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f};
    uint8_t out[TELEM_FRAME_BYTES];
    assert(telem_frame(out, 12.5f, ch) == TELEM_FRAME_BYTES);
    float t, first, last;
    memcpy(&t, out, 4);
    memcpy(&first, out + 4, 4);
    memcpy(&last, out + 4 * TELEM_CHANNELS, 4);
    assert(t == 12.5f && first == 1.5f && last == 11.0f);
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};      /* JustFloat frame tail */
    assert(memcmp(out + TELEM_FRAME_BYTES - 4, tail, 4) == 0);
}

static void test_packing_round_trip_and_saturation(void)
{
    float in[TELEM_CHANNELS] = {1.23f, -4.56f, 5.0f, 20.0f, -0.07f, 26.66f, 130.0f, 0.15f, -18.37f, 112.3f, 4.0f, 1.23f};
    telem_packed_t p = telem_pack(in);
    float out[TELEM_CHANNELS];
    telem_unpack(&p, out);
    for (unsigned i = 0; i < TELEM_CHANNELS; ++i)
        assert(fabsf(out[i] - in[i]) <= 0.5f / telem_scale(i) + 1e-4f);
    float big[TELEM_CHANNELS] = {1e6f, -1e6f, NAN, 0, 0, 0, 0, 0};
    p = telem_pack(big);
    assert(p.v[0] == INT16_MAX && p.v[1] == -INT16_MAX && p.v[2] == 0);
}

static void test_ring_keeps_the_newest_in_order(void)
{
    telem_ring_reset(&ring);
    float ch[TELEM_CHANNELS] = {0};
    const uint32_t extra = 250;
    for (uint32_t k = 0; k < TELEM_RECORD_LEN + extra; ++k) {
        ch[0] = (float)(k % 300);                 /* identifiable, within int16 at x100 */
        telem_ring_push(&ring, ch);
    }
    assert(ring.count == TELEM_RECORD_LEN);
    float out[TELEM_CHANNELS];
    float t0 = telem_ring_get(&ring, 0, out);
    assert(fabsf(out[0] - (float)(extra % 300)) < 0.01f);                 /* oldest kept */
    assert(fabsf(t0 - (float)extra / TELEM_RATE_HZ) < 1e-3f);             /* its time */
    float tn = telem_ring_get(&ring, ring.count - 1, out);
    assert(fabsf(out[0] - (float)((TELEM_RECORD_LEN + extra - 1) % 300)) < 0.01f);
    assert(fabsf(tn - t0 - (float)(TELEM_RECORD_LEN - 1) / TELEM_RATE_HZ) < 1e-2f);
}

static void test_live_divider_fits_the_link(void)
{
    /* 100 Hz x 40-byte frames = 4 kB/s: full rate from 57600 up, decimated below. */
    assert(telem_live_divider(921600U) == 1U);
    assert(telem_live_divider(115200U) == 2U);   /* HC-05: 50 Hz live, see TELEM_LINK_USE */
    assert(telem_live_divider(38400U) >= 2U);
    unsigned d = telem_live_divider(9600U);
    assert(d >= 5U && (float)TELEM_RATE_HZ / (float)d * TELEM_FRAME_BYTES * 10.0f <= 9600.0f);
}

static void test_command_lines(void)
{
    char letter = 0, name[12];
    float value = 0.0f;
    assert(telem_parse_line("  \r\n", &letter, name, sizeof(name), &value) == TELEM_LINE_NONE);
    assert(telem_parse_line("d", &letter, name, sizeof(name), &value) == TELEM_LINE_LETTER && letter == 'D');
    assert(telem_parse_line("G\r\n", &letter, name, sizeof(name), &value) == TELEM_LINE_LETTER && letter == 'G');
    assert(telem_parse_line("S beta 0.3\n", &letter, name, sizeof(name), &value) == TELEM_LINE_SET);
    assert(letter == 'S' && !strcmp(name, "beta") && fabsf(value - 0.3f) < 1e-6f);
    assert(telem_parse_line("s chassisff -1", &letter, name, sizeof(name), &value) == TELEM_LINE_SET);
    assert(!strcmp(name, "chassisff") && value == -1.0f);
    assert(telem_parse_line("S beta", &letter, name, sizeof(name), &value) == TELEM_LINE_BAD);
    assert(telem_parse_line("S beta x", &letter, name, sizeof(name), &value) == TELEM_LINE_BAD);
    assert(telem_parse_line("S beta 0.3 9", &letter, name, sizeof(name), &value) == TELEM_LINE_BAD);
    assert(telem_parse_line("D now", &letter, name, sizeof(name), &value) == TELEM_LINE_BAD);
    assert(telem_parse_line("S averyveryverylongname 2", &letter, name, sizeof(name), &value) == TELEM_LINE_SET);
    assert(strlen(name) == sizeof(name) - 1U);                         /* truncated, no overflow */
}

static void test_reply_goes_out_in_frame_sized_pieces(void)
{
    /* 149-byte "#P ..." reply in frame-sized pieces (56 + 56 + 37), nothing lost or repeated, then 0. */
    char reply[150];
    memset(reply, 'x', sizeof(reply));
    reply[0] = '\n'; reply[148] = '\n'; reply[149] = '\0';
    uint8_t out[TELEM_FRAME_BYTES], joined[160];
    uint32_t off = 0U, total = 0U, n, pieces = 0U;
    while ((n = telem_reply_chunk(reply, 149U, &off, out, TELEM_FRAME_BYTES)) != 0U) {
        assert(n <= TELEM_FRAME_BYTES);
        memcpy(joined + total, out, n);
        total += n; ++pieces;
    }
    assert(pieces == (149U + TELEM_FRAME_BYTES - 1U) / TELEM_FRAME_BYTES && total == 149U && off == 149U && !memcmp(joined, reply, 149U));
    assert(telem_reply_chunk(reply, 149U, &off, out, TELEM_FRAME_BYTES) == 0U);
}

int main(void)
{
    test_command_lines();
    test_reply_goes_out_in_frame_sized_pieces();
    test_justfloat_frame_layout();
    test_packing_round_trip_and_saturation();
    test_ring_keeps_the_newest_in_order();
    test_live_divider_fits_the_link();
    puts("Telemetry tests passed: JustFloat frame, int16 packing, recorder ring, live divider, commands, reply pieces.");
    return 0;
}
