#include "gimbal_protocol.h"
#include "chassis_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static bool parse_body(const char *body, gimbal_packet_t *packet)
{
    char frame[192];
    snprintf(frame, sizeof(frame), "$%s*%04X", body,
             (unsigned)GimbalProtocol_Crc16(body, strlen(body)));
    return GimbalProtocol_Parse(frame, packet);
}

int main(void)
{
    gimbal_packet_t p;
    assert(GimbalProtocol_Crc16("123456789", 9) == 0x29B1);
    assert(parse_body("V,1,65535,1,10.25,-2.5", &p));
    assert(p.kind == GIMBAL_PACKET_VISION && p.valid && p.seq == 65535);
    assert(p.yaw == 10.25f && p.pitch == -2.5f);
    assert(parse_body("V,1,0,0,0,0", &p) && !p.valid);
    assert(parse_body("C,1,12,1,1,90,-5,2,0,0,45", &p));
    assert(p.kind == GIMBAL_PACKET_CHASSIS && p.turning && p.valid && p.gz == 45);
    assert(parse_body("C,1,13,0,0,0,0,0,0,0,0", &p) && !p.valid && !p.turning);
    assert(!p.has_speed);
    /* Version 2: V adds range_mm and age_ms, C adds speed_mm_s; version 1 stays accepted. */
    assert(parse_body("V,2,7,1,1.5,-0.5,812,43", &p) && p.valid && p.has_range);
    assert(p.yaw == 1.5f && fabsf(p.range_m - 0.812f) < 1e-6f && p.age_ms == 43);
    assert(parse_body("V,2,8,0,0,0,0,40", &p) && !p.valid && p.range_m == 0.0f);
    assert(parse_body("C,2,14,0,1,0,0,0,0,0,-30.5,201", &p) && p.has_speed);
    assert(p.gz == -30.5f && fabsf(p.speed_m_s - 0.201f) < 1e-6f);
    assert(parse_body("V,1,9,1,1,1", &p) && !p.has_range);
    /* Version 3 (V only): code and area_x100 for the telemetry. */
    assert(parse_body("V,3,10,0,0,0,0,40,2,123", &p) && !p.valid && p.has_range && p.has_diag);
    assert(p.vision_code == 2 && p.area_x100 == 123 && p.age_ms == 40);
    assert(parse_body("V,3,11,1,0.5,0,900,35,0,10000", &p) && p.valid && p.area_x100 == 10000);
    assert(parse_body("V,2,12,1,0,0,900,35", &p) && !p.has_diag);
    const char *bad[] = {
        "V,2,1,1,0,0", "V,1,65536,1,0,0", "V,1,-1,1,0,0", "V,1,1,2,0,0",
        "V,1,1,1,nan,0", "V,1,1,1,inf,0", "V,1,1,1,1e99,0", "V,1,1,1,181,0",
        "V,1,1,1,1,", "V,1,1,1,0,0,0", "C,1,1,2,1,0,0,0,0,0,0",
        "C,1,1,0,1,0,0,0,0,0,2001", "C,1,1,0,1,0,0,0,0,0", "X,1,1,0,0",
        "V,3,1,1,0,0,0,0", "V,2,1,1,0,0,812", "V,2,1,1,0,0,-1,40", "V,2,1,1,0,0,812,1001",
        "V,2,1,1,0,0,8.5,40", "C,2,1,0,1,0,0,0,0,0,0", "C,2,1,0,1,0,0,0,0,0,0,3001",
        "C,1,1,0,1,0,0,0,0,0,0,200",
        "V,3,1,0,0,0,0,40,2", "V,3,1,0,0,0,0,40,100,0", "V,3,1,0,0,0,0,40,2,10001",
        "V,3,1,0,0,0,0,40,2,1.5", "V,3,1,0,0,0,0,40,2,5,0", "C,3,1,0,1,0,0,0,0,0,0,200",
        "V,4,1,0,0,0,0,40,2,5"
    };
    for (size_t i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) assert(!parse_body(bad[i], &p));
    assert(!GimbalProtocol_Parse("$V,1,1,1,0,0*0000", &p));
    assert(!GimbalProtocol_Parse("$V,1,1,1,0,0*ZZZZ", &p));
    assert(GimbalProtocol_Parse("10,-2,0,0", &p) && p.valid);
    assert(GimbalProtocol_Parse("404,404", &p) && !p.valid);
    assert(GimbalProtocol_Parse("404,2,0,0", &p) && !p.valid);
    assert(!GimbalProtocol_Parse("10,abc", &p));
    assert(!GimbalProtocol_Parse("10,", &p));
    assert(!GimbalProtocol_Parse(",2", &p));
    assert(!GimbalProtocol_Parse("inf,0", &p));
    assert(!GimbalProtocol_Parse("nan,0", &p));
    assert(!GimbalProtocol_Parse("10,2junk", &p));
    assert(!GimbalProtocol_Parse("10,2,0,garbage", &p));
    assert(!GimbalProtocol_Parse("0x10,0", &p));
    assert(!GimbalProtocol_Parse("10,2,0", &p));
    assert(GimbalProtocol_Parse("T,T,T,T", &p) && p.turning);
    assert(GimbalProtocol_Parse("N,N,N,N", &p) && !p.turning);
    assert(!GimbalProtocol_Parse("T,T,T,Tjunk", &p));

    gimbal_line_t line = {0};
    for (unsigned n = 0; n < 191; ++n) assert(GimbalProtocol_Push(&line, '1') == 0);
    assert(GimbalProtocol_Push(&line, '2') == -1);
    assert(GimbalProtocol_Push(&line, '3') == 0);
    assert(GimbalProtocol_Push(&line, '\n') == 0);
    const char *stream = "10,-2\r\n404,404\n";
    unsigned frames = 0;
    for (const char *c = stream; *c; ++c) {
        if (GimbalProtocol_Push(&line, (uint8_t)*c) == 1) {
            assert(GimbalProtocol_Parse(line.data, &p));
            ++frames;
        }
    }
    assert(frames == 2);
    assert(GimbalProtocol_Push(&line, 0) == -1);
    assert(GimbalProtocol_Push(&line, '\n') == 0);
    GimbalProtocol_Discard(&line);
    assert(GimbalProtocol_Push(&line, '1') == 0);
    assert(GimbalProtocol_Push(&line, '\n') == 0);

    gimbal_sequence_t seq = {0};
    assert(GimbalProtocol_AcceptSequence(&seq, 65535, 100, 300));
    assert(GimbalProtocol_AcceptSequence(&seq, 0, 110, 300));
    assert(!GimbalProtocol_AcceptSequence(&seq, 0, 120, 300));
    assert(!GimbalProtocol_AcceptSequence(&seq, 65535, 130, 300));
    assert(!GimbalProtocol_AcceptSequence(&seq, 0, 500, 300));
    assert(GimbalProtocol_AcceptSequence(&seq, 65535, 500, 300));
    seq = (gimbal_sequence_t){true, 10, 0xFFFFFFF0U};
    assert(!GimbalProtocol_AcceptSequence(&seq, 9, 10, 300));
    assert(GimbalProtocol_AcceptSequence(&seq, 11, 10, 300));

    /* $G gimbal -> chassis status: encoded by the gimbal, decoded by the chassis firmware. */
    {
        char out[32];
        unsigned flags = 0;
        const int n = GimbalProtocol_EncodeStatus(out, sizeof(out), 7, GIMBAL_STATUS_READY |
                                                  GIMBAL_STATUS_CONTROL | GIMBAL_STATUS_VISION);
        char expected[32];
        snprintf(expected, sizeof(expected), "$G,1,7,7*%04X\n",
                 (unsigned)GimbalProtocol_Crc16("G,1,7,7", 7));
        assert(n == (int)strlen(expected) && !strcmp(out, expected));
        out[n - 1] = '\0';                                       /* the receiver strips LF */
        assert(chassis_decode_gimbal_status(out, &flags) == 1 && flags == 7U);
        assert(GimbalProtocol_EncodeStatus(out, sizeof(out), 65535, GIMBAL_STATUS_CONTROL) > 0);
        out[strlen(out) - 1] = '\0';
        assert(chassis_decode_gimbal_status(out, &flags) == 1 && flags == GIMBAL_STATUS_CONTROL);
        assert(GimbalProtocol_EncodeStatus(out, 8, 1, 0) < 0);   /* too small: nothing sent */
        assert(chassis_decode_gimbal_status("$G,1,7,7*0000", &flags) == 0);          /* CRC */
        snprintf(out, sizeof(out), "$G,2,7,7*%04X", (unsigned)GimbalProtocol_Crc16("G,2,7,7", 7));
        assert(chassis_decode_gimbal_status(out, &flags) == 0);                       /* version */
        snprintf(out, sizeof(out), "$G,1,7,x*%04X", (unsigned)GimbalProtocol_Crc16("G,1,7,x", 7));
        assert(chassis_decode_gimbal_status(out, &flags) == 0);                       /* digits */
        assert(chassis_decode_gimbal_status("$C,1,7,7*0000", &flags) == 0);          /* not $G */
    }
    /* $C version 3: + circle task flag and run progress (1/1000 lap). */
    {
        gimbal_packet_t c3;
        assert(parse_body("C,3,9,0,1,0,0,0,0,0,12.5,350,1,1250", &c3));
        assert(c3.kind == GIMBAL_PACKET_CHASSIS && c3.has_speed && fabsf(c3.speed_m_s - 0.35f) < 1e-6f);
        assert(c3.has_task && c3.task == 1 && c3.progress_permille == 1250);
        assert(parse_body("C,2,9,0,1,0,0,0,0,0,12.5,350", &c3) && !c3.has_task);
        assert(!parse_body("C,3,9,0,1,0,0,0,0,0,12.5,350,2,0", &c3));        /* task 0/1 */
        assert(!parse_body("C,3,9,0,1,0,0,0,0,0,12.5,350,0,10000", &c3));    /* progress */
        assert(!parse_body("C,3,9,0,1,0,0,0,0,0,12.5,350,0", &c3));          /* field count */
    }
    /* $K gimbal -> K230: the task and progress, decoded by vision/protocol.py (test_encoders.py). */
    {
        char out[40], expected[40];
        const int n = GimbalProtocol_EncodeTask(out, sizeof(out), 12, 1, 2500);
        snprintf(expected, sizeof(expected), "$K,1,12,1,2500*%04X\n",
                 (unsigned)GimbalProtocol_Crc16("K,1,12,1,2500", 13));
        assert(n == (int)strlen(expected) && !strcmp(out, expected));
        assert(GimbalProtocol_EncodeTask(out, 10, 12, 1, 2500) < 0);
        assert(GimbalProtocol_EncodeTask(out, sizeof(out), 12, 2, 0) < 0);
        assert(GimbalProtocol_EncodeTask(out, sizeof(out), 12, 0, 10000) < 0);
    }
    puts("Protocol tests passed: CRC, strict fields, legacy, framing, overflow recovery, sequence/wrap, $G status, $C v3, $K task.");
    return 0;
}
