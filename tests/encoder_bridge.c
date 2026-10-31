#include "gimbal_protocol.h"
int test_parse(const char *frame)
{
    gimbal_packet_t packet;
    if (!GimbalProtocol_Parse(frame, &packet)) return 0;
    return packet.kind == GIMBAL_PACKET_VISION ? 1 : 2;
}
