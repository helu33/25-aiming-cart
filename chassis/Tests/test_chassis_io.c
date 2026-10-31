#include "bsp_line.h"
#include "bsp_gimbal_link.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef TEST_GIMBAL_RECEIVER
#include "gimbal_protocol.h"
#endif

TestDWT test_dwt;
TestCoreDebug test_debug;
uint32_t SystemCoreClock=72000000;
UART_HandleTypeDef huart4={UART4}, huart2={USART2}, huart5={UART5};
static uint32_t now;
static uint8_t address_bits, sensor_mask;
static uint8_t *inflight;
static char saved[192];
static unsigned submissions, aborts;
static HAL_StatusTypeDef send_status=HAL_OK;
uint32_t HAL_GetTick(void) { return now; }
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *pin) {
    assert((port==GPIOC && pin->Pin==7 && pin->Mode==GPIO_MODE_OUTPUT_PP) ||
           (port==GPIOB && pin->Pin==1 && pin->Mode==GPIO_MODE_INPUT));
}
void HAL_GPIO_WritePin(void *port,uint32_t pin,GPIO_PinState state) {
    assert(port==GPIOC && (pin&~7U)==0);
    if (state) address_bits|=(uint8_t)pin; else address_bits&=(uint8_t)~pin;
}
GPIO_PinState HAL_GPIO_ReadPin(void *port,uint32_t pin) {
    assert(port==GPIOB && pin==1);
    return (sensor_mask&(1U<<address_bits))?GPIO_PIN_SET:GPIO_PIN_RESET;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t len) {
    assert(uart==&GIMBAL_LINK_UART && len<sizeof(saved)); submissions++;
    inflight=data; memcpy(saved,data,len); saved[len]=0;
#ifdef TEST_GIMBAL_RECEIVER
    {
        gimbal_line_t line={0}; gimbal_packet_t packet;
        int complete=0;
        for (unsigned i=0;i<len;i++) complete=GimbalProtocol_Push(&line,data[i]);
        assert(complete==1 && GimbalProtocol_Parse(line.data,&packet));
        assert(packet.kind==GIMBAL_PACKET_CHASSIS && !packet.valid);
        assert(packet.yaw==0 && packet.pitch==0 && packet.roll==0 && packet.gx==0 && packet.gy==0 && packet.gz==0);
        assert(packet.turning==(strstr(saved,",1,0,0.000,")!=NULL));
    }
#endif
    return send_status;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart) {
    assert(uart==&GIMBAL_LINK_UART); aborts++; return HAL_OK;
}
static uint8_t *rx_target;
static unsigned rx_arms;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t len) {
    assert(uart==&GIMBAL_LINK_UART && len==1); rx_target=data; rx_arms++; return HAL_OK;
}
static void test_scans(void) {
    uint8_t mask; uint32_t seq;
    now=0; line_bsp_init(); sensor_mask=0xA5;
    assert(!line_bsp_snapshot(&mask,&seq));
    for (unsigned i=0;i<8;i++) {
        assert(address_bits==i);
        test_dwt.CYCCNT+=35999; line_bsp_poll(); assert(address_bits==i);
        test_dwt.CYCCNT++; line_bsp_poll();
        if (i<7) assert(!line_bsp_snapshot(&mask,&seq));
    }
    assert(line_bsp_snapshot(&mask,&seq) && mask==0xA5 && seq==1);
    sensor_mask=0x18;
    for (unsigned i=0;i<7;i++) { test_dwt.CYCCNT+=36000; line_bsp_poll(); }
    assert(line_bsp_snapshot(&mask,&seq) && mask==0xA5 && seq==1);
    test_dwt.CYCCNT+=36000; line_bsp_poll();
    assert(line_bsp_snapshot(&mask,&seq) && mask==0x18 && seq==2);
    now=31; assert(!line_bsp_snapshot(&mask,&seq));
    assert(!line_bsp_key_events());
    /* Cycle counter rollover must still honour the settling delay. */
    test_dwt.CYCCNT=0xFFFFF000U; line_bsp_poll();
    uint8_t ch=address_bits;
    test_dwt.CYCCNT+=35999; line_bsp_poll(); assert(address_bits==ch);
    test_dwt.CYCCNT++; line_bsp_poll(); assert(address_bits==(uint8_t)((ch+1)%8));
}
static void test_link(void) {
    now=0xFFFFFFF0U; gimbal_link_init(); assert(!gimbal_link_healthy());
    gimbal_link_poll(0); assert(submissions==1 && strstr(saved,"$C,3,0,0,0,")==saved);
    now+=5; gimbal_link_poll(1); assert(submissions==1 && !strcmp((char *)inflight,saved));
    gimbal_link_tx_complete(&GIMBAL_LINK_UART); assert(gimbal_link_healthy());
    gimbal_link_poll(1); assert(submissions==2 && strstr(saved,"$C,3,1,1,0,")==saved);
    gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    now+=19; gimbal_link_poll(1); assert(submissions==2);
    now++; gimbal_link_poll(1); assert(submissions==3);
    now+=51; gimbal_link_poll(1); assert(aborts==1 && submissions==4 && gimbal_link_failures()==1);
    now+=301; assert(!gimbal_link_healthy());
    /* UART errors are receive errors ($G): restart reception, leave the $C transmission alone. */
    gimbal_link_error(&GIMBAL_LINK_UART);
    assert(aborts==1 && gimbal_link_failures()==1 && gimbal_link_rx_errors()==1 && rx_arms==2);
    send_status=HAL_BUSY; gimbal_link_poll(0);           /* the stuck frame times out, then BUSY */
    assert(aborts==2 && gimbal_link_failures()==3);
    send_status=HAL_OK; now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(gimbal_link_healthy());
    /* The chassis gyro z rides along (gimbal two-loop yaw cancels the body rotation with it);
     * attitude is not measured and stays 0. Out-of-range rates are clamped, not dropped. */
    gimbal_link_set_gyro(1, 45.5f);
    now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(strstr(saved,",0,1,0.000,0.000,0.000,0.000,0.000,45.500,0,0,0*"));
    gimbal_link_set_gyro(1, 5000.0f);
    now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(strstr(saved,",2000.000,0,0,0*"));
    gimbal_link_set_gyro(0, 45.5f);
    now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(strstr(saved,",0,0,0.000,0.000,0.000,0.000,0.000,0.000,0,0,0*"));
    /* Encoder forward speed rides along in mm/s, whatever the IMU state. */
    gimbal_link_set_speed(0.2014f);
    now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(strstr(saved,",0.000,201,0,0*"));
    gimbal_link_set_speed(9.0f);
    now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(strstr(saved,",3000,0,0*"));
    /* Version 3: the circle task flag and the run progress (1/1000 lap) ride along too. */
    gimbal_link_set_task(1, 1250);
    now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(strstr(saved,",3000,1,1250*"));
    gimbal_link_set_task(0, 99999);                                         /* clamped */
    now+=20; gimbal_link_poll(0); gimbal_link_tx_complete(&GIMBAL_LINK_UART);
    assert(strstr(saved,",3000,0,9999*"));
}
static void feed(const char *text) {
    for (; *text; text++) { *rx_target=(uint8_t)*text; gimbal_link_rx_complete(&GIMBAL_LINK_UART); }
}
/* $G from the gimbal on the same UART's RX (power-up auto-start): READY counts for
 * CHASSIS_GIMBAL_STATUS_TIMEOUT_MS; bad frames, noise and a non-ready gimbal do not. */
static void test_status(void) {
    now=1000; gimbal_link_init(); assert(rx_arms>=1 && rx_target);
    gimbal_link_rx_poll(); assert(!gimbal_link_ready() && gimbal_link_status_frames()==0);
    feed("$G,1,1,7*22E4\r\n"); assert(!gimbal_link_ready());        /* parsed in the main loop */
    gimbal_link_rx_poll(); assert(gimbal_link_ready() && gimbal_link_status_flags()==7);
    now+=CHASSIS_GIMBAL_STATUS_TIMEOUT_MS; gimbal_link_rx_poll(); assert(gimbal_link_ready());
    now++; gimbal_link_rx_poll(); assert(!gimbal_link_ready());            /* stale */
    feed("$G,1,2,2*2B11\n"); gimbal_link_rx_poll();
    assert(!gimbal_link_ready() && gimbal_link_status_flags()==2);          /* not ready */
    feed("$G,1,3,7*0000\n"); gimbal_link_rx_poll(); assert(!gimbal_link_ready());   /* CRC */
    feed("xx\x01$G,1,3,7*4C84\n"); gimbal_link_rx_poll();                  /* resync at '$' */
    assert(gimbal_link_ready() && gimbal_link_status_frames()==3);
    for (unsigned i=0;i<100;i++) { feed("A"); if (i%16==15) gimbal_link_rx_poll(); } /* overlong */
    feed("\n$G,1,3,7*4C84\n"); now++; gimbal_link_rx_poll();
    assert(gimbal_link_ready() && gimbal_link_status_frames()==4);
}
int main(void) {
    test_scans(); test_link(); test_status();
    puts("I/O tests passed: confirmed pins, settling, complete scans, time wrap, TX lifetime and recovery, $G status.");
    return 0;
}
