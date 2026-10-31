#include "bsp_gimbal_link.h"
#include "chassis_protocol.h"
#include "chassis_config.h"
#include "usart.h"

static char tx[192];
static volatile uint8_t busy, completed;
static volatile uint32_t last_completed_ms, failures;
static uint32_t started_ms, last_submit_ms;
static uint16_t sequence;
static uint8_t previous_mode;
static uint8_t gyro_valid;
static float gyro_z_deg_s, speed_m_s;
static uint8_t task;
static uint16_t progress_permille;
/* $G status from the gimbal on this UART's RX: one byte per interrupt into a ring, lines
 * assembled and parsed in the main loop (gimbal_link_rx_poll). */
#define RX_RING_SIZE 64U
static uint8_t rx_byte;
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head;
static uint8_t rx_tail;
static volatile uint32_t rx_errors;
static char rx_line[32];
static uint8_t rx_len, rx_dropping;
static uint8_t status_flags, status_seen;
static uint32_t status_ms, status_frames;

static void rx_arm(void) { (void)HAL_UART_Receive_IT(&GIMBAL_LINK_UART,&rx_byte,1U); }
void gimbal_link_init(void) {
    busy=completed=0; failures=0; sequence=0; previous_mode=255; gyro_valid=0; gyro_z_deg_s=0.0f; speed_m_s=0.0f; task=0; progress_permille=0;
    last_submit_ms=HAL_GetTick()-CHASSIS_LINK_PERIOD_MS;
    rx_head=rx_tail=0; rx_errors=0; rx_len=0; rx_dropping=0;
    status_flags=0; status_seen=0; status_frames=0;
    rx_arm();
}
void gimbal_link_rx_complete(UART_HandleTypeDef *uart) {
    if (uart->Instance!=GIMBAL_LINK_INSTANCE) return;
    if ((uint8_t)((rx_head+1U)%RX_RING_SIZE)!=rx_tail) {
        rx_ring[rx_head]=rx_byte;
        rx_head=(uint8_t)((rx_head+1U)%RX_RING_SIZE);
    }
    rx_arm();
}
static void rx_line_done(void) {
    unsigned flags;
    rx_line[rx_len]=0;
    if (!rx_dropping && rx_len && chassis_decode_gimbal_status(rx_line,&flags)) {
        status_flags=(uint8_t)flags; status_seen=1; status_ms=HAL_GetTick(); status_frames++;
    }
    rx_len=0; rx_dropping=0;
}
void gimbal_link_rx_poll(void) {
    while (rx_tail!=rx_head) {
        const char c=(char)rx_ring[rx_tail];
        rx_tail=(uint8_t)((rx_tail+1U)%RX_RING_SIZE);
        if (c=='$') { rx_len=0; rx_dropping=0; }                    /* resync on a frame start */
        if (c=='\n') { rx_line_done(); continue; }
        if (c=='\r') continue;
        if (rx_len<sizeof(rx_line)-1U) rx_line[rx_len++]=c; else rx_dropping=1;
    }
}
uint8_t gimbal_link_ready(void) {
    return status_seen && (status_flags&CHASSIS_GIMBAL_STATUS_READY) &&
           (uint32_t)(HAL_GetTick()-status_ms)<=CHASSIS_GIMBAL_STATUS_TIMEOUT_MS;
}
uint8_t gimbal_link_status_flags(void) { return status_flags; }
uint32_t gimbal_link_status_frames(void) { return status_frames; }
uint32_t gimbal_link_rx_errors(void) { return rx_errors; }
void gimbal_link_poll(uint8_t hold) {
    uint32_t now=HAL_GetTick(); int length;
    if (busy) {
        if ((uint32_t)(now-started_ms)<=50U) return;
        (void)HAL_UART_AbortTransmit(&GIMBAL_LINK_UART); busy=0; failures++;
    }
    if (hold==previous_mode && (uint32_t)(now-last_submit_ms)<CHASSIS_LINK_PERIOD_MS) return;
    /* Only gyro z is measured (attitude and gx/gy stay 0); imu_valid now means "gz valid". */
    length=chassis_encode(tx,sizeof(tx),sequence++,hold?1U:0U,gyro_valid,0,0,0,0,0,gyro_z_deg_s,speed_m_s,
                          task,progress_permille);
    last_submit_ms=now;
    if (length<=0) { failures++; return; }
    started_ms=now; busy=1;
    if (HAL_UART_Transmit_IT(&GIMBAL_LINK_UART,(uint8_t *)tx,(uint16_t)length)!=HAL_OK) {
        busy=0; failures++; return;
    }
    previous_mode=hold;
}
void gimbal_link_tx_complete(UART_HandleTypeDef *uart) {
    if (uart->Instance==GIMBAL_LINK_INSTANCE) { last_completed_ms=HAL_GetTick(); completed=1; busy=0; }
}
/* UART errors (overrun, framing, noise, parity) are all receive-side: the HAL has stopped the
 * reception, so restart it. A $C transmission in flight is not affected (busy times out). */
void gimbal_link_error(UART_HandleTypeDef *uart) {
    if (uart->Instance==GIMBAL_LINK_INSTANCE) { rx_errors++; rx_arm(); }
}
uint8_t gimbal_link_healthy(void) {
    return completed && (uint32_t)(HAL_GetTick()-last_completed_ms)<=CHASSIS_LINK_TIMEOUT_MS;
}
uint32_t gimbal_link_failures(void) { return failures; }
void gimbal_link_set_speed(float value) {
    if (!(value==value)) value=0.0f;
    if (value>3.0f) value=3.0f;
    if (value<-3.0f) value=-3.0f;
    speed_m_s=value;
}
void gimbal_link_set_gyro(uint8_t valid, float gz_deg_s) {
    if (!(gz_deg_s==gz_deg_s)) valid=0;               /* NaN */
    if (gz_deg_s>2000.0f) gz_deg_s=2000.0f;            /* protocol range */
    if (gz_deg_s<-2000.0f) gz_deg_s=-2000.0f;
    gyro_valid=valid?1U:0U; gyro_z_deg_s=valid?gz_deg_s:0.0f;
}
void gimbal_link_set_task(uint8_t circle, uint32_t permille) {
    task=circle?1U:0U;
    progress_permille=(uint16_t)(permille>9999U?9999U:permille);
}
