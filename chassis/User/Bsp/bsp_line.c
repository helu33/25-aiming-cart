#include "bsp_line.h"
#include "main.h"
#include "chassis_config.h"

static uint8_t channel, assembling, published, ready;
static uint32_t changed_cycles, published_ms, scan_sequence;
#if CHASSIS_KEYS_ENABLE
static uint8_t key_raw, key_stable;
static uint32_t key_changed_ms;
#endif

static void address(uint8_t ch) {
    HAL_GPIO_WritePin(CHASSIS_LINE_ADDRESS_PORT,CHASSIS_LINE_AD0,(ch&1)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CHASSIS_LINE_ADDRESS_PORT,CHASSIS_LINE_AD1,(ch&2)?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CHASSIS_LINE_ADDRESS_PORT,CHASSIS_LINE_AD2,(ch&4)?GPIO_PIN_SET:GPIO_PIN_RESET);
    changed_cycles=DWT->CYCCNT;
}
void line_bsp_init(void) {
    GPIO_InitTypeDef pin={0};
    channel=assembling=published=ready=0; scan_sequence=0;
#if CHASSIS_KEYS_ENABLE
    key_raw=key_stable=0; key_changed_ms=HAL_GetTick();
#endif
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    pin.Pin=CHASSIS_LINE_AD0|CHASSIS_LINE_AD1|CHASSIS_LINE_AD2;
    pin.Mode=GPIO_MODE_OUTPUT_PP; pin.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(CHASSIS_LINE_ADDRESS_PORT,pin.Pin,GPIO_PIN_RESET);
    HAL_GPIO_Init(CHASSIS_LINE_ADDRESS_PORT,&pin);
    pin.Pin=CHASSIS_LINE_OUT; pin.Mode=GPIO_MODE_INPUT; pin.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(CHASSIS_LINE_OUT_PORT,&pin);
#if CHASSIS_KEYS_ENABLE
    pin.Pin=CHASSIS_KEY_LAPS|CHASSIS_KEY_RUN|CHASSIS_KEY_STOP;
    HAL_GPIO_Init(CHASSIS_KEY_PORT,&pin);
#endif
    address(0);
}
void line_bsp_poll(void) {
    if ((uint32_t)(DWT->CYCCNT-changed_cycles)<(SystemCoreClock/1000000U)*CHASSIS_MUX_SETTLE_US) return;
    if (HAL_GPIO_ReadPin(CHASSIS_LINE_OUT_PORT,CHASSIS_LINE_OUT)==CHASSIS_LINE_BLACK_LEVEL)
        assembling|=(uint8_t)(1U<<channel);
    channel++;
    if (channel>=8) {
        published=assembling; assembling=0; channel=0;
        published_ms=HAL_GetTick(); scan_sequence++; ready=1;
    }
    address(channel);
}
uint8_t line_bsp_snapshot(uint8_t *mask, uint32_t *sequence) {
    *mask=published; *sequence=scan_sequence;
    return ready && (uint32_t)(HAL_GetTick()-published_ms)<=CHASSIS_LINE_MAX_AGE_MS;
}
uint8_t line_bsp_key_events(void) {
#if CHASSIS_KEYS_ENABLE
    uint8_t raw=0, presses=0;
    uint32_t now=HAL_GetTick();
    if (HAL_GPIO_ReadPin(CHASSIS_KEY_PORT,CHASSIS_KEY_LAPS)==GPIO_PIN_RESET) raw|=1;
    if (HAL_GPIO_ReadPin(CHASSIS_KEY_PORT,CHASSIS_KEY_RUN)==GPIO_PIN_RESET) raw|=2;
    if (HAL_GPIO_ReadPin(CHASSIS_KEY_PORT,CHASSIS_KEY_STOP)==GPIO_PIN_RESET) raw|=4;
    if (raw!=key_raw) { key_raw=raw; key_changed_ms=now; }
    if ((uint32_t)(now-key_changed_ms)>=20 && raw!=key_stable) {
        presses=raw&(uint8_t)~key_stable; key_stable=raw;
    }
    return presses;
#else
    return 0;
#endif
}

