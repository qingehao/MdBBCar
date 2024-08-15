#ifndef __BSP_TICK_H__
#define __BSP_TICK_H__

#include "stdint.h"
#include "stm32h7xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_TICK_TIMER_INDEX 4
#define BSP_TICK_IRQn TIM4_IRQn
#define BSP_TICK_IRQ_HANDLER

typedef struct
{
    uint8_t     timer_index;
    uint32_t    timer_cnt_freq;
    uint32_t    timer_irq_freq;
} bsp_tick_config_t;

void bsp_tick_init();
uint64_t tim_clock_get(TIM_HandleTypeDef *htim);

uint64_t bsp_tick_get_us();
uint32_t bsp_tick_get_ms();

void bsp_tick_delay_us(uint32_t us);
void bsp_tick_delay_ms(uint32_t ms);

/**
 * @brief 设置tick中断回调函数
 *
 * @param tick_irq_cb 回调函数
 */
void bsp_tick_set_irq_cb(void (*tick_irq_cb)());

void bsp_tick_test(uint32_t pin);

#ifdef __cplusplus
}
#endif
#endif
