#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "stdint.h"


#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx.h"
#define MD_BSP_PWM_CH1_EN (0x0001)
#define MD_BSP_PWM_CH2_EN (0x0001<<1)
#define MD_BSP_PWM_CH3_EN (0x0001<<2)
#define MD_BSP_PWM_CH4_EN (0x0001<<3)
#define MD_BSP_PWM_CH5_EN (0x0001<<4)
#define MD_BSP_PWM_CH6_EN (0x0001<<5)

#define MD_BSP_PWM_CH1C_EN (0x00010000)
#define MD_BSP_PWM_CH2C_EN (0x00010000<<1)
#define MD_BSP_PWM_CH3C_EN (0x00010000<<2)
#define MD_BSP_PWM_CH4C_EN (0x00010000<<3)
#define MD_BSP_PWM_CH5C_EN (0x00010000<<4)
#define MD_BSP_PWM_CH6C_EN (0x00010000<<5)

typedef struct
{
    uint32_t freq; // PWM频率
    uint32_t ch_en;
} md_bsp_pwm_config_t;

typedef struct
{
    md_bsp_pwm_config_t *cfg;
    uint8_t              index;
    TIM_HandleTypeDef    htim;
    uint32_t             period_cnt;
} bsp_pwm_dev_t;

void bsp_pwm_init();
bsp_pwm_dev_t *bsp_pwm_request(uint8_t index);
int32_t bsp_pwm_enable(bsp_pwm_dev_t *pwm_dev, uint8_t ch, uint8_t en);
int32_t bsp_pwm_set_width(bsp_pwm_dev_t *pwm_dev, uint8_t ch, uint32_t width);
int32_t bsp_pwm_set_duty(bsp_pwm_dev_t *pwm_dev, uint8_t ch, float duty);
void bsp_pwm_test();

#ifdef __cplusplus
}
#endif
#endif