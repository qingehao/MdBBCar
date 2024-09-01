#ifndef __BSP_FOC_H__
#define __BSP_FOC_H__

#include "stdint.h"
#include "board_bsp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"

#ifndef BSP_FOC_MOTOR_NUM
    #warning "need define BSP_FOC_MOTOR_NUM"
    #define BSP_FOC_MOTOR_NUM 1
#endif

#ifndef BSP_FOC_ADC_CH_NUM
    #warning "need define BSP_FOC_ADC_CH_NUM"
    #define BSP_FOC_ADC_CH_NUM 7
#endif

#define BSP_FOC_ADC_CH_NOUSE 0xff

typedef struct
{
    uint8_t     timer_index;        // 定时器id
    uint8_t     phaseA_ch;          // A相PWM
    uint8_t     phaseB_ch;          // B相PWM
    uint8_t     phaseC_ch;          // C相PWM
    uint8_t     complementary_en;   // 使能互补PWM
    uint32_t    freq;               // pwm频率
} bsp_foc_pwm_config_t;

typedef struct
{
    uint8_t     adc_index;        // ADC id
    uint8_t     adc_ch_num;
    uint8_t     adc_ch_array[BSP_FOC_ADC_CH_NUM];
    struct
    {
        uint8_t iAch_idx;         // 通道A电流采样在buffer中的偏移
        uint8_t iBch_idx;
        uint8_t iCch_idx;
    } motor[BSP_FOC_MOTOR_NUM];
} bsp_foc_adc_config_t;

typedef struct
{
    bsp_foc_pwm_config_t pwm_config[BSP_FOC_MOTOR_NUM];
    bsp_foc_adc_config_t adc_config;
    uint8_t trigger_ch; // timer_index<<8|ch
} bsp_foc_config_t;

typedef struct
{
    TIM_HandleTypeDef htim;
    uint32_t    chA;
    uint32_t    chB;
    uint32_t    chC;
    uint32_t    period_cnt;
    uint8_t     complementary_en;   // 使能互补PWM
    uint8_t     trigger_en;
    uint32_t    chTrigger;
} bsp_foc_pwm_t;

typedef struct
{
    ADC_HandleTypeDef hadc;
    DMA_HandleTypeDef hdma;
    uint16_t *adc_buffer;
    uint16_t adc_buf_sz;
} bsp_foc_adc_t;

typedef struct
{
    bsp_foc_pwm_t foc_pwm[BSP_FOC_MOTOR_NUM];
    bsp_foc_adc_t foc_adc;

    uint8_t adc_trigger_source_idx;

    // uint16_t adc_buffer[BSP_FOC_ADC_CH_NUM];
    void (*cb)(void *arg, uint8_t *adc_data);
    void *cb_arg;

    uint8_t is_init;
} bsp_foc_t;

/**
 * @brief 初始化foc驱动
 */
void bsp_foc_init();

/**
 * @brief 获得foc句柄
 *
 * @return bsp_foc_t*
 */
bsp_foc_t *bsp_foc_request();

/**
 * @brief foc开始 开始ADC采样
 */
void bsp_foc_start();

/**
 * @brief foc停止 停止ADC采样
 */
void bsp_foc_stop(bsp_foc_t *bsp_foc);

/**
 * @brief
 *
 * @param motor_id  电机id
 * @param en        0 -- 失能
 *                  1 -- 使能
 */
void bsp_foc_pwm_enable(uint8_t motor_id, uint8_t en);

/**
 * @brief 设置foc adc采样完成回调函数
 *
 * @param cb      采样完成回调函数
 * @param arg     参数
 */
void bsp_foc_set_callback(void (*cb)(void *arg, uint8_t *adc_data), void *arg);

/**
 * @brief 设置电机PWM占空比
 *
 * @param bsp_foc  foc句柄
 * @param motor_id 电机id
 * @param dutyA    A相占空比
 * @param dutyB    B相占空比
 * @param dutyC    C相占空比
 */
void bsp_foc_set_pwm(uint8_t motor_id, float dutyA, float dutyB, float dutyC);

/**
 * @brief 设置电机的PWM开启时间
 *
 * @param motor_id 电机id
 * @param ta       A相开启时间
 * @param tb       B相开启时间
 * @param tc       C相开启时间
 */
void bsp_foc_set_width(uint8_t motor_id, uint32_t ta, uint32_t tb, uint32_t tc);

/**
 * @brief 获得当前us时间戳
 *
 * @return uint32_t
 */
uint32_t bsp_foc_cur_us(void);

/**
 * @brief 测试用函数
 */
void bsp_foc_test();

#ifdef __cplusplus
}
#endif
#endif