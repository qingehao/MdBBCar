#include "bsp_pwm.h"
#include "bsp_tick.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

#define MD_BSP_PWM1_CONFIG \
{ \
    .freq=20000, \
    .ch_en=MD_BSP_PWM_CH1_EN|MD_BSP_PWM_CH1C_EN\
            |MD_BSP_PWM_CH2_EN|MD_BSP_PWM_CH2C_EN\
            |MD_BSP_PWM_CH3_EN|MD_BSP_PWM_CH3C_EN\
}

#define MD_BSP_PWM2_CONFIG \
{ \
    .freq=20000, \
    .ch_en=MD_BSP_PWM_CH1_EN\
            |MD_BSP_PWM_CH2_EN\
            |MD_BSP_PWM_CH3_EN\
}

enum
{
    #ifdef MD_BSP_PWM1_CONFIG
        PWM1_INDEX,
    #endif
    #ifdef MD_BSP_PWM2_CONFIG
        PWM2_INDEX,
    #endif
    #ifdef MD_BSP_PWM3_CONFIG
        PWM3_INDEX,
    #endif
    #ifdef MD_BSP_PWM8_CONFIG
        PWM8_INDEX
    #endif
};

static md_bsp_pwm_config_t pwm_config[] =
{
#ifdef MD_BSP_PWM1_CONFIG
    MD_BSP_PWM1_CONFIG,
#endif
#ifdef MD_BSP_PWM2_CONFIG
    MD_BSP_PWM2_CONFIG,
#endif
#ifdef MD_BSP_PWM3_CONFIG
    MD_BSP_PWM3_CONFIG,
#endif
#ifdef MD_BSP_PWM8_CONFIG
    MD_BSP_PWM8_CONFIG,
#endif
};

static bsp_pwm_dev_t pwm_dev[] = {
#ifdef MD_BSP_PWM1_CONFIG
    {
        .index = 1,
        .cfg = &pwm_config[PWM1_INDEX],
    },
#endif
#ifdef MD_BSP_PWM2_CONFIG
    {
        .index = 2,
        .cfg = &pwm_config[PWM2_INDEX],
    },
#endif
#ifdef MD_BSP_PWM3_CONFIG
    {
        .index = 3,
        .cfg = &pwm_config[PWM3_INDEX],
    },
#endif
#ifdef MD_BSP_PWM8_CONFIG
    {
        .index = 8,
        .cfg = &pwm_config[PWM8_INDEX],
    },
#endif
};

extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void bsp_pwm_init()
{
    for (int i=0; i<ARRAY_SIZE(pwm_dev); i++)
    {
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        TIM_OC_InitTypeDef sConfigOC = {0};
        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
        switch(pwm_dev[i].index)
        {
            case 1: pwm_dev[i].htim.Instance = TIM1; break;
            case 2: pwm_dev[i].htim.Instance = TIM2; break;
            case 3: pwm_dev[i].htim.Instance = TIM3; break;
            case 8: pwm_dev[i].htim.Instance = TIM8; break;
        }
        pwm_dev[i].htim.Init.Prescaler = 0;
        pwm_dev[i].htim.Init.CounterMode = TIM_COUNTERMODE_UP;
        uint32_t tim_clk = tim_clock_get(&pwm_dev[i].htim);
        uint32_t tim_period = (tim_clk/(pwm_dev[i].cfg->freq));
        pwm_dev[i].period_cnt = tim_period;
        pwm_dev[i].htim.Init.Period = tim_period-1;
        pwm_dev[i].htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        pwm_dev[i].htim.Init.RepetitionCounter = 0;
        pwm_dev[i].htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        if (HAL_TIM_Base_Init(&(pwm_dev[i].htim)) != HAL_OK)
        {
            // Error_Handler();
        }
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&(pwm_dev[i].htim), &sClockSourceConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        if (HAL_TIM_PWM_Init(&(pwm_dev[i].htim)) != HAL_OK)
        {
            // Error_Handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&(pwm_dev[i].htim), &sMasterConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        uint32_t ch_map = pwm_dev[i].cfg->ch_en;

        for (int j=0; j<8; j++)
        {
            if ( ch_map&(0x01<<j) )
            {
                sConfigOC.OCMode = TIM_OCMODE_PWM1;
                sConfigOC.Pulse = 0;
                sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
                sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
                sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
                if ( ch_map&(0x00010000<<i) )
                {
                    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
                    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
                }
                if (HAL_TIM_PWM_ConfigChannel(&(pwm_dev[i].htim), &sConfigOC, TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*j) != HAL_OK)
                {
                    // Error_Handler();
                }
            }
        }
        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
        sBreakDeadTimeConfig.DeadTime = 0;
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        sBreakDeadTimeConfig.BreakFilter = 0;
        sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
        sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
        sBreakDeadTimeConfig.Break2Filter = 0;
        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(&(pwm_dev[i].htim), &sBreakDeadTimeConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        HAL_TIM_MspPostInit(&(pwm_dev[i].htim));
        __HAL_TIM_URS_ENABLE(&(pwm_dev[i].htim));
    }
}

bsp_pwm_dev_t *bsp_pwm_request(uint8_t index)
{
    for (int i=0; i<ARRAY_SIZE(pwm_dev); i++)
    {
        if (pwm_dev[i].index == index)
        {
            return &pwm_dev[i];
        }
    }
    return NULL;
}

int32_t bsp_pwm_enable(bsp_pwm_dev_t *pwm_dev, uint8_t ch, uint8_t en)
{
    uint32_t ch_map = pwm_dev->cfg->ch_en;
    if ( ch < 1 || ch > 8 ) return -1;
    if ( (ch_map >> (ch-1))&0x01 == 0 ) return -1;

    if (en)
    {
        HAL_TIM_PWM_Start(&pwm_dev->htim, TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*(ch-1));
        if ( ch_map & (0x00010000<<(ch-1))) {
            HAL_TIMEx_PWMN_Start(&pwm_dev->htim, TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*(ch-1));
        }
    }
    else
    {
        HAL_TIM_PWM_Stop(&pwm_dev->htim, TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*(ch-1));
        if ( ch_map & (0x00010000<<(ch-1))) {
            HAL_TIMEx_PWMN_Stop(&pwm_dev->htim, TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*(ch-1));
        }
    }

    return 0;
}

int32_t bsp_pwm_set_width(bsp_pwm_dev_t *pwm_dev, uint8_t ch, uint32_t width)
{
    uint32_t channel = TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*(ch-1);

    __HAL_TIM_SET_COMPARE(&pwm_dev->htim, channel, width);
}

int32_t bsp_pwm_set_duty(bsp_pwm_dev_t *pwm_dev, uint8_t ch, float duty)
{
    uint32_t channel = TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*(ch-1);

    uint32_t pluse = pwm_dev->period_cnt * duty - 1;
    __HAL_TIM_SET_COMPARE(&pwm_dev->htim, channel, pluse);
}

void bsp_pwm_test()
{
    bsp_pwm_init();
    bsp_pwm_dev_t *pwm_dev = bsp_pwm_request(1);

    bsp_pwm_enable(pwm_dev, 1, 1);
    bsp_pwm_enable(pwm_dev, 2, 1);
    bsp_pwm_enable(pwm_dev, 3, 1);

    bsp_pwm_set_duty(pwm_dev, 1, 0.5);
    bsp_pwm_set_duty(pwm_dev, 2, 0.2);
    bsp_pwm_set_duty(pwm_dev, 3, 0.7);
}
