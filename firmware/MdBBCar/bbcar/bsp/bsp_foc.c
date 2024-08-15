#include "bsp_foc.h"
#include "rtthread.h"
#include "bsp_tick.h"
#include "bsp_mem.h"

#define BSP_FOC_CONFIG \
{ \
    .pwm_config[0] = { \
        .timer_index = 1, \
        .phaseA_ch = 1, \
        .phaseB_ch = 2, \
        .phaseC_ch = 3, \
        .complementary_en = 1, \
        .freq = 20000, \
    }, \
    .pwm_config[1] = { \
        .timer_index = 8, \
        .phaseA_ch = 1, \
        .phaseB_ch = 2, \
        .phaseC_ch = 3, \
        .complementary_en = 1, \
        .freq = 20000, \
    }, \
    .pwm_config[2] = { \
        .timer_index = 2, \
        .phaseA_ch = 1, \
        .phaseB_ch = 2, \
        .phaseC_ch = 3, \
        .complementary_en = 0, \
        .freq = 20000, \
    }, \
    .adc_config = {  \
        .adc_index = 1, \
        .adc_ch_num = BSP_FOC_ADC_CH_NUM, \
        .adc_ch_array  = {5, 9, 4, 8, 7, 10, 11}, \
        .motor = { \
            { \
                .iAch_idx = 0, \
                .iBch_idx = 1, \
                .iCch_idx = BSP_FOC_ADC_CH_NOUSE, \
            }, \
            { \
                .iAch_idx = 0, \
                .iBch_idx = 1, \
                .iCch_idx = BSP_FOC_ADC_CH_NOUSE, \
            }, \
        }, \
    }, \
    .trigger_ch = 1<<4|4, \
}

static bsp_foc_config_t foc_config = BSP_FOC_CONFIG;
static bsp_foc_t g_bsp_foc;

extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static void _foc_adc_init(bsp_foc_t *bsp_foc)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    ADC_HandleTypeDef *adc = &(bsp_foc->foc_adc.hadc);

    bsp_foc->foc_adc.adc_buf_sz = foc_config.adc_config.adc_ch_num*2;

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uint8_t unaligned_size = (bsp_foc->foc_adc.adc_buf_sz) % DCACHE_LINE_SIZE;
    if (unaligned_size != 0) {
        bsp_foc->foc_adc.adc_buf_sz = bsp_foc->foc_adc.adc_buf_sz + DCACHE_LINE_SIZE - unaligned_size;
    }
    bsp_foc->foc_adc.adc_buffer = (uint16_t *)bsp_mem_dma_malloc(bsp_foc->foc_adc.adc_buf_sz);
    if (bsp_foc->foc_adc.adc_buffer == NULL) return;
#else
    bsp_foc.foc_adc.adc_buffer = bsp_mem_malloc(bsp_foc.foc_adc.adc_buf_sz);
#endif

    switch (foc_config.adc_config.adc_index)
    {
        case 1: adc->Instance = ADC1; break;
        case 2: adc->Instance = ADC2; break;
        default: break;
    }

    adc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    adc->Init.Resolution = ADC_RESOLUTION_16B;
    adc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    adc->Init.LowPowerAutoWait = DISABLE;
    adc->Init.ContinuousConvMode = DISABLE;
    adc->Init.NbrOfConversion = foc_config.adc_config.adc_ch_num;
    adc->Init.DiscontinuousConvMode = DISABLE;
    switch (((foc_config.trigger_ch>>4)&0x0f))
    {
        case 1: adc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO; break;
        case 2: adc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO; break;
        default: break;
    }
    adc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    adc->Init.Overrun = ADC_OVR_DATA_PRESERVED;
    adc->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    adc->Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(adc) != HAL_OK)
    {
        // Error_Handler();
    }

    __HAL_RCC_DMA2_CLK_ENABLE();

    DMA_HandleTypeDef *dma = &(bsp_foc->foc_adc.hdma);;

    dma->Instance = DMA2_Stream2;
    dma->Init.Request = DMA_REQUEST_ADC1;
    dma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma->Init.PeriphInc = DMA_PINC_DISABLE;
    dma->Init.MemInc = DMA_MINC_ENABLE;
    dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma->Init.Mode = DMA_CIRCULAR;
    dma->Init.Priority = DMA_PRIORITY_HIGH;
    dma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    // dma->XferCpltCallback = adc_xfer_cplt_handler;

    if (HAL_DMA_Init(dma) != HAL_OK)
    {
    //   Error_Handler();
    }
    __HAL_LINKDMA(adc, DMA_Handle, *dma);

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(adc, &multimode) != HAL_OK)
    {
        // Error_Handler();
    }

    for (int i=0; i<foc_config.adc_config.adc_ch_num; i++)
    {
        /** Configure Regular Channel
         */
        switch (foc_config.adc_config.adc_ch_array[i])
        {
            case 0: sConfig.Channel = ADC_CHANNEL_0; break;
            case 1: sConfig.Channel = ADC_CHANNEL_1; break;
            case 2: sConfig.Channel = ADC_CHANNEL_2; break;
            case 3: sConfig.Channel = ADC_CHANNEL_3; break;
            case 4: sConfig.Channel = ADC_CHANNEL_4; break;
            case 5: sConfig.Channel = ADC_CHANNEL_5; break;
            case 6: sConfig.Channel = ADC_CHANNEL_6; break;
            case 7: sConfig.Channel = ADC_CHANNEL_7; break;
            case 8: sConfig.Channel = ADC_CHANNEL_8; break;
            case 9: sConfig.Channel = ADC_CHANNEL_9; break;
            case 10: sConfig.Channel = ADC_CHANNEL_10; break;
            case 11: sConfig.Channel = ADC_CHANNEL_11; break;
            case 12: sConfig.Channel = ADC_CHANNEL_12; break;
            case 13: sConfig.Channel = ADC_CHANNEL_13; break;
            case 14: sConfig.Channel = ADC_CHANNEL_14; break;
            case 15: sConfig.Channel = ADC_CHANNEL_15; break;
            case 16: sConfig.Channel = ADC_CHANNEL_16; break;
            case 17: sConfig.Channel = ADC_CHANNEL_17; break;
            default: break;
        }
        switch(i)
        {
            case 0: sConfig.Rank = ADC_REGULAR_RANK_1; break;
            case 1: sConfig.Rank = ADC_REGULAR_RANK_2; break;
            case 2: sConfig.Rank = ADC_REGULAR_RANK_3; break;
            case 3: sConfig.Rank = ADC_REGULAR_RANK_4; break;
            case 4: sConfig.Rank = ADC_REGULAR_RANK_5; break;
            case 5: sConfig.Rank = ADC_REGULAR_RANK_6; break;
            case 6: sConfig.Rank = ADC_REGULAR_RANK_7; break;
            case 7: sConfig.Rank = ADC_REGULAR_RANK_8; break;
            case 8: sConfig.Rank = ADC_REGULAR_RANK_9; break;
            case 9: sConfig.Rank = ADC_REGULAR_RANK_10; break;
            case 10: sConfig.Rank = ADC_REGULAR_RANK_11; break;
            case 11: sConfig.Rank = ADC_REGULAR_RANK_12; break;
            case 12: sConfig.Rank = ADC_REGULAR_RANK_13; break;
            case 13: sConfig.Rank = ADC_REGULAR_RANK_14; break;
            case 14: sConfig.Rank = ADC_REGULAR_RANK_15; break;
            case 15: sConfig.Rank = ADC_REGULAR_RANK_16; break;
            default: break;
        }
        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        sConfig.OffsetSignedSaturation = DISABLE;
        if (HAL_ADC_ConfigChannel(adc, &sConfig) != HAL_OK)
        {
            // Error_Handler();
        }
    }

    if (HAL_ADC_Start_DMA(adc, (uint32_t *)bsp_foc->foc_adc.adc_buffer,
                          foc_config.adc_config.adc_ch_num*2
                         ) != HAL_OK)
    {
        // return 5;
    }

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

#define GET_TIM_CHANNELX(x) (TIM_CHANNEL_1+(TIM_CHANNEL_2-TIM_CHANNEL_1)*((x)-1))

static void _foc_pwm_init(bsp_foc_t *bsp_foc)
{
    for (int i=0; i<BSP_FOC_MOTOR_NUM; i++)
    {
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};
        TIM_OC_InitTypeDef sConfigOC = {0};
        TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
        bsp_foc_pwm_t *foc_pwm = &(bsp_foc->foc_pwm[i]);

        foc_pwm->chA = GET_TIM_CHANNELX(foc_config.pwm_config[i].phaseA_ch);
        foc_pwm->chB = GET_TIM_CHANNELX(foc_config.pwm_config[i].phaseB_ch);
        foc_pwm->chC = GET_TIM_CHANNELX(foc_config.pwm_config[i].phaseC_ch);

        switch(foc_config.pwm_config[i].timer_index)
        {
            case 1: foc_pwm->htim.Instance = TIM1; break;
            case 2: foc_pwm->htim.Instance = TIM2; break;
            case 3: foc_pwm->htim.Instance = TIM3; break;
            case 8: foc_pwm->htim.Instance = TIM8; break;
        }

        foc_pwm->htim.Init.Prescaler = 0;
        foc_pwm->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
        uint32_t tim_clk = tim_clock_get(&foc_pwm->htim);
        uint32_t tim_period = (tim_clk/(foc_config.pwm_config[i].freq));

        foc_pwm->period_cnt = tim_period;

        foc_pwm->htim.Init.Period = tim_period-1;
        foc_pwm->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        foc_pwm->htim.Init.RepetitionCounter = 0;
        foc_pwm->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        if (HAL_TIM_Base_Init(&foc_pwm->htim) != HAL_OK)
        {
            // Error_Handler();
        }
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&foc_pwm->htim, &sClockSourceConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        if (HAL_TIM_PWM_Init(&foc_pwm->htim) != HAL_OK)
        {
            // Error_Handler();
        }
        if ( ((foc_config.trigger_ch>>4)&0x0f) == foc_config.pwm_config[i].timer_index )
        {
            foc_pwm->trigger_en = 1;
            bsp_foc->adc_trigger_source_idx = i;
            switch(foc_config.trigger_ch&0x0f)
            {
                case 1: sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF; foc_pwm->chTrigger = TIM_CHANNEL_1; break;
                case 2: sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF; foc_pwm->chTrigger = TIM_CHANNEL_2; break;
                case 3: sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF; foc_pwm->chTrigger = TIM_CHANNEL_3; break;
                case 4: sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF; foc_pwm->chTrigger = TIM_CHANNEL_4; break;
                default: break;
            }
        }
        else
        {
            sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        }
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&foc_pwm->htim, &sMasterConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        foc_pwm->complementary_en = foc_config.pwm_config[i].complementary_en;

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = 0;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        if (foc_pwm->complementary_en)
        {
            sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
            sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        }
        if (HAL_TIM_PWM_ConfigChannel(&foc_pwm->htim, &sConfigOC, foc_pwm->chA) != HAL_OK)
        {
            // Error_Handler();
        }
        if (HAL_TIM_PWM_ConfigChannel(&foc_pwm->htim, &sConfigOC, foc_pwm->chB) != HAL_OK)
        {
            // Error_Handler();
        }
        if (HAL_TIM_PWM_ConfigChannel(&foc_pwm->htim, &sConfigOC, foc_pwm->chC) != HAL_OK)
        {
            // Error_Handler();
        }
        if ( ((foc_config.trigger_ch>>4)&0x0f) == foc_config.pwm_config[i].timer_index )
        {
            sConfigOC.OCMode = TIM_OCMODE_PWM2;
            sConfigOC.Pulse = foc_pwm->period_cnt/2;
            if (HAL_TIM_PWM_ConfigChannel(&foc_pwm->htim, &sConfigOC, GET_TIM_CHANNELX(foc_config.trigger_ch&0x0f)) != HAL_OK)
            {
                // Error_Handler();
            }
        }

        sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
        sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
        sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_3;
        sBreakDeadTimeConfig.DeadTime = 48;
        sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
        sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        sBreakDeadTimeConfig.BreakFilter = 0;
        sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
        sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
        sBreakDeadTimeConfig.Break2Filter = 0;
        sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(&foc_pwm->htim, &sBreakDeadTimeConfig) != HAL_OK)
        {
            // Error_Handler();
        }
        HAL_TIM_MspPostInit(&foc_pwm->htim);
        __HAL_TIM_URS_ENABLE(&foc_pwm->htim);
    }
}

bsp_foc_t *bsp_foc_request()
{
    return &g_bsp_foc;
}

void bsp_foc_init()
{
    _foc_pwm_init(&g_bsp_foc);
    _foc_adc_init(&g_bsp_foc);
}

void bsp_foc_start(bsp_foc_t *bsp_foc)
{
    bsp_foc_pwm_t *foc_pwm  = &(bsp_foc->foc_pwm[bsp_foc->adc_trigger_source_idx]);
    HAL_TIM_PWM_Start(&foc_pwm->htim, foc_pwm->chTrigger);
}

void bsp_foc_stop(bsp_foc_t *bsp_foc)
{
    bsp_foc_pwm_t *foc_pwm  = &(bsp_foc->foc_pwm[bsp_foc->adc_trigger_source_idx]);
    HAL_TIM_PWM_Stop(&foc_pwm->htim, foc_pwm->chTrigger);
}

void bsp_foc_pwm_enable(bsp_foc_t *bsp_foc, uint8_t motor_id, uint8_t en)
{
    bsp_foc_pwm_t *foc_pwm  = &(bsp_foc->foc_pwm[motor_id]);

    if (en)
    {
        HAL_TIM_PWM_Start(&foc_pwm->htim, foc_pwm->chA);
        HAL_TIM_PWM_Start(&foc_pwm->htim, foc_pwm->chB);
        HAL_TIM_PWM_Start(&foc_pwm->htim, foc_pwm->chC);
        if (foc_pwm->complementary_en)
        {
            HAL_TIMEx_PWMN_Start(&foc_pwm->htim, foc_pwm->chA);
            HAL_TIMEx_PWMN_Start(&foc_pwm->htim, foc_pwm->chB);
            HAL_TIMEx_PWMN_Start(&foc_pwm->htim, foc_pwm->chC);
        }
    }
    else
    {
        HAL_TIM_PWM_Stop(&foc_pwm->htim, foc_pwm->chA);
        HAL_TIM_PWM_Stop(&foc_pwm->htim, foc_pwm->chB);
        HAL_TIM_PWM_Stop(&foc_pwm->htim, foc_pwm->chC);
        if (foc_pwm->complementary_en)
        {
            HAL_TIMEx_PWMN_Stop(&foc_pwm->htim, foc_pwm->chA);
            HAL_TIMEx_PWMN_Stop(&foc_pwm->htim, foc_pwm->chB);
            HAL_TIMEx_PWMN_Stop(&foc_pwm->htim, foc_pwm->chC);
        }
    }
}

void bsp_foc_set_callback(bsp_foc_t *bsp_foc, void (*cb)(void *arg), void *arg)
{
    bsp_foc->cb = cb;
    bsp_foc->cb_arg = arg;
}

void bsp_foc_set_pwm(bsp_foc_t *bsp_foc, uint8_t motor_id, float dutyA, float dutyB, float dutyC)
{
    bsp_foc_pwm_t *foc_pwm  = &(bsp_foc->foc_pwm[motor_id]);

    uint32_t pluseA = foc_pwm->period_cnt * dutyA - 1;
    uint32_t pluseB = foc_pwm->period_cnt * dutyB - 1;
    uint32_t pluseC = foc_pwm->period_cnt * dutyC - 1;

    __HAL_TIM_SET_COMPARE(&foc_pwm->htim, foc_pwm->chA, pluseA);
    __HAL_TIM_SET_COMPARE(&foc_pwm->htim, foc_pwm->chB, pluseB);
    __HAL_TIM_SET_COMPARE(&foc_pwm->htim, foc_pwm->chC, pluseC);
}

static volatile uint32_t foc_time_us = 0;

uint32_t bsp_foc_cur_us(void)
{
    return foc_time_us;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    rt_interrupt_enter();
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)g_bsp_foc.foc_adc.adc_buffer, g_bsp_foc.foc_adc.adc_buf_sz);
#endif
    foc_time_us += 10;
    if (g_bsp_foc.cb)
    {
        g_bsp_foc.cb(g_bsp_foc.cb_arg);
    }
    rt_interrupt_leave();
}

void ADC_IRQHandler()
{
    rt_interrupt_enter();
    // HAL_ADC_IRQHandler( &(g_bsp_foc.foc_adc.hadc) );
    rt_interrupt_leave();
}

void DMA2_Stream2_IRQHandler()
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler( &(g_bsp_foc.foc_adc.hdma) );
    rt_interrupt_leave();
}

#include "drv_gpio.h"

#define LED1_PIN    GET_PIN(E, 4)

volatile uint32_t dd_cnt =  0;

static void dd_foc_cb(void *arg)
{
    dd_cnt++;
}

void bsp_foc_test()
{
    bsp_foc_init();
    bsp_foc_t *bsp_foc = bsp_foc_request();
    bsp_foc_set_callback(bsp_foc, dd_foc_cb, bsp_foc);

    bsp_foc_pwm_enable(bsp_foc, 0, 1);

    bsp_foc_start(bsp_foc);
    bsp_foc_set_pwm(bsp_foc, 0, 0.2, 0.5, 0.7);
}
