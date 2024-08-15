#include "bsp_tick.h"
#include "stm32h7xx.h"
#include "rtthread.h"
#include "drv_gpio.h"

// typedef struct
// {
//     uint8_t     timer_index;
//     uint32_t    timer_cnt_freq;
//     uint32_t    timer_irq_freq;
// } bsp_tick_config_t;

#define BSP_TICK_CONFIG \
{ \
    .timer_index = BSP_TICK_TIMER_INDEX, \
    .timer_cnt_freq = 10000000, \
    .timer_irq_freq = 8000, \
}

typedef struct
{
    TIM_HandleTypeDef htim;
    uint32_t tim_psc;
    uint32_t tim_period;
    uint32_t irq_cnt;
    volatile uint32_t tick_ms;
    uint32_t ns_per_cnt;
    void (*irq_cb)();
} bsp_tick_t;

static void stm32_tim_pclkx_doubler_get(uint32_t *pclk1_doubler, uint32_t *pclk2_doubler);

static bsp_tick_t bsp_tick = {0};
static bsp_tick_config_t tick_config = BSP_TICK_CONFIG;

void bsp_tick_init()
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    switch (tick_config.timer_index)
    {
        case 1: bsp_tick.htim.Instance = TIM1; break;
        case 2: bsp_tick.htim.Instance = TIM2; break;
        case 3: bsp_tick.htim.Instance = TIM3; break;
        case 4: bsp_tick.htim.Instance = TIM4; break;
        case 5: bsp_tick.htim.Instance = TIM5; break;
        case 6: bsp_tick.htim.Instance = TIM6; break;
        case 7: bsp_tick.htim.Instance = TIM7; break;
        case 8: bsp_tick.htim.Instance = TIM8; break;
        default: break;
    }

    uint64_t tim_clk = tim_clock_get(&bsp_tick.htim);
    bsp_tick.tim_psc = tim_clk/tick_config.timer_cnt_freq;
    bsp_tick.tim_period = tick_config.timer_cnt_freq/tick_config.timer_irq_freq;
    bsp_tick.ns_per_cnt = 1000000000 / tick_config.timer_cnt_freq;

    bsp_tick.htim.Init.Prescaler = bsp_tick.tim_psc - 1;
    bsp_tick.htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    bsp_tick.htim.Init.Period = bsp_tick.tim_period - 1;
    bsp_tick.htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    bsp_tick.htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&bsp_tick.htim) != HAL_OK)
    {
        // Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&bsp_tick.htim, &sClockSourceConfig) != HAL_OK)
    {
        // Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&bsp_tick.htim, &sMasterConfig) != HAL_OK)
    {
        // Error_Handler();
    }

    switch(tick_config.timer_index)
    {
        case 1:
        break;
        case 2:
        break;
        case 3:
        break;
        case 4:
            HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(TIM4_IRQn);
        break;
        default: break;
    }
    HAL_TIM_Base_Start_IT(&bsp_tick.htim);
}

void bsp_tick_set_irq_cb(void (*tick_irq_cb)())
{
    bsp_tick.irq_cb = tick_irq_cb;
}

uint64_t bsp_tick_get_us()
{
    uint32_t ns_per_cnt = bsp_tick.ns_per_cnt;

    uint32_t cur_cnt = __HAL_TIM_GET_COUNTER((&(bsp_tick.htim)));

    uint64_t cur_us = cur_cnt*ns_per_cnt/1000 + bsp_tick.tick_ms * 1000;

    return cur_us;
}

uint32_t bsp_tick_get_ms()
{
    return bsp_tick.tick_ms;
}

void bsp_tick_delay_us(uint32_t us)
{
    uint32_t tcnts = us*1000/bsp_tick.ns_per_cnt;
    uint32_t cur_cnt = 0;
    uint32_t last_cnt = 0;
    uint32_t tcnt = 0;
    uint32_t reload = __HAL_TIM_GET_AUTORELOAD((&(bsp_tick.htim)));

    last_cnt = __HAL_TIM_GET_COUNTER((&(bsp_tick.htim)));
    while (1)
    {
        cur_cnt = __HAL_TIM_GET_COUNTER((&(bsp_tick.htim)));
        if (cur_cnt != last_cnt)
        {
            if (cur_cnt > last_cnt) tcnt += (cur_cnt - last_cnt);
            else tcnt += (reload-last_cnt+cur_cnt);
            last_cnt = cur_cnt;
            if (tcnt >= tcnts) break;
        }
    }
}

void bsp_tick_delay_ms(uint32_t ms)
{
    bsp_tick_delay_us(ms*1000);
}

#if BSP_TICK_TIMER_INDEX == 4
void TIM4_IRQHandler(void)
{
    uint32_t itsource = bsp_tick.htim.Instance->DIER;
    uint32_t itflag   = bsp_tick.htim.Instance->SR;
    rt_interrupt_enter();
    if ((itflag & (TIM_FLAG_UPDATE)) == (TIM_FLAG_UPDATE))
    {
        if ((itsource & (TIM_IT_UPDATE)) == (TIM_IT_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG((&(bsp_tick.htim)), TIM_FLAG_UPDATE);
            bsp_tick.irq_cnt++;
            if (bsp_tick.irq_cnt >= tick_config.timer_irq_freq/1000) {
                bsp_tick.tick_ms++;
                bsp_tick.irq_cnt = 0;
            }
            if (bsp_tick.irq_cb) {
                bsp_tick.irq_cb();
            }
        }
    }
    rt_interrupt_leave();
}
#endif

/* APBx timer clocks frequency doubler state related to APB1CLKDivider value */
static void stm32_tim_pclkx_doubler_get(uint32_t *pclk1_doubler, uint32_t *pclk2_doubler)
{
    uint32_t flatency = 0;
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &flatency);

    *pclk1_doubler = 1;
    *pclk2_doubler = 1;

#if defined(SOC_SERIES_STM32MP1)
    if (RCC_ClkInitStruct.APB1_Div != RCC_APB1_DIV1)
    {
        *pclk1_doubler = 2;
    }
    if (RCC_ClkInitStruct.APB2_Div != RCC_APB2_DIV1)
    {
        *pclk2_doubler = 2;
    }
#else
    if (RCC_ClkInitStruct.APB1CLKDivider != RCC_HCLK_DIV1)
    {
         *pclk1_doubler = 2;
    }
#if !(defined(SOC_SERIES_STM32F0) || defined(SOC_SERIES_STM32G0))
    if (RCC_ClkInitStruct.APB2CLKDivider != RCC_HCLK_DIV1)
    {
         *pclk2_doubler = 2;
    }
#endif /* !(defined(SOC_SERIES_STM32F0) || defined(SOC_SERIES_STM32G0)) */
#endif /* defined(SOC_SERIES_STM32MP1) */
}

uint64_t tim_clock_get(TIM_HandleTypeDef *htim)
{
    uint32_t pclk1_doubler, pclk2_doubler;
    uint64_t tim_clock;

    stm32_tim_pclkx_doubler_get(&pclk1_doubler, &pclk2_doubler);

/* Some series may only have APBPERIPH_BASE, don't have HAL_RCC_GetPCLK2Freq */
#if defined(APBPERIPH_BASE)
    tim_clock = (rt_uint32_t)(HAL_RCC_GetPCLK1Freq() * pclk1_doubler);
#elif defined(APB1PERIPH_BASE) || defined(APB2PERIPH_BASE)
    if ((uint32_t)htim->Instance >= APB2PERIPH_BASE)
    {
        tim_clock = (uint32_t)(HAL_RCC_GetPCLK2Freq() * pclk2_doubler);
    }
    else
    {
        tim_clock = (uint32_t)(HAL_RCC_GetPCLK1Freq() * pclk1_doubler);
    }
#endif

    return tim_clock;
}

static int32_t test_pin = 0;
static rt_sem_t sem_timer;

static void dd_tick_irq_cb()
{
    // rt_pin_write(test_pin, 1);
    // bsp_tick_delay_us(2);
    // rt_pin_write(test_pin, 0);
    rt_sem_release(sem_timer);
}

static void timer_task(void *arg)
{
    while(1)
    {
        rt_sem_take(sem_timer, RT_WAITING_FOREVER);
        rt_pin_write(test_pin, 1);
        bsp_tick_delay_us(2);
        rt_pin_write(test_pin, 0);
    }
}

void bsp_tick_test(uint32_t pin)
{
    test_pin = pin;
    sem_timer = rt_sem_create("timer", 0, RT_IPC_FLAG_PRIO);

    rt_thread_t tid = rt_thread_create("timer task", timer_task, NULL, 1024, 1, 10);
    rt_thread_startup(tid);

    bsp_tick_set_irq_cb(dd_tick_irq_cb);
    // while(1)
    // {
    //     rt_pin_write(pin, 1);
    //     bsp_tick_delay_us(10);
    //     rt_pin_write(pin, 0);
    //     bsp_tick_delay_us(10);
    // }
}
