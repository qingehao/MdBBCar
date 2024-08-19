#ifndef _BOARD_BSP_CONFIG_H_
#define _BOARD_BSP_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/**************************TICK CONFIG***************************/
#define BSP_TICK_TIMER_INDEX 4
#define BSP_TICK_IRQn TIM4_IRQn
#define BSP_TICK_IRQ_HANDLER TIM4_IRQHandler

#define BSP_TICK_CONFIG \
{ \
    .timer_index = BSP_TICK_TIMER_INDEX, \
    .timer_cnt_freq = 10000000, \
    .timer_irq_freq = 1000, \
}

/**************************SPI CONFIG***************************/
/* SPI_CLK 140M*/

// MA732 14BIT SPI_MODE1 时钟最高25M
#define MA732_SPI_MODE     0
#define MA732_SPI_CLK_PSC  4 // 140/8=17.5M

// MY6835 21BIT SPI_MODE3 时钟最高16M
#define MT6835_SPI_MODE 3
#define MT6835_SPI_CLK_PSC 8 // 140/8=17.5M

#define MD_BSP_SPI1_CONFIG \
{ \
    .rb_item_num=32, .spi_mode=MT6835_SPI_MODE, .psc=MT6835_SPI_CLK_PSC, .dma_en=0, \
    .dma_config = { \
        .tx_dma_instance=DMA1_Stream3, .rx_dma_instance=DMA1_Stream2, \
        .tx_irq_en=0,                  .rx_irq_en=1, \
        .tx_irqn=DMA1_Stream3_IRQn,    .rx_irqn=DMA1_Stream2_IRQn, \
        .tx_irq_prio=(1<<8|0),         .rx_irq_prio=(1<<8|0), \
    } \
}
#define MD_BSP_SPI1_DMA_TX_IRQHandler DMA1_Stream3_IRQHandler
#define MD_BSP_SPI1_DMA_RX_IRQHandler DMA1_Stream2_IRQHandler

// SPI2_RX    DMA1_Stream4   IMU
// SPI2_TX    DMA1_Stream5
#define MD_BSP_SPI2_CONFIG \
{ \
    .rb_item_num=8, .spi_mode=3, .psc=256, .dma_en=BSP_SPI_TX_DMA_EN|BSP_SPI_RX_DMA_EN, \
    .dma_config = { \
        .tx_dma_instance=DMA1_Stream5, .rx_dma_instance=DMA1_Stream4, \
        .tx_irq_en=1,                  .rx_irq_en=1, \
        .tx_irqn=DMA1_Stream5_IRQn,    .rx_irqn=DMA1_Stream4_IRQn, \
        .tx_irq_prio=(1<<8|0),         .rx_irq_prio=(1<<8|0), \
    } \
}
#define MD_BSP_SPI2_DMA_TX_IRQHandler DMA1_Stream5_IRQHandler
#define MD_BSP_SPI2_DMA_RX_IRQHandler DMA1_Stream4_IRQHandler


/**************************FOC CONFIG***************************/
#define BSP_FOC_MOTOR_NUM  3
#define BSP_FOC_ADC_CH_NUM 7
#define MOTOR_PWR_EN_PIN   GET_PIN(A, 4)

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

#ifdef __cplusplus
}
#endif

#endif
