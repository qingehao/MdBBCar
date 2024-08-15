#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "lwrb.h"
#include "stm32h7xx.h"

#if 1
typedef enum
{
    SPI_BUS_CPOLL_CPHAL,
    SPI_BUS_CPOLL_CPHAH,
    SPI_BUS_CPOLH_CPHAL,
    SPI_BUS_CPOLH_CPHAH,
} SPI_BUS_MODE;

#define BSP_SPI_TX_DMA_EN (0x01<<0)
#define BSP_SPI_RX_DMA_EN (0x01<<1)

typedef struct
{
    uint8_t    *wbuf;
    uint8_t    *rbuf;
    uint16_t    wrlen;
    uint32_t    flag;
    uint32_t    bus_speed;

    void       *arg;
    void       (*setup)(void *arg);
    void       (*finish)(void *arg);
} bsp_spi_msg_t;

typedef struct
{
    uint32_t    rb_item_num; // ringbuf大小 单位:sizeof(bsp_spi_msg_t)
    uint8_t     spi_mode;
    uint32_t    psc;         // 时钟分频系数
    uint8_t     dma_en;
    struct
    {
        void *tx_dma_instance;
        void *rx_dma_instance;
        uint16_t tx_irqn;
        uint16_t tx_irq_prio;
        uint8_t  tx_irq_en;
        uint16_t rx_irqn;
        uint8_t  rx_irq_en;
        uint16_t rx_irq_prio;
    } dma_config;
} md_bsp_spi_config_t;

typedef struct
{
    md_bsp_spi_config_t *cfg;
    uint8_t              index;
    lwrb_t               msg_rb;
    uint8_t             *rb_buf;
    uint32_t             rb_buf_size;
    volatile uint8_t     is_busy;
    bsp_spi_msg_t        cur_msg;
    SPI_HandleTypeDef    spi_handle;
    struct
    {
        DMA_HandleTypeDef handle_rx;
        DMA_HandleTypeDef handle_tx;
    } dma;
} bsp_spi_dev_t;

bsp_spi_dev_t *bsp_spi_request(uint8_t index);
int32_t bsp_spi_async_transfer(bsp_spi_dev_t *dev, bsp_spi_msg_t *msg);
int32_t bsp_spi_sync_transfer(bsp_spi_dev_t *dev, bsp_spi_msg_t *msg);

void bsp_spi_set_clk_psc(bsp_spi_dev_t *dev, uint32_t psc);

#endif

#ifdef __cplusplus
}
#endif
#endif
