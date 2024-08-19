#include "bsp_spi.h"
#include "board_bsp_config.h"
#include "stm32h7xx.h"
#include "rtthread.h"
#include "string.h"

#define LOG_TAG "bsp.spi"
#define LOG_LVL LOG_LVL_DBG
#include "rtdbg.h"


#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

static SPI_TypeDef *SPI_INSTANCE[] = {SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};

enum
{
    #ifdef MD_BSP_SPI1_CONFIG
        SPI1_INDEX,
    #endif
    #ifdef MD_BSP_SPI2_CONFIG
        SPI2_INDEX,
    #endif
    #ifdef MD_BSP_SPI6_CONFIG
        SPI6_INDEX
    #endif
};

static md_bsp_spi_config_t spi_config[] =
{
#ifdef MD_BSP_SPI1_CONFIG
    MD_BSP_SPI1_CONFIG,
#endif
#ifdef MD_BSP_SPI2_CONFIG
    MD_BSP_SPI2_CONFIG,
#endif
#ifdef MD_BSP_SPI6_CONFIG
    MD_BSP_SPI6_CONFIG,
#endif
};

static bsp_spi_dev_t spi_dev[] = {
#ifdef MD_BSP_SPI1_CONFIG
    {
        .index = 1,
        .cfg = &spi_config[SPI1_INDEX],
    },
#endif
#ifdef MD_BSP_SPI2_CONFIG
    {
        .index = 2,
        .cfg = &spi_config[SPI2_INDEX],
    },
#endif
#ifdef MD_BSP_SPI6_CONFIG
    {
        .index = 6,
        .cfg = &spi_config[SPI6_INDEX],
    },
#endif
};

static int bsp_spi_init(void)
{
    for (int i=0; i<ARRAY_SIZE(spi_dev); i++)
    {
        spi_dev[i].rb_buf_size = spi_dev[i].cfg->rb_item_num * sizeof(bsp_spi_msg_t);
        spi_dev[i].rb_buf = rt_malloc(spi_dev[i].rb_buf_size);
        RT_ASSERT(spi_dev[i].rb_buf != NULL);

        lwrb_init(&spi_dev[i].msg_rb, spi_dev[i].rb_buf, spi_dev[i].rb_buf_size);

        spi_dev[i].spi_handle.Instance = SPI_INSTANCE[spi_dev[i].index-1];
        spi_dev[i].spi_handle.Init.Mode = SPI_MODE_MASTER;
        spi_dev[i].spi_handle.Init.Direction = SPI_DIRECTION_2LINES;
        spi_dev[i].spi_handle.Init.DataSize = SPI_DATASIZE_8BIT;

        switch(spi_dev[i].cfg->spi_mode)
        {
            case SPI_BUS_CPOLL_CPHAL:
            {
                spi_dev[i].spi_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
                spi_dev[i].spi_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
            }
            break;

            case SPI_BUS_CPOLL_CPHAH:
            {
                spi_dev[i].spi_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
                spi_dev[i].spi_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
            }
            break;

            case SPI_BUS_CPOLH_CPHAL:
            {
                spi_dev[i].spi_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
                spi_dev[i].spi_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
            }
            break;

            case SPI_BUS_CPOLH_CPHAH:
            {
                spi_dev[i].spi_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
                spi_dev[i].spi_handle.Init.CLKPhase = SPI_PHASE_2EDGE;
            }
            break;
        }
        spi_dev[i].spi_handle.Init.NSS = SPI_NSS_SOFT;
        switch(spi_dev[i].cfg->psc)
        {
            case 2:   spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; break;
            case 4:   spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; break;
            case 8:   spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; break;
            case 16:  spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; break;
            case 32:  spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; break;
            case 64:  spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; break;
            case 128: spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; break;
            case 256: spi_dev[i].spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; break;
            default: LOG_E("spi%d, psc cfg erorr", spi_dev[i].index); while(1); break;
        }

        spi_dev[i].spi_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
        spi_dev[i].spi_handle.Init.TIMode = SPI_TIMODE_DISABLE;
        spi_dev[i].spi_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        spi_dev[i].spi_handle.Init.CRCPolynomial = 0x0;
        spi_dev[i].spi_handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
        spi_dev[i].spi_handle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
        spi_dev[i].spi_handle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
        spi_dev[i].spi_handle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        spi_dev[i].spi_handle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        spi_dev[i].spi_handle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
        spi_dev[i].spi_handle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
        spi_dev[i].spi_handle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
        spi_dev[i].spi_handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
        spi_dev[i].spi_handle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
        HAL_SPI_Init(&spi_dev[i].spi_handle);


        __HAL_RCC_DMA1_CLK_ENABLE();

        if (spi_dev[i].cfg->dma_en&0x01)
        {
            spi_dev[i].dma.handle_tx.Instance = spi_dev[i].cfg->dma_config.tx_dma_instance;
            switch(spi_dev[i].index)
            {
                case 1: spi_dev[i].dma.handle_tx.Init.Request = DMA_REQUEST_SPI1_TX; break;
                case 2: spi_dev[i].dma.handle_tx.Init.Request = DMA_REQUEST_SPI2_TX; break;
                case 3: spi_dev[i].dma.handle_tx.Init.Request = DMA_REQUEST_SPI3_TX; break;
                case 4: spi_dev[i].dma.handle_tx.Init.Request = DMA_REQUEST_SPI4_TX; break;
                case 5: spi_dev[i].dma.handle_tx.Init.Request = DMA_REQUEST_SPI5_TX; break;
                case 6: spi_dev[i].dma.handle_tx.Init.Request = BDMA_REQUEST_SPI6_TX; break;
            }
            spi_dev[i].dma.handle_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
            spi_dev[i].dma.handle_tx.Init.PeriphInc = DMA_PINC_DISABLE;
            spi_dev[i].dma.handle_tx.Init.MemInc = DMA_MINC_ENABLE;
            spi_dev[i].dma.handle_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            spi_dev[i].dma.handle_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            spi_dev[i].dma.handle_tx.Init.Mode = DMA_NORMAL;
            spi_dev[i].dma.handle_tx.Init.Priority = DMA_PRIORITY_LOW;
            spi_dev[i].dma.handle_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            if (HAL_DMA_Init(&(spi_dev[i].dma.handle_tx)) != HAL_OK)
            {
            }
            __HAL_LINKDMA(&(spi_dev[i].spi_handle), hdmatx, spi_dev[i].dma.handle_tx);
            if (spi_dev[i].cfg->dma_config.tx_irq_en)
            {
                HAL_NVIC_SetPriority(spi_dev[i].cfg->dma_config.tx_irqn,
                                        (spi_dev[i].cfg->dma_config.tx_irq_prio>>8)&0x00ff,
                                        (spi_dev[i].cfg->dma_config.tx_irq_prio)&0x00ff);
                HAL_NVIC_EnableIRQ(spi_dev[i].cfg->dma_config.tx_irqn);
            }
        }

        if (spi_dev[i].cfg->dma_en&0x02)
        {
            /* SPI2_RX Init */
            spi_dev[i].dma.handle_rx.Instance = spi_dev[i].cfg->dma_config.rx_dma_instance;
            switch(spi_dev[i].index)
            {
                case 1: spi_dev[i].dma.handle_rx.Init.Request = DMA_REQUEST_SPI1_RX; break;
                case 2: spi_dev[i].dma.handle_rx.Init.Request = DMA_REQUEST_SPI2_RX; break;
                case 3: spi_dev[i].dma.handle_rx.Init.Request = DMA_REQUEST_SPI3_RX; break;
                case 4: spi_dev[i].dma.handle_rx.Init.Request = DMA_REQUEST_SPI4_RX; break;
                case 5: spi_dev[i].dma.handle_rx.Init.Request = DMA_REQUEST_SPI5_RX; break;
                case 6: spi_dev[i].dma.handle_rx.Init.Request = BDMA_REQUEST_SPI6_RX; break;
            }
            spi_dev[i].dma.handle_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
            spi_dev[i].dma.handle_rx.Init.PeriphInc = DMA_PINC_DISABLE;
            spi_dev[i].dma.handle_rx.Init.MemInc = DMA_MINC_ENABLE;
            spi_dev[i].dma.handle_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            spi_dev[i].dma.handle_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            spi_dev[i].dma.handle_rx.Init.Mode = DMA_NORMAL;
            spi_dev[i].dma.handle_rx.Init.Priority = DMA_PRIORITY_LOW;
            spi_dev[i].dma.handle_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            if (HAL_DMA_Init(&(spi_dev[i].dma.handle_rx)) != HAL_OK)
            {
            }
            __HAL_LINKDMA(&(spi_dev[i].spi_handle), hdmarx, spi_dev[i].dma.handle_rx);
            if (spi_dev[i].cfg->dma_config.rx_irq_en)
            {
                HAL_NVIC_SetPriority(spi_dev[i].cfg->dma_config.rx_irqn,
                                        (spi_dev[i].cfg->dma_config.rx_irq_prio>>8)&0x00ff,
                                        (spi_dev[i].cfg->dma_config.rx_irq_prio)&0x00ff);
                HAL_NVIC_EnableIRQ(spi_dev[i].cfg->dma_config.rx_irqn);
            }
        }
        switch(spi_dev[i].index)
        {
            case 1: HAL_NVIC_SetPriority(SPI1_IRQn, 1, 1);HAL_NVIC_EnableIRQ(SPI1_IRQn);break;
            case 2: HAL_NVIC_SetPriority(SPI2_IRQn, 1, 1);HAL_NVIC_EnableIRQ(SPI2_IRQn);break;
            case 3: HAL_NVIC_SetPriority(SPI3_IRQn, 1, 1);HAL_NVIC_EnableIRQ(SPI3_IRQn);break;
            case 4: HAL_NVIC_SetPriority(SPI4_IRQn, 1, 1);HAL_NVIC_EnableIRQ(SPI4_IRQn);break;
            case 5: HAL_NVIC_SetPriority(SPI5_IRQn, 1, 1);HAL_NVIC_EnableIRQ(SPI5_IRQn);break;
            case 6: HAL_NVIC_SetPriority(SPI6_IRQn, 1, 1);HAL_NVIC_EnableIRQ(SPI6_IRQn);break;
        }
    }
}
INIT_BOARD_EXPORT(bsp_spi_init);

void bsp_spi_set_clk_psc(bsp_spi_dev_t *dev, uint32_t psc)
{
    dev->cfg->psc = psc;

    switch(psc)
    {
        case 2:   dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; break;
        case 4:   dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; break;
        case 8:   dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; break;
        case 16:  dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; break;
        case 32:  dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; break;
        case 64:  dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; break;
        case 128: dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; break;
        case 256: dev->spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; break;
        default: LOG_E("spi%d, psc cfg erorr", dev->index); while(1); break;
    }
    HAL_SPI_Init(&dev->spi_handle);
}

bsp_spi_dev_t *bsp_spi_request(uint8_t index)
{
    for (int i=0; i<ARRAY_SIZE(spi_dev); i++)
    {
        if (spi_dev[i].index == index)
        {
            return &spi_dev[i];
        }
    }
    return NULL;
}

static int32_t bsp_spi_start(bsp_spi_dev_t *dev, bsp_spi_msg_t *msg)
{
    HAL_StatusTypeDef status = HAL_OK;
    dev->is_busy = 1;
    memcpy(&dev->cur_msg, msg, sizeof(bsp_spi_msg_t));

    if (msg->setup != NULL) {
        msg->setup(msg->arg);
    }

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr((uint32_t *)msg->wbuf, msg->wrlen);
#endif

    if (msg->wbuf != NULL && msg->rbuf != NULL)
    {
        status = HAL_SPI_TransmitReceive_DMA(&dev->spi_handle, msg->wbuf, msg->rbuf,
                                              msg->wrlen);
    }
    else if (msg->wbuf != NULL && msg->rbuf == NULL)
    {
        status = HAL_SPI_Transmit_DMA(&dev->spi_handle, msg->wbuf, msg->wrlen);
    }
    else if (msg->wbuf == NULL && msg->rbuf != NULL)
    {
        status = HAL_SPI_Receive_DMA(&dev->spi_handle, msg->rbuf, msg->wrlen);
    }

    if (status !=  HAL_OK) {
        dev->is_busy = 0;
        return status;
    } else {
        return 0;
    }
}

int32_t bsp_spi_async_transfer(bsp_spi_dev_t *dev, bsp_spi_msg_t *msg)
{
    uint32_t w_size = 0;

    if (dev->is_busy)
    {
        /* 直接写入队列中 */
        uint32_t free_size = lwrb_get_free(&dev->msg_rb);
        if (free_size < sizeof(bsp_spi_msg_t)) return -1;
        /* lock */
        w_size = lwrb_write(&dev->msg_rb, msg, sizeof(bsp_spi_msg_t));
        return 0;
    }
    else
    {
        bsp_spi_start(dev, msg);
    }
    return 0;
}

int32_t bsp_spi_sync_transfer(bsp_spi_dev_t *dev, bsp_spi_msg_t *msg)
{
    HAL_StatusTypeDef status;

    if (msg->setup != NULL) {
        msg->setup(msg->arg);
    }

    if (msg->wbuf != NULL && msg->rbuf != NULL) {
        status = HAL_SPI_TransmitReceive(&dev->spi_handle, msg->wbuf, msg->rbuf, msg->wrlen, 20);
    } else if (msg->wbuf != NULL && msg->rbuf == NULL) {
        status = HAL_SPI_Transmit(&dev->spi_handle, msg->wbuf, msg->wrlen, 20);
    } else if (msg->wbuf == NULL && msg->rbuf != NULL) {
        status = HAL_SPI_Receive(&dev->spi_handle, msg->wbuf, msg->wrlen, 20);
    }

    if (msg->finish != NULL)
    {
        msg->finish(msg->arg);
    }
    return 0;
}

static void _spi_transmit_done(bsp_spi_dev_t *spi_dev)
{
    bsp_spi_msg_t cur_msg;
    bsp_spi_msg_t next_msg;
    uint32_t read_size = 0;
    memcpy(&cur_msg, &spi_dev->cur_msg, sizeof(bsp_spi_msg_t));
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (cur_msg.rbuf != NULL) SCB_InvalidateDCache_by_Addr((uint32_t *)cur_msg.rbuf, cur_msg.wrlen);
#endif
    uint32_t remain_size = lwrb_get_full(&spi_dev->msg_rb);
    if (remain_size > 0)
    {
        read_size = lwrb_read(&spi_dev->msg_rb, &next_msg, sizeof(bsp_spi_msg_t));
        if (read_size != sizeof(bsp_spi_msg_t))
        {

        }
        else
        {
            bsp_spi_start(spi_dev, &next_msg);
        }
    }
    else
    {
        spi_dev->is_busy = 0;
    }

    if (cur_msg.finish != NULL) {
        cur_msg.finish(cur_msg.arg);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_dev_t *spi_dev =  rt_container_of(hspi, bsp_spi_dev_t, spi_handle);
    _spi_transmit_done(spi_dev);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_dev_t *spi_dev =  rt_container_of(hspi, bsp_spi_dev_t, spi_handle);
    _spi_transmit_done(spi_dev);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_dev_t *spi_dev =  rt_container_of(hspi, bsp_spi_dev_t, spi_handle);
    _spi_transmit_done(spi_dev);
}

#ifdef MD_BSP_SPI1_CONFIG
void SPI1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_SPI_IRQHandler(&spi_dev[SPI1_INDEX].spi_handle);
    rt_interrupt_leave();
}

#ifdef MD_BSP_SPI1_DMA_TX_IRQHandler
void MD_BSP_SPI1_DMA_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI1_INDEX].dma.handle_tx);
    rt_interrupt_leave();
}
#endif

#ifdef MD_BSP_SPI1_DMA_RX_IRQHandler
void MD_BSP_SPI1_DMA_RX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI1_INDEX].dma.handle_rx);
    rt_interrupt_leave();
}
#endif
#endif /* MD_BSP_SPI1_CONFIG */

#ifdef MD_BSP_SPI2_CONFIG
void SPI2_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_SPI_IRQHandler(&spi_dev[SPI2_INDEX].spi_handle);
    rt_interrupt_leave();
}

#ifdef MD_BSP_SPI2_DMA_TX_IRQHandler
void MD_BSP_SPI2_DMA_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI2_INDEX].dma.handle_tx);
    rt_interrupt_leave();
}
#endif

#ifdef MD_BSP_SPI2_DMA_RX_IRQHandler
void MD_BSP_SPI2_DMA_RX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI2_INDEX].dma.handle_rx);
    rt_interrupt_leave();
}
#endif
#endif /* MD_BSP_SPI2_CONFIG */

#ifdef MD_BSP_SPI3_CONFIG
void SPI1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_SPI_IRQHandler(&spi_dev[SPI3_INDEX].spi_handle);
    rt_interrupt_leave();
}

#ifdef MD_BSP_SPI3_DMA_TX_IRQHandler
void MD_BSP_SPI3_DMA_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI3_INDEX].dma.handle_tx);
    rt_interrupt_leave();
}
#endif

#ifdef MD_BSP_SPI3_DMA_RX_IRQHandler
void MD_BSP_SPI3_DMA_RX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI3_INDEX].dma.handle_rx);
    rt_interrupt_leave();
}
#endif
#endif /* MD_BSP_SPI3_CONFIG */

#ifdef MD_BSP_SPI6_CONFIG
void SPI6_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_SPI_IRQHandler(&spi_dev[SPI6_INDEX].spi_handle);
    rt_interrupt_leave();
}

#ifdef MD_BSP_SPI6_DMA_TX_IRQHandler
void MD_BSP_SPI6_DMA_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI6_INDEX].dma.handle_tx);
    rt_interrupt_leave();
}
#endif

#ifdef MD_BSP_SPI6_DMA_RX_IRQHandler
void MD_BSP_SPI6_DMA_RX_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&spi_dev[SPI6_INDEX].dma.handle_rx);
    rt_interrupt_leave();
}
#endif
#endif /* MD_BSP_SPI6_CONFIG */

