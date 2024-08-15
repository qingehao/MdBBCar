#include "bsp_mem.h"
#include "rtthread.h"

#define BSP_AXI_RAM_FOR_DMA_SIZE 1024

__aligned(DCACHE_LINE_SIZE) static uint8_t bsp_axi_ram_for_dma[BSP_AXI_RAM_FOR_DMA_SIZE] __attribute__((section(".axiram_data"))) = {0};
static volatile uint32_t axi_ram_alloced_size = 0;

void *bsp_mem_dma_malloc(uint32_t size)
{
    void *p = NULL;

    if (axi_ram_alloced_size + size > BSP_AXI_RAM_FOR_DMA_SIZE) return NULL;

    p = &bsp_axi_ram_for_dma[axi_ram_alloced_size];
    axi_ram_alloced_size += size;

    return p;
}

void *bsp_mem_malloc(uint32_t size)
{
    return rt_malloc(size);
}

void bsp_mem_free(void *p)
{
    rt_free(p);
}
