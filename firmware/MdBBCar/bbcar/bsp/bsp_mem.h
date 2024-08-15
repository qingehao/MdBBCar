#ifndef __BSP_MEM_H__
#define __BSP_MEM_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DCACHE_LINE_SIZE 32

void *bsp_mem_dma_malloc(uint32_t size);
void *bsp_mem_malloc(uint32_t size);
void bsp_mem_free(void *p);


#ifdef __cplusplus
}
#endif
#endif