#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void bsp_encoder_init(void);
uint32_t bsp_encoder_read(uint8_t index);



#ifdef __cplusplus
}
#endif
#endif