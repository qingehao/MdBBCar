#ifndef __DRV_MA732_H__
#define __DRV_MA732_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "bsp_spi.h"

typedef struct
{
    bsp_spi_dev_t *spi_dev;
    uint8_t  bus_id;
    uint32_t cs_pin;
    uint8_t  is_ready;
} ma732_dev_t;

uint16_t ma732_read(ma732_dev_t *dev);
uint32_t ma732_get_cpr(ma732_dev_t *dev);
int32_t ma732_init(ma732_dev_t *dev);

void bsp_encoder_init(void);
uint32_t bsp_encoder_read(uint8_t index);


#ifdef __cplusplus
}
#endif

#endif

