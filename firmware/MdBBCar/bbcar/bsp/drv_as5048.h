#ifndef __DRV_AS5048_H__
#define __DRV_AS5048_H__

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
} as5048_dev_t;

uint16_t as5048_read(as5048_dev_t *dev);
uint32_t as5048_get_cpr(as5048_dev_t *dev);
int32_t as5048_init(as5048_dev_t *dev);

int32_t bsp_as5048_test(void);

#ifdef __cplusplus
}
#endif

#endif
