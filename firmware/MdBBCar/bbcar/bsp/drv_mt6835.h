#ifndef __DRV_MT6835_H__
#define __DRV_MT6835_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "bsp_spi.h"

#define MA6835_CPR 2097152

typedef struct
{
    bsp_spi_dev_t *spi_dev;
    uint8_t  bus_id;
    uint32_t cs_pin;
    uint8_t  is_ready;
} mt6835_dev_t;

uint32_t mt6835_read(mt6835_dev_t *dev);
uint32_t mt6835_get_cpr(mt6835_dev_t *dev);
int32_t mt6835_init(mt6835_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif