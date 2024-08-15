#include "bsp_encoder.h"
#include "drv_ma732.h"
#include "drv_gpio.h"
#include <rtdevice.h>

static ma732_dev_t ec_dev[2] = {
    {
        .bus_id = 1,
        .cs_pin = GET_PIN(D, 9),
        .is_ready = 0
    },
    {
        .bus_id = 1,
        .cs_pin = GET_PIN(A, 3),
        .is_ready = 0
    }
};

void bsp_encoder_init(void)
{
    for (int i=0; i<2; i++)
    {
        ma732_init(&ec_dev[i]);
    }
}

uint32_t bsp_encoder_read(uint8_t index)
{
    uint16_t reg_val = ma732_read(&ec_dev[index]);
    return (uint32_t)reg_val;
}
