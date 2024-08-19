#include "drv_mt6835.h"
#include "rtthread.h"
#include "drv_gpio.h"
#include "bsp_spi.h"
#include "rtthread.h"
#include <rtdevice.h>
#define LOG_TAG "bsp.drv_mt6835"
#define LOG_LVL LOG_LVL_DBG
#include "rtdbg.h"

static void _spi_transfer_setup(void *arg);
static void _spi_transfer_finish(void *arg);

static void _spi_transfer_setup(void *arg)
{
    mt6835_dev_t *ec_dev = (mt6835_dev_t *)arg;

    rt_pin_write(ec_dev->cs_pin, 0);
}

static void _spi_transfer_finish(void *arg)
{
    mt6835_dev_t *ec_dev = (mt6835_dev_t *)arg;

    rt_pin_write(ec_dev->cs_pin, 1);
}

int32_t mt6835_init(mt6835_dev_t *dev)
{
    int32_t ret = 0;

    if (dev == NULL) return -1;
    dev->spi_dev = bsp_spi_request(dev->bus_id);

    if (dev->spi_dev == NULL)
    {
        LOG_E("not find spi%d", dev->bus_id);
        return -2;
    }

    rt_pin_mode(dev->cs_pin, PIN_MODE_OUTPUT);
    rt_pin_write(dev->cs_pin, 1);

    return 0;
}

uint32_t mt6835_get_cpr(mt6835_dev_t *dev)
{
    return 2097152;
}

uint32_t mt6835_read(mt6835_dev_t *dev)
{
    uint8_t send_buf[6] = {0xA0, 0x03, 0xff, 0xff, 0xff, 0xff};
    uint8_t recv_buf[6] = {0};

    bsp_spi_msg_t msg = {
            .wbuf = send_buf,
            .rbuf = recv_buf,
            .wrlen = 6,
            .flag = 0,
            .bus_speed = 0,
            .arg = dev,
            .setup = _spi_transfer_setup,
            .finish = _spi_transfer_finish,
    };
    bsp_spi_sync_transfer(dev->spi_dev, &msg);

    uint32_t reg_val = recv_buf[2]<<13 | recv_buf[3]<<5 | recv_buf[4]>>3;
    return reg_val;
}
