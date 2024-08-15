#include "bsp_spi.h"
#include "rtthread.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_ma732.h"
#include <rtdevice.h>

#define LOG_TAG "bsp.drv_ma732"
#define LOG_LVL LOG_LVL_DBG
#include "rtdbg.h"

__aligned(32) static uint8_t send_buf[4]  __attribute__((section(".axiram_data"))) = {0x0e|0x40, 0x00, 0x00, 0x00};
__aligned(32) static uint8_t recv_buf[4]  __attribute__((section(".axiram_data"))) = {0};

static void ma732_spi_transfer_setup(void *arg);
static void ma732_spi_transfer_finish(void *arg);

static void ma732_spi_transfer_setup(void *arg)
{
    ma732_dev_t *ec_dev = (ma732_dev_t *)arg;

    rt_pin_write(ec_dev->cs_pin, 0);
}

static void ma732_spi_transfer_finish(void *arg)
{
    ma732_dev_t *ec_dev = (ma732_dev_t *)arg;

    rt_pin_write(ec_dev->cs_pin, 1);
}

static uint8_t ma732_reg_val[][2] =
{
    {0x02, 0x00},
    {0x03, 0x00},
    {0x04, 0xc0},
    {0x05, 0xff},
    {0x06, 0x1c},
    {0x07, 0x00},
    {0x08, 0xc0},
    {0x09, 0x00},
    {0x0e, 0x88},
    {0x10, 0x00},
    {0x1b, 0x0f},
};

static int32_t ma732_read_reg(ma732_dev_t *dev, uint8_t reg, uint8_t *data)
{
    if (dev == NULL) return -1;

    uint8_t send_buf[4] = {reg|0x40, 0x00, 0x00, 0x00};
    uint8_t recv_buf[2] = {0};

    bsp_spi_msg_t msg[2] = {
        {
            .arg = dev,
            .setup = ma732_spi_transfer_setup,
            .finish = ma732_spi_transfer_finish,
            .wbuf = send_buf,
            .rbuf = recv_buf,
            .wrlen = 2
        },
        {
            .arg = dev,
            .setup = ma732_spi_transfer_setup,
            .finish = ma732_spi_transfer_finish,
            .wbuf = &send_buf[2],
            .rbuf = recv_buf,
            .wrlen = 2
        }
    };
    bsp_spi_sync_transfer(dev->spi_dev, &(msg[0]));
    rt_thread_mdelay(5);
    bsp_spi_sync_transfer(dev->spi_dev, &(msg[1]));
    *data = recv_buf[0];

    return 0;
}

static int32_t ma732_write_reg(ma732_dev_t *dev, uint8_t reg, uint8_t data)
{
    if (dev == NULL) return -1;
    uint8_t send_buf[4] = {reg|0x80, data, 0x00, 0x00};
    uint8_t recv_buf[2] = {0};
    bsp_spi_msg_t msg[2] = {
        {
            .arg = dev,
            .setup = ma732_spi_transfer_setup,
            .finish = ma732_spi_transfer_finish,
            .wbuf = send_buf,
            .rbuf = recv_buf,
            .wrlen = 2
        },
        {
            .arg = dev,
            .setup = ma732_spi_transfer_setup,
            .finish = ma732_spi_transfer_finish,
            .wbuf = &send_buf[2],
            .rbuf = recv_buf,
            .wrlen = 2
        }
    };
    bsp_spi_sync_transfer(dev->spi_dev, &(msg[0]));
    rt_thread_mdelay(50);
    bsp_spi_sync_transfer(dev->spi_dev, &(msg[1]));

    return 0;
}

uint16_t ma732_read(ma732_dev_t *dev)
{
    uint8_t send_buf[2] = {0x00, 0x00};
    uint8_t recv_buf[2] = {0};

    bsp_spi_msg_t msg = {
            .arg = dev,
            .setup = ma732_spi_transfer_setup,
            .finish = ma732_spi_transfer_finish,
            .wbuf = send_buf,
            .rbuf = recv_buf,
            .wrlen = 2
    };
    bsp_spi_sync_transfer(dev->spi_dev, &msg);

    uint16_t reg_val = recv_buf[0]<<8 | recv_buf[1];
    int32_t ret = (reg_val>>2)&(0x3fff);
    return ret;
}

uint32_t ma732_get_cpr(ma732_dev_t *dev)
{
    (void)(dev);

    return 16384; // 2^14
}

int32_t ma732_init(ma732_dev_t *dev)
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

    for(int i=0; i<sizeof(ma732_reg_val)/2; i++)
    {
        uint8_t read_val = 0;
        ret = ma732_read_reg(dev, ma732_reg_val[i][0], &read_val);
        if (ret != 0)
        {
            LOG_E("dev read reg fail");
            return -2;
        }
        LOG_D("read reg %02x %02x", ma732_reg_val[i][0], read_val);
        if (read_val != ma732_reg_val[i][1])
        {
            ma732_write_reg(dev, ma732_reg_val[i][0], ma732_reg_val[i][1]);
        }
    }
}
