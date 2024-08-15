#include "drv_as5048.h"
#include "rtthread.h"
#include "drv_gpio.h"

static void _spi_transfer_setup(void *arg);
static void _spi_transfer_finish(void *arg);

static void _spi_transfer_setup(void *arg)
{
    as5048_dev_t *ec_dev = (as5048_dev_t *)arg;

    rt_pin_write(ec_dev->cs_pin, 0);
}

static void _spi_transfer_finish(void *arg)
{
    as5048_dev_t *ec_dev = (as5048_dev_t *)arg;

    rt_pin_write(ec_dev->cs_pin, 1);
}

int32_t as5048_init(as5048_dev_t *dev)
{
    int32_t ret = 0;

    if (dev == NULL) return -1;
    dev->spi_dev = bsp_spi_request(dev->bus_id);

    if (dev->spi_dev == NULL)
    {
        return -2;
    }

    rt_pin_mode(dev->cs_pin, PIN_MODE_OUTPUT);
    rt_pin_write(dev->cs_pin, 1);
}

uint32_t as5048_get_cpr(as5048_dev_t *dev)
{
    (void)(dev);

    return 16384; // 2^14
}

uint16_t as5048_read(as5048_dev_t *dev)
{
    uint8_t send_buf[2] = {0x3F, 0xFF};
    uint8_t recv_buf[2] = {0};

    bsp_spi_msg_t msg = {
            .arg = dev,
            .setup = _spi_transfer_setup,
            .finish =_spi_transfer_finish,
            .wbuf = send_buf,
            .rbuf = recv_buf,
            .wrlen = 2
    };
    bsp_spi_sync_transfer(dev->spi_dev, &msg);

    uint16_t reg_val = recv_buf[0]<<8 | recv_buf[1];
    // int32_t ret = (reg_val)&(0x3fff);
    return (reg_val)&(0x3fff);
}

static as5048_dev_t ec_dev[1] = {
    {
        .bus_id = 1,
        .cs_pin = GET_PIN(D, 10),
        .is_ready = 0
    },
};

volatile uint16_t dd_ang = 0;

float dd_sfoc_vel = 0;
static float get_sensor_angle(void *user_data);

static void bsp_ec_test_thread(void *arg)
{
    as5048_dev_t *ec_dev = (as5048_dev_t *)arg;
    uint32_t cnt = 0;
    while(1)
    {
        cnt ++;
        rt_thread_mdelay(2);
        // sfoc_sensor_update(&dd_sfoc_sensor);
        dd_sfoc_vel = get_sensor_angle(ec_dev);
        if (cnt % 2 == 0)
        {
            // dd_sfoc_vel = sfoc_sensor_getVelocity(&dd_sfoc_sensor);
        }
    }
}

static float get_sensor_angle(void *user_data)
{
    as5048_dev_t *ec_dev = (as5048_dev_t *)user_data;
    uint16_t reg_val = as5048_read(ec_dev);
    dd_ang = reg_val;
    return (reg_val / (float)16384) * 6.28318530718f;
}

int32_t bsp_as5048_test(void)
{
    for (int i=0; i<1; i++)
    {
        as5048_init(&ec_dev[i]);
    }

    // sfoc_sensor_init(&dd_sfoc_sensor, get_sensor_angle, &ec_dev[0]);

    rt_thread_t tid = rt_thread_create("ec_test", bsp_ec_test_thread, &ec_dev[0], 1024, 6, 10);
    rt_thread_startup(tid);

    return 0;
}
