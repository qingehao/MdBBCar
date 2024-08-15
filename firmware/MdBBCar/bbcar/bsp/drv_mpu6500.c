#include "drv_mpu6500.h"
#include "bsp_spi.h"
#include "bsp_mem.h"
#include "drv_gpio.h"
#include <rtdevice.h>

#define LOG_TAG "bsp.drv_ma732"
#define LOG_LVL LOG_LVL_DBG
#include "rtdbg.h"

__aligned(DCACHE_LINE_SIZE) static uint8_t mpu6500_burst_buf[15] __attribute__((section(".axiram_data")))= {0x00};

static void mpu6500_spi_transfer_setup(void *arg)
{
    imu_mpu6500_dev_t *dev = (imu_mpu6500_dev_t *)arg;

    rt_pin_write(dev->cs_pin, 0);
}

static void mpu6500_spi_transfer_finish(void *arg)
{
    imu_mpu6500_dev_t *dev = (imu_mpu6500_dev_t *)arg;

    rt_pin_write(dev->cs_pin, 1);
}

static void mpu6500_spi_seq_transfer_start(void *arg)
{
    imu_mpu6500_dev_t *dev = (imu_mpu6500_dev_t *)arg;

    dev->is_busy = 1;
    rt_pin_write(dev->cs_pin, 0);
}

static void mpu6500_spi_seq_transfer_finish(void *arg)
{
    imu_mpu6500_dev_t *dev = (imu_mpu6500_dev_t *)arg;

    dev->is_busy = 0;
    rt_pin_write(dev->cs_pin, 1);
}

static int32_t mpu6500_write_reg(imu_mpu6500_dev_t *dev, uint8_t reg, uint8_t val)
{
    if (dev == NULL) return -1;

    uint8_t send_buf[2] = {reg, val};
    bsp_spi_msg_t msg = {
        .arg = dev,
        .setup = mpu6500_spi_transfer_setup,
        .finish = mpu6500_spi_transfer_finish,
        .wbuf = send_buf,
        .rbuf = NULL,
        .wrlen = 2
    };
    bsp_spi_sync_transfer((bsp_spi_dev_t *)dev->bus_dev, &msg);

    return 0;
}

static int32_t mpu6500_read_reg(imu_mpu6500_dev_t *dev, uint8_t reg, uint8_t *val)
{
    if (dev == NULL) return -1;

    uint8_t send_buf[2] = {reg|0x80, 0xff};
    uint8_t recv_buf[2] = {0, 0};

    bsp_spi_msg_t msg = {
        .arg = dev,
        .setup = mpu6500_spi_transfer_setup,
        .finish = mpu6500_spi_transfer_finish,
        .wbuf = send_buf,
        .rbuf = recv_buf,
        .wrlen = 2
    };
    bsp_spi_sync_transfer((bsp_spi_dev_t *)dev->bus_dev, &msg);

    *val = recv_buf[1];

    return 0;
}

static int32_t mpu6500_config(imu_mpu6500_dev_t *dev)
{
    int32_t ret = 0;
    int err_cnt = 0;
    uint8_t reg_val = 0;
    bsp_spi_set_clk_psc((bsp_spi_dev_t *)dev->bus_dev, 256);
    do {
        ret = mpu6500_read_reg(dev, MPU6500_WHO_AM_I, &reg_val);
        if (ret == 0) {
            if (reg_val != 0x70) {
                err_cnt ++;
            } else {
                err_cnt = 0;
                break;
            }
        } else {
            err_cnt ++;
        }
        if (err_cnt > 5) {
            return -1;
        }
        rt_thread_mdelay(5);
    } while(reg_val != 0x70);

    mpu6500_write_reg(dev, MPU6500_PWR_MGMT_1, 0X80);
    rt_thread_mdelay(10);
    mpu6500_write_reg(dev, MPU6500_PWR_MGMT_1, 0X01);
    rt_thread_mdelay(10);
    mpu6500_write_reg(dev, MPU6500_PWR_MGMT_2, 0X00);

    mpu6500_write_reg(dev, MPU6500_SIGNAL_PATH_RESET, 0X07);
    mpu6500_write_reg(dev, MPU6500_CONFIG, 0X0);
    mpu6500_write_reg(dev, MPU6500_GYRO_CONFIG, 0x18);
    mpu6500_write_reg(dev, MPU6500_ACCEL_CONFIG, 0x10);
    bsp_spi_set_clk_psc((bsp_spi_dev_t *)dev->bus_dev, 8);
    return 0;
}

int32_t imu_mpu6500_update(imu_mpu6500_dev_t *dev)
{
    if (dev->is_init != 1) return -1;

    if (dev->is_busy) return 0; // is_busy
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)dev->dma_buf, dev->dma_buf_size);
#endif

    dev->imu_data.acc_x = dev->dma_buf[1] << 8 | dev->dma_buf[2];
    dev->imu_data.acc_y = dev->dma_buf[3] << 8 | dev->dma_buf[4];
    dev->imu_data.acc_z = dev->dma_buf[5] << 8 | dev->dma_buf[6];
    dev->imu_data.tem   = dev->dma_buf[7] << 8 | dev->dma_buf[8];
    dev->imu_data.gyro_x = dev->dma_buf[9] << 8 | dev->dma_buf[10];
    dev->imu_data.gyro_y = dev->dma_buf[11] << 8 | dev->dma_buf[12];
    dev->imu_data.gyro_z = dev->dma_buf[13] << 8 | dev->dma_buf[14];

    dev->imu_data.acc_x_f = (float)dev->imu_data.acc_x/4096.0f;
    dev->imu_data.acc_y_f = (float)dev->imu_data.acc_y/4096.0f;
    dev->imu_data.acc_z_f = (float)dev->imu_data.acc_z/4096.0f;

    bsp_spi_msg_t msg = {
        .arg = dev,
        .setup = mpu6500_spi_seq_transfer_start,
        .finish = mpu6500_spi_seq_transfer_finish,
        .wbuf = mpu6500_burst_buf,
        .rbuf = dev->dma_buf,
        .wrlen = 15
    };

    bsp_spi_async_transfer((bsp_spi_dev_t *)dev->bus_dev, &msg);

    return 1;
}

int32_t imu_mpu6500_get_ready_data(imu_mpu6500_dev_t *dev)
{

}

int32_t imu_mpu6500_init(imu_mpu6500_dev_t *dev)
{
    int32_t ret = 0;

    if (dev == NULL) return -1;

    dev->bus_dev = (void *)bsp_spi_request(dev->bus_index);
    if (dev->bus_dev == NULL) return -2;

    rt_pin_mode(dev->cs_pin, PIN_MODE_OUTPUT);
    rt_pin_write(dev->cs_pin, 1);

    ret = mpu6500_config(dev);
    if (ret !=0 ) return -3;

    dev->dma_buf_size = 15;

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uint8_t unaligned_size = (dev->dma_buf_size) % DCACHE_LINE_SIZE;
    if (unaligned_size != 0) {
        dev->dma_buf_size= dev->dma_buf_size + DCACHE_LINE_SIZE - unaligned_size;
    }
    dev->dma_buf = (uint8_t *)bsp_mem_dma_malloc(dev->dma_buf_size);
#else
    dev->dma_buf = bsp_mem_malloc(dev->dma_buf_size);
#endif
    if (dev->dma_buf == NULL) return -4;

    memset(mpu6500_burst_buf, 0xff, 15);
    mpu6500_burst_buf[0] = MPU6500_ACCEL_XOUT_H|0x80;
    SCB_CleanDCache_by_Addr((uint32_t *)mpu6500_burst_buf, 15);
    dev->is_init = 1;

    return 0;
}

#include "bsp_tick.h"

imu_mpu6500_dev_t dd_imu_dev;
static rt_sem_t sem_timer;
static int32_t test_pin = 0;

static void dd_tick_irq_cb()
{
    if (sem_timer)
        rt_sem_release(sem_timer);
}

static void dd_imu_test_thread(void *arg)
{
    imu_mpu6500_dev_t *imu = (imu_mpu6500_dev_t *)(arg);
    while(1)
    {
        rt_sem_take(sem_timer, RT_WAITING_FOREVER);
        rt_pin_write(test_pin, 1);
        imu_mpu6500_update(imu);
        rt_pin_write(test_pin, 0);
    }
}

void imu_6500_test(int pin)
{
    test_pin = pin;
    sem_timer = rt_sem_create("timer", 0, RT_IPC_FLAG_PRIO);

    dd_imu_dev.bus_index = 2;
    dd_imu_dev.cs_pin = GET_PIN(B, 12);
    imu_mpu6500_init(&dd_imu_dev);

    rt_thread_t tid = rt_thread_create("imu test", dd_imu_test_thread, &dd_imu_dev, 1024, 2, 10);
    rt_thread_startup(tid);

    bsp_tick_set_irq_cb(dd_tick_irq_cb);
}
