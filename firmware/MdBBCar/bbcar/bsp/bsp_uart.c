#include "rtdevice.h"
#include "link.h"


#define LINK_UART3_RB_SIZE 1024
#define LINK_UART3_LINE_SIZE 512

static rt_device_t serial3;
static uint8_t link_uart3_rb_buffer[LINK_UART3_RB_SIZE] = {0};
static uint8_t link_uart3_line_buf[LINK_UART3_LINE_SIZE] = {0};
static uint8_t link_uart3_index = 0;

static uint8_t rx_buffer[512] = {0};

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    uint32_t rx_length = 0;
    if (size > 512) size = 512;
    rx_length = rt_device_read(dev, 0, rx_buffer, size);

    link_recv_data(link_uart3_index, rx_buffer, rx_length);
    return 0;
}

static int32_t uart3_link_send(uint8_t *buf, uint32_t size)
{
    int32_t ret = 0;
    ret = rt_device_write(serial3, 0, buf, size);
    return ret;
}

int32_t bsp_uart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* 查找串口设备 */
    serial3 = rt_device_find("uart3");
    if (!serial3)
    {
        rt_kprintf("find %s failed!\n", "uart3");
        return RT_ERROR;
    }

    config.baud_rate = BAUD_RATE_921600;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.rx_bufsz  = 256;
    config.tx_bufsz  = 512;
    config.parity    = PARITY_NONE;

    rt_device_control(serial3, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_open(serial3, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_NON_BLOCKING | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
    rt_device_set_rx_indicate(serial3, uart_input);

    link_uart3_index = link_add(link_uart3_rb_buffer, LINK_UART3_RB_SIZE,
                link_uart3_line_buf, LINK_UART3_LINE_SIZE,
                uart3_link_send);

    return 0;
}
