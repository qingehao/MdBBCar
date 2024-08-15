
#include "MagneticSensorSPI.h"
#include "bsp_spi.h"
#include "drv_gpio.h"
#include "drv_ma732.h"

MagneticSensorSPI::MagneticSensorSPI(uint8_t spi_index, int cs_pin, int bit_resolution, int angle_register)
{
    this->spi_index = spi_index;
    this->cs_pin = cs_pin;

    // this->send_buf[0] = (uint8_t)((angle_register&0x00003f00)>>8);
    // this->send_buf[1] = (uint8_t)(angle_register&0x000000ff);

    this->send_buf[0] = 0xA0;
    this->send_buf[1] = 0x03;
    this->send_buf[2] = 0xff;
    this->send_buf[3] = 0xff;
    this->send_buf[4] = 0xff;
    this->send_buf[5] = 0xff;
}

void MagneticSensorSPI::init()
{
    // spi_dev = bsp_spi_request(spi_index);
    // rt_pin_mode(cs_pin, PIN_MODE_OUTPUT);
    // rt_pin_write(cs_pin, 1);
    bsp_encoder_init();
    this->Sensor::init(); // call base class init
}

void MagneticSensorSPI::_spi_transfer_setup(void *arg)
{
    MagneticSensorSPI *mag = (MagneticSensorSPI *)arg;
    rt_pin_write(mag->cs_pin, 0);
}

void MagneticSensorSPI::_spi_transfer_finish(void *arg)
{
    MagneticSensorSPI *mag = (MagneticSensorSPI *)arg;
    rt_pin_write(mag->cs_pin, 1);
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSPI::getSensorAngle()
{
    /* AS5648A */
    // bsp_spi_msg_t msg = {
    //         .wbuf = send_buf,
    //         .rbuf = recv_buf,
    //         .wrlen = 2,
    //         .flag = 0,
    //         .bus_speed = 0,
    //         .arg = this,
    //         .setup = _spi_transfer_setup,
    //         .finish = _spi_transfer_finish,
    // };
    // bsp_spi_sync_transfer(spi_dev, &msg);

    // uint16_t reg_val = recv_buf[0]<<8 | recv_buf[1];
    // reg_val = reg_val&0x3fff;
    // angle = (reg_val / (float)16384) * 6.28318530718f;
    /* AS5648A */

    /* MT6835 */
    // bsp_spi_msg_t msg = {
    //         .wbuf = send_buf,
    //         .rbuf = recv_buf,
    //         .wrlen = 6,
    //         .flag = 0,
    //         .bus_speed = 0,
    //         .arg = this,
    //         .setup = _spi_transfer_setup,
    //         .finish = _spi_transfer_finish,
    // };
    // bsp_spi_sync_transfer(spi_dev, &msg);

    // reg_val = recv_buf[2]<<13 | recv_buf[3]<<5 | recv_buf[4]>>3;
    // angle = (reg_val / (float)2097152) * 6.28318530718f;

    /* MT6835 */

    uint32_t reg_val = bsp_encoder_read(0);
    angle = (reg_val / (float)16384) * 6.28318530718f;

    return angle;
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void MagneticSensorSPI::close(){

}


