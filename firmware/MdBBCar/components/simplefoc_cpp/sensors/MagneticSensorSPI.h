#ifndef MAGNETICSENSORSPI_LIB_H
#define MAGNETICSENSORSPI_LIB_H


#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "bsp_spi.h"

class MagneticSensorSPI: public Sensor
{
 public:
    /**
     *  MagneticSensorSPI class constructor
     * @param cs  SPI chip select pin
     * @param bit_resolution   sensor resolution bit number
     * @param angle_register  (optional) angle read register - default 0x3FFF
     */
    MagneticSensorSPI(uint8_t spi_index, int cs_pin, int bit_resolution, int angle_register);

    void init();

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;

    int cs_pin;
    int spi_index;
    bsp_spi_dev_t *spi_dev;

  private:
    static void _spi_transfer_setup(void *arg);
    static void _spi_transfer_finish(void *arg);

    float cpr; //!< Maximum range of the magnetic sensor
    // spi variables
    int angle_register; //!< SPI angle register to read

    // spi functions
    /** Stop SPI communication */
    void close();

    uint8_t send_buf[6];
    uint8_t recv_buf[6];
    float angle;
    uint32_t reg_val;

    int bit_resolution; //!< the number of bites of angle data
    int command_parity_bit; //!< the bit where parity flag is stored in command
    int command_rw_bit; //!< the bit where read/write flag is stored in command
    int data_start_bit; //!< the the position of first bit
};


#endif
