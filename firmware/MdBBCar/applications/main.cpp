/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-10-25     zylx         first version
 */

#include <rtdevice.h>
#include <drv_gpio.h>
#include "bbcar.h"
#include "arm_math.h"

/* defined the LED1 pin: PC13 */

#define BEEP_PIN    GET_PIN(C, 9)

/**
 *  DMA Config
 *  U3_RX      DMA1_Stream0
 *  U3_TX      DMA1_Stream1
 *  SPI1_RX    DMA1_Stream2   ENCODER
 *  SPI1_TX    DMA1_Stream3
 *  SPI2_RX    DMA1_Stream4   IMU
 *  SPI2_TX    DMA1_Stream5
 *  SPI3_RX    DMA1_Stream6   FLASH
 *  SPI3_TX    DMA1_Stream7
 *  SPI6_RX    DMA2_Stream0   FPV_IMU
 *  SPI6_TX    DMA2_Stream1
 *  ADC        DMA2_Stream2   FPV_IMU
 */

#include "SimpleFOC.h"

MagneticSensorSPI sensor = MagneticSensorSPI(1, GET_PIN(D, 10), 14, 0x3fff);

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, GET_PIN(A, 4));
float target_velocity = 0;

static void foc_loop(void *arg)
{
    while(1)
    {
        rt_thread_mdelay(1);
        motor.loopFOC();
        motor.move(target_velocity);
    }
}

#define LED1_PIN    GET_PIN(E, 4)

volatile uint32_t dd_foc_is_init = 0;

static void dd_foc_cb(void *arg)
{
    if (dd_foc_is_init)
    {
        rt_pin_write(LED1_PIN, 1);
        motor.loopFOC();
        motor.move(target_velocity);
        rt_pin_write(LED1_PIN, 0);
    }
}

void simplefoc_test()
{
    dd_foc_is_init = 0;
    // bsp_pwm_init();

    bsp_foc_init();
    bsp_foc_t *bsp_foc = bsp_foc_request();
    bsp_foc_set_callback(bsp_foc, dd_foc_cb, bsp_foc);
    bsp_foc_start(bsp_foc);

    sensor.min_elapsed_time = 0.0005;
    sensor.sensor_update_freq = 10000;
    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 8.4;
    // limit the maximal dc voltage the driver can set
    // as a protection measure for the low-resistance motors
    // this value is fixed on startup
    driver.voltage_limit = 8.4;
    driver.init();
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::velocity; // velocity angle
    // velocity PI controller parameters
    // 2808 P:0.04 I:0.21 D:0
    motor.PID_velocity.P = 0.04;
    motor.PID_velocity.I = 0.21;
    motor.PID_velocity.D = 0;

    motor.voltage_limit = 8.4;   // [V]
    motor.PID_velocity.output_ramp = 1000;

    motor.LPF_velocity.Tf = 0.01f;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.P_angle.P = 2;

    motor.voltage_sensor_align = 1;
    motor.init();
    // align sensor and start FOC
    motor.initFOC();
    // rt_thread_t tid = rt_thread_create("foc", foc_loop, NULL, 1024, 1, 10);
    // rt_thread_startup(tid);
    dd_foc_is_init = 1;
}

extern "C" int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    bsp_tick_init();
    rt_pin_mode(BEEP_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BEEP_PIN, PIN_LOW);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED1_PIN, 0);
    bsp_uart_init();

    // bsp_pwm_init();
    // simplefoc_test();
    // bsp_tick_test(LED1_PIN);

    imu_6500_test(LED1_PIN);
    // bsp_pwm_test();
    // bsp_foc_test();

    while (count++)
    {
        rt_thread_mdelay(100);
    }
    return RT_EOK;
}


