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

#define LED1_PIN    GET_PIN(E, 4)

// 16384 2097152
MagneticSensorSPI SensorL = MagneticSensorSPI(1, 2097152);

BLDCDriver3PWM DriverL = BLDCDriver3PWM(1, GET_PIN(A, 4));
BLDCMotor MotorL = BLDCMotor(7);


float target_velocity = 0;

volatile uint32_t dd_foc_is_init = 0;

static void dd_foc_cb(void *arg)
{
    if (dd_foc_is_init)
    {
        rt_pin_write(LED1_PIN, 1);
        MotorL.loopFOC();
        MotorL.move(target_velocity);
        rt_pin_write(LED1_PIN, 0);
    }
}

void simplefoc_test()
{
    dd_foc_is_init = 0;

    bsp_encoder_init(); // 编码器初始化
    bsp_foc_init();     // pwm adc初始化

    bsp_foc_t *bsp_foc = bsp_foc_request();
    bsp_foc_set_callback(bsp_foc, dd_foc_cb, bsp_foc); // 设置ADC采样完成回调函数
    bsp_foc_start(bsp_foc); // // ADC开始采样

    SensorL.min_elapsed_time = 0.0005;
    SensorL.sensor_update_freq = 10000;
    SensorL.init();
    MotorL.linkSensor(&SensorL);

    DriverL.voltage_power_supply = 8.4; // 设置供电电压
    DriverL.voltage_limit = 8.4;        // 设置电压限制
    DriverL.init();
    MotorL.linkDriver(&DriverL);

    MotorL.controller = MotionControlType::velocity; // velocity angle
    MotorL.voltage_limit = 8.4;   // [V]
    MotorL.foc_modulation = FOCModulationType::SpaceVectorPWM;
    MotorL.voltage_sensor_align = 1; // 校准使用的电压

    // 速度环 PID
    // 2808 P:0.04 I:0.21 D:0
    MotorL.PID_velocity.P = 0.04;
    MotorL.PID_velocity.I = 0.21;
    MotorL.PID_velocity.D = 0;
    MotorL.PID_velocity.output_ramp = 1000; // 输出斜坡

    // 速度 LPF
    MotorL.LPF_velocity.Tf = 0.01f;

    MotorL.P_angle.P = 2;
    MotorL.init();
    MotorL.initFOC();
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

    simplefoc_test();
    // bsp_tick_test(LED1_PIN);

    // imu_6500_test(LED1_PIN);
    // bsp_pwm_test();
    // bsp_foc_test();

    while (count++)
    {
        rt_thread_mdelay(100);
    }
    return RT_EOK;
}


