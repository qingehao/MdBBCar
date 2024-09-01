#ifndef __MC_BSP_CONFIG_H__
#define __MC_BSP_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "motor_application_fun.h"


typedef struct
{
    int16_t ia;/* data */
    int16_t ib;
    int16_t ic;
    int16_t vbus;
    int16_t v_np;
    int32_t angle;
    int16_t ref;
}mc_input_package_t;

typedef struct
{
    int16_t ta;/* data */
    int16_t tb;
    int16_t tc;
}mcv_output_t;

typedef struct
{
    uint8_t ia_ch;/* data */
    uint8_t ib_ch;
    uint8_t ic_ch;
    uint8_t vbus_ch;
    uint8_t v_np_ch;
}mc_adc_channel_config_t;

#define MOTOR1_IDX 0
#define MOTOR2_IDX 1

void mc_bsp_init();
mcv_output_t mc_task_loop(uint8_t motor_index);
void get_motor_para_fun(mc_signal_t *mc_sigal, uint8_t i);
void set_motor_output_para(mc_foc_t *mc_foc, uint8_t i);
void mc_init_fun(MCVector_t *MCV, uint8_t i);

#ifdef __cplusplus
}
#endif

#endif
