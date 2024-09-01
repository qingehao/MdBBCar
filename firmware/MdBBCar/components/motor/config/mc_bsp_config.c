#include "mc_bsp_config.h"
#include "mc_app_config.h"
#include "motor_application_fun.h"
//#include "mc_app_config.c"
#include "bsp_foc.h"
#include "string.h"

extern MCVector_t *mcv[MOTOR_NUM];

static uint16_t mc_adc_value[MC_ADC_CH_NUM];

mcv_output_t mcv_output_para[MOTOR_NUM];

const mc_adc_channel_config_t mc_adc_channel[MOTOR_NUM] =
{
    /*motor1*/
    {
        .ia_ch  = 2,
        .ib_ch  = 3,
        .ic_ch  = 0, // no use
        .vbus_ch  = 11,
        .v_np_ch  = 0,
    },
    /*motor2*/
    {
        .ia_ch  = 0,
        .ib_ch  = 1,
        .ic_ch  = 0, // no use
        .vbus_ch  = 11,
        .v_np_ch  = 0,
    },
};


/*获取电机参数*/
void get_motor_para_fun(mc_signal_t *mc_sigal, uint8_t i)
{
    mc_sigal->adc_data.ia = mc_adc_value[mc_adc_channel[i].ia_ch];
    mc_sigal->adc_data.ib = mc_adc_value[mc_adc_channel[i].ib_ch];
    mc_sigal->adc_data.ic = mc_adc_value[mc_adc_channel[i].ic_ch];
    mc_sigal->adc_data.vbus = mc_adc_value[mc_adc_channel[i].vbus_ch];
    mc_sigal->adc_data.v_np = mc_adc_value[mc_adc_channel[i].v_np_ch];

//    mc_sigal->org_angle = mc_in.angle;
//    mc_sigal->ref = mc_in.ref;
}

void set_motor_output_para(mc_foc_t *mc_foc, uint8_t i)
{
    bsp_foc_set_pwm(i, mc_foc->pwm.taOn, mc_foc->pwm.tbOn, mc_foc->pwm.tcOn);
}

static void mc_adc_sample_callback(void *arg, uint8_t *adc_data)
{
    memcpy(mc_adc_value, adc_data, MC_ADC_CH_NUM*2);
    for (int i=0; i<MOTOR_NUM; i++)
    {
       mc_task_loop(i);
    }
}

void mc_bsp_init()
{
    bsp_foc_init();
    bsp_foc_set_callback(mc_adc_sample_callback, NULL);

    bsp_foc_pwm_enable(MOTOR1_IDX, 1); // 电机1 使能
    bsp_foc_pwm_enable(MOTOR2_IDX, 1); // 电机2 使能

    bsp_foc_start();
}

mcv_output_t mc_task_loop(uint8_t motor_index)
{
    mc_control_loop(motor_index);
    switch (mcv[motor_index]->mcstate)
    {
        case MC_INIT:
            mc_init_fun(mcv[motor_index], motor_index);
            break;
        case MC_PREPARE:
            mc_prepare_fun(mcv[motor_index]);
            break;
        default:
            break;
    }
    return mcv_output_para[motor_index];
}
