#include "motor_application_fun.h"
#include "mc_app_config.h"
#include "string.h"

const motor_para_t init_motor_para[MOTOR_NUM] =
{
    {
        .pole           = 8,
        .rs             = 0.0f,
        .ld             = 0.0f,
        .lq             = 0.0f,
        .flux           = 0.0f,
    },
    {
        .pole           = 8,
        .rs             = 0.0f,
        .ld             = 0.0f,
        .lq             = 0.0f,
        .flux           = 0.0f,
    },
};

const control_system_config_t init_pos_ctl[MOTOR_NUM] =
{
//  motor1
    {
        .manage =
        {
            .mode               = ORG_DIRECT,
            // .step               = 10, // 梯度步长
            // .time               = 3,  // 线性插值法，插值点数
        },
        .pid_ctl =
        {
            .out_limit          = 32767,
            .kp                 = 1.0f,
            .ki                 = 0.0f,
            .ki_out             = 2000,
            .kd                 = 0.0f,
            .kd_limit           = 2000,

            .fp                 = 10,    // 极点，暂时不配置，默认零即可
            .fz                 = 1,     // 零点，暂时不配置，默认零即可
            .lpf_fc             = 2000,
            .fs                 = MC_CONTROL_FRE,
            .init               = 1,

            .pid_mode           = PID_NORM,
        },
        .filt =
        {
            {
                .filt_type          = NOTCH_FILTER,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
            {
                .filt_type          = PHASE_PEAK,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
        },
        .filt_num               = 0, //配置滤波器数量
    },
//  motor2
    {
        .manage =
        {
            .mode               = ORG_DIRECT,
            // .step               = 10, // 梯度步长
            // .time               = 3,  // 线性插值法，插值点数
        },
        .pid_ctl =
        {
            .out_limit          = 32767,
            .kp                 = 1.0f,
            .ki                 = 0.0f,
            .ki_out             = 2000,
            .kd                 = 0.0f,
            .kd_limit           = 2000,

            .fp                 = 10,    // 极点，暂时不配置，默认零即可
            .fz                 = 1,     // 零点，暂时不配置，默认零即可
            .lpf_fc             = 2000,
            .fs                 = MC_CONTROL_FRE,
            .init               = 1,

            .pid_mode           = PID_NORM,
        },
        .filt =
        {
            {
                .filt_type          = NOTCH_FILTER,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
            {
                .filt_type          = PHASE_PEAK,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
        },
        .filt_num               = 0, //配置滤波器数量
    },
};

const control_system_config_t init_spe_ctl[MOTOR_NUM] =
{
    // motor1
    {
        .manage =
        {
            .mode               = ORG_DIRECT,
            // .step               = 10, // 梯度步长
            // .time               = 3,  // 线性插值法，插值点数
        },
        .pid_ctl =
        {
            .out_limit          = 32767,
            .kp                 = 1.0f,
            .ki                 = 0.0f,
            .ki_out             = 2000,
            .kd                 = 0.0f,
            .kd_limit           = 2000,

            .fp                 = 10,    // 极点，暂时不配置，默认零即可
            .fz                 = 1,     // 零点，暂时不配置，默认零即可
            .lpf_fc             = 2000,
            .fs                 = MC_CONTROL_FRE,
            .init               = 1,

            .pid_mode           = PID_NORM,
        },
        .filt =
        {
            {
                .filt_type          = NOTCH_FILTER,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
            {
                .filt_type          = PHASE_PEAK,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
        },
        .filt_num               = 0, //配置滤波器数量
    },
    // motor2
    {
        .manage =
        {
            .mode               = ORG_DIRECT,
            // .step               = 10, // 梯度步长
            // .time               = 3,  // 线性插值法，插值点数
        },
        .pid_ctl =
        {
            .out_limit          = 32767,
            .kp                 = 1.0f,
            .ki                 = 0.0f,
            .ki_out             = 2000,
            .kd                 = 0.0f,
            .kd_limit           = 2000,

            .fp                 = 10,    // 极点，暂时不配置，默认零即可
            .fz                 = 1,     // 零点，暂时不配置，默认零即可
            .lpf_fc             = 2000,
            .fs                 = MC_CONTROL_FRE,
            .init               = 1,

            .pid_mode           = PID_NORM,
        },
        .filt =
        {
            {
                .filt_type          = NOTCH_FILTER,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
            {
                .filt_type          = PHASE_PEAK,

                .para               = {1000, 0, 0, 50, 3},      // [fc  fz  fp  bs  As]
                .fs                 = MC_CONTROL_FRE,
                .init               = 0,
            },
        },
        .filt_num               = 0, //配置滤波器数量
    },
};

const pid_ctl_t init_id_ctl[MOTOR_NUM] =
{
    // motor1
    {
        .pid_org =
        {
            .mode           = ORG_DIRECT,
        },
        .out_limit          = 32767,
        .kp                 = 1.0f,
        .ki                 = 0.0f,
        .ki_out             = 2000,
        .kd                 = 0.0f,
        .kd_limit           = 2000,

        .fp                 = 10,    // 极点，暂时不配置，默认零即可
        .fz                 = 1,     // 零点，暂时不配置，默认零即可
        .lpf_fc             = 2000,
        .fs                 = MC_CONTROL_FRE,
        .init               = 1,

        .pid_mode           = PID_NORM,
    },
    // motor2
    {
        .pid_org =
        {
            .mode           = ORG_DIRECT,
        },
        .out_limit          = 32767,
        .kp                 = 1.0f,
        .ki                 = 0.0f,
        .ki_out             = 2000,
        .kd                 = 0.0f,
        .kd_limit           = 2000,

        .fp                 = 10,    // 极点，暂时不配置，默认零即可
        .fz                 = 1,     // 零点，暂时不配置，默认零即可
        .lpf_fc             = 2000,
        .fs                 = MC_CONTROL_FRE,
        .init               = 1,

        .pid_mode           = PID_NORM,
    },
};

const pid_ctl_t init_iq_ctl[MOTOR_NUM] =
{
    // motor1
    {
        .pid_org =
        {
            .mode           = ORG_DIRECT,
        },
        .out_limit          = 32767,
        .kp                 = 1.0f,
        .ki                 = 0.0f,
        .ki_out             = 2000,
        .kd                 = 0.0f,
        .kd_limit           = 2000,

        .fp                 = 10,    // 极点，暂时不配置，默认零即可
        .fz                 = 1,     // 零点，暂时不配置，默认零即可
        .lpf_fc             = 2000,
        .fs                 = MC_CONTROL_FRE,
        .init               = 1,

        .pid_mode           = PID_NORM,
    },
    // motor2
    {
        .pid_org =
        {
            .mode           = ORG_DIRECT,
        },
        .out_limit          = 32767,
        .kp                 = 1.0f,
        .ki                 = 0.0f,
        .ki_out             = 2000,
        .kd                 = 0.0f,
        .kd_limit           = 2000,

        .fp                 = 10,    // 极点，暂时不配置，默认零即可
        .fz                 = 1,     // 零点，暂时不配置，默认零即可
        .lpf_fc             = 2000,
        .fs                 = MC_CONTROL_FRE,
        .init               = 1,

        .pid_mode           = PID_NORM,
    },
};

const mc_eleangle_offset_cali_t init_eleangle_offset_cali[MOTOR_NUM] =
{
    // motor1
    {
        .state          = INIT,
        .eangle_offset  = 0,
        .vs             = 5000,
        .time           = MC_CONTROL_FRE * 3,
    },
    // motor2
    {
        .state          = INIT,
        .eangle_offset  = 0,
        .vs             = 5000,
        .time           = MC_CONTROL_FRE * 3,
    },
};

const mc_cur_offset_cali_t init_cur_offset_cali[MOTOR_NUM] =
{
    // motor1
    {
        .state          = NONE,
    },
    // motor2
    {
        .state          = NONE,
    },
};

void mc_init_fun(MCVector_t *MCV, uint8_t i)
{
    MCV->motorid = i;
    memcpy(&MCV->foc.para, &init_motor_para[i], sizeof(motor_para_t));
    memcpy(&MCV->pos_ctl, &init_pos_ctl[i], sizeof(control_system_config_t));
    memcpy(&MCV->spe_ctl, &init_spe_ctl[i], sizeof(control_system_config_t));
    memcpy(&MCV->foc.id_ctl, &init_id_ctl[i], sizeof(pid_ctl_t));
    memcpy(&MCV->foc.iq_ctl, &init_iq_ctl[i], sizeof(pid_ctl_t));
    memcpy(&MCV->eleangle_offset_cali, &init_eleangle_offset_cali[i], sizeof(mc_eleangle_offset_cali_t));
    memcpy(&MCV->cur_offset_cali, &init_cur_offset_cali[i], sizeof(mc_cur_offset_cali_t));
}

