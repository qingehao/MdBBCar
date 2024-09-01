#ifndef __MC_CONTROL_FUN_H__
#define __MC_CONTROL_FUN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "mc_cordic.h"
#include "mc_basic_def.h"
#include "math.h"

typedef enum
{
    BUTTER_LP       = 0,
    BUTTER_HP       = 1,
    CHEBY_LP        = 2,
    NOTCH_FILTER    = 3,
    PHASE_PEAK      = 4,
    LEAD_LAG        = 5,
    FREE_MODE       = 6,
}filt_type_t;

typedef struct
{
    filt_type_t filt_type;

    float para[5];      // [fc  fz  fp  bs  As]
    float num[3];
    float den[3];
    float input[3];
    float output[3];
    uint16_t fs;

    uint8_t init;
}filt2_t;

typedef enum
{
    ORG_DIRECT      = 0,
    ORG_STEP        = 1,
    LINEAR_STEP     = 2,
}manage_mode_t;

typedef struct
{
    manage_mode_t   mode;
    float           ref;
    float           ref_last;
    float           managed_ref;
    float           step;
    int16_t         time;
}mc_reference_manage_t;

typedef enum
{
    PID_NORM    = 0,
    PID_FZP     = 1,
}pid_mode_t;

typedef struct
{
    mc_reference_manage_t pid_org;

    float input;
    float output[2];
    int32_t out_limit;
    float err;
    float err_last;
    /*kp������*/
    float kp;
    float kp_out;
    /*ki������*/
    float ki;
    int32_t ki_out;
    int32_t ki_limit;
    /*kd������*/
    float kd;
    float kd_input[2];
    float kd_out[2];
    float kd_fc;
    uint16_t fp;        //����
    uint16_t fz;        //���
    float kd_num[2];
    float kd_den;
    int32_t kd_limit;
    float kd_lpf;

    uint16_t fs;        //����Ƶ��
    uint16_t lpf_fc;    //��ͨ�˲�����Ƶ��
    float lpf_k;        //��ͨ�˲�ϵ��

    pid_mode_t pid_mode;
    int8_t init;
}pid_ctl_t;

typedef struct
{
    mc_reference_manage_t manage;
    pid_ctl_t pid_ctl;
    filt2_t filt[5];
    uint8_t filt_num;

    float ref;
    float fb;
    float out;
    /* data */
}control_system_config_t;

extern float limit_f (float input, float up, float down);
extern void free_filter_fun(filt2_t *filt2, float input);
extern void pid_nrom_ctl(pid_ctl_t *pidctl, float reference, float fb);
extern void pid_zp_ctl (pid_ctl_t *pidctl, float reference, float fb);
extern void reference_managed_fun(mc_reference_manage_t *manage, float reference);
extern void control_sys_turning_loop(control_system_config_t *csc, float reference, float fb);

#ifdef __cplusplus
}
#endif

#endif
