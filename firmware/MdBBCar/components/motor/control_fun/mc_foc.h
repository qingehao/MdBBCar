#ifndef __MC_FOC_H__
#define __MC_FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

// #include <stdio.h>
#include "mc_cordic.h"
#include "mc_app_config.h"
#include "mc_basic_def.h"
#include "stdio.h"
#include "stdint.h"
#include "mc_control_fun.h"


typedef struct
{
    int32_t alpha;
    int32_t beta;

    int32_t d_axis;
    int32_t q_axis;

    int32_t a;
    int32_t b;
    int32_t c;
}clark_park_t;

typedef enum
{
    PWM_OFF     = 0,
    PWM_ON      = 1,
    PWM_NORM    = 2,
    PWM_INJ     = 3,
}PWM_mode_t;

typedef struct
{
    PWM_mode_t pwm_mode;
    int32_t UAlpha; //���룬��ֹ����ϵAlpha�ᶨ�ӵ�ѹ
    int32_t UBeta;  //���룬��ֹ����ϵBeta�ᶨ�ӵ�ѹ
    int32_t Ua;   //
    int32_t Ub;   //
    int32_t Uc;   //
    int32_t taOn; //A��ʱ��
    int32_t tbOn; //B��ʱ��
    int32_t tcOn; //C��ʱ��
    uint16_t pwm_period;  // ����MC_PWM_PERIOD
    float   pwm_modulate_ratio;  // ����MC_PWM_PERIOD/32767
}pwm_module_t;

typedef enum
{
    UQ_MODE     = 0,
    IQ_MODE     = 1,
    MTPA_MODE   = 2,
    UDQ_MODE    = 3, // ��������ǰ������
    /*������Ҫ���ڲ���ģʽ*/
    UD_MODE     = 4,
    ID_MODE     = 5,
}foc_control_mode_t;

typedef struct
{
    uint8_t pole; // ������
    float   rs;
    float   ld;
    float   lq;
    float   flux;
}motor_para_t;

typedef struct
{
    general_state_t cur_state;
    foc_control_mode_t mode;
    motor_para_t para;
    clark_park_t vol;
    clark_park_t cur;

    clark_park_t inj_signal;
    int16_t      vs_ref;
    int16_t      id_ref;
    pid_ctl_t    id_ctl;
    int16_t      iq_ref;
    pid_ctl_t    iq_ctl;
    int16_t      ud_ref;
    int16_t      uq_ref;
    pwm_module_t pwm;
    sincos_t     sincos;
}mc_foc_t;

extern void Clark_Calc(clark_park_t *mc_clark);
extern void Park_Calc(clark_park_t *mc_park, sincos_t *mc_sincos);
extern void Anti_Park_Calc(clark_park_t *mc_park, sincos_t *mc_sincos);
extern void Svpwm_Module(pwm_module_t *pstrSvpwm);
extern void foc_algorithm(mc_foc_t *foc);

#ifdef __cplusplus
}
#endif

#endif
