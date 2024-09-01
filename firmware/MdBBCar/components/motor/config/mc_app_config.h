#ifndef __MC_APP_CONFIG_H__
#define __MC_APP_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BSP_FOC中配置实际PWM频率
 *
 */
#define MC_PWM_FRE                  20000
#define MC_CONTROL_FRE              20000
#define MC_CPU_FRE                  280000000
#define MC_PWM_MODE                 2
#define MC_PWM_PERIOD               MC_CPU_FRE / MC_PWM_FRE / MC_PWM_MODE
#define MOTOR_NUM                   2
#define MC_ADC_CH_NUM               7

#ifdef __cplusplus
}
#endif

#endif
