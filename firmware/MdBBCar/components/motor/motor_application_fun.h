#ifndef __MOTOR_APPLICATION_FUN_H__
#define __MOTOR_APPLICATION_FUN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_proj_config.h"
#include "mc_basic_def.h"

typedef enum
{
    CURRENT_MODE        = 0,
    SPEED_MODE          = 1,
    POSITION_MODE       = 2,
    SPEED_POSITION_MODE = 3,
}mc_control_t;

typedef enum
{
    MC_INIT                = 0,
    MC_PREPARE             = 1,
    MC_NORAML              = 2,
    MC_DEBUG               = 3,
    MC_ERROR               = 4,
}mcstate_t;

typedef enum
{
    VALID_CHECK     = 0,
    CURRENT_CALI    = 1,
    ELEANGLE_CALI   = 2,
    MECANGLE_CALI   = 3,
    PREPARE_DONE    = 4,
}mc_prepare_t;

typedef struct
{
    int16_t ia;
    int16_t ib;
    int16_t ic;
    int16_t ibus;
    int16_t vbus;
    int16_t v_np;  //电机三相中性点
}mc_adc_data_t;

typedef enum
{
    HALL_ANGLE        = 0,
    MT6825            = 1,
    MT6835            = 2,
    SENSORLESS        = 3,
}mc_angletype_t;


typedef struct
{
    /*电机电流信息*/
    mc_adc_data_t adc_data;
    int16_t ia;
    int16_t ib;
    int16_t ic;
    int16_t ia_offset;
    int16_t ib_offset;
    int16_t ic_offset;

    /*电机角度信息*/
    mc_angletype_t angletype;
    int32_t org_angle;
    float   angle;
    int32_t angle_last;
    int32_t angle_sum;
    int32_t angle_delta;
    int16_t eleangle;

    float speed;    // 电机速度，单位rpm
    float speedfilt;
    float lpf_k;
    float w;        // 电机速度，单位弧度

    int16_t reference; // 电机参考输入信号
}mc_signal_t;

typedef struct
{
    int16_t vs_ref;
		int16_t vs;
    int16_t ud;
    int16_t uq;
    int16_t vs_ext;
    int16_t id;
    int16_t iq;
}mc_torquectl_t;

typedef struct
{
    pid_ctl_t pid_ctl;

    float speed;
    float speed_filt;
    float lpf_k;  // lpf_k = (fs - 2pi*fc)/(fs + 2pi*fc)

    float out; // 控制最终输出
}mc_speedctl_t;

typedef struct
{
    pid_ctl_t pid_ctl;

    float pos;
    float pos_filt;
    float lpf_k;  // lpf_k = (fs - 2pi*fc)/(fs + 2pi*fc)

    float out; // 控制最终输出
}mc_positctl_t;

typedef enum
{
    DEBUG_NONE  = 0,
    UDQ_OP      = 1,
    IDQ_OP      = 2,
    UDQ_CL      = 3,
    IDQ_CL      = 4,
    UDQ_SIN     = 5,
    IDQ_SIN     = 6,
}foc_debug_mode_t;

typedef struct
{
    foc_debug_mode_t debug_mode;
    int16_t ud;
    int16_t uq;
    int16_t id;
    int16_t iq;
    float angle;
    int16_t freq;
    sincos_t foc_debug_scos;
}foc_debug_t;


typedef struct
{
    general_state_t state;
    int16_t ia_offset;
    int16_t ib_offset;
    int16_t ic_offset;
    int32_t ia_record;
    int32_t ib_record;
    int32_t ic_record;
    uint32_t cnt;
}mc_cur_offset_cali_t;

typedef struct
{
    general_state_t state;
    int16_t eangle_offset;
    int16_t vs;
    uint32_t cnt;
    uint32_t time;
}mc_eleangle_offset_cali_t;


typedef struct
{
    uint16_t state;
    int16_t eangle_offset;
		uint8_t write_bit;
}eleangle_offset_cali_t;

typedef struct
{
    eleangle_offset_cali_t eleangle_offset_cali;
    uint8_t write_bit;
}general_flash_t;

typedef struct
{
    uint8_t             motorid;
    mcstate_t           mcstate;
    mc_prepare_t        preparecmd;
    mc_control_t        control_type;
    mc_signal_t         signal;

    mc_foc_t            foc;
    foc_debug_t    			foc_debug;
    mc_torquectl_t      torq_ctl;
    control_system_config_t pos_ctl;
    control_system_config_t spe_ctl;

    mc_eleangle_offset_cali_t   eleangle_offset_cali;
    mc_cur_offset_cali_t        cur_offset_cali;
    general_flash_t             general_flash;
}MCVector_t;

extern void mc_get_signal_procession(MCVector_t *MCV);
extern void mc_closeloop_fun(MCVector_t *MCV);
extern void mc_cur_sensor_cali(MCVector_t *MCV);
extern void mc_eleangle_offset_cali(MCVector_t *MCV);
extern void mc_mecangle_offset_cali(MCVector_t *MCV);
extern void foc_debug_fun(MCVector_t *MCV);
extern void mc_prepare_fun(MCVector_t *MCV);
extern void mc_control_loop(uint8_t i);

#ifdef __cplusplus
}
#endif

#endif
