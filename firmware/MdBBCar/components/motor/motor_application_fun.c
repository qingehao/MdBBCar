#include "motor_application_fun.h"
#include "mc_bsp_config.h"
#include "mc_app_config.h"

#if(MOTOR_NUM == 1)
MCVector_t mcv_1 = {0};
MCVector_t *mcv[MOTOR_NUM];
mcv[0] = &mcv_1;
#endif

#if(MOTOR_NUM == 2)
MCVector_t mcv_1 = {0};
MCVector_t mcv_2 = {0};
MCVector_t *mcv[MOTOR_NUM] = {&mcv_1, &mcv_2};
#endif

#if(MOTOR_NUM == 3)
MCVector_t mcv_1 = {0};
MCVector_t mcv_2 = {0};
MCVector_t mcv_3 = {0};
MCVector_t *mcv[MOTOR_NUM];
mcv[0] = &mcv_1;
mcv[1] = &mcv_2;
mcv[2] = &mcv_3;
#endif


/*获取电机各类信号*/
void mc_get_signal_procession(MCVector_t *MCV)
{
    /*获取电机角度信号,速度信号*/
    switch (MCV->signal.angletype)
    {
        case HALL_ANGLE/* constant-expression */:
            break;
        case MT6825:
            MCV->signal.angle = MCV->signal.org_angle >> 3;
            MCV->signal.eleangle = MCV->signal.angle * MCV->foc.para.pole - MCV->eleangle_offset_cali.eangle_offset;
            MCV->foc.sincos.angle = (int16_t)MCV->signal.eleangle;
            MCV->signal.angle_delta = MCV->signal.org_angle - MCV->signal.angle_last;
            if(MCV->signal.angle_delta > 131072)
            {
                MCV->signal.angle_delta -= 262144;
            }
            else if(MCV->signal.angle_delta < -131072)
            {
                MCV->signal.angle_delta += 262144;
            }
            else{}
            MCV->signal.speed = MCV->signal.angle_delta * MC_CONTROL_FRE * 0.0002288818359f;
            MCV->signal.angle_last = MCV->signal.org_angle;
            break;
        case MT6835:
            MCV->signal.angle = MCV->signal.org_angle >> 6;
            MCV->signal.eleangle = MCV->signal.angle * MCV->foc.para.pole - MCV->eleangle_offset_cali.eangle_offset;
            MCV->foc.sincos.angle = (int16_t)MCV->signal.eleangle;
            MCV->signal.angle_delta = MCV->signal.org_angle - MCV->signal.angle_last;
            if(MCV->signal.angle_delta > 1048576)
            {
                MCV->signal.angle_delta -= 2097152;
            }
            else if(MCV->signal.angle_delta < -1048576)
            {
                MCV->signal.angle_delta += 2097152;
            }
            else{}
            MCV->signal.speed = MCV->signal.angle_delta * MC_CONTROL_FRE * 0.0000286102294922f;
            MCV->signal.angle_last = MCV->signal.org_angle;
            break;
        default:
            break;
    }
    MCV->signal.speedfilt = LPF(MCV->signal.speedfilt, MCV->signal.speed, MCV->signal.lpf_k);
    MCV->signal.w = MCV->signal.speedfilt * W_RATIO;
    /*获取电机电流*/
    MCV->foc.cur.a = MCV->signal.ia - MCV->signal.ia_offset;
    MCV->foc.cur.b = MCV->signal.ib - MCV->signal.ib_offset;
    MCV->foc.cur.c = MCV->signal.ic - MCV->signal.ic_offset;
}

void mc_closeloop_fun(MCVector_t *MCV)
{
    switch (MCV->control_type)
    {
        case CURRENT_MODE:
            MCV->torq_ctl.vs = MCV->signal.reference;
            MCV->torq_ctl.vs_ref = MCV->torq_ctl.vs + MCV->torq_ctl.vs_ext;
            break;
        case SPEED_MODE:
            control_sys_turning_loop(&MCV->spe_ctl, (float)MCV->signal.reference, MCV->signal.w);
            MCV->torq_ctl.vs = MCV->spe_ctl.out;
            MCV->torq_ctl.vs_ref = MCV->torq_ctl.vs + MCV->torq_ctl.vs_ext;
            break;
        case POSITION_MODE:
            control_sys_turning_loop(&MCV->pos_ctl, (float)MCV->signal.reference, MCV->signal.angle);
            MCV->torq_ctl.vs = MCV->pos_ctl.out;
            MCV->torq_ctl.vs_ref = MCV->torq_ctl.vs + MCV->torq_ctl.vs_ext;
            break;
        case SPEED_POSITION_MODE:
            control_sys_turning_loop(&MCV->pos_ctl, (float)MCV->signal.reference, MCV->signal.angle);
            control_sys_turning_loop(&MCV->spe_ctl, (float)MCV->pos_ctl.out, MCV->signal.w);
            MCV->torq_ctl.vs = MCV->spe_ctl.out;
            MCV->torq_ctl.vs_ref = MCV->torq_ctl.vs + MCV->torq_ctl.vs_ext;
            break;
        default:
            break;
    }

    switch (MCV->foc.mode)
    {
        case UQ_MODE/* constant-expression */:
            MCV->foc.uq_ref = MCV->torq_ctl.vs_ref;
            break;
        case UDQ_MODE:
            MCV->foc.vs_ref = MCV->torq_ctl.vs_ref;
            break;
        case UD_MODE:
            MCV->foc.ud_ref = MCV->torq_ctl.vs_ref;
            break;
        case IQ_MODE/* constant-expression */:
            MCV->foc.iq_ref = MCV->torq_ctl.vs_ref;
            break;
        case MTPA_MODE/* constant-expression */:
            MCV->foc.vs_ref = MCV->torq_ctl.vs_ref;
            break;
        case ID_MODE/* constant-expression */:
            MCV->foc.id_ref = MCV->torq_ctl.vs_ref;
            break;
        default:
            break;
    }
}


void mc_cur_sensor_cali(MCVector_t *MCV)
{
    MCV->cur_offset_cali.cnt++;
    if(MCV->cur_offset_cali.state == NONE)
    {
        MCV->cur_offset_cali.cnt = 0;
        MCV->cur_offset_cali.ia_record = 0;
        MCV->cur_offset_cali.ib_record = 0;
        MCV->cur_offset_cali.ic_record = 0;
        MCV->foc.pwm.pwm_mode = PWM_OFF;
        MCV->cur_offset_cali.state = START;
    }

    if(MCV->cur_offset_cali.state == START)
    {
        if(MCV->cur_offset_cali.cnt >= 1)
        {
            MCV->cur_offset_cali.ia_record += MCV->signal.adc_data.ia;
            MCV->cur_offset_cali.ib_record += MCV->signal.adc_data.ib;
            MCV->cur_offset_cali.ic_record += MCV->signal.adc_data.ic;
        }
        if(MCV->cur_offset_cali.cnt >= 1024)
        {
            MCV->signal.ia_offset = MCV->cur_offset_cali.ia_record >> 10;
            MCV->signal.ib_offset = MCV->cur_offset_cali.ib_record >> 10;
            MCV->signal.ic_offset = MCV->cur_offset_cali.ic_record >> 10;
        }
        MCV->cur_offset_cali.state = END;
    }

    if(MCV->cur_offset_cali.state == END)
    {
        MCV->foc.pwm.pwm_mode = PWM_ON;
        MCV->foc.cur_state = DONE;
        MCV->preparecmd = ELEANGLE_CALI;
        MCV->cur_offset_cali.state = DONE;
    }
}

void mc_eleangle_offset_cali(MCVector_t *MCV)
{
    MCV->eleangle_offset_cali.cnt++;
    if(MCV->eleangle_offset_cali.state == NONE)
    {
        if(MCV->general_flash.eleangle_offset_cali.state == 0x5a5a)
        {
            MCV->eleangle_offset_cali.eangle_offset = MCV->general_flash.eleangle_offset_cali.eangle_offset;
        }
        else
        {
            MCV->eleangle_offset_cali.state = INIT;
        }
    }

    if(MCV->eleangle_offset_cali.state == INIT)
    {
        MCV->eleangle_offset_cali.cnt = 0;
        MCV->foc_debug.debug_mode = UDQ_OP;
        MCV->foc_debug.ud = MCV->eleangle_offset_cali.vs;
        MCV->foc_debug.uq = 0;
        MCV->foc_debug.angle = 0;
        MCV->foc_debug.freq = 0;
        MCV->eleangle_offset_cali.state = START;
    }

    if(MCV->eleangle_offset_cali.state == START)
    {
        if(MCV->eleangle_offset_cali.cnt < MCV->eleangle_offset_cali.time)
        {
            return;
        }
        switch (MCV->signal.angletype)
        {
            case HALL_ANGLE:
                MCV->general_flash.eleangle_offset_cali.eangle_offset = (int16_t)(MCV->signal.angle);
                break;
            case MT6835:
            case MT6825:
                MCV->general_flash.eleangle_offset_cali.eangle_offset = (int16_t)(MCV->signal.angle * MCV->foc.para.pole);
                break;
            default:
                break;
        }
        MCV->eleangle_offset_cali.state = END;
    }

    if(MCV->eleangle_offset_cali.state == END)
    {
        MCV->foc_debug.debug_mode = DEBUG_NONE;
        MCV->foc_debug.ud = 0;
        MCV->foc_debug.uq = 0;
        MCV->foc_debug.angle = 0;
        MCV->foc_debug.freq = 0;

        MCV->general_flash.eleangle_offset_cali.state = 0x5a5a;
        MCV->general_flash.eleangle_offset_cali.write_bit = 1;
        MCV->eleangle_offset_cali.eangle_offset = MCV->general_flash.eleangle_offset_cali.eangle_offset;
        MCV->preparecmd = MECANGLE_CALI;
        MCV->eleangle_offset_cali.state = DONE;
    }
}

void mc_mecangle_offset_cali(MCVector_t *MCV)
{
    MCV->preparecmd = PREPARE_DONE;
}

void foc_debug_fun(MCVector_t *MCV)
{
    switch(MCV->foc_debug.debug_mode)
    {
        case UDQ_OP:
            MCV->foc_debug.angle += (float)MCV->foc_debug.freq * 32767.0f / MC_CONTROL_FRE;
            MCV->signal.eleangle = MCV->foc_debug.angle;
            MCV->foc.ud_ref = MCV->foc_debug.ud;
            MCV->foc.uq_ref = MCV->foc_debug.uq;
            MCV->foc.mode = UQ_MODE;
            break;
        case IDQ_OP:
            MCV->foc_debug.angle += (float)MCV->foc_debug.freq * 32767.0f / MC_CONTROL_FRE;
            MCV->signal.eleangle = MCV->foc_debug.angle;
            MCV->foc.id_ref = MCV->foc_debug.id;
            MCV->foc.iq_ref = MCV->foc_debug.iq;
            MCV->foc.mode = IQ_MODE;
            break;
        case UDQ_CL:
            MCV->foc.ud_ref = MCV->foc_debug.ud;
            MCV->foc.uq_ref = MCV->foc_debug.uq;
            MCV->foc.mode = UQ_MODE;
            break;
        case IDQ_CL:
            MCV->foc.id_ref = MCV->foc_debug.id;
            MCV->foc.iq_ref = MCV->foc_debug.iq;
            MCV->foc.mode = IQ_MODE;
            break;
        case UDQ_SIN:
            MCV->foc_debug.foc_debug_scos.angle = MCV->foc_debug.angle;
            Sincos_table_fun(&MCV->foc_debug.foc_debug_scos);
            MCV->foc.ud_ref = MCV->foc_debug.ud * MCV->foc_debug.foc_debug_scos.sin;
            MCV->foc.uq_ref = MCV->foc_debug.uq * MCV->foc_debug.foc_debug_scos.sin;
            MCV->foc.mode = UQ_MODE;
            break;
        case IDQ_SIN:
            MCV->foc_debug.foc_debug_scos.angle = MCV->foc_debug.angle;
            Sincos_table_fun(&MCV->foc_debug.foc_debug_scos);
            MCV->foc.id_ref = MCV->foc_debug.id * MCV->foc_debug.foc_debug_scos.sin;
            MCV->foc.iq_ref = MCV->foc_debug.iq * MCV->foc_debug.foc_debug_scos.sin;
            MCV->foc.mode = IQ_MODE;
            break;
        default:
            break;
    }
}

void mc_prepare_fun(MCVector_t *MCV)
{
    switch (MCV->preparecmd)
    {
        case VALID_CHECK:
        case CURRENT_CALI:
            mc_cur_sensor_cali(MCV);
            break;
        case ELEANGLE_CALI:
            mc_eleangle_offset_cali(MCV);
            break;
        case MECANGLE_CALI:
            mc_mecangle_offset_cali(MCV);
            break;
        default:
            MCV->mcstate = MC_NORAML;
            break;
    }
}

void mc_control_loop(uint8_t i)
{
    // 获取电机传感器参数
    get_motor_para_fun(&mcv[i]->signal, i);
    // 传感器信号处理
    mc_get_signal_procession(mcv[i]);
    // 电机闭环控制
    mc_closeloop_fun(mcv[i]);
    // foc输出
    foc_algorithm(&mcv[i]->foc);
    // 设置电机输出参数
    set_motor_output_para(&mcv[i]->foc, i);
}
