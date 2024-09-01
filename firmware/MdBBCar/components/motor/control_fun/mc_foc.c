#include "mc_foc.h"
#include "mc_basic_def.h"

void Clark_Calc(clark_park_t *mc_clark)
{
    mc_clark->alpha = mc_clark->a;
    mc_clark->beta  = mc_clark->a + (mc_clark->b << 1);
    mc_clark->beta = 0.57735f * mc_clark->beta;
}

void Park_Calc(clark_park_t *mc_park, sincos_t *mc_sincos)
{
    // mc_cordic(&mc_park->sincos);
    mc_park->d_axis = mc_park->alpha * mc_sincos->cos + mc_park->beta * mc_sincos->sin;
    mc_park->q_axis = -mc_park->alpha * mc_sincos->sin + mc_park->beta * mc_sincos->cos;
    mc_park->d_axis = mc_park->d_axis >> 15;
    mc_park->q_axis = mc_park->q_axis >> 15;
}

void Anti_Park_Calc(clark_park_t *clark_park, sincos_t *mc_sincos)
{
    // mc_cordic(&clark_park->sincos);
    clark_park->alpha = clark_park->d_axis * mc_sincos->cos - clark_park->q_axis * mc_sincos->sin;
    clark_park->beta = clark_park->d_axis * mc_sincos->sin + clark_park->q_axis * mc_sincos->cos;
    clark_park->alpha = clark_park->alpha >> 15;
    clark_park->beta = clark_park->beta >> 15;
}

void Svpwm_Module(pwm_module_t *pstrSvpwm)
{
    uint8_t Sector=0;
    int t1,t2;

    pstrSvpwm->Ua = pstrSvpwm->UBeta;
    float mc_data1 = SIN_60_f * (float)pstrSvpwm->UAlpha;
    int32_t mc_data2_int = pstrSvpwm->UBeta >> 1;
    pstrSvpwm->Ub = -(int32_t)mc_data1 - mc_data2_int;
    pstrSvpwm->Uc = (int32_t)mc_data1- mc_data2_int;
    if(pstrSvpwm->Ua > 0)
    {
        Sector += 1;
    }
    if(pstrSvpwm->Ub > 0)
    {
        Sector += 2;
    }
    if(pstrSvpwm->Uc > 0)
    {
        Sector += 4;
    }

    /*        婀�式     X  Y  Z     Ts为Timer1_Period,Udc为MOTOR_POWER*/
    pstrSvpwm->Ua = pstrSvpwm->Ua * pstrSvpwm->pwm_modulate_ratio;//X=sqrt(3)*beta*Ts/Udc
    pstrSvpwm->Ub = pstrSvpwm->Ub * pstrSvpwm->pwm_modulate_ratio;//Y=(sqrt(3)/2*beta+3/2*alpha)*Ts/Udc
    pstrSvpwm->Uc = pstrSvpwm->Uc * pstrSvpwm->pwm_modulate_ratio;//Z=(sqrt(3)/2*beta-3/2*alpha)*Ts/Udc
    /*     SVPWM占 毡  */
    switch(Sector)
    {
        case 0:
            pstrSvpwm->taOn = pstrSvpwm->pwm_period / 2;
            pstrSvpwm->tbOn = pstrSvpwm->pwm_period / 2;
            pstrSvpwm->tcOn = pstrSvpwm->pwm_period / 2;
            break;
        case 1:
            t1 = -pstrSvpwm->Ub;
            t2 = -pstrSvpwm->Uc;
            if(pstrSvpwm->pwm_period < (t1+t2))
            {
                t1 = t1*pstrSvpwm->pwm_period / (t1+t2);
                t2 = t2*pstrSvpwm->pwm_period / (t1+t2);
            }
            pstrSvpwm->tcOn = (pstrSvpwm->pwm_period- t1 - t2)>>1;          //Tbon = (1-t1-t2)/
            pstrSvpwm->taOn = pstrSvpwm->tcOn + t1;     //Taon = Tbon + t1/2
            pstrSvpwm->tbOn = pstrSvpwm->taOn + t2;     //Tcon = Taon + t2/2
            break; //2鍙锋墖鍖�
        case 2:
            t1 = -pstrSvpwm->Uc;//Ut4
            t2 = -pstrSvpwm->Ua;//Ut5
            if(pstrSvpwm->pwm_period<(t1+t2))
            {
                t1 = t1*pstrSvpwm->pwm_period / (t1+t2);
                t2 = t2*pstrSvpwm->pwm_period / (t1+t2);
            }
            pstrSvpwm->taOn = (pstrSvpwm->pwm_period - t1 - t2)>>1;
            pstrSvpwm->tbOn = pstrSvpwm->taOn + t1;              //Tcon = Taon + t1/2
            pstrSvpwm->tcOn = pstrSvpwm->tbOn + t2;              //Tbon = Tcon + t2/2
            break;//6鍙锋墖鍖�
        case 3:
            t1 = pstrSvpwm->Ub;//Ut4
            t2 = pstrSvpwm->Ua;//Ut6
            if(pstrSvpwm->pwm_period < (t1+t2))
            {
                t1 = t1*pstrSvpwm->pwm_period / (t1+t2);
                t2 = t2*pstrSvpwm->pwm_period / (t1+t2);
            }
            pstrSvpwm->taOn = (pstrSvpwm->pwm_period - t1 - t2)>>1;//Taon = (1-t1-t2)/4
            pstrSvpwm->tcOn = pstrSvpwm->taOn + t1;              //Tbon = Taon + t1/2
            pstrSvpwm->tbOn = pstrSvpwm->tcOn + t2;              //Tcon = Tbon + t2/2
            break;//1鍙锋墖鍖�
        case 4:
            t1 = -pstrSvpwm->Ua;//Ut1
            t2 = -pstrSvpwm->Ub;//Ut3
            if(pstrSvpwm->pwm_period < (t1+t2))
            {
                t1 = t1*pstrSvpwm->pwm_period / (t1+t2);
                t2 = t2*pstrSvpwm->pwm_period / (t1+t2);
            }
            pstrSvpwm->tbOn = (pstrSvpwm->pwm_period - t1 - t2)>>1;             //Tcon = (1-t1-t2)/4
            pstrSvpwm->tcOn = pstrSvpwm->tbOn + t1;         //Tbon = Tcon + t1/2
            pstrSvpwm->taOn = pstrSvpwm->tcOn + t2;         //Taon = Tbon + t2/2
            break;//4鍙锋墖鍖�
        case 5:
            t1 = pstrSvpwm->Ua;//Ut2
            t2 = pstrSvpwm->Uc;//Ut3
            if(pstrSvpwm->pwm_period < (t1+t2))//路闃叉�㈠彂鐢熻繃璋冩暣瀵艰嚧鍦嗗舰鐢靛帇鐭㈤噺澶辩湡锛屾墍浠ラ噰鍙栨瘮鍒楃缉灏�
            {
                t1 = t1*pstrSvpwm->pwm_period / (t1+t2);
                t2 = t2*pstrSvpwm->pwm_period / (t1+t2);
            }
            pstrSvpwm->tcOn = (pstrSvpwm->pwm_period - t1 - t2)>>1;//Tbon = (1-t1-t2)/4
            pstrSvpwm->tbOn = pstrSvpwm->tcOn + t1;              //Tcon = Tbon + t1/2
            pstrSvpwm->taOn = pstrSvpwm->tbOn + t2;              //Taon = Tcon + t2/2
            break;//3鍙锋墖鍖�
        case 6:
            t1 = pstrSvpwm->Uc;//Ut1
            t2 = pstrSvpwm->Ub;//Ut5
            if(pstrSvpwm->pwm_period < (t1+t2))
            {
                t1 = t1*pstrSvpwm->pwm_period / (t1+t2);
                t2 = t2*pstrSvpwm->pwm_period / (t1+t2);
            }
            pstrSvpwm->tbOn = (pstrSvpwm->pwm_period - t1 - t2)>>1;//Tcon = (1-t1-t2)/4
            pstrSvpwm->taOn = pstrSvpwm->tbOn + t1;              //Taon = Tcon + t1/2
            pstrSvpwm->tcOn = pstrSvpwm->taOn + t2;              //Tbon = Taon + t2/2
            break;//5鍙锋墖鍖�
        default:
            break;
    }
}

void foc_algorithm(mc_foc_t *foc)
{
    /*鐢垫満clark锛宲ark鍙樺寲锛岃幏寰梔q杞寸數娴�*/
    Clark_Calc(&foc->cur);
    Sincos_table_fun(&foc->sincos);
    Park_Calc(&foc->cur, &foc->sincos);

    /*鍙傛暟杈ㄨ瘑绠楁硶*/

    /*鏃犳劅绠楁硶*/

    switch (foc->mode)
    {
        case UQ_MODE/* constant-expression */:
            foc->vol.q_axis = foc->uq_ref;
            foc->vol.d_axis = 0;
            break;
        case UDQ_MODE:
            foc->vol.q_axis = foc->uq_ref;
            foc->vol.d_axis = foc->uq_ref;
            break;
        case UD_MODE:
            foc->vol.q_axis = 0;
            foc->vol.d_axis = foc->uq_ref;
            break;
        case IQ_MODE:/* constant-expression */
            if(foc->cur_state == DONE)
            {
                pid_nrom_ctl(&foc->id_ctl, 0, foc->cur.d_axis);
                pid_nrom_ctl(&foc->iq_ctl, foc->iq_ref, foc->cur.q_axis);
                foc->vol.d_axis = foc->id_ctl.output[1];
                foc->vol.q_axis = foc->iq_ctl.output[1];
            }
            break;
        case MTPA_MODE:/* MTPA妯″紡锛屾煡琛� */
            if(foc->cur_state == DONE)
            {
                pid_nrom_ctl(&foc->id_ctl, foc->id_ref, foc->cur.d_axis);
                pid_nrom_ctl(&foc->iq_ctl, foc->iq_ref, foc->cur.q_axis);
                foc->vol.d_axis = foc->id_ctl.output[1];
                foc->vol.q_axis = foc->iq_ctl.output[1];
            }
            break;
        case ID_MODE:/* constant-expression */
            if(foc->cur_state == DONE)
            {
                pid_nrom_ctl(&foc->id_ctl, foc->id_ref, foc->cur.d_axis);
                pid_nrom_ctl(&foc->iq_ctl, 0, foc->cur.q_axis);
                foc->vol.d_axis = foc->id_ctl.output[1];
                foc->vol.q_axis = foc->iq_ctl.output[1];
            }
            break;
        default:
            break;
    }

    /*璋愭尝鐢垫祦鎶戝埗*/

    /*鍙峱ark鍙樺寲锛孲VPWM*/
    Anti_Park_Calc(&foc->vol, &foc->sincos);
    foc->pwm.UAlpha = foc->vol.alpha;
    foc->pwm.UBeta = foc->vol.beta;
    Svpwm_Module(&foc->pwm);

    /*閫嗗彉鍣ㄦ�诲尯琛ュ伩*/
}
