
#include "mc_basic_def.h"
#include "mc_control_fun.h"

/****************************************************************
 * ????????????????????pid???????????
 * ???????????????
 ***************************************************************/


float limit_f (float input, float up, float down)
{
    if (input > up)
    {
        input = up;
    }
    else if (input < down)
    {
        input = down;
    }
    else
    {
        input = input;
    }
    return input;
}
/*
 * ??para?? [fc  fz  fp  bs  As]?
 * fc????????????????????fz?????????fp?????????
 * bs?????????Hz??As?????????db?
 * */

void free_filter_fun(filt2_t *filt2, float input)
{
	  static float notch_peak_k;
		float ts;
		float wid;
		float wn;
		float A;
    if (filt2->init != 0)
    {
        switch (filt2->filt_type)
        {
            case BUTTER_LP:
                break;
            case BUTTER_HP:
                break;
            case CHEBY_LP:
                break;
            case NOTCH_FILTER:/* constant-expression */
                /* code */
                ts = 1 / filt2->fs;
                wid = filt2->para[3] * 2 * PI;
                wn = 2 / ts * tanf(2 * PI * filt2->para[0] * ts / 2);
                A = pow(10, filt2->para[4] / 20);
                if (filt2->para[4] > 0)
                {
                    notch_peak_k = 1.12202;
                }
                else
                {
                    notch_peak_k = 0.89125;
                }
                float lamda_a = sqrt(2 * (1 - notch_peak_k * notch_peak_k))\
                            * (sqrt(wid*wid/wn/wn + 1) - 1) * (notch_peak_k * notch_peak_k - A * A);
                float lamda_b = lamda_a * A;
                filt2->den[0] = 4/(ts*ts) + 2*lamda_a * wn / ts + wn*wn;
                filt2->den[1] = (-8/(ts*ts) + 2*wn*wn) / filt2->den[0];
                filt2->den[2] = (4/(ts*ts) - 2*lamda_a * wn / ts + wn*wn) / filt2->den[0];

                filt2->num[0] = (4/(ts*ts) + 2*lamda_b * wn / ts + wn*wn) / filt2->den[0];
                filt2->num[1] = (-8/(ts*ts) + 2*wn*wn) / filt2->den[0];
                filt2->num[2] = (4/(ts*ts) - 2*lamda_b * wn / ts + wn*wn) / filt2->den[0];
                break;
            case PHASE_PEAK:/* constant-expression */
                ts = 1 / filt2->fs;
                A = pow(10, filt2->para[4] / 20);
                float fai;
                if(fai >= 0)
                {
                    fai = (fai - 180) / 180 * PI;
                }
                else
                {
                    fai = (fai + 180) / 180 * PI;
                }

                wn = 2 / ts * tanf(2 * PI * filt2->para[3] * ts / 2);
                float A_sqrt = sqrt(A);
                float wz = wn / sqrt(A_sqrt);
                float wp = wn / sqrt(A_sqrt);
                float mu = (wz - wp) * tanf(fai / 2) / wn / 2;

                filt2->den[0] = 1;
                filt2->den[1] = (2*wp*wp-8*filt2->fs*filt2->fs)/(4*filt2->fs*filt2->fs+4*mu*wp*filt2->fs+wp*wp);
                filt2->den[2] = (4*filt2->fs*filt2->fs-4*mu*wp*filt2->fs+wp*wp)/(4*filt2->fs*filt2->fs+4*mu*wp*filt2->fs+wp*wp);

                filt2->num[0] = (4*filt2->fs*filt2->fs+4*mu*wz*filt2->fs + wz*wz)\
                                /(4*filt2->fs*filt2->fs+4*mu*wp*filt2->fs+wp*wp)*wp*wp/wz/wz;
                filt2->num[1] = (2*wz*wz-8*filt2->fs*filt2->fs)/(4*filt2->fs*filt2->fs+4*mu*wp*filt2->fs+wp*wp)*wp*wp/wz/wz;
                filt2->num[2] = (4*filt2->fs*filt2->fs- 4*mu*wz*filt2->fs + wz*wz)\
                                /(4*filt2->fs*filt2->fs+4*mu*wp*filt2->fs+wp*wp)*wp*wp/wz/wz;
                break;
            case LEAD_LAG:
                ts = 1 / filt2->fs;
                float w = filt2->para[0] * 2 * PI;
                filt2->para[4] = pow(10, filt2->para[4] / 20);
                float a = sqrt(filt2->para[4] / (w*w));
                float b = sqrt(1 / filt2->para[4] / (w*w));

                filt2->den[0] = 2 / b + ts;
                filt2->num[0] = (2 / a + ts) / filt2->den[0];
                filt2->num[1] = (ts - 2 / a) / filt2->den[0];
                filt2->den[1] = (ts - 2 / b) / filt2->den[0];
                filt2->den[2] = 0;
                filt2->num[2] = 0;
                break;
            default:
                break;
        }
        filt2->init = 0;
    }
    filt2->output[2] = filt2->output[1];
    filt2->output[1] = filt2->output[0];
    filt2->input[2] = filt2->input[1];
    filt2->input[1] = filt2->input[0];
    filt2->input[0] = input;

    filt2->output[0] = -filt2->output[1] * filt2->den[1] - filt2->output[2] * filt2->den[2] + \
                            filt2->input[0] * filt2->num[0] + filt2->input[1] * filt2->num[1] + filt2->input[2] * filt2->num[2];
}
/******************************************
pid????
????????????
%        2 * (z - 1)
% s =  ----------------
%        Ts * (z + 1)
????????????,????????????
         (1/fz/2/pi)s+1
 out =  ------------------ * kd * err
         (1/fp/2/pi)s+1

 out * (1/(T*2*pi*fp) + 1) - out * (z^-1)*(1/(T*2*pi*fp)) =  err * (1/(T*2*pi*fz) + 1) - err * (z^-1)*((1/(T*2*pi*fz))
 out + out * (z^-1)*(Tz * 2 * pi * fp - 1) =  err / fz * fp + err * (z^-1)*(Tz * 2 * pi * fp - fp / fz)

 lpf     1 / (1/((2pi*fs)s + 1))
    out(1 + (z-1)/Tz * (1 / 2pi*fs)) = input
    out(1+ 1/T*(1 / 2pi*fs)) = out * (z^-1) * 1/T*(1 / 2pi*fs) + input

        1/T*(1 / 2pi*fs)
    -----------------------
      1 + 1/T*(1 / 2pi*fs)
*******************************************/
float kd_out_debug[10];
void pid_nrom_ctl(pid_ctl_t *pidctl, float reference, float fb)//?????????????10k
{
    float kd_out = 0;
//    float output = 0;
    if (pidctl->init != 0)
    {
        pidctl->kd_lpf = 2 * PI * pidctl->kd_fc / pidctl->fs;
        pidctl->lpf_k = 2 * PI * pidctl->lpf_fc / pidctl->fs;
        pidctl->init = 0;
    }
    pidctl->err_last = pidctl->err;
    pidctl->input = fb;
    pidctl->err = reference - pidctl->input;
    pidctl->kp_out = pidctl->kp * pidctl->err;//1024?????1.0
    pidctl->ki_out = pidctl->ki * pidctl->err + pidctl->ki_out;//1024?????1.0
    pidctl->ki_out = limit_f(pidctl->ki_out, pidctl->ki_limit, -pidctl->ki_limit);

    if(pidctl->kd != 0)
    {
        kd_out = (pidctl->err - pidctl->err_last) * pidctl->kd;//
        pidctl->kd_out[0] = kd_out * (1 - pidctl->kd_lpf) + pidctl->kd_out[1] * pidctl->kd_lpf;
        pidctl->kd_out[1] = pidctl->kd_out[0];
        pidctl->kd_out[1] = limit_f(pidctl->kd_out[1], pidctl->kd_limit, -pidctl->kd_limit);
    }
    else
    {
        pidctl->kd_out[1] = 0;
    }

    pidctl->output[0] = pidctl->kp_out + pidctl->ki_out + pidctl->kd_out[1];
    pidctl->output[1] = pidctl->output[0] * (1 - pidctl->lpf_k) + pidctl->output[1] * pidctl->lpf_k;
    pidctl->output[1] = limit_f(pidctl->output[1], pidctl->out_limit, -pidctl->out_limit);
}

void pid_zp_ctl(pid_ctl_t *pidctl, float reference, float fb)//?????????????10k
{
    if (pidctl->init == 1)
    {
        if(pidctl->fp != 0)
        {
            float wb = INV_2PI / (float)pidctl->fz;//INV_2PI * pidctl->fp;
            float wa = INV_2PI / (float)pidctl->fp;//INV_2PI * pidctl->fz;
            float a1 = (2*wa + 1.0f / (float)pidctl->fs) * pidctl->kd;
            float a2 = (-2*wa + 1.0f / (float)pidctl->fs) * pidctl->kd;
            float b1 = 2*wb + 1.0f / (float)pidctl->fs;
            float b2 = -2*wb + 1.0f / (float)pidctl->fs;
            /*????????????PID?��?kd??????��?1??��??????????????
             * ?????????????????��?????????????kp????????*/
            pidctl->kd_num[0] = a1/b1;// * pidctl->kd;
            pidctl->kd_num[1] = a2/b1;// * pidctl->kd;
            pidctl->kd_den = -b2/b1;// * pidctl->kd;
        }
        pidctl->lpf_k = 2 * PI * pidctl->lpf_fc / pidctl->fs;
        pidctl->init = 0;
    }
    pidctl->err_last = pidctl->err;
    pidctl->input = fb;
    pidctl->err = reference - pidctl->input;

    kd_out_debug[0] = pidctl->err - pidctl->err_last;
    kd_out_debug[1] = pidctl->err;

    pidctl->kp_out = pidctl->err;//1024?????1.0
    pidctl->ki_out = pidctl->ki * pidctl->err + pidctl->ki_out;//1024?????1.0
    pidctl->kd_input[1] = pidctl->kd_input[0];
    pidctl->kd_input[0] = pidctl->err; //(pidctl->err - pidctl->err_last) * pidctl->kd;//
    pidctl->kd_out[0] = pidctl->kd_input[0]*pidctl->kd_num[0] + pidctl->kd_input[1]*pidctl->kd_num[1] + pidctl->kd_out[1]*pidctl->kd_den;
    pidctl->kd_out[1] = pidctl->kd_out[0];

    kd_out_debug[2] = pidctl->kd_out[1];

    pidctl->ki_out = limit_f(pidctl->ki_out, pidctl->ki_limit, -pidctl->ki_limit);
    pidctl->kd_out[1] = limit_f(pidctl->kd_out[1], pidctl->kd_limit, -pidctl->kd_limit);

    pidctl->output[0] = (pidctl->kp_out + pidctl->ki_out + pidctl->kd_out[1]) * pidctl->kp;
    pidctl->output[1] = pidctl->output[0] * (1 - pidctl->lpf_k) + pidctl->output[1] * pidctl->lpf_k;
    pidctl->output[1] = limit_f(pidctl->output[1], pidctl->out_limit, -pidctl->out_limit);
}


void reference_managed_fun(mc_reference_manage_t *manage, float reference)
{
		float manage_error;
    switch (manage->mode)
    {
        case ORG_DIRECT:
            manage->managed_ref = reference;
            break;
        case ORG_STEP:
					  manage_error = reference - manage->managed_ref;
            if(manage_error > manage->step)
            {
                manage->managed_ref += manage->step;
            }
            else if(manage_error < manage->step)
            {
                manage->managed_ref -= manage->step;
            }
            else
            {
                manage->managed_ref = reference;
            }
        case LINEAR_STEP:
            manage_error = reference - manage->managed_ref;
            if(reference != manage->ref_last)
            {
                manage->step = manage_error / manage->time;
                manage->managed_ref += manage->step;
            }
            else
            {
                manage->managed_ref += manage->step;
            }
            manage->ref_last = reference;
        default:
            break;
    }
}

void control_sys_turning_loop(control_system_config_t *csc, float reference, float fb)
{
		float filt_ref;
    csc->ref = reference;
    csc->fb = fb;
    reference_managed_fun(&csc->manage, csc->ref);
    switch (csc->pid_ctl.pid_mode)
    {
        case PID_NORM:
            pid_zp_ctl(&csc->pid_ctl, csc->manage.managed_ref, fb);
            break;
        case PID_FZP:
            pid_zp_ctl(&csc->pid_ctl, csc->manage.managed_ref, fb);
            break;
        default:
            csc->pid_ctl.output[1] = csc->manage.managed_ref;
            break;
    }
    if(csc->filt_num != 0)
    {
        filt_ref = csc->pid_ctl.output[1];
        for (uint8_t i = 0; i < csc->filt_num; i++)
        {
            free_filter_fun(&csc->filt[i], filt_ref);
            filt_ref = csc->filt[i].output[0];
        }
        csc->out = filt_ref;
    }
    else
    {
        csc->out = csc->pid_ctl.output[1];
    }
}

