#include "mc_sensorless_fun.h"

/* 在估计的 dq 轴系注入的方波电压信号,当高频电压 Vinj 注入到估计 d 轴，
 * 相当于方波电压被施加到电感的两侧，激励的 d 轴响应电流 idh 为三角波，
 * 其频率与注入电压频率相等，q 轴响应电流理论上为 0                 */

// void square_wave_injection(void)
