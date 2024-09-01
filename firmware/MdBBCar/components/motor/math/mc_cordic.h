#ifndef __MC_CORDIC_H__
#define __MC_CORDIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"

typedef struct
{
    int16_t angle;
    int32_t sin;
    int32_t cos;
}sincos_t;

extern void mc_cordic (sincos_t *Sincos);
extern void Sincos_table_fun (sincos_t *Sincos);

#ifdef __cplusplus
}
#endif

#endif
