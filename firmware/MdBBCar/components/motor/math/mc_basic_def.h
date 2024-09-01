#ifndef __MC_BASIC_DEF_H__
#define __MC_BASIC_DEF_H__
#ifdef __cplusplus
extern "C" {
#endif

#define DATABASE                32767

#define COS_30_INT              28377
#define COS_45_INT              23170
#define COS_60_INT              16384
#define COS_72_INT              10126

#define SIN_30_INT              16384
#define SIN_45_INT              23170
#define SIN_60_INT              28377
#define SIN_72_INT              31163

#define COS_30_f                0.86602540378f
#define COS_45_f                0.70710678119f
#define COS_60_f                0.5f

#define SIN_30_f                0.5f
#define SIN_45_f                0.70710678119f
#define SIN_60_f                0.86602540378f

#define PI                  (3.1415926535898)
#define PI2                 (6.28318531f)
#define INV_PI              (0.3183098862f)
#define INV_2PI             (0.1591549431f)

#define W_RATIO             (0.10471975512f)

#define mc_sign(x)          (x > 0)?1:((x<0)?-1:0)
#define LPF(out, in, k)     in*k + out*(1-k)
#define OVER(x,a,b)         (x < a)?(x+b):((x > b)?(x-b):x)
#define LIMIT(x,a,b)        (x < a)?(a):((x > b)?(b):x)
#define ABS(x)              (x < 0)?(-x):(x)

typedef enum
{
    NONE    = 0,
    INIT    = 1,
    READY   = 2,
    START   = 3,
    END     = 4,
    DONE    = 5,
    NORM    = 6,
    FAIL    = 7,
    DEBUG   = 8,
}general_state_t;

#ifdef __cplusplus
}
#endif

#endif
