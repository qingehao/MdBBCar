#include "mc_cordic.h"

/*
在控制系统中这是最底层的数学库层
一共分为三层
电机应用层
电机控制器层
电机数学库层
*/

const float sintable[32] = {0.0f,              1607.80048468651f, 3211.72763717871f, 4807.91745647884f, 6392.52458150248f, 7961.73155489025f,\
                            9511.75801959697f,  11038.8698261029f, 12539.3880283069f, 14009.6977464301f, 15446.2568755775f, 16845.6046189793f,\
                            18204.3698253533f,  19519.2791103036f, 20787.1647421902f, 22004.9722734723f, 23169.7678991396f, 24278.7455245059f,\
                            25329.2335253368f,  26318.7011840263f, 27244.7647863175f, 28105.1933638789f, 28897.9140689025f, 29621.0171677759f,\
                            30272.7606417973f,  30851.5743838520f, 31356.0639809373f, 31785.0140734256f, 32137.3912829727f, 32412.3467020160f,\
                            32609.2179388679f, 32727.5307134749};
// {0.0f,         402.101495f,   804.142435f,  1206.062274f,  1607.800485f,  2009.296566f,  2410.490055f,  2811.320534f, \
//                             3211.727637f,  3611.651066f,  4011.03059f,  4409.806076f,  4807.917456f,  5205.304782f,  5601.908209f,  5997.668009f, \
//                             6392.524582f,  6786.418464f,  7179.29034f,  7571.081034f,  7961.731555f,  8351.183068f,  8739.376924f,  9126.254662f, \
//                             9511.758020f,  9895.828941f,  10278.4096f,  10659.44234f,  11038.86983f,  11416.63490f,  11792.68066f,  12166.95049f, \
//                             12539.38803f,  12909.93718f,  13278.5421f,  13645.14740f,  14009.69775f,  14372.13829f,  14732.41444f,  15090.47194f, \
//                             15446.25688f,  15799.71566f,  16150.7951f,  16499.44222f,  16845.60462f,  17189.23013f,  17530.26701f,  17868.66389f, \
//                             18204.36983f,  18537.33424f,  18867.5070f,  19194.83840f,  19519.27911f,  19840.78029f,  20159.29353f,  20474.77085f, \
//                             20787.16474f,  21096.42817f,  21402.5145f,  21705.37778f,  22004.97227f,  22301.25289f,  22594.17503f,  22883.69456f, \
//                             23169.76790f,  23452.35195f,  23731.4042f,  24006.88252f,  24278.74552f,  24546.95224f,  24811.46227f,  25072.23579f, \
//                             25329.23353f,  25582.41677f,  25831.7474f,  26077.18785f,  26318.70118f,  26556.25101f,  26789.80157f,  27019.31768f, \
//                             27244.76478f,  27466.10892f,  27683.3168f,  27896.35561f,  28105.19336f,  28309.79858f,  28510.14044f,  28706.18878f, \
//                             28897.91407f,  29085.28744f,  29268.2807f,  29446.86622f,  29621.01717f,  29790.70730f,  29955.91107f,  30116.60358f, \
//                             30272.76064f,  30424.35874f,  30571.3750f,  30713.78740f,  30851.57438f,  30984.71523f,  31113.18989f,  31236.97902f, \
//                             31356.06398f,  31470.42683f,  31580.0503f,  31684.91803f,  31785.01407f,  31880.32341f,  31970.83169f,  32056.52529f, \
//                             32137.39128f,  32213.41751f,  32284.5925f,  32350.90557f,  32412.34670f,  32468.90665f,  32520.57691f,  32567.34969f, \
//                             32609.21794f,  32646.17536f,  32678.2164f,  32705.33621f,  32727.53071f,  32744.79658f,  32757.13119f,  32764.53271};
const float sin_div[32]  = {6.28047064330666f, 6.26534043942268f, 6.23511648164112f, 6.18987158212358f, 6.12971473979599f, 6.05479087776063f,\
                            5.96528049416370f, 5.86139922735946f, 5.74339733641858f, 5.61155909823208f, 5.46620212266340f, 5.30767658739848f,\
                            5.13636439433695f, 4.95267824955707f, 4.75706066907061f, 4.54998291276310f, 4.33194384908728f, 4.10346875324552f,\
                            3.86510804175582f, 3.61743594645004f, 3.36104913109928f, 3.09656525399856f, 2.82462147997390f, 2.54587294539635f,\
                            2.26099117990121f, 1.97066248861425f, 1.67558629878255f, 1.37647347479307f, 1.07404460563801f, 0.76902826895271f,\
                            0.462159275808631f, 0.154176900488736};
// {1.57070896487572f,  1.57047242188313f,  1.56999937152046f,  1.56928988502735f,  1.56834406924985f,  1.56716206662435f,  1.56574405515609f,  1.56409024839238f,  1.56220089539044f, \
//                             1.56007628067989f,  1.55771672421987f,  1.55512258135092f,  1.55229424274141f,  1.54923213432872f,  1.54593671725513f,  1.54240848779832f,  1.53864797729668f,  1.53465575206925f, \
//                             1.53043241333046f,  1.52597859709960f,  1.52129497410501f,  1.51638224968307f,  1.51124116367203f,  1.50587249030053f,  1.50027703807101f,  1.49445564963801f,  1.48840920168121f, \
//                             1.48213860477346f,  1.47564480324355f,  1.46892877503411f,  1.46199153155431f,  1.45483411752750f,  1.44745761083385f,  1.43986312234820f,  1.43205179577256f,  1.42402480746397f, \
//                             1.41578336625736f,  1.40732871328342f,  1.39866212178187f,  1.38978489690943f,  1.38069837554355f,  1.37140392608094f,  1.36190294823147f,  1.35219687280744f,  1.34228716150820f, \
//                             1.33217530669968f,  1.32186283119017f,  1.31135128800042f,  1.30064226013026f,  1.28973736031988f,  1.27863823080700f,  1.26734654307981f,  1.25586399762489f,  1.24419232367134f, \
//                             1.23233327893037f,  1.22028864933047f,  1.20806024874852f,  1.19564991873675f,  1.18305952824514f,  1.17029097334020f,  1.15734617691938f,  1.14422708842133f,  1.13093568353260f, \
//                             1.11747396388979f,  1.10384395677845f,  1.09004771482746f,  1.07608731570014f,  1.06196486178123f,  1.04768247986033f,  1.03324232081168f,  1.01864655927005f,  1.00389739330346f, \
//                             0.988997044082097f, 0.973947755543705f, 0.958751794055786f, 0.943411448074230f, 0.927929027798768f, 0.912306864824885f, 0.896547311792887f, 0.880652742033504f, 0.864625549210444f, \
//                             0.848468146960073f, 0.832182968527619f, 0.815772466401143f, 0.799239111941859f, 0.782585395012163f, 0.765813823600553f, 0.748926923443989f, 0.731927237647525f, 0.714817326301315f, \
//                             0.697599766095053f, 0.680277149930006f, 0.662852086528446f, 0.645327200040782f, 0.627705129650522f, 0.609988529176604f, 0.592180066673919f, 0.574282424031367f, 0.556298296568187f, \
//                             0.538230392627739f, 0.520081433169906f, 0.501854151361300f, 0.483551292163412f, 0.465175611919634f, 0.446729877939660f, 0.428216868083325f, 0.409639370341765f, 0.391000182417798f, \
//                             0.372302111304549f, 0.353547972862799f, 0.334740591396781f, 0.315882799228945f, 0.296977436273494f, 0.278027349608479f, 0.259035393047313f, 0.240004426708722f, 0.220937316586273f, \
//                             0.201836934116656f, 0.182706155747155f, 0.163547862502625f, 0.144364939551608f, 0.125160275771705f, 0.105936763314631f, 0.0866972971706872f, 0.0674447747326781f, 0.0481820953596639f, \
//                             0.028912159940333f, 0.009637870456061f};


const float tantable[128];


const float cordic_angle[18] = {4096, 2418.01, 1277.61, 648.5348, 325.5259, 162.9216,81.4807, 40.742836,\
                                 20.3717291, 10.1859034, 5.09295656, 2.546478887,1.273239519,0.6366197692,\
                                 0.3183098858,0.159154943,0.07957747154,0.03978873577};

void mc_cordic (sincos_t *Sincos)
{
    int8_t i;
    int8_t sector;
    float angle;
    int32_t cos_curr;
    int32_t sin_curr;
    /*单位模长设为65536，39797=65536/1.64676，
    * 后续可以考虑更大的模长，这样精度更高  */
    int32_t cos_last = 40752061;//159188;
    int32_t sin_last = 0;

    if (Sincos->angle < 0)
    {
        angle = Sincos->angle + 32768;
    }
    else
    {
        angle = Sincos->angle;
    }

    if ((angle >= 0) && (angle <= 8192))
    {
        sector = 1;
    }
    else if ((angle > 8192) && (angle <= 16384))
    {
        sector = 2;
        angle = 16384 - angle;
    }
    else if ((angle > 16384) && (angle <= 24576))
    {
        sector = 3;
        angle = angle - 16384;
    }
    else
    {
        sector = 4;
        angle = 32768 - angle;
    }

    for (i = 0; i < 12; i++)
    {

        if (angle < 0)
        {
            cos_curr = cos_last + (sin_last >> (i));
            sin_curr = sin_last - (cos_last >> (i));
            cos_last = cos_curr;
            sin_last = sin_curr;
            angle = angle + cordic_angle[i];
        }
        else
        {
            cos_curr = cos_last - (sin_last >> (i));
            sin_curr = sin_last + (cos_last >> (i));
            cos_last = cos_curr;
            sin_last = sin_curr;
            angle = angle - cordic_angle[i];
        }
    }

    switch (sector)
    {
        case 1:
            Sincos->cos = cos_last >> 11;
            Sincos->sin = sin_last >> 11;
            break;
        case 2:
            Sincos->cos = -cos_last >> 11;
            Sincos->sin = sin_last >> 11;
            break;
        case 3:
            Sincos->cos = -cos_last >> 11;
            Sincos->sin = -sin_last >> 11;
            break;
        case 4:
            Sincos->cos = cos_last >> 11;
            Sincos->sin = sin_last >> 11;
            break;
        default:
            Sincos->cos = 0;
            Sincos->sin = 0;
            break;
    }
    // Sincos->cos = Sincos->cos / 2048.0f;
    // Sincos->sin = Sincos->sin / 2048.0f;
}

void Sincos_table_fun(sincos_t *Sincos)
{
    uint8_t decimals;
    uint8_t mc_sin_index;
    uint8_t mc_cos_index;
    int16_t mc_angle;
    float mc_sin;
    float mc_cos;

    if(Sincos->angle < 0)
    {
        mc_angle = Sincos->angle + 32767;
    }
    else
    {
        mc_angle = Sincos->angle;
    }

    if ((mc_angle >= 0) && (mc_angle <= 8192))
    {
        mc_sin_index = mc_angle >> 8;
        mc_cos_index = (8192 - mc_angle) >> 8;
        decimals = (uint8_t)mc_angle;
        mc_sin = sintable[mc_sin_index] + sin_div[mc_sin_index] * decimals;
        mc_cos = sintable[mc_cos_index] + sin_div[mc_cos_index] * (256 - decimals);
        Sincos->sin = mc_sin;
        Sincos->cos = mc_cos;
    }
    else if ((mc_angle > 8192) && (mc_angle <= 16384))
    {
        mc_angle = 16384 - mc_angle;
        mc_sin_index = mc_angle >> 8;
        mc_cos_index = (8192 - mc_angle) >> 8;
        decimals = (uint8_t)mc_angle;
        mc_sin = sintable[mc_sin_index] + sin_div[mc_sin_index] * decimals;
        mc_cos = -sintable[mc_cos_index] - sin_div[mc_cos_index] * (256 - decimals);
        Sincos->sin = mc_sin;
        Sincos->cos = mc_cos;
    }
    else if ((mc_angle > 16384) && (mc_angle <= 24576))
    {
        mc_angle = mc_angle - 16384;
        mc_sin_index = mc_angle >> 8;
        mc_cos_index = (8192 - mc_angle) >> 8;
        decimals = (uint8_t)mc_angle;
        mc_sin = -sintable[mc_sin_index] - sin_div[mc_sin_index] * decimals;
        mc_cos = -sintable[mc_cos_index] - sin_div[mc_cos_index] * (256 - decimals);
        Sincos->sin = mc_sin;
        Sincos->cos = mc_cos;
    }
    else
    {
        mc_angle = 32768 - mc_angle;
        mc_sin_index = mc_angle >> 8;
        mc_cos_index = (8192 - mc_angle) >> 8;
        decimals = (uint8_t)mc_angle;
        mc_sin = -sintable[mc_sin_index] - sin_div[mc_sin_index] * decimals;
        mc_cos = sintable[mc_cos_index] + sin_div[mc_cos_index] * (256 - decimals);
        Sincos->sin = mc_sin;
        Sincos->cos = mc_cos;
    }
}
