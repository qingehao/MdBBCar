#ifndef _LINK_PROTOCOL_X_H_
#define _LINK_PROTOCOL_X_H_
#include "stdint.h"
#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

typedef struct
{
    uint8_t sof;
    struct{
        uint16_t ver :2;
        uint16_t len :10;
        uint16_t resv :4;
    } ver_len;
    uint8_t head_crc;
} x_prehead_t;

typedef struct
{
    uint8_t sof;
    struct{
        uint16_t ver :2;
        uint16_t len :10;
        uint16_t resv :4;
    } ver_len;
    uint8_t head_crc;
    struct
    {
        uint8_t is_ack   :1;
        uint8_t need_ack :2;
        uint8_t is_enc   :1;
        uint8_t resv     :4;
    } cmd_type;
    uint16_t sender;
    uint16_t recvier;
    uint8_t  cmdSet;
    uint8_t  cmdId;
} x_head_t;

#pragma pack()

#ifdef __cplusplus
}
#endif

#endif /* _LINK_UNPACK_H_ */