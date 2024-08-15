#ifndef _LINK_H_
#define _LINK_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "lwrb.h"
#include "link_unpack.h"

typedef struct
{
    int32_t (*open)(void *arg);
    int32_t (*read)(uint8_t *buf, uint32_t size);
    int32_t (*write)(uint8_t *buf, uint32_t size);
    int32_t (*ioctl)(uint8_t cmd, void *arg);
} link_ops_t;

typedef int32_t (*link_send_t)(uint8_t *buf, uint32_t size);

typedef struct
{
    uint8_t     index;
    uint8_t    *line_buf;
    uint32_t    line_size;
    lwrb_t      comm_rb;
    link_send_t send;
    uint8_t     is_ready;
    link_unpack_sta_t unpack_sta;
} link_obj_t;

link_obj_t *link_get_obj(uint8_t index);
uint8_t link_get_obj_num(void);

int32_t link_recv_data(uint8_t index, uint8_t *data, uint32_t len);
int32_t link_add(uint8_t *rb_buf, uint32_t rb_size,
                uint8_t *line_buf, uint32_t line_buf_size,
                link_send_t pSend);

#ifdef __cplusplus
}
#endif

#endif /* _LINK_H_ */
