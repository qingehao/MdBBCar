#ifndef _LINK_UNPACK_H_
#define _LINK_UNPACK_H_
#include "stdint.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "protocol/link_protocol_x.h"

typedef struct
{
    uint8_t cmdSet;
    uint8_t cmdId;
    void (*req_cb)(x_head_t *head, uint8_t *body, void *arg);
    void (*ack_cb)(x_head_t *head, uint8_t *body, void *arg);
    void *arg;
} comm_cb_desc_t;

typedef enum
{
    UNPACK_STEP_SEARCH_SOF = 0,
    UNPACK_STEP_CHECK_HEAD,
    UNPACK_STEP_FETCH_DATA,
    UNPACK_STEP_EXEC_CBF,
} unpack_step_e;

typedef struct
{
    uint32_t line_buf_write_index;
    uint32_t expected_size;
    unpack_step_e step;
    uint32_t timeout;
} link_unpack_sta_t;

void link_unpack_thread(void *arg);

#define LINK_ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#ifdef __cplusplus
}
#endif

#endif /* _LINK_UNPACK_H_ */