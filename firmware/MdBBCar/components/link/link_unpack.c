#include "link_unpack.h"
#include "link.h"
#include "protocol/link_protocol_x.h"
#include "lwrb.h"
#include "rtthread.h"
#define LOG_TAG "link.unpack"
#define LOG_LVL LOG_LVL_DBG
#include "rtdbg.h"

#define SOF 0xcc
volatile uint32_t dd_cb2_cnt = 0;
static void dd_cb1(x_head_t *head, uint8_t *body, void *arg)
{
    LOG_I("dd_cb1");
}

uint8_t dd_send_buf[12] = {0x00};
static void dd_cb2(x_head_t *head, uint8_t *body, void *arg)
{
    dd_cb2_cnt++;
    LOG_I("dd_cb2 %d", dd_cb2_cnt);
    link_obj_t *link_obj = link_get_obj(0);

    for (int i=0; i<12; i++)
    {
        dd_send_buf[i] = i;
    }
    link_obj->send(dd_send_buf, 12);
}

#define COMM_CB_CFG \
{\
    {0x00, 0x00, dd_cb1, NULL, NULL}, \
    {0x00, 0x01, dd_cb2, NULL, NULL}, \
}

static comm_cb_desc_t comm_cb[] = COMM_CB_CFG;

static uint8_t _bcc_calc(uint8_t *data, uint32_t len)
{
    if (len == 0) return 0;

    uint8_t val = data[0];
    for (int i=1; i<len; i++)
    {
        val = val ^ data[i];
    }
    return val;
}

void link_unpack_thread(void *arg)
{
    uint32_t read_size = 0;
    uint32_t ready_size_in_rb = 0;
    uint8_t *line_buf = NULL;

    while (1)
    {
        for (int i=0; i<link_get_obj_num(); i++)
        {
            link_obj_t *link_obj = link_get_obj(i);
            line_buf = link_obj->line_buf;
            link_unpack_sta_t *unpack_sta = &link_obj->unpack_sta;

            if (unpack_sta->step == UNPACK_STEP_SEARCH_SOF)
            {
                unpack_sta->line_buf_write_index = 0;
                unpack_sta->expected_size = 1;
                /* 找SOF */
                read_size = lwrb_read(&link_obj->comm_rb, line_buf, unpack_sta->expected_size);
                if (read_size > 0)
                {
                    if (line_buf[0] == SOF)
                    {
                        unpack_sta->timeout = 0;
                        unpack_sta->step = UNPACK_STEP_CHECK_HEAD;
                        unpack_sta->line_buf_write_index += 1;
                    }
                }
            }
            if (unpack_sta->step == UNPACK_STEP_CHECK_HEAD)
            {
                unpack_sta->expected_size = 3;
                ready_size_in_rb = lwrb_get_full(&link_obj->comm_rb);
                if (ready_size_in_rb >= unpack_sta->expected_size)
                {
                    read_size = lwrb_peek(&link_obj->comm_rb, 0, &(line_buf[unpack_sta->line_buf_write_index]), unpack_sta->expected_size);
                    if (read_size == unpack_sta->expected_size)
                    {
                        x_prehead_t *prehead = (x_prehead_t *)line_buf;
                        uint8_t bcc_val = _bcc_calc(line_buf, 3);
                        if (bcc_val == prehead->head_crc)
                        {
                            unpack_sta->step = UNPACK_STEP_FETCH_DATA;
                            unpack_sta->expected_size = prehead->ver_len.len - 1;
                        }
                        else
                        {
                            unpack_sta->step = UNPACK_STEP_SEARCH_SOF;
                        }
                    }
                }
            }
            if (unpack_sta->step == UNPACK_STEP_FETCH_DATA)
            {
                if (unpack_sta->expected_size > 0)
                {
                    read_size = lwrb_read(&link_obj->comm_rb, &(line_buf[unpack_sta->line_buf_write_index]), unpack_sta->expected_size);
                    unpack_sta->line_buf_write_index += read_size;
                    unpack_sta->expected_size -= read_size;
                }
                if (unpack_sta->expected_size == 0) {
                    unpack_sta->step = UNPACK_STEP_EXEC_CBF;
                }
            }

            if (unpack_sta->step == UNPACK_STEP_EXEC_CBF)
            {
                x_head_t *x_head = (x_head_t *)(line_buf);
                for (int i=0; i<LINK_ARRAY_SIZE(comm_cb); i++)
                {
                    if (x_head->cmdSet == comm_cb[i].cmdSet &&
                        x_head->cmdId == comm_cb[i].cmdId)
                    {
                        if (x_head->cmd_type.is_ack) {
                            if (comm_cb[i].ack_cb != NULL) {
                                comm_cb[i].ack_cb(x_head, &line_buf[sizeof(x_head_t)], comm_cb[i].arg);
                            }
                        } else {
                            if (comm_cb[i].req_cb != NULL) {
                                comm_cb[i].req_cb(x_head, &line_buf[sizeof(x_head_t)], comm_cb[i].arg);
                            }
                        }
                        break;
                    }
                }
                unpack_sta->step = UNPACK_STEP_SEARCH_SOF;
            }
            unpack_sta->timeout += 1;
            if (unpack_sta->timeout > 20)
            {
                unpack_sta->timeout = 0;
                unpack_sta->step = UNPACK_STEP_SEARCH_SOF;
            }
        }
        rt_thread_mdelay(1);
    }
}
