#include "link.h"
#include "link_unpack.h"
#include "rtthread.h"

static link_obj_t link_obj[3];
static uint8_t link_obj_num = 0;

link_obj_t *link_get_obj(uint8_t index)
{
    return &link_obj[index];
}

uint8_t link_get_obj_num(void)
{
    return link_obj_num;
}

int32_t link_add(uint8_t *rb_buf, uint32_t rb_size,
                uint8_t *line_buf, uint32_t line_buf_size,
                link_send_t pSend)
{
    if (link_obj_num >= 3) return -1;
    link_obj_t *obj = &(link_obj[link_obj_num]);

    obj->line_buf = line_buf;
    obj->line_size = line_buf_size;
    lwrb_init(&obj->comm_rb, rb_buf, rb_size); /* Initialize buffer */
    obj->send = pSend;
    obj->is_ready = 1;
    obj->index = link_obj_num;
    link_obj_num++;

    return obj->index;
}

int32_t link_recv_data(uint8_t index, uint8_t *data, uint32_t len)
{
    if (index >= link_obj_num) return -1;
    uint32_t size = 0;

    link_obj_t *obj = &(link_obj[index]);
    size = lwrb_write(&obj->comm_rb, data, len);
    if (size != len) return -2;
    else return 0;
}

static int link_init(void)
{
    rt_thread_t tid = rt_thread_create("link_unpack", link_unpack_thread, NULL, 4096, 2, 10);
    if (tid != NULL)
    {
        rt_thread_startup(tid);
    }
    return 0;
}

INIT_APP_EXPORT(link_init);
