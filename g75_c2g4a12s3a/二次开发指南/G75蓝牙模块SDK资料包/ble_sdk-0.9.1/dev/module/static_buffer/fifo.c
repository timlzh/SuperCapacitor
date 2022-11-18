#include <stdlib.h>
#include "fifo.h"
#include "compile_flag.h"
#define GET_WR_IDX(fifo) ((fifo)->wr_idx>=(fifo)->length?(fifo)->wr_idx - (fifo)->length:(fifo)->wr_idx)
#define GET_RD_IDX(fifo) ((fifo)->rd_idx>=(fifo)->length?(fifo)->rd_idx - (fifo)->length:(fifo)->rd_idx)

ROM_SYMBOL void sw_fifo_init(struct fifo_env *ptr,void *buf,uint16_t length)
{
    ptr->buf = buf;
    ptr->rd_idx = 0;
    ptr->wr_idx = 0;
    ptr->length = length;
}

ROM_SYMBOL bool sw_fifo_full(struct fifo_env *ptr)
{
    return abs(ptr->wr_idx - ptr->rd_idx) == ptr->length;
}

ROM_SYMBOL bool sw_fifo_empty(struct fifo_env *ptr)
{
    return ptr->wr_idx == ptr->rd_idx;
}

ROM_SYMBOL void sw_fifo_flush(struct fifo_env *ptr)
{
    ptr->wr_idx = ptr->rd_idx; 
}

ROM_SYMBOL uint16_t sw_fifo_element_amount(struct fifo_env *ptr)
{
    return ptr->wr_idx-ptr->rd_idx;
}

ROM_SYMBOL bool dword_fifo_put(struct fifo_env *ptr,void *data)
{
    if(sw_fifo_full(ptr))
    {
        return false;
    }else
    {
        uint64_t *elem = ptr->buf;
        elem[GET_WR_IDX(ptr)] = *(uint64_t *)data;
        ptr->wr_idx = (ptr->wr_idx + 1)%(2*ptr->length);
        return true;
    }
}

ROM_SYMBOL bool dword_fifo_get(struct fifo_env *ptr,void *data)
{
    if(sw_fifo_empty(ptr))
    {
        return false;
    }else
    {
        uint64_t *elem = ptr->buf;
        *(uint64_t *)data = elem[GET_RD_IDX(ptr)];
        ptr->rd_idx = (ptr->rd_idx + 1)%(2*ptr->length);
        return true;
    }
}
