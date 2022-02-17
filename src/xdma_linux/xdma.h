#ifndef _FSERVO_XDMA_H_
#define _FSERVO_XDMA_H_

#include <stdint.h>
#include <sys/types.h>

typedef struct
{
    const char *mem_dev_name;
    const char *reg_dev_name;
} dev_init_param_st;

int32_t xdma_open(const dev_init_param_st &param);

void xdma_close(void);

void xdma_write(void *dst_ptr, void *data_in_ptr, uint32_t data_size);

void xdma_write_with_reg(void *dst_ptr, void *data_in_ptr, uint32_t data_size, int32_t reg_index);

void xdma_read(void *src_ptr, void *data_out_ptr, uint32_t data_size);

int32_t xdma_write_reg(uint32_t reg_index, int32_t data);

int32_t xdma_read_reg(uint32_t reg_index, int32_t &data_out);

//define for read and write XDMA device(refer to dma_utils.c)
extern "C" {
ssize_t read_to_buffer(char *fname, int fd, char *buffer, uint64_t size, uint64_t base);
ssize_t write_from_buffer(char *fname, int fd, char *buffer, uint64_t size, uint64_t base);
}

#endif //_FSERVO_XDMA_H_
