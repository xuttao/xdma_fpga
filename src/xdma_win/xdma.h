
#ifndef _FSERVO_XDMA_H_
#define _FSERVO_XDMA_H_

#include <windows.h>
#include <stdint.h>

#define ERR_NONE 0
#define ERR_NORM -1

int32_t xdma_open();

int32_t xdma_close(void);

int32_t xdma_write(void *data_in_ptr, uint64_t offset, uint32_t data_size);

int32_t xdma_write_with_reg(void *data_in_ptr, uint64_t offset, uint32_t data_size, int reg_index);

int32_t xdma_read(void *data_out_ptr, uint64_t offset, uint32_t data_size);

int32_t xdma_write_reg(uint32_t reg_index, int32_t data);

int32_t xdma_read_reg(uint32_t reg_index, int32_t &data_out);

// define for read and write XDMA device(refer to dma_utils.c)
extern "C"
{
        HANDLE open_device(char device_base_path[], const char *device_name, LARGE_INTEGER base_address);
        void CleanupDevice(HANDLE device);
        int get_devices(GUID guid, char *devpath, size_t len_devpath);
        BYTE *allocate_buffer(size_t size, size_t alignment);
}

#endif //_FSERVO_XDMA_H_
