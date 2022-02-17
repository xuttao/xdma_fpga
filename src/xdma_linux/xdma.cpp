#include <fcntl.h>
#include <stdexcept>
#include <sys/mman.h>
#include <unistd.h>

#include "log.h"
#include "xdma.h"

#include <pthread.h>

#define REG_MEMORY_BASE (0x20000)

#define MAP_SIZE (64 * 1024)

#define INVALID_FILE_ID -1

#define XDMA_CHANNEL_CNT 1

#define DEV_MEM_ADDR_BASE_0 0x0
#define DEV_MEM_ADDR_BASE_1 0x400000000
#define DEV_MEM_ADDR_BASE_2 0x800000000
#define DEV_MEM_ADDR_BASE_3 0xc00000000

#define FPGA_DOWN 0

#define FPGA_START 1

#define FPGA_CALC_START()              \
    do {                               \
        xdma_write_reg(0, FPGA_DOWN);  \
        usleep(1);                     \
        xdma_write_reg(0, FPGA_START); \
    } while (false)

#define FPGA_CALC_START_INDEX(reg_index)       \
    do {                                       \
        xdma_write_reg(reg_index, FPGA_DOWN);  \
        usleep(1);                             \
        xdma_write_reg(reg_index, FPGA_START); \
    } while (false)

//Read from FPGA devices
static const char *xdma_list_c2h[XDMA_CHANNEL_CNT] = {
    "/dev/xdma0_c2h_0",
    // "/dev/xdma0_c2h_1",
};

//Write to FPGA devices
static const char *xdma_list_h2c[XDMA_CHANNEL_CNT] = {
    "/dev/xdma0_h2c_0",
    // "/dev/xdma0_h2c_1",
};

static int32_t dma_reg_fd = INVALID_FILE_ID;
static uint32_t *dma_reg_base = NULL;

static int32_t dma_read_fds[XDMA_CHANNEL_CNT] = {INVALID_FILE_ID};
static int32_t dma_write_fds[XDMA_CHANNEL_CNT] = {INVALID_FILE_ID};

static uint64_t dma_channels_base_addr[XDMA_CHANNEL_CNT] = {
    DEV_MEM_ADDR_BASE_0,
    // DEV_MEM_ADDR_BASE_1,
};

// static pthread_mutex_t mutex_reg_read=PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_reg=PTHREAD_MUTEX_INITIALIZER;
// static pthread_mutex_t mutex_write=PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_read_write=PTHREAD_MUTEX_INITIALIZER;

int32_t xdma_open(const dev_init_param_st &param)
{
    if (dma_reg_base) {
        LOG_ERR("DMA is already opend\n");
        return -1;
    }

    //Open registers to access FPGA
    dma_reg_fd = open(param.reg_dev_name, O_RDWR | O_SYNC);
    if (INVALID_FILE_ID == dma_reg_fd) {
        LOG_ERR("DMA_InitReg:unable to open device:[%s].\n", param.reg_dev_name);
        return -1;
    }

    dma_reg_base = (uint32_t *)mmap((void *)0,
                                    MAP_SIZE,
                                    PROT_READ | PROT_WRITE,
                                    MAP_SHARED,
                                    dma_reg_fd,
                                    REG_MEMORY_BASE);
    if (NULL == dma_reg_base) {
        LOG_ERR("DMA_InitReg:unable to map memory");
        return -1;
    }

    //Open files to access XDMA
    for (int i = 0; i < XDMA_CHANNEL_CNT; ++i) {
        dma_write_fds[i] = open(xdma_list_h2c[i], O_RDWR | O_NONBLOCK);
        if (INVALID_FILE_ID == dma_write_fds[i]) {
            LOG_ERR("DMA_InitDma:unable to open device:[%s].", xdma_list_h2c[i]);
            return -1;
        }
    }

    for (int i = 0; i < XDMA_CHANNEL_CNT; ++i) {
        dma_read_fds[i] = open(xdma_list_c2h[i], O_RDWR | O_NONBLOCK);
        if (INVALID_FILE_ID == dma_read_fds[i]) {
            LOG_ERR("DMA_InitDma:unable to open device:[%s].", xdma_list_c2h[i]);
            close(dma_write_fds[i]);
            dma_write_fds[i] = INVALID_FILE_ID;
            return -1;
        }
    }

    return 0;
}

void xdma_close(void)
{
    if (dma_reg_base) {
        munmap((void *)dma_reg_base, MAP_SIZE);
        dma_reg_base = NULL;
    }
    if (INVALID_FILE_ID != dma_reg_fd) {
        close(dma_reg_fd);
        dma_reg_fd = INVALID_FILE_ID;
    }
    //Close files opened for XDMA
    for (int i = 0; i < XDMA_CHANNEL_CNT; ++i) {
        if (INVALID_FILE_ID != dma_read_fds[i]) {
            close(dma_read_fds[i]);
            dma_read_fds[i] = INVALID_FILE_ID;
        }
    }

    for (int i = 0; i < XDMA_CHANNEL_CNT; ++i) {
        if (INVALID_FILE_ID != dma_write_fds[i]) {
            close(dma_write_fds[i]);
            dma_write_fds[i] = INVALID_FILE_ID;
        }
    }
}

int32_t xdma_write_reg(uint32_t reg_index, int32_t data)
{
	pthread_mutex_lock(&mutex_reg);
    if (NULL != dma_reg_base) {
        dma_reg_base[reg_index] = data;
		pthread_mutex_unlock(&mutex_reg);
        return 0;
    }
    LOG_ERR("DMA reg not open");
	pthread_mutex_unlock(&mutex_reg);
    return -1;

}

int32_t xdma_read_reg(uint32_t reg_index, int32_t &data_out)
{
	pthread_mutex_lock(&mutex_reg);
    if (NULL == dma_reg_base) {
        LOG_ERR("DMA reg not open");
		pthread_mutex_unlock(&mutex_reg);
        return -1;
    }

    char *ptr = (char *)dma_reg_base;
    int index_bits = (uint8_t)ptr[0];
    int32_t index_num = (uint8_t)ptr[1];
    int32_t index_bytes = (uint8_t)ptr[2];
    int32_t i, index, shift_bits, mapped_index;

    uint32_t index_mask = (1 << index_bits) - 1;

    if (index_num == 0 || index_bits != 8) {
        LOG_ERR("DMA reg mem err");
		pthread_mutex_unlock(&mutex_reg);
        return -1;
    }

    for (i = 0; i < index_num; i++) {
        mapped_index = ((i * index_bits) / 32) + 1;
        shift_bits = (i * index_bits) % 32;

        if (shift_bits + index_bits > 32) {
            index = ((dma_reg_base[mapped_index] >> shift_bits) & ((1 << (32 - shift_bits)) - 1)) |
                    ((dma_reg_base[mapped_index + 1] & ((1 << (shift_bits + index_bits - 32)) - 1))
                     << (32 - shift_bits));
        } else {
            index = (dma_reg_base[mapped_index] >> shift_bits) & index_mask;
        }

        if (index == reg_index) {
            data_out = dma_reg_base[1 + index_bytes + i];
        }
    }
	pthread_mutex_unlock(&mutex_reg);
    return 0;
}

void xdma_write(void *dst_ptr, void *data_in_ptr, uint32_t data_size)
{
	pthread_mutex_lock(&mutex_read_write);

    write_from_buffer((char *)xdma_list_h2c[0],
                      dma_write_fds[0],
                      (char *)data_in_ptr,
                      data_size,
                      (uint64_t)dst_ptr);

	pthread_mutex_unlock(&mutex_read_write);
}

void xdma_write_with_reg(void *dst_ptr, void *data_in_ptr, uint32_t data_size, int32_t reg_index)
{
    xdma_write(dst_ptr, data_in_ptr, data_size);
    FPGA_CALC_START_INDEX(reg_index);
}

void xdma_read(void *src_ptr, void *data_out_ptr, uint32_t data_size)
{
	pthread_mutex_lock(&mutex_read_write);

    if (INVALID_FILE_ID != dma_read_fds[0]) {
        ssize_t ret = read_to_buffer((char *)xdma_list_c2h[0], dma_read_fds[0], (char *)data_out_ptr, data_size,
                                     (uint64_t)src_ptr);
        FSERVO_CHECK(ret == data_size);
    }

	pthread_mutex_unlock(&mutex_read_write);
}
