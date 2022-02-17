#include "xdma.h"
#include "log.h"
#include "xdma_public.h"
#include <chrono>
#include <strsafe.h>
#include <thread>

#pragma comment(lib, "setupapi.lib")

#define REG_MEMORY_BASE (0x20000)

#define MAP_SIZE (64 * 1024)

#define INVALID_FILE_ID -1

#define DEV_CNT_TO_WRITE_DATA 1

#define DEV_CNT_TO_WRITE_WEIGHT 1

#define XDMA_CHANNEL_CNT 1

#define DEV_MEM_ADDR_BASE_0 0x00000000
#define DEV_MEM_ADDR_BASE_1 0x00000000
#define DEV_MEM_ADDR_BASE_2 0x800000000
#define DEV_MEM_ADDR_BASE_3 0xc00000000

#define FPGA_DOWN 0
#define FPGA_START 1

#define MAX_PATH 260

#define FPGA_CALC_START()                                                  \
        do                                                                 \
        {                                                                  \
                xdma_write_reg(0, FPGA_DOWN);                              \
                std::this_thread::sleep_for(std::chrono::microseconds(1)); \
                xdma_write_reg(0, FPGA_START);                             \
        } while (false)

#define FPGA_CALC_START_INDEX(reg_index)                                   \
        do                                                                 \
        {                                                                  \
                xdma_write_reg(reg_index, FPGA_DOWN);                      \
                std::this_thread::sleep_for(std::chrono::microseconds(1)); \
                xdma_write_reg(reg_index, FPGA_START);                     \
        } while (false)

// reg dev name
static const char *reg_dev_name = "user";

//Read from FPGA devices
static const char *xdma_list_c2h[XDMA_CHANNEL_CNT] = {
    "c2h_0",
    /*"c2h_1",*/};

//Write to FPGA devices
static const char *xdma_list_h2c[XDMA_CHANNEL_CNT] = {
    "h2c_0",
    /*"h2c_1",*/};

HANDLE dma_reg_fd = INVALID_HANDLE_VALUE;

HANDLE dma_read_fds[XDMA_CHANNEL_CNT] = {INVALID_HANDLE_VALUE};
HANDLE dma_write_fds[XDMA_CHANNEL_CNT] = {INVALID_HANDLE_VALUE};

static const uint32_t SIZE_8M = 8UL * 1024 * 1024;

static uint64_t dma_channels_base_addr[XDMA_CHANNEL_CNT] = {
    DEV_MEM_ADDR_BASE_0,
    /*DEV_MEM_ADDR_BASE_1,*/
};

bool is_xdma_dev_opened = false;

int32_t xdma_open()
{
        do
        {
                if (true == is_xdma_dev_opened)
                        return ERR_NONE;

                // get device path from GUID
                char device_base_path[MAX_PATH + 1] = "";
                DWORD num_devices = get_devices(GUID_DEVINTERFACE_XDMA, device_base_path, sizeof(device_base_path));
                LOG_INFO("Devices found: %d\n", num_devices);
                LOG_INFO("Device base path: %s\n", device_base_path);
                if (num_devices < 1)
                {
                        return ERR_NORM;
                }
                LARGE_INTEGER reg_memory_base;
                reg_memory_base.QuadPart = REG_MEMORY_BASE;
                dma_reg_fd = open_device(device_base_path, reg_dev_name, reg_memory_base);

                //Open files to access XDMA
                for (int i = 0; i < XDMA_CHANNEL_CNT; ++i)
                {
                        LARGE_INTEGER h2c_memory_base;
                        h2c_memory_base.QuadPart = dma_channels_base_addr[i];
                        dma_write_fds[i] = open_device(device_base_path, xdma_list_h2c[i], h2c_memory_base);
                        if (INVALID_HANDLE_VALUE == dma_write_fds[i])
                        {
                                LOG_ERR("DMA_InitDma:unable to open device:[%s].", xdma_list_h2c[i]);
                                return ERR_NORM;
                        }
                }

                for (int i = 0; i < XDMA_CHANNEL_CNT; ++i)
                {
                        LARGE_INTEGER c2h_memory_base;
                        c2h_memory_base.QuadPart = dma_channels_base_addr[i];
                        dma_read_fds[i] = open_device(device_base_path, xdma_list_c2h[i], c2h_memory_base);
                        if (INVALID_HANDLE_VALUE == dma_read_fds[i])
                        {
                                LOG_ERR("DMA_InitDma:unable to open device:[%s].", xdma_list_c2h[i]);
                                return ERR_NORM;
                        }
                }

                is_xdma_dev_opened = true;

        } while (false);

        return ERR_NONE;
}

int32_t xdma_close(void)
{
        if (is_xdma_dev_opened)
        {
                if (INVALID_HANDLE_VALUE != dma_reg_fd)
                {
                        CleanupDevice(dma_reg_fd);
                        dma_reg_fd = INVALID_HANDLE_VALUE;
                }
                //Close files opened for XDMA
                for (int i = 0; i < XDMA_CHANNEL_CNT; ++i)
                {
                        if (INVALID_HANDLE_VALUE != dma_read_fds[i])
                        {
                                CleanupDevice(dma_read_fds[i]);
                                dma_read_fds[i] = INVALID_HANDLE_VALUE;
                        }
                }
                for (int i = 0; i < XDMA_CHANNEL_CNT; ++i)
                {
                        if (INVALID_HANDLE_VALUE != dma_write_fds[i])
                        {
                                CleanupDevice(dma_write_fds[i]);
                                dma_write_fds[i] = INVALID_HANDLE_VALUE;
                        }
                }
                is_xdma_dev_opened = false;
        }
        return ERR_NONE;
}

int32_t xdma_write_reg(uint32_t reg_index, int32_t data)
{

        int32_t ret = ERR_NONE;
        FSERVO_CHECK(true == is_xdma_dev_opened);

        LARGE_INTEGER offset;
        offset.QuadPart = reg_index * sizeof(DWORD);
        if (INVALID_SET_FILE_POINTER == SetFilePointerEx(dma_reg_fd, offset, NULL, FILE_BEGIN))
        {
                LOG_ERR("Error setting file pointer, win32 error code: %ld\n", GetLastError());
                CleanupDevice(dma_reg_fd);

                assert(false);
        }

        DWORD len = sizeof(data);
        if (!WriteFile(dma_reg_fd, &data, len, &len, NULL))
        {
                fprintf(stderr, "WriteFile to device %s failed with Win32 error code: %d\n", "user", GetLastError());
                CleanupDevice(dma_reg_fd);

                assert(false);
        }

        return ret;
}

int32_t xdma_read_reg(uint32_t reg_index, int32_t &data_out)
{

        int32_t ret = ERR_NONE;
        FSERVO_CHECK(true == is_xdma_dev_opened);

        // allocate and zero-initialize memory to hold read bytes
        DWORD len = MAP_SIZE;
        uint32_t *data = (uint32_t *)allocate_buffer(len, 0);
        if (!ReadFile(dma_reg_fd, data, len, &len, NULL))
        {
                fprintf(stderr, "ReadFile from device %s failed with Win32 error code: %ld\n", "user", GetLastError());
                CleanupDevice(dma_reg_fd);

                assert(false);
        }

        do
        {
                char *ptr = (char *)data;
                int index_bits = (uint8_t)ptr[0];
                int32_t index_num = (uint8_t)ptr[1];
                int32_t index_bytes = (uint8_t)ptr[2];
                int32_t i, index, shift_bits, mapped_index;

                uint32_t index_mask = (1 << index_bits) - 1;

                if (index_bits != 8)
                {
                        fprintf(stderr, "reg bits is not 8!");
                        assert(false);
                }

                if (index_num <= 0)
                {
                        fprintf(stderr, "reg num is %d!", index_num);
                        assert(false);
                }

                // the num of reg
                for (i = 0; i < index_num; i++)
                {
                        mapped_index = ((i * index_bits) / 32) + 1;
                        shift_bits = (i * index_bits) % 32;

                        if (shift_bits + index_bits > 32)
                        {
                                index = ((data[mapped_index] >> shift_bits) & ((1 << (32 - shift_bits)) - 1)) |
                                        ((data[mapped_index + 1] & ((1 << (shift_bits + index_bits - 32)) - 1)) << (32 - shift_bits));
                        }
                        else
                        {
                                index = (data[mapped_index] >> shift_bits) & index_mask;
                        }

                        if (index == reg_index)
                        {
                                data_out = data[1 + index_bytes + i];
                                break;
                        }
                }
                _aligned_free(data);

        } while (false);

        return ret;
}

int32_t xdma_write(void *data_in_ptr, uint64_t offset, uint32_t data_size)
{
        FSERVO_CHECK(true == is_xdma_dev_opened);

        for (int i = 0; i < DEV_CNT_TO_WRITE_DATA; ++i)
        {
            uint32_t current_pos = 0;
            do
            {
                LARGE_INTEGER forward_offset;
                forward_offset.QuadPart = offset+current_pos;
                if (INVALID_SET_FILE_POINTER == SetFilePointerEx(dma_write_fds[i], forward_offset, NULL, FILE_BEGIN))
                {
                    LOG_ERR("Error setting file pointer, win32 error code: %ld\n", GetLastError());
                    CleanupDevice(dma_write_fds[i]);

                    assert(false);
                }
                DWORD len = data_size>SIZE_8M? SIZE_8M:data_size;
                
                if (!WriteFile(dma_write_fds[i], (char*)data_in_ptr+ current_pos, len, &len, NULL))
                {
                    fprintf(stderr, "WriteFile to device %s%d failed with Win32 error code: %d\n", "h2c_", i, GetLastError());
                    CleanupDevice(dma_write_fds[i]);

                    assert(false);
                }
                current_pos += len;
                data_size -= len;
            } while (data_size > 0);
        }

        return ERR_NONE;
}

int32_t xdma_write_with_reg(void *data_in_ptr, uint64_t offset, uint32_t data_size, int reg_index)
{

        FSERVO_CHECK(true == is_xdma_dev_opened);

        LARGE_INTEGER forward_offset;
        forward_offset.QuadPart = offset;

        for (int i = 0; i < DEV_CNT_TO_WRITE_DATA; ++i)
        {

                if (INVALID_SET_FILE_POINTER == SetFilePointerEx(dma_write_fds[i], forward_offset, NULL, FILE_BEGIN))
                {
                        LOG_ERR("Error setting file pointer, win32 error code: %ld\n", GetLastError());
                        CleanupDevice(dma_write_fds[i]);

                        assert(false);
                }
                DWORD len = data_size;
                if (!WriteFile(dma_write_fds[i], data_in_ptr, len, &len, NULL))
                {
                        fprintf(stderr, "WriteFile to device %s%d failed with Win32 error code: %d\n", "h2c_", i, GetLastError());
                        CleanupDevice(dma_write_fds[i]);

                        assert(false);
                }
        }

        FPGA_CALC_START_INDEX(reg_index);

        return ERR_NONE;
}

int32_t xdma_read(void *data_out_ptr, uint64_t offset, uint32_t data_size)
{

        LARGE_INTEGER forward_offset;
        forward_offset.QuadPart = offset;

        if (INVALID_HANDLE_VALUE != dma_read_fds[0])
        {
            uint32_t current_pos = 0;
            do
            {
                LARGE_INTEGER forward_offset;
                forward_offset.QuadPart = offset+current_pos;
                if (INVALID_SET_FILE_POINTER == SetFilePointerEx(dma_read_fds[0], forward_offset, NULL, FILE_BEGIN))
                {
                    LOG_ERR("Error setting file pointer, win32 error code: %ld\n", GetLastError());
                    CleanupDevice(dma_read_fds[0]);

                    assert(false);
                }

                DWORD len = data_size>SIZE_8M? SIZE_8M:data_size;
                if (!ReadFile(dma_read_fds[0], (char*)data_out_ptr+current_pos, len, &len, NULL))
                {
                    fprintf(stderr, "ReadFile from device %s failed with Win32 error code: %ld\n",
                        "user", GetLastError());
                    CleanupDevice(dma_reg_fd);

                    assert(false);
                }
                current_pos += len;
                data_size -= len;
                //if (len != data_size)
                //{
                //    fprintf(stderr, "Read len is %d,Result len is %d", data_size, len);

                //    return ERR_NORM;
                //}

            } while (data_size > 0);
        }
        return ERR_NONE;
}