/*
 * @Author: xtt
 * @Date: 2021-05-17 12:06:45
 * @Description: ...
 * @LastEditTime: 2021-05-17 12:26:24
 */

#pragma once

#ifdef _UNIX
#define XDMA_FPGA_API
#else

#ifdef XDMAFPGA_EXPORTS
#define XDMA_FPGA_API __declspec(dllexport)
#else
#define XDMA_FPGA_API __declspec(dllimport)
#endif

#endif
