#ifndef __XDMAAPI_H__
#define __XDMAAPI_H__

#include "config.h"
#include "dataModel.h"

#ifdef __cplusplus
extern "C" {
#endif

XDMA_FPGA_API bool fpga_open();

XDMA_FPGA_API void fpga_close();

XDMA_FPGA_API void set_callback(pf_fpga_callback _pf, void *);

XDMA_FPGA_API void fpga_start();

XDMA_FPGA_API void fpga_stop();

XDMA_FPGA_API bool push_data(void *);

XDMA_FPGA_API bool push_data_slave(void *);

XDMA_FPGA_API FpgaParaCfg *get_fpga_param();

XDMA_FPGA_API void update_fpga_param(const FpgaParaCfg *);
#ifdef _DOUBLE_V7
XDMA_FPGA_API NNC_CommonStruct::NNC_StateModel get_fpga_status();
#endif
#ifdef __cplusplus
}
#endif

#endif