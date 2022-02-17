#include "xdmaApi.h"
#include "fpgaControl.h"

bool fpga_open()
{
    return FpgaControl::getInstance()->open();
}

void fpga_close()
{
    FpgaControl::getInstance()->close();
}

void set_callback(pf_fpga_callback _pf, void *_para)
{
    FpgaControl::getInstance()->setCallBack(_pf, _para);
}

void fpga_start()
{
    FpgaControl::getInstance()->start();
}

void fpga_stop()
{
    FpgaControl::getInstance()->stop();
}

bool push_data(void *pInput)
{
    return FpgaControl::getInstance()->push(pInput);
}

bool push_data_slave(void *pInput)
{
	return FpgaControl::getInstance()->push_slave(pInput);
}

FpgaParaCfg *get_fpga_param()
{
    return FpgaControl::getInstance()->getFpgaPara();
}

void update_fpga_param(const FpgaParaCfg *pdata)
{
    FpgaControl::getInstance()->setFpgaPara(pdata);
}
#ifdef _DOUBLE_V7
NNC_CommonStruct::NNC_StateModel get_fpga_status()
{
    return FpgaControl::getInstance()->getFpgaStatus();
}
#endif