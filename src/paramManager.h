#pragma once
#include "dataModel.h"
#include <map>

class ParamManager
{
private:
    ParamManager() = default;
    ~ParamManager() = default;

public:
    static ParamManager *getInstance();
    void parserCfg(const char *_file);
    void initParam();
    inline const FpgaParaCfg &getFpgaCfg() const { return fpgaParam; }
    inline const PicParaCfg &getPicCfg() const { return picParam; }
    inline const DetectParaCfg &getNetCfg() const { return netParam; }

private:
    void setParam(const std::string &group, const std::string &key, const std::string &val);
    // void updateFpgaCfg();

public:
    void updateFpgaParam(const FpgaParaCfg *pCfg);

private:
    std::map<std::string, std::string> mpFpga;
    std::map<std::string, std::string> mpPic;
    std::map<std::string, std::string> mpNet;

    FpgaParaCfg fpgaParam;
    PicParaCfg picParam;
    DetectParaCfg netParam;
};