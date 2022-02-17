#pragma once
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
typedef struct BoxInfo {
    float left;
    float top;
    float right;
    float bottom;
    int classNo;
    float score;

    BoxInfo() { memset(this, 0, sizeof(BoxInfo)); }
    BoxInfo(float t, float l, float b, float r) : left(l), top(t), right(r), bottom(b) {}

} BoxInfo;

struct FpgaInput {
    uint8_t *data;     //must rgb, not bgr
    uint8_t *src_data; //must rgb, not bgr
    uint32_t len;
    uint32_t height;
    uint32_t width;
    std::string name;
};

struct FpgaParaCfg {
    std::string paramFiles;
    std::string paramAddress;
    std::string keyFiles;
    uint32_t inputNetHeight;
    uint32_t inputNetWidth;
    uint32_t inputOffset;
    uint32_t outputOffset;
    uint32_t outputSize;
    uint32_t inputBufferSize;
    uint32_t inputBufferNum;
    uint8_t outputSigType;
    uint32_t outputWaitTime;
    uint32_t inputReg;
    uint32_t outputReg;
    std::string interuptDev;

    bool isValid() const
    {
        return !(paramFiles.empty() || paramAddress.empty());
    }
};

struct PicParaCfg {
    uint8_t processMode = 0;
    double inputScale = 1.f;
    float inputZero = 0.f;
    float rMean = 0.f;
    float gMean = 0.f;
    float bMean = 0.f;
    float rStd = 0.f;
    float gStd = 0.f;
    float bStd = 0.f;
};

struct DetectParaCfg {
    std::string netFile;
    std::string labelPath;
    std::string classFile;
};

struct DetectData {
    uint8_t *outData = nullptr;
    void *inputData = nullptr;

    DetectData(uint8_t *data1, void *data2) : outData(data1), inputData(data2) {}
    DetectData() = default;
};
#ifdef _DOUBLE_V7
#include "NNC_InterfaceControlDocument.h"
typedef void (*pf_fpga_callback)(NNC_InterfaceControlDocument::OutParameterAllInOne *,
                                 NNC_InterfaceControlDocument::InParameterAllInOne *, void *);
#else
typedef void (*pf_fpga_callback)(FpgaInput *, BoxInfo2 *, int, void *);
#endif