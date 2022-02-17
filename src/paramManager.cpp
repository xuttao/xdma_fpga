#include "paramManager.h"
#include "log.h"
#include <fstream>
#include <sstream>
#include <vector>
namespace
{
    void rmStrSpace(std::string &str)
    {
        for (;;) {
            size_t pos = str.find_first_of(' ');
            if (pos != std::string::npos) {
                str.erase(pos, 1);
            } else
                break;
        }
    }

    std::vector<std::string> split_string(const std::string &str,
                                          const char splitStr)
    {
        std::vector<std::string> vecStr;
        std::string::size_type beginPos =
            str.find_first_not_of(splitStr, 0);
        std::string::size_type endPos =
            str.find_first_of(splitStr, beginPos);
        while (
            std::string::npos != endPos ||
            std::string::npos !=
                beginPos) {
            vecStr.push_back(str.substr(
                beginPos, endPos - beginPos));
            beginPos = str.find_first_not_of(
                splitStr, endPos);
            endPos =
                str.find_first_of(splitStr, beginPos);
        }

        return vecStr;
    }

    const char *FPGA_CFG_KEY = "fpga_cfg";
    const char *PIC_CFG_KEY = "pic_cfg";
    const char *NET_CFG_KEY = "detect_cfg";
} // namespace

ParamManager *ParamManager::getInstance()
{
    static ParamManager ins;
    return &ins;
}

void ParamManager::parserCfg(const char *_file)
{
    std::ifstream in(_file);

    FSERVO_CHECK(in.is_open());

    std::stringstream strStream;
    strStream << in.rdbuf();
    in.close();

    while (!strStream.eof() && !strStream.fail()) {
        std::string strLine;
        try {
            getline(strStream, strLine);
            if (strLine.empty())
                continue;

            static std::string key;
            static std::string val;

            rmStrSpace(strLine);

            if (strLine.empty())
                continue;
            if (strLine[0] == '#')
                continue;

            if (strLine[0] == '[') {
                key = strLine.substr(1, strLine.find_first_of(']') - 1);
            } else {
                size_t pos = strLine.find_first_of('#');
                if (pos != std::string::npos) {
                    strLine = strLine.substr(0, pos);
                }
                size_t posMid = strLine.find_first_of('=');
                auto str1 = strLine.substr(0, posMid);
                auto str2 = strLine.substr(posMid + 1, strLine.length() - posMid - 1);

                setParam(key, str1, str2);
            }
        } catch (...) {
            LOG_ERR("parser cfg fail:%s in line:%s", _file, strLine.c_str());
            exit(-1);
        }
    }
}

void ParamManager::setParam(const std::string &group, const std::string &key,
                            const std::string &val)
{
    if (FPGA_CFG_KEY == group) {
        mpFpga.insert(std::make_pair(key, val));
    } else if (PIC_CFG_KEY == group) {
        mpPic.insert(std::make_pair(key, val));
    } else if (NET_CFG_KEY == group) {
        mpNet.insert(std::make_pair(key, val));
    } else {
        LOG_ERR("key :%s is invalid", key.c_str());
        assert(false);
    }
}

void ParamManager::initParam()
{
    try {
        fpgaParam.paramFiles = mpFpga["param_files_path"];
        fpgaParam.paramAddress = mpFpga["param_files_addrs"];
        fpgaParam.keyFiles = mpFpga["key_file_path"];
        fpgaParam.inputNetHeight = std::stoi(split_string(mpFpga["input_resolution"], 'x')[0]);
        fpgaParam.inputNetWidth = std::stoi(split_string(mpFpga["input_resolution"], 'x')[1]);
        fpgaParam.inputOffset = std::stoul(mpFpga["input_offest"], nullptr, 16);
        fpgaParam.outputOffset = std::stoul(mpFpga["output_offest"], nullptr, 16);
        fpgaParam.outputSize = std::stoul(mpFpga["output_size"], nullptr, 16);
        fpgaParam.inputBufferSize = std::stoul(mpFpga["input_size"], nullptr, 16);
        fpgaParam.inputBufferNum = std::stoi(mpFpga["input_buffer_num"]);
        fpgaParam.outputSigType = std::stoi(mpFpga["output_signal_type"]);
        fpgaParam.outputWaitTime = std::stoul(mpFpga["output_wait_time"]);
        fpgaParam.inputReg = std::stoi(mpFpga["input_reg"]);
        fpgaParam.outputReg = std::stoi(mpFpga["output_reg"]);
        fpgaParam.interuptDev = mpFpga["fpga_interupt_dev"];

        picParam.processMode = std::stoi(mpPic["process_mode"]);
        picParam.inputScale = std::stod(mpPic["input_scale"]);
        picParam.inputZero = std::stof(mpPic["input_zero"]);
        picParam.rMean = std::stof(mpPic["r_mean"]);
        picParam.gMean = std::stof(mpPic["g_mean"]);
        picParam.bMean = std::stof(mpPic["b_mean"]);
        picParam.rStd = std::stof(mpPic["r_std"]);
        picParam.gStd = std::stof(mpPic["g_std"]);
        picParam.bStd = std::stof(mpPic["b_std"]);

        netParam.netFile = mpNet["model_cfg_file"];
        netParam.labelPath = mpNet["label_path"];
        netParam.classFile = mpNet["class_file"];
    } catch (...) {
        LOG_ERR("init cfg fail!");
        exit(-1);
    }
    mpFpga.clear();
    mpPic.clear();
    mpNet.clear();

    FSERVO_CHECK(fpgaParam.isValid());
}

void ParamManager::updateFpgaParam(const FpgaParaCfg *pCfg)
{
    fpgaParam = *pCfg;
}