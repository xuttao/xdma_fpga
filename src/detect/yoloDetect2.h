/*
 * @Author: xtt
 * @Date: 2022-02-05 19:47:24
 * @Description: ...
 * @LastEditTime: 2022-02-11 17:23:32
 */
#pragma once
#include <stdio.h>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "dataModel.h"
#include "log.h"


// struct BoxInfo {
//     float left;
//     float top;
//     float right;
//     float bottom;
//     int classNo;
//     float score;

//     BoxInfo() { memset(this, 0, sizeof(BoxInfo)); }
//     BoxInfo(float t, float l, float b, float r) : left(l), top(t), right(r), bottom(b) {}

// };

struct YoloLayer {
    int net_h;
    int net_w;
    int net_c;
    int output_size; //字节大小
    int *mask;
    int *anchors;
    int anchor_num;
    float *box = nullptr;
    float *box_scores = nullptr;
};

class NetDetect
{
public:
    NetDetect(const char *file, int fpgaNetw, int fpgaNeth);
    ~NetDetect();

public:
    void parserNetCfg(const char *file);
    BoxInfo *getDetectBoxInfo(const char *pData, int src_w, int src_h, int &num);

private:
    void yoloHead(const char *pData, YoloLayer &layer, int src_w, int src_h);
    void yoloCorrectBoxes(const char *pData, const YoloLayer &layer, int src_w, int src_h) = delete;
    float calIou(BoxInfo &box1, BoxInfo &box2);

private:
    int maxBoxNum = 100; //最大框个数
    float thresh = .1;   //置信度阈值
    float iou = .45;     // iou阈值
    int classNum = 80;   //类别个数
    int fpgaNetw = 416;
    int fpgaNeth = 416;
    int *anchorArr = nullptr;
    char *inputData = nullptr;
    float *boxMinMax = nullptr;
    BoxInfo *boxArr = nullptr;
    std::vector<YoloLayer> vecLayer;
    std::map<std::string, std::map<std::string, std::string>> mpNetData;

    float *pBoxValid = nullptr;
    float *pBoxScoresValid = nullptr;
    int *pClassValid = nullptr;
    int boxValidNum = 0;
};