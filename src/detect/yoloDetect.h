/*
 * @Author: xtt
 * @Date: 2020-10-24 17:16:02
 * @Description: ...
 * @LastEditTime: 2022-02-11 17:20:39
 */

#pragma once
#include <stdio.h>

#include "dataModel.h"
#include "log.h"
#include <cmath>
#include <map>
#include <string>
#include <vector>

enum NMSKIND {
    DEFAULT_NMS,
    GREEDY_NMS,
    DIOU_NMS,
    CORNERS_NMS
};

struct YoloLayer {
    int net_h;
    int net_w;
    int net_c;
    int output_size;
    int grid_box_num;
    int *mask;
    int *anchors;
    NMSKIND nms_kind;
    float beta_nms;
};

struct box {
    float x, y, w, h;
};

struct detection {
    box bbox;
    int classes;
    float *prob;
    float *mask;
    float objectness;
    int sort_class;
    float *uc;
    int points;
    float *embeddings;
    int embedding_size;
    float sim;
    int track_id;
};

struct detection_with_class {
    detection det;
    // The most probable class id: the best class index in this->prob.
    // Is filled temporary when processing results, otherwise not initialized
    int best_class;
};

struct boxabs {
    float left, right, top, bot;
};

class NetDetect
{
public:
    NetDetect(const char *file, int fpgaNetw, int fpgaNeth) noexcept;
    ~NetDetect();

public:
    void parserNetCfg(const char *file);
    BoxInfo *getDetectBoxInfo(char *pData, int src_w, int src_h, int &num);

private:
    inline int getIndex(int n, int loc, int layer_size, int entry) { return n * layer_size * (4 + classNum + 1) + entry * layer_size + loc; }
    inline int getIndex2(int n, int loc, int full_channel, int one_channel, int index) { return loc * full_channel + n * one_channel + index; }
    inline box getYoloBox(float *x, int n, int index, int i, int j, int lw, int lh, int stride)
    { // pData, layer.mask[n], box_index, col, row, layer.net_w,
        // layer.net_h, signle_chanel_size
        box b;
        b.x = (i + x[index + 0 * stride]) / lw;
        b.y = (j + x[index + 1 * stride]) / lh;
        b.w = exp(x[index + 2 * stride]) * anchorArr[2 * n] / fpgaNetw;
        b.h = exp(x[index + 3 * stride]) * anchorArr[2 * n + 1] / fpgaNeth;
        return b;
    }

    inline box getYoloBox2(float *x, int n, int index, int col, int row, int lw, int lh)
    { // pData, layer.mask[n], box_index, col, row, layer.net_w,
        // layer.net_h, signle_chanel_size
        box b;
        b.x = (col + x[index]) / lw;
        b.y = (row + x[index + 1]) / lh;
        float wtemp = x[index + 2];
        wtemp = wtemp / (1.0 - wtemp);
        b.w = wtemp * anchorArr[2 * n] / fpgaNetw;
        float htemp = x[index + 3];
        htemp = htemp / (1.0 - htemp);
        b.h = htemp * anchorArr[2 * n + 1] / fpgaNeth;
        return b;
    }

private:
    detection *makeNetworkBoxes(float thresh, int &num);
    void fillNetworkBoxes(int src_w, int src_h, float thresh, float hier, detection *dets);
    void correctYoloBoxes(detection *dets, int n, int src_w, int src_h);
    void diounms_sort(detection *dets, int total, float nms_thresh, NMSKIND nms_kind, float beta1);
    void do_nms_sort(detection *dets, int total, float nms_thresh);
    detection_with_class *get_actual_detections(detection *dets, int dets_num, float thresh, int &selected_detections_num);

private:
    int maxBoxNum = 100;
    float thresh = .1;
    float iou = .5;
    int classNum = 80;
    int fpgaNetw = 416;
    int fpgaNeth = 416;
    BoxInfo *boxArr = nullptr;
    int *anchorArr = nullptr;
    char *inputData = nullptr;
    detection *detectArr = nullptr;
    NMSKIND nmsType = DEFAULT_NMS;
    float betaNms = 0.6;
    std::vector<YoloLayer> vecLayer;
    std::map<std::string, std::map<std::string, std::string>> mpNetData;
};
