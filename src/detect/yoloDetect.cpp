/*
 * @Author: xtt
 * @Date: 2020-10-24 17:16:14
 * @Description: ...
 * @LastEditTime: 2022-02-11 17:20:50
 */
#include "yoloDetect.h"

#include <cstdlib>
#include <fstream>
#include <sstream>

#define YOLO_FPGA 23

namespace
{
    inline void rmStrSpace(std::string &str)
    {
        for (;;) {
            size_t pos = str.find_first_of(' ');
            if (pos != std::string::npos) {
                str.erase(pos, 1);
            } else
                break;
        }
    }

    inline std::vector<std::string> split_string(const std::string &str,
                                                 const char flag)
    {
        std::vector<std::string> vecStr;

        std::istringstream in(str);
        std::string temp;
        while (getline(in, temp, flag)) {
            vecStr.push_back(temp);
        }

        return vecStr;
    }

    template <class T>
    T stringToNum(const std::string &str)
    {
        T data;
        std::istringstream ss(str);
        ss >> data;
        return data;
    }

    inline int nms_comparator_v3(const void *pa, const void *pb)
    {
        detection a = *(detection *)pa;
        detection b = *(detection *)pb;
        float diff = 0;
        if (b.sort_class >= 0) {
            diff = a.prob[b.sort_class] -
                   b.prob[b.sort_class]; // there is already: prob = objectness*prob
        } else {
            diff = a.objectness - b.objectness;
        }
        if (diff < 0)
            return 1;
        else if (diff > 0)
            return -1;
        return 0;
    }

    inline float overlap(float x1, float w1, float x2, float w2)
    {
        float l1 = x1 - w1 / 2;
        float l2 = x2 - w2 / 2;
        float left = l1 > l2 ? l1 : l2;
        float r1 = x1 + w1 / 2;
        float r2 = x2 + w2 / 2;
        float right = r1 < r2 ? r1 : r2;
        return right - left;
    }
    inline float box_intersection(const box &a, const box &b)
    {
        float w = overlap(a.x, a.w, b.x, b.w);
        float h = overlap(a.y, a.h, b.y, b.h);
        if (w < 0 || h < 0)
            return 0;
        float area = w * h;
        return area;
    }

    inline float box_union(const box &a, const box &b)
    {
        float i = box_intersection(a, b);
        float u = a.w * a.h + b.w * b.h - i;
        return u;
    }

    inline float box_iou(const box &a, const box &b)
    {
        float I = box_intersection(a, b);
        float U = box_union(a, b);
        if (I == 0 || U == 0) {
            return 0;
        }
        return I / U;
    }

    inline float box_iou2(const box &a, const box &b)
    {
        float I = box_intersection(a, b);
        float U = box_union(a, b);
        if (I == 0 || U == 0) {
            return 0;
        }

        if (((I - a.w * a.h) < 0.00001 || (I - b.w * b.h) < 0.00001)) {
            return 1;
        }

        return I / U;
    }

    inline boxabs box_c(const box &a, const box &b)
    {
        boxabs ba = {0};
        ba.top = fmin(a.y - a.h / 2, b.y - b.h / 2);
        ba.bot = fmax(a.y + a.h / 2, b.y + b.h / 2);
        ba.left = fmin(a.x - a.w / 2, b.x - b.w / 2);
        ba.right = fmax(a.x + a.w / 2, b.x + b.w / 2);
        return ba;
    }

    inline float box_diounms(const box &a, const box &b, float beta1)
    {
        boxabs ba = box_c(a, b);
        float w = ba.right - ba.left;
        float h = ba.bot - ba.top;
        float c = w * w + h * h;
        float iou = box_iou(a, b);
        if (c == 0) {
            return iou;
        }
        float d = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
        float u = pow(d / c, beta1);
        float diou_term = u;
        return iou - diou_term;
    }

    inline int compare_by_lefts(const void *a_ptr, const void *b_ptr)
    {
        const detection_with_class *a = (detection_with_class *)a_ptr;
        const detection_with_class *b = (detection_with_class *)b_ptr;
        const float delta = (a->det.bbox.x - a->det.bbox.w / 2) - (b->det.bbox.x - b->det.bbox.w / 2);
        return delta < 0 ? -1 : delta > 0 ? 1 : 0;
    }

    inline int compare_by_probs(const void *a_ptr, const void *b_ptr)
    {
        const detection_with_class *a = (detection_with_class *)a_ptr;
        const detection_with_class *b = (detection_with_class *)b_ptr;
        float delta = a->det.prob[a->best_class] - b->det.prob[b->best_class];
        return delta < 0 ? -1 : delta > 0 ? 1 : 0;
    }

    const char *NET = "net";
    const char *YOLO = "yolo";
    const char *MASK = "mask";
    const char *ANCHORS = "anchors";
    const char *CLASS = "classes";
    const char *IOU = "iou";
    const char *THRESH = "thresh";
    const char *MAXBOX = "maxboxes";
    const char *NET_H = "net_h";
    const char *NET_W = "net_w";
    const char *NET_C = "net_c";
    const char *OUTPUT_SIZE = "output_size";
    const char *NMS_KIND = "nms_kind";
    const char *MAX_DELTA = "max_delta";
    const char *BETA_NMS = "beta_nms";

} // namespace

NetDetect::NetDetect(const char *file, int _fpgaNetw, int _fpgaNeth) noexcept
    : fpgaNetw(_fpgaNetw), fpgaNeth(_fpgaNeth)
{
    boxArr = new BoxInfo[maxBoxNum];
    parserNetCfg(file);

    thresh = stringToNum<float>(mpNetData[NET][THRESH]);
    iou = stringToNum<float>(mpNetData[NET][IOU]);
    classNum = stringToNum<int>(mpNetData[NET][CLASS]);
    maxBoxNum = stringToNum<int>(mpNetData[NET][MAXBOX]);
    std::string &anchorStr = mpNetData[NET][ANCHORS];

    LOG_INFO("thresh:%.3f,iou:%.3f,class:%d,maxBox:%d", thresh, iou, classNum,
             maxBoxNum);
    LOG_INFO("anchors:%s", anchorStr.c_str());

    auto vec = split_string(anchorStr, ',');
    anchorArr = new int[vec.size()];
    for (size_t i = 0; i < vec.size(); i++) {
        anchorArr[i] = stringToNum<int>(vec[i]);
    }

    for (auto ite = mpNetData.cbegin(); ite != mpNetData.cend(); ite++) {
        if (ite->first.find(YOLO) != std::string::npos) {
            YoloLayer layer;
            layer.net_h = stringToNum<int>(ite->second.at(NET_H));
            layer.net_w = stringToNum<int>(ite->second.at(NET_W));
            layer.net_c = stringToNum<int>(ite->second.at(NET_C));
            layer.output_size = stringToNum<int>(ite->second.at(OUTPUT_SIZE));
            auto vec = split_string(ite->second.at(MASK), ',');
            layer.grid_box_num = vec.size();
            layer.mask = new int[vec.size()];
            layer.anchors = new int[vec.size() * 2];

            for (size_t i = 0; i < vec.size(); i++) {
                layer.mask[i] = stringToNum<int>(vec[i]);
                layer.anchors[i * 2] = anchorArr[i * 2];
                layer.anchors[i * 2 + 1] = anchorArr[i * 2 + 1];
            }
            layer.beta_nms = stringToNum<float>(ite->second.at(BETA_NMS));
            if (ite->second.at(NMS_KIND) == "greedynms")
                layer.nms_kind = GREEDY_NMS;
            else if (ite->second.at(NMS_KIND) == "diounms")
                layer.nms_kind = DIOU_NMS;
            else if (ite->second.at(NMS_KIND) == "nms")
                layer.nms_kind = DEFAULT_NMS;
            nmsType = layer.nms_kind;
            betaNms = layer.beta_nms;
            vecLayer.push_back(layer);
        }
    }
}

NetDetect::~NetDetect()
{
    delete[] boxArr;
    delete[] anchorArr;
    if (detectArr)
        delete[] detectArr;
    for (int i = 0; i < vecLayer.size(); i++) {
        delete[] vecLayer[i].mask;
        delete[] vecLayer[i].anchors;
    }
}

void NetDetect::parserNetCfg(const char *file)
{
    std::ifstream in(file);

    FSERVO_CHECK(in.is_open());

    std::stringstream strStream;
    strStream << in.rdbuf();
    in.close();

    // std::string strContent=strStream.str();
    while (!strStream.eof() && !strStream.fail()) {
        std::string strLine;
        try {
            getline(strStream, strLine);
            if (strLine.empty())
                continue;

            static std::string key;
            static std::string val;

            rmStrSpace(strLine); //去除空格

            if (strLine.empty()) continue;
            if (strLine[0] == '#') continue; //跳过注释

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
                mpNetData[key][str1] = str2;
            }
        } catch (...) {
            LOG_ERR("parser cfg fail:%s in line:%s", file, strLine.c_str());
            exit(-1);
        }
    }
}

BoxInfo *NetDetect::getDetectBoxInfo(char *pData, int src_w, int src_h, int &num)
{
    // float thresh = thresh;
    float hier = iou;
    inputData = pData;

    detection *dets = makeNetworkBoxes(thresh, num);

    fillNetworkBoxes(src_w, src_h, thresh, hier, dets);

    if (nmsType == DEFAULT_NMS) {
        do_nms_sort(dets, num, hier);
    } else
        diounms_sort(dets, num, hier, nmsType, betaNms);

    int selected_detections_num;
    detection_with_class *selected_detections = get_actual_detections(dets, num, thresh, selected_detections_num);
    //到此处，就已经拿到所有框信息
    num = selected_detections_num;
    // BoxInfo2 *pInfo = new BoxInfo2[selected_detections_num];
    BoxInfo *pInfo = boxArr;
    for (int i = 0; i < selected_detections_num; i++) {
        pInfo[i].classNo = selected_detections[i].best_class;
        pInfo[i].score = selected_detections[i].det.prob[pInfo[i].classNo];

        box &b = selected_detections[i].det.bbox;

        int left = (b.x - b.w / 2.) * src_w;
        int right = (b.x + b.w / 2.) * src_w;
        int top = (b.y - b.h / 2.) * src_h;
        int bot = (b.y + b.h / 2.) * src_h;

        if (left < 0) left = 0;
        if (right > src_w - 1) right = src_w - 1;
        if (top < 0) top = 0;
        if (bot > src_h - 1) bot = src_h - 1;

        pInfo[i].left = left;
        pInfo[i].top = top;
        pInfo[i].right = right;
        pInfo[i].bottom = bot;

        LOG_DEBUG("detect res:class:%d,score:%.3f,left:%d,top:%d,right:%d,bot:%d",
                  pInfo[i].classNo, pInfo[i].score, left, top, right, bot);
    }
    delete[] selected_detections;
    return pInfo;
}

detection *NetDetect::makeNetworkBoxes(float thresh, int &num)
{
    float *pData = (float *)inputData;
    int boxNum = 0;
    for (int i = 0; i < vecLayer.size(); i++) {
        YoloLayer &layer = vecLayer[i];

        for (int n = 0; n < layer.grid_box_num; ++n) {
            const int signle_chanel_size = layer.net_w * layer.net_h;
            for (int i = 0; i < signle_chanel_size; ++i) {
                // int obj_index = n * signle_chanel_size * (1 + 4 + classNum) +
                //                 4 * signle_chanel_size;
#ifdef YOLO_FPGA
                int obj_index = getIndex2(n, i, layer.net_c, 85, 4);
#else
                int obj_index = getIndex(n, i, signle_chanel_size, 4);
#endif
                if (pData[obj_index] > thresh) {
                    boxNum++;
                }

                if (boxNum > maxBoxNum) {
                    LOG_WARN("box num is too more! current box num:%d", boxNum);
					exit(-1);
                }
            }
        }
        pData += layer.output_size >> 2;
    }

    boxNum = boxNum > maxBoxNum ? maxBoxNum : boxNum;
    num = boxNum;

    static int lastNum = boxNum;

    if (detectArr) {
        for (int i = 0; i < lastNum; i++) {
            delete[] detectArr[i].prob;
        }

        delete[] detectArr;
        detectArr = nullptr;
    }
    detectArr = new detection[boxNum];
    for (int i = 0; i < boxNum; i++) {
        detectArr[i].prob = new float[classNum];
    }
    lastNum = boxNum;
    return detectArr;
}

void NetDetect::fillNetworkBoxes(int src_w, int src_h, float thresh, float hier, detection *dets)
{
    float *pData = (float *)inputData;
    int count = 0;
    for (int i = 0; i < vecLayer.size(); i++) {
        YoloLayer &layer = vecLayer[i];

        const int signle_chanel_size = layer.net_h * layer.net_w;
        const int one_channel = 80 + 5;
        for (int j = 0; j < signle_chanel_size; j++) {
            int row = j / layer.net_w;
            int col = j % layer.net_h;
            for (int n = 0; n < layer.grid_box_num; n++) {
#ifdef YOLO_FPGA
                int obj_index = getIndex2(n, j, layer.net_c, one_channel, 4);
#else
                int obj_index = getIndex(n, j, signle_chanel_size, 4);
#endif

                float objectness = pData[obj_index];
                if (objectness > thresh && count < maxBoxNum) {
#ifdef YOLO_FPGA
                    int box_index = getIndex2(n, j, layer.net_c, one_channel, 0);
                    dets[count].bbox = getYoloBox2(pData, layer.mask[n], box_index, col,
                                                   row, layer.net_w, layer.net_h);
#else
                    int box_index = getIndex(n, j, signle_chanel_size, 0);
                    dets[count].bbox =
                        getYoloBox(pData, layer.mask[n], box_index, col, row, layer.net_w,
                                   layer.net_h, signle_chanel_size);
#endif

                    // dets[count].bbox = getYoloBox(pData, layer.mask[n], box_index, col,
                    // row, layer.net_w, layer.net_h, signle_chanel_size);
                    dets[count].objectness = objectness;
                    dets[count].classes = classNum;

                    for (int k = 0; k < classNum; k++) {
#ifdef YOLO_FPGA
                        int class_index =
                            getIndex2(n, j, layer.net_c, one_channel, 4 + 1 + k);
#else
                        int class_index = getIndex(n, j, signle_chanel_size, 4 + 1 + k);
#endif

                        float prob = objectness * pData[class_index];
                        dets[count].prob[k] = (prob > thresh) ? prob : 0;
                    }
                    ++count;
                }
            }
        }
        // correctYoloBoxes(dets, count, src_w, src_h);
        // dets += count;
        pData += layer.output_size >> 2;
    }
    correctYoloBoxes(dets, count, src_w, src_h);
}

void NetDetect::correctYoloBoxes(detection *dets, int n, int src_w, int src_h)
{
    int i;
    int new_w = fpgaNetw;
    int new_h = fpgaNeth;

    if (((float)fpgaNetw / src_w) < ((float)fpgaNeth / src_h)) {
        new_w = fpgaNetw;
        new_h = round((src_h * fpgaNetw) / src_w);
    } else {
        new_h = fpgaNeth;
        new_w = round((src_w * fpgaNeth) / src_h);
    }

    // difference between network width and "rotated" width
    float deltaw = fpgaNetw - new_w;
    // difference between network height and "rotated" height
    float deltah = fpgaNeth - new_h;
    // ratio between rotated network width and network width
    float ratiow = (float)new_w / fpgaNetw;
    // ratio between rotated network width and network width
    float ratioh = (float)new_h / fpgaNeth;
    for (i = 0; i < n; ++i) {
        box b = dets[i].bbox;
        b.x = (b.x - deltaw / 2. / fpgaNetw) / ratiow;
        b.y = (b.y - deltah / 2. / fpgaNeth) / ratioh;
        b.w *= 1 / ratiow;
        b.h *= 1 / ratioh;

        dets[i].bbox = b;
    }
}

void NetDetect::do_nms_sort(detection *dets, int total, float thresh)
{
    int i, j, k;
    k = total - 1;
    for (i = 0; i <= k; ++i) {
        if (dets[i].objectness == 0) {
            detection swap = dets[i];
            dets[i] = dets[k];
            dets[k] = swap;
            --k;
            --i;
        }
    }
    total = k + 1;

    for (k = 0; k < classNum; ++k) {
        for (i = 0; i < total; ++i) {
            dets[i].sort_class = k;
        }
        qsort(dets, total, sizeof(detection), nms_comparator_v3);
        for (i = 0; i < total; ++i) {
            // printf("  k = %d, \t i = %d \n", k, i);
            if (dets[i].prob[k] == 0)
                continue;
            box a = dets[i].bbox;
            for (j = i + 1; j < total; ++j) {
                box b = dets[j].bbox;
                if (box_iou2(a, b) > thresh) {
                    dets[j].prob[k] = 0;
                }
            }
        }
    }
}

void NetDetect::diounms_sort(detection *dets, int total, float thresh, NMSKIND nms_kind, float beta1)
{
    int i, j, k;
    k = total - 1;
    for (i = 0; i <= k; ++i) {
        if (dets[i].objectness == 0) {
            detection swap = dets[i];
            dets[i] = dets[k];
            dets[k] = swap;
            --k;
            --i;
        }
    }
    total = k + 1;

    for (k = 0; k < classNum; ++k) {
        for (i = 0; i < total; ++i) {
            dets[i].sort_class = k;
        }
        qsort(dets, total, sizeof(detection), nms_comparator_v3);
        for (i = 0; i < total; ++i) {
            if (dets[i].prob[k] == 0)
                continue;
            box a = dets[i].bbox;
            for (j = i + 1; j < total; ++j) {
                box b = dets[j].bbox;

                if (box_iou(a, b) > thresh && nms_kind == GREEDY_NMS) {
                    dets[j].prob[k] = 0;
                } else {
                    if (box_diounms(a, b, beta1) > thresh && nms_kind == DIOU_NMS) {
                        dets[j].prob[k] = 0;
                    }
                }
            }
        }
    }
}

detection_with_class *NetDetect::get_actual_detections(detection *dets, int dets_num, float thresh, int &selected_detections_num)
{
    int selected_num = 0;
    detection_with_class *result_arr = new detection_with_class[dets_num];
    int i;
    for (i = 0; i < dets_num; ++i) {
        int best_class = -1;
        float best_class_prob = thresh;
        int j;
        auto dd1 = dets[i].classes;
        for (j = 0; j < dets[i].classes; ++j) {
            if (dets[i].prob[j] > best_class_prob) {
                best_class = j;
                best_class_prob = dets[i].prob[j];
            }
        }
        if (best_class >= 0) {
            result_arr[selected_num].det = dets[i];
            result_arr[selected_num].best_class = best_class;
            ++selected_num;
        }
    }
    selected_detections_num = selected_num;
    return result_arr;
}