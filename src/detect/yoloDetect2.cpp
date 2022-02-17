/*
 * @Author: xtt
 * @Date: 2022-02-05 19:47:14
 * @Description: ...
 * @LastEditTime: 2022-02-11 17:22:30
 */
#include "yoloDetect2.h"
#include <algorithm>
#include <fstream>
#include <numeric>
#include <sstream>

namespace
{
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

    std::vector<std::string> split_string(const std::string &str, const char splitStr)
    {
        std::vector<std::string> vecStr;
        std::string::size_type beginPos = str.find_first_not_of(splitStr, 0);  //查找起始位置，即第一个不是分隔符的字符的位置
        std::string::size_type endPos = str.find_first_of(splitStr, beginPos); //根据起始位置，查找第一个分隔符位置　
        while (std::string::npos != endPos || std::string::npos != beginPos) { //开始查找，未找到返回npos,此处条件考虑到尾部结束为分割符的情况以及头部开始就是分隔符并且只有这一个分割符的情况
            vecStr.push_back(str.substr(beginPos, endPos - beginPos));         //从指定位置截断指定长度的字符串，str本身不变
            beginPos = str.find_first_not_of(splitStr, endPos);                //再从上次截断地方开始，查找下一个不是分隔符的起始位置
            endPos = str.find_first_of(splitStr, beginPos);                    //再次开始从指定位置查找下一个分隔符位置
        }

        return vecStr;
    }
} // namespace

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

template <class T>
T stringToNum(const std::string &str)
{
    T data;
    std::istringstream ss(str);
    ss >> data;
    return data;
}

NetDetect::NetDetect(const char *file, int _fpgaNetw, int _fpgaNeth)
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

    size_t box_valid_tot = 0;
    size_t box_score_valid_tot = 0;
    size_t box_class_valid_tot = 0;

    for (auto ite = mpNetData.cbegin(); ite != mpNetData.cend(); ite++) {
        if (ite->first.find(YOLO) != std::string::npos) {
            YoloLayer layer;
            layer.net_h = stringToNum<int>(ite->second.at(NET_H));
            layer.net_w = stringToNum<int>(ite->second.at(NET_W));
            layer.net_c = stringToNum<int>(ite->second.at(NET_C));
            layer.output_size = stringToNum<int>(ite->second.at(OUTPUT_SIZE));
            auto vec = split_string(ite->second.at(MASK), ',');

            layer.mask = new int[vec.size()];
            layer.anchors = new int[vec.size() * 2];
            layer.anchor_num = vec.size();

            layer.box = new float[layer.net_h * layer.net_w * layer.anchor_num * 4];
            layer.box_scores = new float[layer.net_h * layer.net_w * layer.anchor_num * classNum];

            box_valid_tot += layer.net_h * layer.net_w * layer.anchor_num * 4;
            box_score_valid_tot += layer.net_h * layer.net_w * layer.anchor_num * classNum;
            box_class_valid_tot += layer.net_h * layer.net_w * layer.anchor_num * classNum;

            for (size_t i = 0; i < vec.size(); i++) {
                layer.mask[i] = stringToNum<int>(vec[i]);
                layer.anchors[i * 2] = anchorArr[layer.mask[i] * 2];
                layer.anchors[i * 2 + 1] = anchorArr[layer.mask[i] * 2 + 1];
            }
            vecLayer.push_back(layer);
        }
    }
    pBoxValid = new float[box_valid_tot];
    pBoxScoresValid = new float[box_score_valid_tot];
    pClassValid = new int[box_class_valid_tot];
}

NetDetect::~NetDetect()
{
    delete[] boxArr;
    delete[] anchorArr;
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

BoxInfo *NetDetect::getDetectBoxInfo(const char *pData, int src_w, int src_h, int &num)
{
    static int file_name = 0;
    size_t offset = 0;
    for (size_t i = 0; i < vecLayer.size(); ++i) {
        YoloLayer &layer = vecLayer[i];
        yoloHead(pData + offset, vecLayer[i], src_w, src_h);
        offset += layer.output_size;

        //save_binary_file((std::string("/media/xtt/hdd/zchw/test/box64_") + std::to_string(file_name) + "_" + std::to_string(i) + ".bin").c_str(), layer.box, layer.net_h * layer.net_w * layer.anchor_num * 4 * 4);
        //save_binary_file((std::string("/media/xtt/hdd/zchw/test/score64_") + std::to_string(file_name) + "_" + std::to_string(i) + ".bin").c_str(), layer.box_scores, layer.net_h * layer.net_w * layer.anchor_num * classNum * 4);
    }
    //file_name++;
    {
        boxValidNum = 0;
        size_t pos = 0;
        for (int cls = 0; cls < classNum; ++cls) {
            for (size_t i = 0; i < vecLayer.size(); ++i) {
                YoloLayer layer = vecLayer[i];
                const int bx_num = layer.net_h * layer.net_w * layer.anchor_num;
                for (int bx = 0; bx < bx_num; ++bx) {
                    if (layer.box_scores[bx * classNum + cls] >= thresh) {
                        memcpy(pBoxValid + pos * 4, layer.box + bx * 4, 4 * 4);
                        pBoxScoresValid[pos] = layer.box_scores[bx * classNum + cls];
                        pClassValid[pos] = cls;
                        boxValidNum++;
                        pos++;
                    }
                }
            }
        }
    }

    //nms
    {
        if (boxValidNum > maxBoxNum) {
            LOG_WARN("box num too more: %d", boxValidNum);
        }
        std::vector<int> vec_sorted_index(boxValidNum);
        std::iota(vec_sorted_index.begin(), vec_sorted_index.begin() + boxValidNum, 0);
        std::sort(vec_sorted_index.begin(), vec_sorted_index.begin() + boxValidNum, [&](int index1, int index2) { return (pBoxScoresValid[index1] > pBoxScoresValid[index2]); });

        std::vector<int> vec_selected_index;
        vec_selected_index.reserve(10);
        for (int i = 0; i < boxValidNum; ++i) {
            int index = vec_sorted_index[i];
            bool keep = true;
            for (int j = 0; j < vec_selected_index.size(); ++j) {
                if (keep) {
                    int index2 = vec_selected_index[j];

                    BoxInfo b1(pBoxValid[index * 4], pBoxValid[index * 4 + 1], pBoxValid[index * 4 + 2], pBoxValid[index * 4 + 3]);
                    BoxInfo b2(pBoxValid[index2 * 4], pBoxValid[index2 * 4 + 1], pBoxValid[index2 * 4 + 2], pBoxValid[index2 * 4 + 3]);

                    float b_iou = calIou(b1, b2);
                    keep = (b_iou <= iou ? true : false);
                } else {
                    break;
                }
            }
            if (keep) {
                vec_selected_index.push_back(index);
            }
        }
        num = vec_selected_index.size();
        BoxInfo *temp = boxArr;
        size_t valid_num = 0;
        for (auto index : vec_selected_index) {
            BoxInfo box(pBoxValid[index * 4], pBoxValid[index * 4 + 1], pBoxValid[index * 4 + 2], pBoxValid[index * 4 + 3]);
            box.classNo = pClassValid[index];
            box.score = pBoxScoresValid[index];
            *temp++ = box;

            LOG_INFO("box info: left :%f, top:%f, right:%f,bottom:%f ,class:%d, score:%f",
                     box.left, box.top, box.right, box.bottom, box.classNo, box.score);

            valid_num++;
            if (valid_num > maxBoxNum) {
                num = maxBoxNum;
                break;
            }
        }
    }
    return boxArr;
}

void NetDetect::yoloHead(const char *pData, YoloLayer &layer, int src_w, int src_h)
{
    //hwc格式数据，x,y,w,h,框置信度,类别置信度,类别置信度,类别置信度......，依次顺序，然后重复anchor次数
    float *pSrcData = (float *)const_cast<char *>(pData);

    float ratio_h = (float)fpgaNeth / src_h;
    float ratio_w = (float)fpgaNetw / src_w;
    float ratio = std::min(ratio_h, ratio_w);

    int new_h = src_w * ratio;
    int new_w = src_h * ratio;

    int offset_h = ((float)fpgaNeth - new_h) / 2.f / fpgaNeth;
    int offset_w = ((float)fpgaNetw - new_w) / 2.f / fpgaNetw;

    size_t index = 0;
    size_t index_box = 0;
    size_t index_box_scores = 0;

    for (int32_t grid_height = 0; grid_height < layer.net_h; ++grid_height) {
        for (int32_t grid_width = 0; grid_width < layer.net_w; ++grid_width) {
            for (int32_t grid_anchor = 0; grid_anchor < layer.anchor_num; ++grid_anchor) {
                pSrcData[index] = ((pSrcData[index] + grid_width) / layer.net_w - offset_w) * ratio;
                pSrcData[index + 1] = ((pSrcData[index + 1] + grid_height) / layer.net_h - offset_h) * ratio;

                pSrcData[index + 2] = (pSrcData[index + 2] / (1 - pSrcData[index + 2]) * layer.anchors[grid_anchor * 2] / fpgaNetw) * ratio;
                pSrcData[index + 3] = (pSrcData[index + 3] / (1 - pSrcData[index + 3]) * layer.anchors[grid_anchor * 2 + 1] / fpgaNeth) * ratio;

                layer.box[index_box++] = std::max((pSrcData[index + 1] - (pSrcData[index + 3] / 2.)) * src_h, 0.0); //y_left
                layer.box[index_box++] = std::max((pSrcData[index] - (pSrcData[index + 2] / 2.)) * src_w, 0.0);     //x_left
                layer.box[index_box++] = std::max((pSrcData[index + 1] + (pSrcData[index + 3] / 2.)) * src_h, 0.0); //y_right
                layer.box[index_box++] = std::max((pSrcData[index] + (pSrcData[index + 2] / 2.)) * src_w, 0.0);     //x_right

                for (int i = 0; i < classNum; ++i) {
                    layer.box_scores[index_box_scores++] = pSrcData[index + 4] * pSrcData[index + 5 + i]; //框置信度*类别置信度
                }

                index += layer.net_c / 3;
            }
            index += 13;
        }
    }
}

float NetDetect::calIou(BoxInfo &box1, BoxInfo &box2)
{
    float box1_area = (box1.right - box1.left) * (box1.bottom - box1.top);
    float box2_area = (box2.right - box2.left) * (box2.bottom - box2.top);

    float x_min = std::max(box1.left, box2.left);
    float x_max = std::min(box1.right, box2.right);
    float y_min = std::max(box1.top, box2.top);
    float y_max = std::min(box1.bottom, box2.bottom);

    float inter_area = (x_max - x_min + 1) * (y_max - y_min + 1);

    return inter_area / (box1_area + box2_area - inter_area);
}

// void NetDetect::yoloCorrectBoxes(const char *pData, const YoloLayer &layer, int src_w, int src_h)
// {
//     float ratio_h = (float)fpgaNeth / src_h;
//     float ratio_w = (float)fpgaNetw / src_w;
//     float ratio = std::min(ratio_h, ratio_w);

//     int new_h = src_w * ratio;
//     int new_w = src_h * ratio;

//     int offset_h = ((float)fpgaNeth - new_h) / 2.f / fpgaNeth;
//     int offset_w = ((float)fpgaNetw - new_w) / 2.f / fpgaNetw;

//     float *pSrcData = (float *)const_cast<char *>(pData);
//     size_t index = 0;
//     size_t index_box = 0;
//     size_t index_box_scores = 0;
//     for (int32_t grid_height = 0; grid_height < layer.net_h; ++grid_height) {
//         for (int32_t grid_width = 0; grid_width < layer.net_w; ++grid_width) {
//             for (int32_t grid_anchor = 0; grid_anchor < layer.anchor_num; ++grid_anchor) {
//                 pSrcData[index] = (pSrcData[index] - offset_w) * ratio;         //x
//                 pSrcData[index + 1] = (pSrcData[index + 1] - offset_h) * ratio; //y

//                 pSrcData[index + 2] *= ratio; //w
//                 pSrcData[index + 3] *= ratio; //h

//                 layer.box[index_box++] = (pSrcData[index + 1] - (pSrcData[index + 3] / 2.)) * 512; //y_min
//                 layer.box[index_box++] = (pSrcData[index] - (pSrcData[index + 2] / 2.)) * 704;     //x_min
//                 layer.box[index_box++] = (pSrcData[index + 1] + (pSrcData[index + 3] / 2.)) * 512; //y_max
//                 layer.box[index_box++] = (pSrcData[index] + (pSrcData[index + 2] / 2.)) * 704;     //y_max

//                 for (int i = 0; i < classNum; ++i) {
//                     layer.box_scores[index_box_scores++] = pSrcData[index + 4] * pSrcData[index + 5 + i]; //框置信度*类别置信度
//                 }

//                 index += layer.net_c / 3;
//             }
//         }
//     }
// }