#pragma once
#define _USE_MATH_DEFINES

#include <fstream>
#include <iostream>
#include <math.h>
// #include <io.h>
// #include <direct.h>
#include "opencv2/opencv.hpp"

namespace NNC_CommonStruct
{
    /*----------------------------
	 * 板卡状态
	 *----------------------------
	 */
    enum NNC_StateModel {
        MODEL_STATE_NNC_OFFLINE = 0,
        MODEL_STATE_NNC_NORMAL = 1,
        MODEL_STATE_NNC_FREE = 2,
        MODEL_STATE_NNC_ERROR = 99,
        MODEL_STATE_NNC_XXXXXX = 999,
        // 其他状态可根据情况自定义
    };

    /*----------------------------
	 * 记录时间结构体
	 *----------------------------
	 */
    struct NNC_Timer {
        double year;
        double mouth;
        double day;

        long double absTime;
        long double relTime;
        // 其他状态可根据情况自定义
    };

    /*----------------------------
	 * 感受野位置设置结构体
	 *----------------------------
	 */
    struct NNC_ReceptiveField {
        int layer;    // 确定取哪层特征
        int widthRf;  // 确定取特征的宽度
        int heightRf; // 确定取特征的高度
    };

    /*----------------------------
	 * 检测到目标框结构体
	 *----------------------------
	 */
    struct NNC_DetectBox {
        float xpointLT;  // 框左上点像素x坐标，原图坐标系
        float ypointLT;  // 框左上点像素y坐标，原图坐标系
        float widthBox;  // 框宽度，原图坐标系
        float heightBox; // 框高度，原图坐标系
    };

    /*----------------------------
	 * 检测目标属性结构体
	 *----------------------------
	 */
    struct NNC_Detection {
        NNC_DetectBox detectBox; // 检测到目标框位置大小
        float confidence;        // 目标置信度
        int category;            // 目标种类
        float feature[128];      // 目标特征
    };
} // namespace NNC_CommonStruct

/*----------------------------
 * 板卡与CMU的接口，某一时刻
 *----------------------------
 */
namespace NNC_InterfaceControlDocument
{
    /*----------------------------
	 * 板卡输入数据结构体，某一时刻
	 *----------------------------
	 */
    struct InParameterAllInOne {
        int idStreamVideo;                                   // 视频源ID
        cv::Mat currentFrame;                                // 当前视频帧
        NNC_CommonStruct::NNC_StateModel nncStateModel;      // 当前板卡状态
        NNC_CommonStruct::NNC_Timer nncTimer;                // 当前时间，包括相对时间和绝对时间
        NNC_CommonStruct::NNC_ReceptiveField receptiveField; // 设置感受野位置
    };

    /*----------------------------
	 * 板卡输出数据结构体，某一时刻
	 *----------------------------
	 */
    struct OutParameterAllInOne {
        int idStreamVideo;                              // 视频源ID
        cv::Mat currentFrame;                           // 当前视频帧
        NNC_CommonStruct::NNC_StateModel nncStateModel; // 当前板卡状态
        NNC_CommonStruct::NNC_Timer nncTimer;           // 当前时间，包括相对时间和绝对时间

        std::vector<NNC_CommonStruct::NNC_Detection> detectionVec; // 检测目标列表
    };
} // namespace NNC_InterfaceControlDocument
