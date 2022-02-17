/*
 * @Author: xtt
 * @Date: 2021-05-17 20:12:33
 * @Description: ...
 * @LastEditTime: 2022-02-15 16:03:16
 */
#include "semaphore.h"
#include "xdmaApi.h"
#ifdef _UNIX
#include <dirent.h>
#include <dlfcn.h>
#include <fcntl.h>
#else
#include <filesystem>
#endif
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <unistd.h>
#include <sys/time.h>

using namespace NNC_InterfaceControlDocument;

inline uint64_t get_current_Mtime()
{
        using namespace std;

        timeval tv;
        gettimeofday(&tv, nullptr);

        //tv.tv_sec 秒
        //tv.tv_sec*1000+tv.tv_usec/1000 毫秒
        //tv.tv_sec*1000000+tv.tv_usec 微秒

        uint64_t time = tv.tv_sec * 1000 + tv.tv_usec / 1000;

        return time;
}

static std::vector<std::string>
read_dir_file(const char *dir, const char *file_suffix)
{
#ifdef _UNIX
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(dir)) == NULL) {
        fprintf(stderr, "dir not exit:%s", dir);
        assert(false);
    }
    std::string dir_str(dir);
    if (dir_str[dir_str.size() - 1] == '/') {
        dir_str = dir_str.substr(0, dir_str.size() - 1);
    }
    std::vector<std::string> res;
    res.reserve(100);
    while ((dirp = readdir(dp)) != NULL) {
        if (DT_REG == dirp->d_type) {
            std::string file = dirp->d_name;
            auto pos2 = file.rfind('.');
            auto suffix = file.substr(pos2 + 1, file.size() - pos2 - 1);
            if (file_suffix == suffix) {
                res.push_back(dir_str + "/" + file);
            }
        }
    }
    return res;
#else
    std::filesystem::directory_entry filedir(dir);
    std::filesystem::directory_iterator filelist(dir);
    std::vector<std::string> vec;
    vec.reserve(100);
    for (auto &file : filelist) {
        vec.push_back(file.path().string());
    }
    return vec;
#endif
}

std::vector<std::string> string_split(const std::string &str, const char splitStr)
{
    std::vector<std::string> vecStr;
    std::string::size_type beginPos = str.find_first_not_of(splitStr, 0);
    std::string::size_type endPos = str.find_first_of(splitStr, beginPos);
    while (std::string::npos != endPos || std::string::npos != beginPos) {
        vecStr.push_back(str.substr(beginPos, endPos - beginPos));
        beginPos = str.find_first_not_of(splitStr, endPos);
        endPos = str.find_first_of(splitStr, beginPos);
    }

    return vecStr;
}

bool sort_by_custom(const std::string &a, const std::string &b)
{
    int pos1 = a.find_last_of('/');
    std::string name_a = a.substr(pos1 + 1, a.size() - pos1 - 4);
    std::string name_b = b.substr(pos1 + 1, b.size() - pos1 - 4);

    auto vec_a = string_split(name_a, '_');
    auto vec_b = string_split(name_b, '_');

    int num_a_1 = std::stoi(vec_a[1]);
    int num_b_1 = std::stoi(vec_b[1]);

    int num_a_2 = std::stoi(vec_a[2]);
    int num_b_2 = std::stoi(vec_b[2]);

    if (num_a_2 == num_b_2) {
        return num_a_1 < num_b_1;
    } else {
        return num_a_2 < num_b_2;
    }
}

void pic_process(FpgaInput *pInput, const char *file)
{
    int dst_height = 416;
    int dst_width = dst_height;

    // inputData.data = pInput->data;
    // cv::Mat inputData;
    // try
    // {
    cv::Mat inputData = cv::imread(file, cv::ImreadModes::IMREAD_COLOR);

    //     cv::Mat src_res(608, 608, CV_8UC3, cv::Scalar(0, 0, 0));
    //     cv::resize(inputData,src_res,cv::Size(608, 608), cv::InterpolationFlags::INTER_LINEAR)

    int height = pInput->height = inputData.rows;
    int width = pInput->width = inputData.cols;

    float ratio1 = (float)dst_height / (float)height;
    float ratio2 = (float)dst_width / (float)width;
    float ratio = ratio1 < ratio2 ? ratio1 : ratio2;

    int real_height = ratio * height;
    int real_width = ratio * width;

    cv::Mat resData(real_height, real_width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::resize(inputData, resData, cv::Size(real_width, real_height), cv::InterpolationFlags::INTER_LINEAR);

    int begin_h = (dst_height - real_height) / 2;
    int begin_w = (dst_width - real_width) / 2;
    int end_h = real_height + begin_h;
    int end_w = real_width + begin_w;

    auto rgb_len = dst_height * dst_width * 4;
    unsigned char *pRes = new unsigned char[rgb_len];
    memset(pRes, 0, rgb_len);
    unsigned char *pRgb = (unsigned char *)resData.data;
    for (int i = begin_h; i < end_h; i++) {
        for (int j = begin_w; j < end_w; j++) {
            int pos = (i * dst_width + j) * 4;
            int pos2 = (i - begin_h) * resData.step + (j - begin_w) * resData.channels();

            pRes[pos] = pRgb[pos2 + 2];
            pRes[pos + 1] = pRgb[pos2 + 1];
            pRes[pos + 2] = pRgb[pos2];

            // memcpy(pRes + pos, pRgb + pos2, 3);

            // pRes[pos] = pRgb[pos2];
            // pRes[pos + 1] = pRgb[pos2 + 1];
            // pRes[pos + 2] = pRgb[pos2 + 2];
        }
    }
    resData.release();

    pInput->name = file;
    pInput->data = pRes;
    pInput->len = rgb_len;
}

void drawbox(cv::Mat &imgdata, void *_pBox, int boxNum)
{
    BoxInfo *pBox = (BoxInfo *)_pBox;
    for (int num = 0; num < boxNum; num++) {
        BoxInfo *pBoxData = pBox + num;

        int x1 = pBoxData->left;
        int y1 = pBoxData->top;
        int x2 = pBoxData->right;
        int y2 = pBoxData->bottom;

        float score = pBoxData->score;

        cv::Rect r(x1, y1, x2 - x1, y2 - y1);
        cv::rectangle(imgdata, r, cv::Scalar(0, 255, 203), 1);

        /*std::cout << "---------------draw box :" << x1 << " " << y1 << " " << x2 - x1 << " " << y2 - y1 << " " << std::endl;*/

        // int fontFace = CV_FONT_HERSHEY_SIMPLEX;
        int fontFace = 1;
        double fontScale = 1;
        int thickness = 1;
        int baseline = 0;

        cv::Size textSize = cv::getTextSize(std::string("unknow") + std::to_string(score), fontFace, fontScale, thickness, &baseline);
        cv::rectangle(imgdata, cv::Point(x1, y1), cv::Point(x1 + textSize.width + thickness, y1 - textSize.height - 4), cv::Scalar(0, 255, 203), -1);
        cv::Point txt_org(x1, y1 - 4);
        cv::putText(imgdata, std::string("unknow") + " " + std::to_string(score), txt_org, fontFace, fontScale, cv::Scalar(0, 0, 0));
    }
}

void callback(FpgaInput *input, BoxInfo *pbox, int num, void *)
{
    //fprintf(stderr, "fpga call back,box num:%d\n", num);
    static uint64_t tot_num = 0;
    tot_num++;
    if (tot_num % 30 == 0) {
        fprintf(stderr, "fpga call back,pic num:%d\n", tot_num);
    }
#if 1

    std::string &pic_file_name = input->name;

    static int pos1 = pic_file_name.find_last_of('/');
    static int pos2 = pic_file_name.find_last_of('.');
    std::string fileTemp = pic_file_name.substr(pos1 + 1, pic_file_name.length() - pos1 - 1);
    for (int i = 0; i < num; i++) {
        BoxInfo &box = pbox[i];
        std::ostringstream ostr;
        ostr << fileTemp << " "
             << "unkonw"
             << " " << box.score << " "
             << box.left << " " << box.top << " " << box.right << " " << box.bottom;

        std::cout << ostr.rdbuf()->str() << std::endl;
    }
#endif

#if 0
    auto cstr = "out/" + fileTemp;
    cv::Mat im = cv::imread(pic_file_name);
    drawbox(im, pbox, num);
    bool res = cv::imwrite(cstr, im);
    if (!res) {
        fprintf(stderr, "## save file fail : %s\n", cstr.c_str());
    }
#endif

#if 0
    {
        cv::Mat im = cv::imread(input->name, cv::IMREAD_COLOR);
        drawbox(im, pbox, num);

        cv::Mat resize_mat(608, 608, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::resize(im, resize_mat, cv::Size(608, 608), cv::InterpolationFlags::INTER_LINEAR);

        cv::imshow("win", resize_mat);
        cv::waitKey(40);
    }
#endif

    delete[] input->data;
    delete input;
}

static void call_back2v7(OutParameterAllInOne *output, InParameterAllInOne *input, void *arg)
{
	static uint64_t picNum=0;
    static auto t1 = get_current_Mtime();
    picNum++;
    if (picNum % 30 == 0 && picNum != 30) {
        auto t2 = get_current_Mtime();
        auto fps = 30.f / (t2 - t1) * 1000;
        std::cout << "--------------fps:" << 30.f / (t2 - t1) * 1000 << std::endl;
        t1 = t2;
    }
	delete input;
}
#include <stdio.h>
#include <thread>
static void push_2()
{
    for (;;) {
			cv::Mat frame=cv::imread("vlcsnap.jpg",cv::IMREAD_COLOR);
			cv::Mat frame_rgba;
			
			cv::cvtColor(frame,frame_rgba,cv::COLOR_BGR2RGBA);

			InParameterAllInOne *pInput = new InParameterAllInOne;
			pInput->idStreamVideo = 0;
        	pInput->currentFrame = frame_rgba;
			pInput->nncStateModel = get_fpga_status();
			
			push_data_slave(pInput);
			usleep(160*1000);
    }
}

int main(int argc, char *argv[])
{
    // auto &&vec = read_dir_file(argv[1], "jpg");
    // std::sort(vec.begin(), vec.end(), sort_by_custom);
    fprintf(stderr,"fpga open\n");
    bool ret = fpga_open();
    if (!ret)
        return 0;
    fprintf(stderr,"fpga init\n");
    set_callback(call_back2v7, NULL);
    fpga_start();
    fprintf(stderr,"fpga start\n");
	// FILE *fd = fopen("/home/sl/input_2192_2752_4_u8_2.bin", "rb");
    // if (fd == NULL) {
    //     return NULL;
    // }
    // fseek(fd, 0, SEEK_END);
    // long len = ftell(fd);
    // fseek(fd, 0, SEEK_SET);

    // char *buffer = new char[len];
	// assert(len==2192*2752*4);
    // fread(buffer, 1, len, fd);

    // fclose(fd);	
	std::thread* th=new std::thread(push_2);
	th->detach();
    for (;;) {
			cv::Mat frame=cv::imread("vlcsnap.jpg",cv::IMREAD_COLOR);
			cv::Mat frame_rgba;
			
			cv::cvtColor(frame,frame_rgba,cv::COLOR_BGR2RGBA);

			InParameterAllInOne *pInput = new InParameterAllInOne;
			pInput->idStreamVideo = 0;
        	pInput->currentFrame = frame_rgba;
			pInput->nncStateModel = get_fpga_status();
			
			push_data(pInput);
			usleep(150*1000);
    }
	sleep(10000);
    return 0;
}
