#include "fpgaControl.h"
#include "log.h"
#include "paramManager.h"
#include "xdma.h"

#ifdef _DOUBLE_V7
using namespace NNC_InterfaceControlDocument;
#include <opencv2/opencv.hpp>
#include "yoloDetect2.h"
#else
#include "yoloDetect.h"
#endif

#include <sys/time.h>
#include <unistd.h>
#include <inttypes.h>
#include <fstream>
#include <thread>

static constexpr int32_t FPS = 15;

static constexpr int32_t REAL_OUT_SIZE = 0x1ce000;

static char *readBinaryFile(const char *_file, int &length)
{
    char *pRes = nullptr;

    std::ifstream i_f_stream(_file, std::ifstream::binary);
    FSERVO_CHECK(i_f_stream.is_open());

    i_f_stream.seekg(0, i_f_stream.end);
    length = i_f_stream.tellg();
    i_f_stream.seekg(0, i_f_stream.beg);

    pRes = new char[length];
    i_f_stream.read(pRes, length);

    i_f_stream.close();

    return pRes;
}

// inline char *read_binary_file(const char *file, ssize_t *len)
// {
//     /******************************************************************************************************************************************************
// 	 	1	"r"		Opens a file for reading. The file must exist.
// 		2	"w"		Creates an empty file for writing. If a file with the same name already exists, its content is erased and the file is considered as a new empty file.
// 		3	"a"		Appends to a file. Writing operations, append data at the end of the file. The file is created if it does not exist.
// 		4	"r+"	Opens a file to update both reading and writing. The file must exist.
// 		5	"w+"  Creates an empty file for both reading and writing.
// 		6	"a+"   Opens a file for reading and appending.
// 	 * ****************************************************************************************************************************************************/
//     FILE *fd = fopen(file, "rb");
//     if (fd == NULL) {
//         return NULL;
//     }
//     fseek(fd, 0, SEEK_END);
//     *len = ftell(fd);
//     fseek(fd, 0, SEEK_SET);

//     char *buffer = new char[*len];
//     fread(buffer, 1, *len, fd);

//     fclose(fd);

//     return buffer;
// }

inline int save_binary_file(const char *file, const void *buffer, size_t len)
{
    FILE *fd = fopen(file, "w");
    size_t ret = fwrite(buffer, 1, len, fd);
    fclose(fd);

    return ret == len ? 0 : -1;
}

static std::vector<std::string> split_string(const std::string &str, const char splitStr)
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

FpgaParaCfg *FpgaControl::getFpgaPara() const
{
    return const_cast<FpgaParaCfg *>(&ParamManager::getInstance()->getFpgaCfg());
}

void FpgaControl::setFpgaPara(const FpgaParaCfg *pdata)
{
    pause = true;
    ParamManager::getInstance()->updateFpgaParam(pdata);
    for (int i = 0;; ++i) {
        if (DIE_NUM == i) i = 0;

        if (input_index[i] == output_index[i])
            break;
        LOG_INFO("wait fpga finish...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    for (int i = 0; i < DIE_NUM; ++i) {
        FSERVO_CHECK(que_input[i].empty());
    }
    FSERVO_CHECK(que_detect.empty());
    initParam();
    initIndex();
    for (int i = 0; i < DIE_NUM; ++i) {
        sem_input[i].init(FPGA_INPUT_BUFFER_NUM);
    }
    pause = false;
}

FpgaControl::FpgaControl() noexcept
{
    ParamManager::getInstance()->parserCfg("cfg/fservo.cfg");
    ParamManager::getInstance()->initParam();
}
#ifdef _DOUBLE_V7
NNC_CommonStruct::NNC_StateModel FpgaControl::getFpgaStatus()
{
    return nnc_state;
}
#endif
FpgaControl *FpgaControl::getInstance()
{
    static FpgaControl ins;
    return &ins;
}

bool FpgaControl::open()
{
#ifdef _UNIX
    dev_init_param_st dev;
    dev.mem_dev_name = "/dev/mem";
    dev.reg_dev_name = "/dev/xdma0_user";
    auto ret = xdma_open(dev);
#else
    auto ret = xdma_open();
#endif
    if (0 == ret) {
        status = FPGA_OPEN;
        return true;
    }
    return false;
}

void FpgaControl::close()
{
    stop();
    xdma_close();
}

void FpgaControl::start()
{
    if (is_stop == true) {
        init();

        is_stop = false;

        while (!que_detect.empty()) {
            delete[] que_detect.front().outData;
#ifdef _DOUBLE_V7
#else
            delete[]((FpgaInput *)que_detect.front().inputData)->data;
            delete que_detect.front().inputData;
#endif
            que_detect.pop();
        }

        for (int i = 0; i < DIE_NUM; ++i) {
            while (!que_input[i].empty()) {
                que_input[i].pop();
            }
        }

        thread_calculate = new std::thread(calculate_thread, this);
        thread_output = new std::thread(output_thread, this);
#if DIE_NUM >1
		thread_output2 = new std::thread(output_thread2, this);
#endif
    } else
        LOG_INFO("already started!");
}

void FpgaControl::stop()
{
    if (false == is_stop) {
        is_stop = true;

        sem_detect.post();
#ifdef _DOUBLE_V7
        sem_cal[0].post();
        sem_input[0].post();

        sem_cal[1].post();
        sem_input[1].post();
#else
        sem_cal.post();
        sem_input.post();
#endif
        if (thread_calculate && thread_output) {
            thread_output->join();
            thread_calculate->join();

            delete thread_calculate;
            delete thread_output;

            thread_calculate = nullptr;
            thread_output = nullptr;
        }
    }
}

void FpgaControl::init()
{
    // ParamManager::getInstance()->parserCfg("cfg/fservo.cfg");
    // ParamManager::getInstance()->initParam();
#ifdef _DOUBLE_V7
	int len=0;
	char* data=readBinaryFile("cfg/Slave_new3.bin",len);
	FSERVO_CHECK(len);

	xdma_write(0x0,data,len);
	xdma_write_reg(1,1);

	usleep(10);
	xdma_write_reg(1,0);
	usleep(1000*1000);

	//150reg 口变0表示成功
	for(;;){
		int32_t reg_150_data=1;
		xdma_read_reg(150,reg_150_data);
		if(reg_150_data==0) break;
		else{
			LOG_INFO("wait slave configure done");
		}
		usleep(100*1000);
	}

	xdma_write_reg(3,1);
	xdma_write_reg(13,0);
	xdma_write_reg(14,0X1B6757);
	xdma_write_reg(11,1);
	usleep(10);
	xdma_write_reg(11,0);
	
	for(;;){
		int32_t reg_151_data=0;
		xdma_read_reg(151,reg_151_data);
		if(reg_151_data==1) break;
		else{
			LOG_INFO("wait slave configure done");
		}
		usleep(100*1000);
	}
	LOG_INFO("slave configure done!");
	xdma_write_reg(3,0);
#endif

    *(const_cast<uint32_t *>(&FPGA_INPUT_DATA_SIZE)) = ParamManager::getInstance()->getFpgaCfg().inputBufferSize;
    *(const_cast<uint32_t *>(&FPGA_INPUT_OFFSET)) = ParamManager::getInstance()->getFpgaCfg().inputOffset;
    *(const_cast<uint32_t *>(&FPGA_INPUT_BUFFER_NUM)) = ParamManager::getInstance()->getFpgaCfg().inputBufferNum;
    *(const_cast<uint32_t *>(&FPGA_INPUT_REG)) = ParamManager::getInstance()->getFpgaCfg().inputReg;

    LOG_INFO("fpga input offset:0x%lx "
             "fpga input data size:0x%lx "
             "fpga input buffer num:%d "
             "fpga input reg:%d",
             FPGA_INPUT_OFFSET, FPGA_INPUT_DATA_SIZE, FPGA_INPUT_BUFFER_NUM, FPGA_INPUT_REG);

    initParam();
    initIndex();
}

void FpgaControl::initIndex()
{
#ifdef _DOUBLE_V7
    uint32_t out_reg = ParamManager::getInstance()->getFpgaCfg().outputReg;
    int32_t reg_data = 0;
    xdma_read_reg(out_reg, reg_data);
    start_index[0] = reg_data;
#if DIE_NUM > 1
    xdma_read_reg(FPGA_OUTPUT_REG2, reg_data);
    start_index[1] = reg_data;
#endif

#else
    uint32_t out_reg = ParamManager::getInstance()->getFpgaCfg().outputReg;
    int32_t reg_data = 0;
    xdma_read_reg(out_reg, reg_data);
    start_index = reg_data;
#endif

    for (int i = 0; i < DIE_NUM; ++i) {
        input_index[i] = 0;
        output_index[i] = 0;

        sem_input[i].init(FPGA_INPUT_BUFFER_NUM);
        sem_cal[i].init(0);
    }
    sem_detect.init(0);
}

void FpgaControl::initParam()
{
    const FpgaParaCfg &fpgaCfg = ParamManager::getInstance()->getFpgaCfg();
    std::vector<std::string> vecParam_files, vecParam_addrs;
    vecParam_files = split_string(fpgaCfg.paramFiles, ',');
    vecParam_addrs = split_string(fpgaCfg.paramAddress, ',');

    FSERVO_CHECK(vecParam_files.size() == vecParam_addrs.size());

    for (int i = 0; i < vecParam_files.size(); ++i) {
        const char *param_path = vecParam_files[i].c_str();
        int len = 0;
        char *param_buf = readBinaryFile(param_path, len);
#ifdef _UNIX
        xdma_write((void *)std::stoul(vecParam_addrs[i], nullptr, 16), param_buf, len);
#else
        xdma_write(param_buf, std::stoul(vecParam_addrs[i], nullptr, 16), len);
#endif

        LOG_INFO("ddr:0x%x; file:%s; len: %d",
                 stoul(vecParam_addrs[i], nullptr, 16), param_path, len);
#ifdef _DOUBLE_V7
		// LOG_INFO("write slave param start");
		xdma_write_reg(15, 0);
		usleep(10);
        xdma_write_reg(15, 1);
		xdma_write_reg(13, std::stoul(vecParam_addrs[i], nullptr, 16));
		xdma_write_reg(14, len/16);
		xdma_write_reg(10, 1);
		usleep(10);
        xdma_write_reg(10, 0);

		for (;;) {
            int32_t reg_data = 0;
            xdma_read_reg(207, reg_data);
            if (reg_data) break;
			else {
				// LOG_INFO("wait reg 207 val 1");
			}
            usleep(1000);
        }
		xdma_write_reg(15, 0);
		usleep(10);
        xdma_write_reg(15, 1);
        xdma_write_reg(11, 1);
		usleep(10);
        xdma_write_reg(11, 0);

		for (;;) {
            int32_t reg_data = 0;
            xdma_read_reg(207, reg_data);
            if (reg_data) break;
			else{
				// LOG_INFO("wait reg 207 val 1");
			}
            usleep(1000);
        }
		LOG_INFO("slave reg13:0x%x; reg14:%d done!",std::stoul(vecParam_addrs[i], nullptr, 16),len/16);
#endif

        if (param_buf)
            delete[] param_buf;
    }

    return;
#if 0
    //yolov4,双v7模式下,将第一块die的权重包头mish拷贝到第二块die

    LOG_INFO("slave push param start");

    uint32_t die2_data_addr[6] = {0x0, 0x1000000, 0x28000, 0X2C000, 0X30000, 0X34000};
    uint32_t die2_data_len[6] = {3744, 2381672, 1024, 1024, 1024, 1024};

    for (int i = 0; i < vecParam_files.size(); ++i) {
        xdma_write_reg(15, 0);
        xdma_write_reg(15, 1);
        xdma_write_reg(13, std::stoul(vecParam_addrs[i], nullptr, 16));
        xdma_write_reg(14, die2_data_len[i]);
        xdma_write_reg(10, 1);
        xdma_write_reg(10, 0);

        for (;;) {
            int32_t reg_data = 0;
            xdma_read_reg(207, reg_data);
            if (reg_data) break;
            usleep(1000);
        }

        xdma_write_reg(15, 0);
        xdma_write_reg(15, 1);
        xdma_write_reg(11, 1);
        xdma_write_reg(11, 0);

        for (;;) {
            int32_t reg_data = 0;
            xdma_read_reg(207, reg_data);
            if (reg_data) break;
            usleep(1000);
        }
    }

    LOG_INFO("die2 push param end");

#endif
}

void FpgaControl::setCallBack(pf_fpga_callback _pf, void *_para)
{
    pf_callback = _pf;
    pf_param = _para;
}
#ifdef _DOUBLE_V7
static uint8_t *pic_process(InParameterAllInOne *pInput, int &rgb_len)
{
    int height = pInput->currentFrame.rows;
    int width = pInput->currentFrame.cols;

    const static int dst_height = ParamManager::getInstance()->getFpgaCfg().inputNetHeight;
    const static int dst_width = ParamManager::getInstance()->getFpgaCfg().inputNetWidth;

    float ratio1 = (float)dst_height / (float)height;
    float ratio2 = (float)dst_width / (float)width;
    float ratio = ratio1 < ratio2 ? ratio1 : ratio2;

    int real_height = ratio * height;
    int real_width = ratio * width;

    cv::Mat resData(real_height, real_width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::resize(pInput->currentFrame, resData, cv::Size(real_width, real_height), cv::InterpolationFlags::INTER_LINEAR);

    int begin_h = (dst_height - real_height) / 2;
    int begin_w = (dst_width - real_width) / 2;
    int end_h = real_height + begin_h;
    int end_w = real_width + begin_w;

    rgb_len = dst_height * dst_width * 4;
    static unsigned char *pRes = new unsigned char[rgb_len];
    memset(pRes, 0, rgb_len);
    unsigned char *pRgb = (unsigned char *)resData.data;
    for (int i = begin_h; i < end_h; i++) {
        for (int j = begin_w; j < end_w; j++) {
            int pos = (i * dst_width + j) * 4;
            int pos2 = (i - begin_h) * resData.step + (j - begin_w) * resData.channels();

            // pRes[pos] = pRgb[pos2 + 2];
            // pRes[pos + 1] = pRgb[pos2 + 1];
            // pRes[pos + 2] = pRgb[pos2];-+

            memcpy(pRes + pos, pRgb + pos2, 3);

            // pRes[pos] = pRgb[pos2];
            // pRes[pos + 1] = pRgb[pos2 + 1];
            // pRes[pos + 2] = pRgb[pos2 + 2];
        }
    }
    resData.release();
    return pRes;
}
static OutParameterAllInOne *structor_output(InParameterAllInOne *pInput, BoxInfo *pbox, int num)
{
    static OutParameterAllInOne *res = new OutParameterAllInOne;
    res->idStreamVideo = pInput->idStreamVideo;
    res->currentFrame = pInput->currentFrame;
    res->nncStateModel = pInput->nncStateModel;
    res->nncTimer = pInput->nncTimer;
    if (res->detectionVec.empty())
        res->detectionVec.reserve(num);
    res->detectionVec.clear();

    static const int detect_w = ParamManager::getInstance()->getFpgaCfg().inputNetWidth;
    static const int detect_h = ParamManager::getInstance()->getFpgaCfg().inputNetHeight;

    const static float ratio_w = (float)res->currentFrame.cols / detect_w;
    const static float ratio_h = (float)res->currentFrame.rows / detect_h;

    for (int i = 0; i < num; ++i) {
        BoxInfo &box = pbox[i];

        box.left *= ratio_w;
        box.right *= ratio_w;
        box.top *= ratio_h;
        box.bottom *= ratio_h;

        NNC_CommonStruct::NNC_Detection temp;
        temp.detectBox = {(float)box.left, (float)box.top, (float)box.right - (float)box.left, (float)box.bottom - (float)box.top};
        temp.confidence = box.score;
        temp.category = box.classNo;
        res->detectionVec.push_back(temp);
    }

    return res;
}
#endif
bool FpgaControl::push(void *in_data)
{
    FSERVO_CHECK(FPGA_OPEN == status);

    if (pause)
        return false;
    if (is_stop)
        return false;

#if DIE_NUM > 1
    const uint64_t FPGA_INPUT_OFFSET_ARR[DIE_NUM] = {FPGA_INPUT_OFFSET, FPGA_INPUT_OFFSET2};
#else
    const uint64_t FPGA_INPUT_OFFSET_ARR[DIE_NUM] = {FPGA_INPUT_OFFSET};
#endif

	static unsigned char pic_param[16]={0x90,0x08,0xc0,0x0a,0xd1,0x45,0x1f,0x00,
                                                                        0x00,0x40,0x22,0x00,0x01,0x03,0x17,0x00};

    if (!FPGA_RESIZE_SCALE_X) {
        cv::Mat &frame = ((InParameterAllInOne *)in_data)->currentFrame;
        *(const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_X)) = ((float)frame.cols / ParamManager::getInstance()->getFpgaCfg().inputNetWidth) * (1 << 19);
        *(const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_Y)) = ((float)frame.rows / ParamManager::getInstance()->getFpgaCfg().inputNetHeight) * (1 << 19);

        uint16_t cols = frame.cols;
        uint16_t rows = frame.rows;

        memcpy(pic_param, &rows, sizeof(uint16_t));
        memcpy(pic_param + sizeof(uint16_t), &cols, sizeof(uint16_t));
        memcpy(pic_param + sizeof(uint16_t) * 2, const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_X), sizeof(uint32_t));
        memcpy(pic_param + sizeof(uint16_t) * 4, const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_Y), sizeof(uint32_t));
    }

#ifdef _DOUBLE_V7
#if 0
	static uint32_t i=0; 
    for (;; ++i) {
		if(i>=DIE_NUM) i=0;

        if (!sem_input[i].wait(10 * 1000)) continue;

        mutex_input[i].lock();

        uint64_t offset = FPGA_INPUT_OFFSET_ARR[i] + (uint32_t)(FPGA_INPUT_DATA_SIZE *
                                                                ((input_index[i] + start_index[i]) % FPGA_INPUT_BUFFER_NUM));

        cv::Mat frame = ((InParameterAllInOne *)in_data)->currentFrame;
        int rgb_len = frame.cols * frame.rows * frame.channels();
        uint8_t *rgb_data = frame.data;

        if (0 == i) //die 1
        {
			static uint32_t count=0;
            xdma_write_reg(13, offset);
            xdma_write_reg(14, rgb_len/16+1);

            xdma_write((void *)offset, pic_param, 16);
            xdma_write_with_reg((void *)(offset + 16), rgb_data, rgb_len, FPGA_INPUT_REG);

            que_input[i].push(in_data);

			LOG_INFO("master push done count:%d",++count);
        } else //die 2
        {
			static uint32_t push_count=0;
            xdma_write_reg(15, 0);
            usleep(1);
            xdma_write_reg(15, 1);
			// usleep(10);
            xdma_write_reg(12, 1);
			// usleep(10);
            xdma_write_reg(13, offset);
			// usleep(10);
            xdma_write_reg(14, rgb_len/16+1);

            // usleep(10);
            xdma_write_reg(10, 1);
			usleep(1);
            xdma_write_reg(10, 0);
            //读207
            // LOG_INFO("slave write reg 13 0x%lx,len:0x%lx", offset,rgb_len/16+1);
            uint32_t count = 0;
            for (;;) {
                count++;
                if (count > 100) {
                    LOG_ERR("slave wait 207 fail");
                    mutex_input[i].unlock();
                    return;
                }
                int32_t reg_data = 0;
                xdma_read_reg(207, reg_data);
                if (reg_data) break;
                usleep(1000);
            }
            xdma_write_reg(15, 0);
			usleep(1);
            xdma_write_reg(15, 1);

            xdma_write((void *)offset, pic_param, 16);
            xdma_write((void *)(offset + 16), rgb_data, rgb_len);

            que_input[i].push(in_data);

            xdma_write_reg(11, 1);
            usleep(1);
            xdma_write_reg(11, 0);

            LOG_INFO("slave push done count:%d",++push_count);
        }

        input_index[i]++;

        mutex_input[i].unlock();

        sem_cal[i].post();

		i++;
		
		break;
    }
#else
	const uint32_t i=0; 

	sem_input[i].wait();

	mutex_input[i].lock();

	uint64_t offset = FPGA_INPUT_OFFSET_ARR[i] + (uint32_t)(FPGA_INPUT_DATA_SIZE *
															((input_index[i] + start_index[i]) % FPGA_INPUT_BUFFER_NUM));

	cv::Mat frame = ((InParameterAllInOne *)in_data)->currentFrame;
	int rgb_len = frame.cols * frame.rows * frame.channels();
	uint8_t *rgb_data = frame.data;

	static uint32_t count=0;
	xdma_write_reg(13, offset);
	xdma_write_reg(14, rgb_len/16+1);
	// xdma_write_reg(14, 0x170301);

	xdma_write((void *)offset, pic_param, 16);
	xdma_write_with_reg((void *)(offset + 16), rgb_data, rgb_len, FPGA_INPUT_REG);

	// static int len=0;
	// static char* data2192=readBinaryFile("/home/sl/input_2192_2752_4_u8.bin",len);
	// xdma_write_with_reg((void *)(offset), data2192, len, FPGA_INPUT_REG);

	que_input[i].push(in_data);

	input_index[i]++;

	mutex_input[i].unlock();

	sem_cal[i].post();	

	LOG_INFO("master push done count:%d,offset:0x%lx",++count,offset);
	
#endif

#else
    sem_input.wait();

    if (is_stop)
        return;

    mutex_input.lock();
    que_input.push(in_data);

    uint64_t offset = FPGA_INPUT_OFFSET + (uint32_t)(FPGA_INPUT_DATA_SIZE *
                                                     ((input_index + start_index) % FPGA_INPUT_BUFFER_NUM));
#ifdef _UNIX
#ifdef _DOUBLE_V7
    int rgb_len = 0;
    uint8_t *rgb_data = pic_process((InParameterAllInOne *)in_data, rgb_len);
    xdma_write_with_reg((void *)offset, rgb_data, rgb_len, FPGA_INPUT_REG);
#else
    FpgaInput *ptr_input = (FpgaInput *)in_data;
    xdma_write_with_reg((void *)offset, ptr_input->data, ptr_input->len, FPGA_INPUT_REG);
#endif
#else
    xdma_write_with_reg(ptr_input->data, offset, ptr_input->len, FPGA_INPUT_REG);
#endif

    input_index++;

    mutex_input.unlock();

    sem_cal.post();
#endif
}

bool FpgaControl::push_slave(void *in_data)
{
	FSERVO_CHECK(FPGA_OPEN == status);

    if (pause)
        return false;
    if (is_stop)
        return false;

#if DIE_NUM > 1
    const uint64_t FPGA_INPUT_OFFSET_ARR[DIE_NUM] = {FPGA_INPUT_OFFSET, FPGA_INPUT_OFFSET2};
#else
    const uint64_t FPGA_INPUT_OFFSET_ARR[DIE_NUM] = {FPGA_INPUT_OFFSET};
#endif

	static unsigned char pic_param[16]={0x90,0x08,0xc0,0x0a,0xd1,0x45,0x1f,0x00,
                                                                        0x00,0x40,0x22,0x00,0x01,0x03,0x17,0x00};

    if (!FPGA_RESIZE_SCALE_X2) {
        cv::Mat &frame = ((InParameterAllInOne *)in_data)->currentFrame;
        *(const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_X2)) = ((float)frame.cols / ParamManager::getInstance()->getFpgaCfg().inputNetWidth) * (1 << 19);
        *(const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_Y2)) = ((float)frame.rows / ParamManager::getInstance()->getFpgaCfg().inputNetHeight) * (1 << 19);

        uint16_t cols = frame.cols;
        uint16_t rows = frame.rows;

        memcpy(pic_param, &rows, sizeof(uint16_t));
        memcpy(pic_param + sizeof(uint16_t), &cols, sizeof(uint16_t));
        memcpy(pic_param + sizeof(uint16_t) * 2, const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_X2), sizeof(uint32_t));
        memcpy(pic_param + sizeof(uint16_t) * 4, const_cast<uint32_t *>(&FPGA_RESIZE_SCALE_Y2), sizeof(uint32_t));
    }

#ifdef _DOUBLE_V7
	const uint32_t i=1; 

	sem_input[i].wait();

	mutex_input[i].lock();

	uint64_t offset = FPGA_INPUT_OFFSET_ARR[i] + (uint32_t)(FPGA_INPUT_DATA_SIZE *
															((input_index[i] + start_index[i]) % FPGA_INPUT_BUFFER_NUM));

	cv::Mat frame = ((InParameterAllInOne *)in_data)->currentFrame;
	int rgb_len = frame.cols * frame.rows * frame.channels();
	uint8_t *rgb_data = frame.data;
	
	static uint32_t push_count=0;
	xdma_write_reg(15, 0);
	usleep(1);
	xdma_write_reg(15, 1);
	// usleep(10);
	xdma_write_reg(12, 1);
	// usleep(10);
	xdma_write_reg(13, offset);
	// usleep(10);
	xdma_write_reg(14, rgb_len/16+1);
	// xdma_write_reg(14, 0x170301);

	// usleep(10);
	xdma_write_reg(10, 1);
	usleep(1);
	xdma_write_reg(10, 0);
	//读207
	// LOG_INFO("slave write reg 13 0x%lx,len:0x%lx", offset,rgb_len/16+1);
	uint32_t count = 0;
	for (;;) {
		count++;
		if (count > 100) {
			LOG_ERR("slave wait 207 fail");
			mutex_input[i].unlock();
			return false;
		}
		int32_t reg_data = 0;
		xdma_read_reg(207, reg_data);
		if (reg_data) break;
		usleep(1000);
	}
	xdma_write_reg(15, 0);
	usleep(1);
	xdma_write_reg(15, 1);

	xdma_write((void *)offset, pic_param, 16);
	xdma_write((void *)(offset + 16), rgb_data, rgb_len);

	// static int len=0;
	// static char* data2192=readBinaryFile("/home/sl/input_2192_2752_4_u8.bin",len);
	// xdma_write_with_reg((void *)(offset), data2192, len, FPGA_INPUT_REG);

	que_input[i].push(in_data);

	xdma_write_reg(11, 1);
	usleep(1);
	xdma_write_reg(11, 0);

	input_index[i]++;

	mutex_input[i].unlock();

	sem_cal[i].post();
	
	LOG_INFO("slave push done count:%d,offset:0x%lx",++push_count,offset);
#endif
}

void FpgaControl::input_thread(void *arg)
{
    FpgaControl *pThis = static_cast<FpgaControl *>(arg);

    LOG_INFO("Fpga input thread start:%lu", std::this_thread::get_id());

    for (;;) {
        if (pThis->is_stop)
            break;
    }

    LOG_INFO("Fpga input thread exit");
}

void FpgaControl::output_thread(void *arg)
{
    FpgaControl *pThis = static_cast<FpgaControl *>(arg);

    LOG_INFO("Fpga output thread start:%lu", std::this_thread::get_id());

    const uint32_t WAIT_TIMES = ParamManager::getInstance()->getFpgaCfg().outputWaitTime;
    const uint32_t FPGA_OUTPUT_REG = ParamManager::getInstance()->getFpgaCfg().outputReg;
    const uint32_t FPGA_OUTPUT_OFFSET = ParamManager::getInstance()->getFpgaCfg().outputOffset;
    const uint32_t FPGA_OUTPUT_SIZE = ParamManager::getInstance()->getFpgaCfg().outputSize;

    LOG_INFO("fpga output offset:0x%x "
             "fpga output data size:0x%x "
             "fpga output reg:%d",
             FPGA_OUTPUT_OFFSET, FPGA_OUTPUT_SIZE, FPGA_OUTPUT_REG);

#ifdef _DOUBLE_V7
#if DIE_NUM > 1
    const uint32_t FPGA_OUTPUT_REG_ARR[DIE_NUM] = {FPGA_OUTPUT_REG, FPGA_OUTPUT_REG2};
    const uint32_t FPGA_OUTPUT_OFFSET_ARR[DIE_NUM] = {FPGA_OUTPUT_OFFSET, FPGA_OUTPUT_OFFSET2};
#else
    const uint32_t FPGA_OUTPUT_REG_ARR[DIE_NUM] = {FPGA_OUTPUT_REG};
    const uint32_t FPGA_OUTPUT_OFFSET_ARR[DIE_NUM] = {FPGA_OUTPUT_OFFSET};
#endif
#endif

#if 0

    for (uint16_t i = 0;; ++i) {
    STEP:
        if (DIE_NUM == i) i = 0;

#ifdef _DOUBLE_V7
        pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_FREE;
#endif
        if (!pThis->sem_cal[i].wait(1000 / FPS * 1000)) continue;

#ifdef _DOUBLE_V7
        pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_NORMAL;
#endif
        if (pThis->is_stop)
            break;

        int32_t read_count = 0;
        for (;;) {
            if (pThis->is_stop)
                return;

            read_count++;
            if (read_count >= 1000) {
                LOG_ERR("interrupt fail!!! die %d", i + 1);
#ifdef _DOUBLE_V7
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_ERROR;
#else
                exit(-1);
#endif
            }

            int32_t reg_data = 0;
            xdma_read_reg(FPGA_OUTPUT_REG_ARR[i], reg_data);

            if (reg_data - pThis->start_index[i] - pThis->output_index[i] > 0) {
                // LOG_INFO("die %d out",i+1);
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TIMES));
        }

        pThis->mutex_input[i].lock();
        if (pThis->que_input[i].empty()) {
            LOG_ERR("empty input!");
            pThis->output_index[i]++;
            pThis->mutex_input[i].unlock();

            continue;
        }
        void *pInput = pThis->que_input[i].front();
        pThis->que_input[i].pop();
        pThis->mutex_input[i].unlock();

        uint64_t offset = FPGA_OUTPUT_OFFSET_ARR[i] + (FPGA_OUTPUT_SIZE * ((pThis->output_index[i] + pThis->start_index[i]) % pThis->FPGA_INPUT_BUFFER_NUM));
        uint8_t *res_buffer = new uint8_t[REAL_OUT_SIZE];
#ifdef _UNIX
        xdma_read((void *)offset, res_buffer, REAL_OUT_SIZE);
#else
        xdma_read(res_buffer, offset, FPGA_OUTPUT_SIZE);
#endif

		static bool iss=true;
		if(iss){
			static int n=0;
			std::ofstream out(std::string("out_")+std::to_string(i+1)+"_"+std::to_string(n++)+".bin",std::ofstream::binary);
			out.write((char*)res_buffer,REAL_OUT_SIZE);
			out.close();
			LOG_INFO("out addr:%lx",offset);
			// fprintf(stderr,"---out addr:%"PRIu64"\n",offset);
			// char* buff2=new char[11534336];
			// xdma_read((void*)0x3f9f700,buff2,11534336);
			// save_binary_file("conv1.bin",buff2,11534336);

			// char* buff3=new char[5767168];
			// xdma_read((void*)0x59bf730,buff3,5767168);
			// save_binary_file("conv3.bin",buff3,5767168);

			// char* buff4=new char[5767168];
			// xdma_read((void*)0x61ff700,buff4,5767168);
			// save_binary_file("conv8.bin",buff4,5767168);

			// char* buff5=new char[1441792];
			// xdma_read((void*)0x6937700,buff5,1441792);
			// save_binary_file("conv12.bin",buff5,1441792);

			// char* buff7=new char[720896];
			// xdma_read((void*)0x748f700,buff7,720896);
			// save_binary_file("conv24.bin",buff7,720896);

			// char* buff6=new char[360448];
			// xdma_read((void*)0x7d95700,buff6,360448);
			// save_binary_file("conv46.bin",buff6,360448);

			// char* buff8=new char[180224];
			// xdma_read((void*)0x8194700,buff8,180224);
			// save_binary_file("conv64.bin",buff8,180224);

			// char* buff9=new char[180224];
			// xdma_read((void*)0x804a700,buff9,180224);
			// save_binary_file("conv59.bin",buff9,180224);

			// char* buff10=new char[360448];
			// xdma_read((void*)0x7e9d700,buff10,360448);
			// save_binary_file("conv48.bin",buff10,360448);

			// char* buff11=new char[360448];
			// xdma_read((void*)0x8008700,buff11,360448);
			// save_binary_file("conv58.bin",buff11,360448);
			
			// char* buff12=new char[180224];
			// xdma_read((void*)0x8131700,buff12,180224);
			// save_binary_file("conv62.bin",buff12,180224);

			// char* buff13=new char[360448];
			// xdma_read((void*)0x804a700,buff13,360448);
			// save_binary_file("conv60.bin",buff13,360448);

			// char* buff3=new char[1441792];
			// xdma_read((void*)0x3457700,buff3,1441792);
			// save_binary_file("resize.bin",buff3,1441792);

			iss=false;
		}


        DetectData calData(res_buffer, pInput);

        for (;;) {
            std::unique_lock<std::mutex> locker(pThis->mutex_detect);
            if (pThis->que_detect.size() > 60) {
                locker.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            pThis->que_detect.push(calData);
            break;
        }

        // pThis->mutex_detect.lock();
        // pThis->que_detect.push(calData);
        // pThis->mutex_detect.unlock();

        pThis->output_index[i]++;

        pThis->sem_detect.post();
        pThis->sem_input[i].post();
    }

#else

	const uint16_t i=0;
	for (;;) {
#ifdef _DOUBLE_V7
        pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_FREE;
#endif
        pThis->sem_cal[i].wait();

#ifdef _DOUBLE_V7
        pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_NORMAL;
#endif
        if (pThis->is_stop)
            break;

        int32_t read_count = 0;
        for (;;) {
            if (pThis->is_stop)
                return;

            read_count++;
            if (read_count >= 1000) {
                LOG_ERR("interrupt fail!!! die %d", i + 1);
#ifdef _DOUBLE_V7
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_ERROR;
#else
                exit(-1);
#endif
            }

            int32_t reg_data = 0;
            xdma_read_reg(FPGA_OUTPUT_REG_ARR[i], reg_data);

            if (reg_data - pThis->start_index[i] - pThis->output_index[i] > 0) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TIMES));
        }

        pThis->mutex_input[i].lock();
        if (pThis->que_input[i].empty()) {
            LOG_ERR("empty input!");
            pThis->output_index[i]++;
            pThis->mutex_input[i].unlock();

            continue;
        }
        void *pInput = pThis->que_input[i].front();
        pThis->que_input[i].pop();
        pThis->mutex_input[i].unlock();

        uint64_t offset = FPGA_OUTPUT_OFFSET_ARR[i] + (FPGA_OUTPUT_SIZE * ((pThis->output_index[i] + pThis->start_index[i]) % pThis->FPGA_INPUT_BUFFER_NUM));
        uint8_t *res_buffer = new uint8_t[REAL_OUT_SIZE];
#ifdef _UNIX
        xdma_read((void *)offset, res_buffer, REAL_OUT_SIZE);
#else
        xdma_read(res_buffer, offset, FPGA_OUTPUT_SIZE);
#endif

		// static bool iss=true;
		// if(iss){
		// 	static int n=0;
		// 	std::ofstream out(std::string("out_")+std::to_string(i+1)+"_"+std::to_string(n++)+".bin",std::ofstream::binary);
		// 	out.write((char*)res_buffer,REAL_OUT_SIZE);
		// 	out.close();
		// 	LOG_INFO("out addr:%lx",offset);

		// 	iss=false;
		// }


        DetectData calData(res_buffer, pInput);

        for (;;) {
            std::unique_lock<std::mutex> locker(pThis->mutex_detect);
            if (pThis->que_detect.size() > 60) {
                locker.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            pThis->que_detect.push(calData);
            break;
        }

        pThis->output_index[i]++;

        pThis->sem_detect.post();
        pThis->sem_input[i].post();
    }
#endif

    LOG_INFO("Fpga output thread exit");
}

void FpgaControl::output_thread2(void *arg)
{
    FpgaControl *pThis = static_cast<FpgaControl *>(arg);

    LOG_INFO("Fpga output thread2 start:%lu", std::this_thread::get_id());

    const uint32_t WAIT_TIMES = ParamManager::getInstance()->getFpgaCfg().outputWaitTime;
    const uint32_t FPGA_OUTPUT_REG = ParamManager::getInstance()->getFpgaCfg().outputReg;
    const uint32_t FPGA_OUTPUT_OFFSET = ParamManager::getInstance()->getFpgaCfg().outputOffset;
    const uint32_t FPGA_OUTPUT_SIZE = ParamManager::getInstance()->getFpgaCfg().outputSize;

    LOG_INFO("fpga output offset2:0x%x "
             "fpga output data size2:0x%x "
             "fpga output reg2:%d",
             FPGA_OUTPUT_OFFSET2, FPGA_OUTPUT_SIZE, FPGA_OUTPUT_REG2);

#ifdef _DOUBLE_V7
#if DIE_NUM > 1
    const uint32_t FPGA_OUTPUT_REG_ARR[DIE_NUM] = {FPGA_OUTPUT_REG, FPGA_OUTPUT_REG2};
    const uint32_t FPGA_OUTPUT_OFFSET_ARR[DIE_NUM] = {FPGA_OUTPUT_OFFSET, FPGA_OUTPUT_OFFSET2};
#else
    const uint32_t FPGA_OUTPUT_REG_ARR[DIE_NUM] = {FPGA_OUTPUT_REG};
    const uint32_t FPGA_OUTPUT_OFFSET_ARR[DIE_NUM] = {FPGA_OUTPUT_OFFSET};
#endif
#endif

	const uint16_t i=1;
    for (;;) {
#ifdef _DOUBLE_V7
        pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_FREE;
#endif
        pThis->sem_cal[i].wait();

#ifdef _DOUBLE_V7
        pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_NORMAL;
#endif
        if (pThis->is_stop)
            break;

        int32_t read_count = 0;
        for (;;) {
            if (pThis->is_stop)
                return;

            read_count++;
            if (read_count >= 1000) {
                LOG_ERR("interrupt fail!!! die %d", i + 1);
#ifdef _DOUBLE_V7
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                pThis->nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_ERROR;
#else
                exit(-1);
#endif
            }

            int32_t reg_data = 0;
            xdma_read_reg(FPGA_OUTPUT_REG_ARR[i], reg_data);

            if (reg_data - pThis->start_index[i] - pThis->output_index[i] > 0) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(WAIT_TIMES));
        }

        pThis->mutex_input[i].lock();
        if (pThis->que_input[i].empty()) {
            LOG_ERR("empty input!");
            pThis->output_index[i]++;
            pThis->mutex_input[i].unlock();

            continue;
        }
        void *pInput = pThis->que_input[i].front();
        pThis->que_input[i].pop();
        pThis->mutex_input[i].unlock();

        uint64_t offset = FPGA_OUTPUT_OFFSET_ARR[i] + (FPGA_OUTPUT_SIZE * ((pThis->output_index[i] + pThis->start_index[i]) % pThis->FPGA_INPUT_BUFFER_NUM));
        uint8_t *res_buffer = new uint8_t[REAL_OUT_SIZE];
#ifdef _UNIX
        xdma_read((void *)offset, res_buffer, REAL_OUT_SIZE);
#else
        xdma_read(res_buffer, offset, FPGA_OUTPUT_SIZE);
#endif

		static bool iss=true;
		if(iss){
			static int n=0;
			std::ofstream out(std::string("out_")+std::to_string(i+1)+"_"+std::to_string(n++)+".bin",std::ofstream::binary);
			out.write((char*)res_buffer,REAL_OUT_SIZE);
			out.close();
			LOG_INFO("out addr:%lx",offset);

			iss=false;
		}


        DetectData calData(res_buffer, pInput);

        for (;;) {
            std::unique_lock<std::mutex> locker(pThis->mutex_detect);
            if (pThis->que_detect.size() > 60) {
                locker.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            pThis->que_detect.push(calData);
            break;
        }

        pThis->output_index[i]++;

        pThis->sem_detect.post();
        pThis->sem_input[i].post();
    }

    LOG_INFO("Fpga output thread2 exit");
}

void FpgaControl::calculate_thread(void *arg)
{
    FpgaControl *pThis = static_cast<FpgaControl *>(arg);

    LOG_INFO("Fpga calculate thread start:%lu", std::this_thread::get_id());

    for (;;) {
        pThis->sem_detect.wait();

        if (pThis->is_stop)
            break;

        pThis->mutex_detect.lock();
        if (pThis->que_detect.empty()) {
            LOG_ERR("empty detect!");
            pThis->mutex_detect.unlock();
            continue;
        }
        DetectData calData = pThis->que_detect.front();
        pThis->que_detect.pop();
        pThis->mutex_detect.unlock();

        static const char *netFile = ParamManager::getInstance()->getNetCfg().netFile.c_str();
        static const int detect_w = ParamManager::getInstance()->getFpgaCfg().inputNetWidth;
        static const int detect_h = ParamManager::getInstance()->getFpgaCfg().inputNetHeight;

        static NetDetect netDetect(netFile, detect_w, detect_h);
        int num = 0;
#ifdef _DOUBLE_V7
        InParameterAllInOne *pInput = (InParameterAllInOne *)calData.inputData;
        BoxInfo *pBox = netDetect.getDetectBoxInfo((char *)calData.outData, detect_w, detect_h, num);

        LOG_INFO("box num is %d ，%d,%d",num,detect_w, detect_h);

        auto pOut = structor_output(pInput, pBox, num);

        pThis->pf_callback(pOut, pInput, pThis->pf_param);
	
#else
        FpgaInput *pInput = (FpgaInput *)calData.inputData;
        BoxInfo2 *pBox = netDetect.getDetectBoxInfo((char *)calData.outData, pInput->width, pInput->height, num);
        pThis->pf_callback((FpgaInput *)calData.inputData, pBox, num, pThis->pf_param);
#endif
		// LOG_INFO("delete");
        delete[] calData.outData;
    }

    LOG_INFO("Fpga calculate thread exit");
}
