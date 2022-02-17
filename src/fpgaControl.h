#pragma once

#include "dataModel.h"
#include "semaphore.h"
#include <atomic>
#include <mutex>
#include <queue>

namespace std
{
    class thread;
}

#ifdef _DOUBLE_V7
#define DIE_NUM 2
constexpr uint32_t FPGA_OUTPUT_REG2 = 135;
constexpr uint32_t FPGA_OUTPUT_OFFSET2 = 0x9000000 + 0x1ec000 * 8;
constexpr uint32_t FPGA_INPUT_OFFSET2 = 0x10000000 + 8 * 0x1e00000;
#else
constexpr int16_t DIE_NUM = 1;
#endif

class FpgaControl
{
public:
    enum Status {
        FPGA_OPEN,
        FPGA_CLOSE,
        FPGA_ERR,
    };

private:
    FpgaControl() noexcept;
    ~FpgaControl() { close(); }

public:
    static FpgaControl *getInstance();
    bool open();
    void close();
    void start();
    void stop();
    bool push(void *);
	bool push_slave(void *);
    void setCallBack(pf_fpga_callback _pf, void *_para);
    FpgaParaCfg *getFpgaPara() const;
    void setFpgaPara(const FpgaParaCfg *);
#ifdef _DOUBLE_V7
    NNC_CommonStruct::NNC_StateModel getFpgaStatus();
#endif
private:
    void init();
    void initIndex();
    void initParam();
    // uint8_t *picProcess(FpgaInput *, int &);

    static void input_thread(void *);
    static void output_thread(void *);
	static void output_thread2(void *);
    static void calculate_thread(void *);

private:
    void *pf_param = nullptr;
    pf_fpga_callback pf_callback = nullptr;

#ifdef _DOUBLE_V7
    std::atomic<uint64_t> input_index[DIE_NUM];
    std::atomic<uint64_t> output_index[DIE_NUM];
    uint64_t start_index[DIE_NUM];
    const volatile uint32_t FPGA_RESIZE_SCALE_X = 0;
    const volatile uint32_t FPGA_RESIZE_SCALE_Y = 0;

	const volatile uint32_t FPGA_RESIZE_SCALE_X2 = 0;
    const volatile uint32_t FPGA_RESIZE_SCALE_Y2 = 0;
#endif
    const volatile uint32_t FPGA_INPUT_DATA_SIZE = 0;
    const volatile uint32_t FPGA_INPUT_OFFSET = 0;
    const volatile uint32_t FPGA_INPUT_BUFFER_NUM = 0;
    const volatile uint32_t FPGA_INPUT_REG = 0;

    Status status = FPGA_CLOSE;
#ifdef _DOUBLE_V7
    NNC_CommonStruct::NNC_StateModel nnc_state = NNC_CommonStruct::MODEL_STATE_NNC_FREE;
#endif

#ifdef _DOUBLE_V7
    std::Semaphore sem_input[DIE_NUM];
    std::Semaphore sem_cal[DIE_NUM];
    std::Semaphore sem_detect;
    std::mutex mutex_input[DIE_NUM];
    std::mutex mutex_detect;
    std::queue<void *> que_input[DIE_NUM];
    std::queue<DetectData> que_detect;
#else
    std::Semaphore sem_input;
    std::Semaphore sem_cal;
    std::Semaphore sem_detect;
    std::mutex mutex_input;
    std::mutex mutex_detect;
    std::queue<void *> que_input;
    std::queue<DetectData> que_detect;
#endif
    std::atomic<bool> is_stop = {true};
    std::atomic<bool> pause = {false};

    std::thread *thread_output = nullptr;
	std::thread *thread_output2 = nullptr;
    std::thread *thread_calculate = nullptr;
    std::thread *thread_input = nullptr;
};