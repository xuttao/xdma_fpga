#pragma once

#include <condition_variable>
#include <mutex>

namespace std
{
    class Semaphore
    {
    public:
        explicit Semaphore(int32_t _count) noexcept;
        Semaphore() noexcept = default;
        ~Semaphore();

    public:
        void wait();
        bool wait(int _msec);
        void post(int32_t _count = 1);
        void init(int32_t _count);
        int32_t count();

    private:
        int32_t m_count = 0;
        std::mutex mutex_lock;
        std::condition_variable condition_var;
    };

} // namespace std