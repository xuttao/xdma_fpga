#include "semaphore.h"
#include <chrono>
namespace std
{

    Semaphore::Semaphore(int32_t _count) noexcept : m_count(_count)
    {
    }

    Semaphore::~Semaphore()
    {
    }

    void Semaphore::wait()
    {
        std::unique_lock<std::mutex> locker(mutex_lock);
        --m_count;
        while (m_count < 0) {
            condition_var.wait(locker);
        }
    }

    bool Semaphore::wait(int _msec)
    {
        std::unique_lock<std::mutex> locker(mutex_lock);
        --m_count;
        bool res = condition_var.wait_for(locker, std::chrono::duration<int, std::milli>(_msec), [&] { return m_count >= 0; });
        if (!res) {
            ++m_count;
        }
        return res;
    }

    void Semaphore::post(int32_t _count)
    {
        std::lock_guard<std::mutex> locker(mutex_lock);
        m_count += _count;

        if (m_count <= 0)
            condition_var.notify_one();
    }

    void Semaphore::init(int32_t _count)
    {
        std::lock_guard<std::mutex> locker(mutex_lock);
        m_count = _count;
    }

    int32_t Semaphore::count()
    {
        std::lock_guard<std::mutex> locker(mutex_lock);
        return m_count;
    }

} // namespace std