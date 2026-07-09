#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <utility>

template <typename T>
class ThreadSafeQueue
{
public:
    void push(T value)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(value));
        }
        condition_.notify_one();
    }

    bool tryPop(T& value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
        {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    bool waitPop(T& value)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return !queue_.empty() || closed_; });
        if (queue_.empty())
        {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    void close()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        condition_.notify_all();
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::queue<T> queue_;
    bool closed_ = false;
};
