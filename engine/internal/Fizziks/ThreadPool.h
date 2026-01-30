#pragma once

#include <queue>

#include <functional>

#include <future>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace Fizziks::internal
{
class ThreadPool
{
public:
    ThreadPool(size_t num_threads = std::thread::hardware_concurrency());
    ~ThreadPool();

    template<typename T>
    std::future<std::invoke_result_t<T>> submit(T&& t)
    {
        auto task = std::make_shared<std::packaged_task<std::invoke_result_t<T>()>>(std::forward<T>(t));
        std::future<std::invoke_result_t<T>> result = task->get_future();

        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.emplace([task]() { (*task)(); });
        }

        cv.notify_one();
        return result;
    }

    void wait();

    size_t size() const { return threads.size(); }
private:
    using Task = std::function<void()>;

    std::vector<std::thread> threads;
    std::queue<Task> tasks;

    std::mutex queue_mutex;
    std::condition_variable cv;

    bool stop = false;
    size_t running_tasks = 0;
};
}
