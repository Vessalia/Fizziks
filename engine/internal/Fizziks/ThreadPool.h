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
    using Task = std::function<void()>;

    ThreadPool(size_t num_threads = std::thread::hardware_concurrency());
    ~ThreadPool();

    void submit(Task&& );

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
