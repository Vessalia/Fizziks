#include <Fizziks/ThreadPool.h>

namespace Fizziks::internal
{
ThreadPool::ThreadPool(size_t num_threads)
{
    for (size_t i = 0; i < num_threads; ++i)
    {
        threads.emplace_back([this] 
        {
            while (true)
            {
                Task task;

                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    cv.wait(lock, [this] { return !tasks.empty() || stop; });
                    if (stop && tasks.empty()) return;

                    task = move(tasks.front()); tasks.pop();
                    ++running_tasks;
                }

                task();

                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    --running_tasks;
                    if (tasks.empty() && !running_tasks) cv.notify_all();
                }
            }
        });
    }
}

ThreadPool::~ThreadPool()
{
    std::unique_lock<std::mutex> lock(queue_mutex);
    stop = true;
    cv.notify_all();
    for (auto& thread : threads) thread.join();
}

void ThreadPool::wait()
{
    std::unique_lock<std::mutex> lock(queue_mutex);
    cv.wait(lock, [this] { return tasks.empty() && running_tasks == 0; });
}
}
