#pragma once
#include <Fizziks/Allocator.h>
#include <mutex>

namespace Fizziks::internal
{
class LinearAllocator : public Allocator
{
public:
    LinearAllocator(size_t tot_bytes);
    ~LinearAllocator();

    virtual void reset() override;

    virtual Block write(void* data, size_t byte_count, size_t alignment = 0) override;
    virtual void* read(Block block) const override;

private:
    void* start = nullptr;
    size_t cursor;

    std::mutex mutex;

    virtual void* allocate(size_t size, size_t alignment = 0) override;
};
}
