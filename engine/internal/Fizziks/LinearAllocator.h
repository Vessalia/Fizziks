#pragma once
#include <Fizziks/Allocator.h>
#include <mutex>

namespace Fizziks::internal
{
class LinearAllocator : public Allocator
{
public:
    LinearAllocator(size_t tot_bytes);
    virtual ~LinearAllocator();

    virtual void reset() override;
    void shrink();

    virtual Block write(void* data, size_t byte_count, size_t alignment = 0) override;
    virtual void* read(Block block) const override;

private:
    void* start = nullptr;
    uintptr_t cursor;
    size_t init_bytes;

    std::mutex mutex;

    bool timing;
    size_t counter;

    virtual void resize(size_t new_size) override;
    virtual void* allocate(size_t size, size_t alignment = 0) override;
};
}
