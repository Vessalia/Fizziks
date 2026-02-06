#pragma once
#include <mutex>

namespace Fizziks::internal
{
class Allocator
{
public:
    struct Block
    {
        size_t base;
        size_t byte_count;
    };

    virtual ~Allocator() = 0;

    virtual void reset() = 0;

    virtual Block write(void* data, size_t byte_count, size_t alignment = 0) = 0;
    virtual void* read(Block block) const = 0;

protected:
    size_t tot_bytes;

    Allocator(size_t tot_bytes);
    
    virtual void* allocate(size_t size, size_t alignment = 0) = 0;
};
}