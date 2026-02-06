#include <Fizziks/LinearAllocator.h>
#include <stdlib.h>
#include <algorithm>

namespace Fizziks::internal
{
LinearAllocator::LinearAllocator(size_t tot_bytes) 
    : Allocator(tot_bytes)
    , cursor(0)
{
    start = malloc(tot_bytes);
}

static inline size_t calculatePadding(const size_t baseAddress, const size_t alignment) 
{
    const size_t multiplier = (baseAddress / alignment) + 1; // next aligned block index
    const size_t alignedAddress = multiplier * alignment; // start of the next aligned block
    const size_t padding = alignedAddress - baseAddress; // how many bytes we need to skip to get to the next block

    return padding;
}

LinearAllocator::~LinearAllocator()
{
    free(start);
    start = nullptr;
}

void* LinearAllocator::allocate(size_t size, size_t alignment)
{
    size_t padding = 0;
    size_t currAddress = (size_t)start + cursor;

    // if we care about alignment, and the cursor is not currently
    if (alignment && cursor % alignment) padding = calculatePadding(currAddress, alignment);

    if (cursor + padding + size > tot_bytes) return nullptr;

    size_t nextAddress = currAddress + padding;
    cursor += padding + size;

    return (void*)nextAddress;
}

void LinearAllocator::reset()
{
    cursor = 0;
}

LinearAllocator::Block LinearAllocator::write(void* data, size_t byte_count, size_t alignment)
{
    std::lock_guard lock(mutex);

    size_t writeStart = (size_t)start + cursor;
    void* location = allocate(byte_count, alignment);
    if (location) memcpy(location, data, byte_count);

    return { writeStart, byte_count };
}

void* LinearAllocator::read(Block block) const
{
    if (block.base + block.byte_count > tot_bytes) return nullptr;
    return static_cast<char*>(start) + block.base;
}
}
