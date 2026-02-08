#include <Fizziks/LinearAllocator.h>
#include <cstdlib>
#include <algorithm>

namespace Fizziks::internal
{
LinearAllocator::LinearAllocator(size_t tot_bytes) 
    : Allocator(tot_bytes)
    , init_bytes(tot_bytes)
    , cursor(0)
    , timing(false)
    , counter(0)
{
    start = malloc(tot_bytes); // need to align start address?
    if (!start) throw std::bad_alloc();
}
    
static inline size_t calculatePadding(const uintptr_t baseAddress, const size_t alignment) 
{
    const size_t multiplier = (baseAddress / alignment) + 1; // next aligned block index
    const uintptr_t alignedAddress = multiplier * alignment; // start of the next aligned block
    const size_t padding = alignedAddress - baseAddress; // how many bytes we need to skip to get to the next block

    return padding;
}

LinearAllocator::~LinearAllocator()
{
    free(start);
    start = nullptr;
}

void LinearAllocator::resize(size_t new_size)
{
    void* newStart = malloc(new_size);
    if (!newStart) throw std::bad_alloc();

    size_t bytesToCopy = std::min(new_size, cursor);
    memcpy(newStart, start, bytesToCopy);

    free(start);

    start = newStart;
    tot_bytes = new_size;
    cursor = bytesToCopy;
}

void* LinearAllocator::allocate(size_t size, size_t alignment)
{
    size_t padding = 0;
    uintptr_t currAddress = (uintptr_t)start + cursor;

    // if we care about alignment, and the cursor is not currently
    bool align = alignment && currAddress % alignment;
    padding = align ? calculatePadding(currAddress, alignment) : 0;

    if (cursor + padding + size > tot_bytes) 
    {
        size_t newSize = std::max(tot_bytes * 2, cursor + padding + size);
        resize(newSize);

        // start has moved, need to update addresses
        currAddress = (uintptr_t)start + cursor;
        padding = align ? calculatePadding(currAddress, alignment) : 0;
        counter = 0;
        timing = true;
    }

    uintptr_t nextAddress = currAddress + padding;
    cursor += padding + size;
    peak = std::max(peak, cursor);

    return (void*)nextAddress;
}

constexpr size_t COLLECTION_TIMER = 30;
void LinearAllocator::reset()
{
    if (timing && counter++ == COLLECTION_TIMER)
    {
        shrink();
        timing = false;
        counter = 0;
        peak = 0;
    }

    cursor = 0;
}

void LinearAllocator::shrink()
{
    size_t target = std::max(peak * 2, init_bytes);
    if (target < tot_bytes)resize(target);
}

LinearAllocator::Block LinearAllocator::write(void* data, size_t byte_count, size_t alignment)
{
    std::lock_guard lock(mutex);

    uintptr_t writeStart = cursor;
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
