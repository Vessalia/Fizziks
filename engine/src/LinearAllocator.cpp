#include <Fizziks/LinearAllocator.h>
#include <cstdlib>
#include <cassert>
#include <algorithm>

namespace Fizziks::internal
{
static inline size_t calculatePadding(const uintptr_t baseAddress, const size_t alignment)
{
	if (alignment <= 1) return 0;
	assert((alignment & (alignment - 1)) == 0 && "alignment must be a power of two");

	// round up to the next multiple of alignment
	// ex: if alignment is 0b1000, then add 0b0111 and mask out 0b0111 (bottom 3 bits) to round
	const uintptr_t alignedAddress = (baseAddress + alignment - 1) & ~(uintptr_t)(alignment - 1);
	return static_cast<size_t>(alignedAddress - baseAddress);
}

void LinearAllocator::allocateBuffer(size_t bytes, void** raw, void** aligned)
{
	*raw = malloc(bytes + MAX_ALIGN); // slack by MAX_ALIGN so we can round up
	if (!*raw) throw std::bad_alloc();

	uintptr_t addr = (uintptr_t)*raw;
	*aligned = (void*)(addr + calculatePadding(addr, MAX_ALIGN));
}

LinearAllocator::LinearAllocator(size_t tot_bytes)
	: Allocator(tot_bytes)
	, cursor(0)
	, init_bytes(tot_bytes)
	, timing(false)
	, counter(0)
{
	allocateBuffer(tot_bytes, &raw_start, &start);
}

LinearAllocator::~LinearAllocator()
{
	free(raw_start);
	raw_start = nullptr;
	start = nullptr;
}

void LinearAllocator::resize(size_t new_size)
{
	void* newRaw;
	void* newStart;
	allocateBuffer(new_size, &newRaw, &newStart);

	size_t bytesToCopy = std::min(new_size, cursor);
	memcpy(newStart, start, bytesToCopy);

	free(raw_start);
	raw_start = newRaw;
	start = newStart;
	tot_bytes = new_size;
	cursor = bytesToCopy;
}

void* LinearAllocator::allocate(size_t size, size_t alignment)
{
	assert(alignment <= MAX_ALIGN && "alignment exceeds MAX_ALIGN");

	uintptr_t currAddress = (uintptr_t)start + cursor;
	size_t padding = calculatePadding(currAddress, alignment);

	if (cursor + padding + size > tot_bytes)
	{
		resize(std::max(tot_bytes * 2, cursor + padding + size));
		currAddress = (uintptr_t)start + cursor; // start has moved
	}

	uintptr_t nextAddress = currAddress + padding;
	cursor += padding + size;
	peak = std::max(peak, cursor);

	return (void*)nextAddress;
}

constexpr size_t COLLECTION_TIMER = 30;
void LinearAllocator::reset()
{
	std::unique_lock lock(mutex);

	peak = std::max(peak, cursor);
	cursor = 0;
	++gen;

	if (counter++ == COLLECTION_TIMER)
	{
		shrink();
		counter = 0;
		peak = 0;
	}
}

void LinearAllocator::shrink()
{
	size_t target = std::max(peak * 2, init_bytes);
	if (target < tot_bytes) resize(target);
}

Allocator::Block LinearAllocator::write_internal(const void* data, size_t byte_count, size_t alignment)
{
	std::unique_lock lock(mutex);

	void* location = allocate(byte_count, alignment);
	size_t offset = (size_t)((uintptr_t)location - (uintptr_t)start);
	memcpy(location, data, byte_count);

	return { offset, byte_count, gen };
}

bool LinearAllocator::read_internal(Block block, void** out) const
{
	std::shared_lock lock(mutex);

	if (block.gen != gen) return false;
	if (block.byte_count > cursor || block.base > cursor - block.byte_count) return false;

	*out = static_cast<char*>(start) + block.base;
	return true;
}
}
