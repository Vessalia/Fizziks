#pragma once
#include <cstdint>

namespace Fizziks::internal
{
class Allocator
{
public:
	struct Block
	{
		size_t base;
		size_t byte_count;
		size_t gen;
	};

	virtual ~Allocator() = default;

	virtual void reset() = 0;

	template<typename T>
	Block write(const T* data, size_t size = sizeof(T), size_t alignment = alignof(T)) { return write_internal(data, size, alignment); }

	template<typename T>
	bool read(Block block, T** out) const { return read_internal(block, (void**)out); }

protected:
	size_t tot_bytes;
	size_t peak;
	size_t gen;

	Allocator(size_t tot_bytes) : tot_bytes(tot_bytes), peak(0), gen(0) { };

	virtual Block write_internal(const void* data, size_t byte_count, size_t alignment = 0) = 0;
	virtual bool read_internal(Block block, void** out) const = 0;
	virtual void resize(size_t new_size) = 0;
	virtual void* allocate(size_t size, size_t alignment = 0) = 0;
};
}