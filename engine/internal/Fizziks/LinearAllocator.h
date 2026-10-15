#pragma once
#include <Fizziks/Allocator.h>
#include <shared_mutex>

namespace Fizziks::internal
{
class LinearAllocator : public Allocator
{
public:
	LinearAllocator(size_t tot_bytes);
	virtual ~LinearAllocator();

	virtual void reset() override;

protected:
	virtual Block write_internal(const void* data, size_t byte_count, size_t alignment = 0) override;
	virtual bool read_internal(Block block, void** out) const override;
	virtual void resize(size_t new_size) override;
	virtual void* allocate(size_t size, size_t alignment = 0) override;

private:
	static constexpr size_t MAX_ALIGN = 64;

	void* start = nullptr; // aligned
	void* raw_start = nullptr; // malloced, needs freeing
	size_t cursor;
	size_t init_bytes;

	mutable std::shared_mutex mutex; // needs mutable to be called in read since it's const

	bool timing;
	size_t counter;

	void shrink();
	void allocateBuffer(size_t bytes, void** raw, void** aligned);
};
}
