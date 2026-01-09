#pragma once
#include <Fizziks/Handle.h>

#include <cstdint>
#include <queue>

namespace Fizziks::internal
{
template<typename T>
class Pool
{
private:
    struct Slot
    {
        T data;
        uint32_t gen = 0;
        bool active = false;
    };

    std::vector<Slot> slots;
    std::queue<uint32_t> free_indices;

public:
    Pool(size_t max_capacity = 1024)
    {
        slots.resize(max_capacity);
        for (uint32_t i = 0; i < max_capacity; ++i)
        {
            free_indices.push(i);
        }
    }

    std::pair<Handle, T*> get()
    {
        assert(!free_indices.empty() && "Pool is exhausted.");

        uint32_t index = free_indices.front(); free_indices.pop();
        slots[index].active = true;
        Handle handle{ index, slots[index].gen };
        return { handle, &slots[index].data };
    }

    bool release(Handle h)
    {
        if(isValid(h))
        {
            slots[h.index].gen++;
            slots[h.index].active = false;
            free_indices.push(h.index);
            return true;
        }
        else
        {
            assert(false && "Releasing unowned handle.");
            return false;
        }
    }

    bool isValid(Handle h) const 
    {
        return h.index < slots.size()
            && slots[h.index].active
            && slots[h.index].gen == h.gen;
    }

    void clear()
    {
        for (auto& s : slots) s.active = false;

        while (!free_indices.empty()) free_indices.pop();
        for (uint32_t i = 0; i < slots.size(); ++i) free_indices.push(i);
    }
};
}
