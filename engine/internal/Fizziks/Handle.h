#pragma once

#include <functional>

namespace Fizziks::internal
{
struct Handle 
{
	uint32_t index;
	uint32_t gen;

	bool operator==(const Handle&) const = default;
};
}

namespace std
{
template <>
struct hash<Fizziks::internal::Handle>
{
	size_t operator()(const Fizziks::internal::Handle h) const
	{
		size_t h1 = std::hash<uint32_t>{}(h.index);
		size_t h2 = std::hash<uint32_t>{}(h.gen);
		return h1 ^ (h2 << 1);
	}
};
}
