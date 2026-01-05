#pragma once
#include <inttypes.h>
#include <functional>

namespace Fizziks
{
struct ContactKey
{
    uint32_t bodyAId, bodyBId;
    uint32_t collIdA, collIdB;
    uint32_t featureA, featureB;

    bool operator==(const ContactKey&) const = default;
};
}

namespace std
{
    template<>
    struct std::hash<Fizziks::ContactKey>
    {
        std::size_t operator()(const Fizziks::ContactKey& ck) const
        {
            size_t h = 0;
            auto hashCombine = [&](uint32_t v)
                {
                    h ^= std::hash<uint32_t>{}(v)+0x9e3779b9 + (h << 6) + (h >> 2);
                };

            hashCombine(ck.bodyAId);
            hashCombine(ck.bodyBId);
            hashCombine(ck.collIdA);
            hashCombine(ck.collIdB);
            hashCombine(ck.featureA);
            hashCombine(ck.featureB);

            return h;
        }
    };
};