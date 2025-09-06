#pragma once
#include <vector>
#include <ostream>
#include <bitset>

namespace Fizziks
{
static constexpr uint8_t BYTE_SIZE = 8;

class BitArray
{
private:
    std::vector<std::bitset<BYTE_SIZE>> bytes;
    size_t bitCount;

    size_t countBits() const;

public:
    static const BitArray Zero;

    BitArray(size_t val);
    BitArray();

    explicit operator bool() const
    {
        for(const auto& byte : bytes)
        {
            if(byte.any())
            {
                return true;
            }
        }

        return false;
    }
    explicit operator size_t() const
    {
        if (bytes.size() > sizeof(size_t)) return SIZE_MAX;

        size_t val = 0;
        for(size_t i = 0; i < bytes.size(); ++i)
        {
            val += bytes[i].to_ullong() << (BYTE_SIZE * i);
        }

        return val;
    }

    const std::vector<std::bitset<BYTE_SIZE>>& getBytes() const;

    BitArray operator~() const;
    uint8_t  operator[](size_t index) const;

    BitArray& operator|=(const BitArray& other);
    BitArray& operator&=(const BitArray& other);
    BitArray& operator^=(const BitArray& other);
    BitArray& operator<<=(size_t shift);
    BitArray& operator>>=(size_t shift);
    
    BitArray operator|(const BitArray& other) const;
    BitArray operator&(const BitArray& other) const;
    BitArray operator^(const BitArray& other) const;
    BitArray operator<<(size_t shift) const;
    BitArray operator>>(size_t shift) const;

    bool operator==(const BitArray& other) const;
    bool operator!=(const BitArray& other) const;

    BitArray& set(size_t index, bool val = true);
    BitArray& clear(size_t index);
    BitArray& flip(size_t index);
    bool      read(size_t index) const;

    BitArray& trim();

    size_t size() const;
};

inline std::ostream& operator<<(std::ostream& os, const BitArray& ba)
{
    for (auto it = ba.getBytes().rbegin(); it != ba.getBytes().rend(); ++it)
    {
        os << *it;
        os << " ";
    }
    return os;
};
};
