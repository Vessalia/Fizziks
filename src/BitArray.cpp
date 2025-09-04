#include "BitArray.h"

namespace Fizziks
{

BitArray::BitArray(size_t val)
{
    uint8_t mask = 0b11111111;
    size_t numBits = 0;
    do
    {
        std::bitset<BYTE_SIZE> byte { ((val >> numBits) & mask) };
        bytes.push_back(byte);
        numBits += BYTE_SIZE;
    }
    while((val >> numBits) & mask);

    auto lastByte = bytes[bytes.size() - 1];
    while(lastByte.any())
    {
        lastByte >>= 1;
        --numBits;
    }

    bitCount = numBits;
}

BitArray::BitArray() : BitArray(0) { }

const std::vector<std::bitset<BYTE_SIZE>>& BitArray::getBytes() const
{
    return bytes;
}

#pragma region unary operators

BitArray BitArray::operator~() const
{
    BitArray result;
    result.bytes.resize(bytes.size());
    for(size_t i = 0; i < bytes.size() - 1; ++i)
        result.bytes[i] = ~bytes[i];

    uint8_t count = bitCount % BYTE_SIZE;
    uint8_t convert = (1 << count) - 1;
    std::bitset<BYTE_SIZE> mask { convert };
    result.bytes[bytes.size() - 1] = ~bytes[bytes.size() - 1] & mask;

    return result;
}
uint8_t BitArray::operator[](size_t index) const
{
    size_t byteIndex = index / BYTE_SIZE;
    uint8_t bitIndex = index % BYTE_SIZE;

    if(byteIndex > bytes.size()) return 0;

    return (bytes[byteIndex] & (std::bitset<BYTE_SIZE>(1) << bitIndex)).to_ulong();
}

#pragma endregion

#pragma region binary assignment

BitArray& BitArray::operator|=(const BitArray& other)
{
    bytes.resize(std::max(bytes.size(), other.bytes.size()));

    for(size_t i = 0; i < other.bytes.size(); ++i)
        bytes[i] |= other.bytes[i];

    return *this;
}
BitArray& BitArray::operator&=(const BitArray& other)
{
    bytes.resize(std::max(bytes.size(), other.bytes.size()));

    size_t i;
    for(i = 0; i < other.bytes.size(); ++i)
        bytes[i] &= other.bytes[i];

    for (; i < bytes.size(); ++i)
        bytes[i] &= 0;

    return *this;
}
BitArray& BitArray::operator^=(const BitArray& other)
{
    bytes.resize(std::max(bytes.size(), other.bytes.size()));

    for(size_t i = 0; i < other.bytes.size(); ++i)
        bytes[i] ^= other.bytes[i];

    return *this;
}
BitArray& BitArray::operator<<=(size_t shift)
{
    if (!shift || bytes.empty()) return *this;

    size_t byteShift = shift / BYTE_SIZE;
    uint8_t bitShift = shift % BYTE_SIZE;

    if (byteShift > 0)
    {
        bitCount += BYTE_SIZE * byteShift;
        bytes.insert(bytes.begin(), byteShift, std::bitset<BYTE_SIZE>{}); // prepend zero bytes
    }

    if (!bitShift) return *this;

    bitCount += bitShift;
    std::bitset<BYTE_SIZE> carry{};
    for (size_t i = byteShift; i < bytes.size(); ++i)
    {
        std::bitset<BYTE_SIZE> newCarry = (bytes[i] >> (BYTE_SIZE - bitShift));
        bytes[i] <<= bitShift;
        bytes[i] |= carry;
        carry = newCarry;
    }

    return *this;
}
BitArray& BitArray::operator>>=(size_t shift)
{
    if (!shift || bytes.empty()) return *this;

    size_t byteShift = shift / BYTE_SIZE;
    uint8_t bitShift = shift % BYTE_SIZE;

    if (byteShift > 0)
    {
        if (byteShift >= bytes.size())
        {
            bytes.clear(); bitCount = 0;
            return *this;
        }

        bytes.erase(bytes.begin(), bytes.begin() + byteShift);
    }

    if (bitShift == 0) return *this;

    std::bitset<BYTE_SIZE> carry{};
    for (size_t i = bytes.size(); i > 0; --i)
    {
        std::bitset<BYTE_SIZE> newCarry = (bytes[i] >> (BYTE_SIZE - bitShift));
        bytes[i] >>= bitShift;
        bytes[i] |= carry;
        carry = newCarry;
    }

    return *this;
}

#pragma endregion

#pragma region binary operators

BitArray BitArray::operator|(const BitArray& other) const
{
    BitArray result(other);
    result |= *this;
    return result;
}
BitArray BitArray::operator&(const BitArray& other) const
{
    BitArray result(other);
    result &= *this;
    return result;
}
BitArray BitArray::operator^(const BitArray& other) const
{
    BitArray result(other);
    result ^= *this;
    return result;
}
BitArray BitArray::operator<<(size_t shift) const
{
    BitArray result(*this);
    result <<= shift;
    return result;
}
BitArray BitArray::operator>>(size_t shift) const
{
    BitArray result(*this);
    result >>= shift;
    return result;
}

#pragma endregion

#pragma region equality

bool BitArray::operator==(const BitArray& other) const
{
    if(other.bytes.size() != bytes.size()) return false;

    for(size_t i = 0; i < bytes.size(); ++i)
        if(bytes[i] != other.bytes[i]) return false;

    return true;
}
bool BitArray::operator!=(const BitArray& other) const
{
    return !(*this == other);
}

#pragma endregion

#pragma region bit operations

BitArray& BitArray::set(size_t index, bool val)
{
    size_t byteIndex = index / BYTE_SIZE;
    size_t bitIndex  = index % BYTE_SIZE;

    if (byteIndex >= bytes.size() && !val) return *this;

    if (byteIndex >= bytes.size())
        bytes.resize(byteIndex + 1);

    bytes[byteIndex].set(bitIndex, val);

    if (val && index + 1 > bitCount) 
        bitCount = index + 1;

    return *this;
}
BitArray& BitArray::clear(size_t index)
{
    return set(index, false);
}
BitArray& BitArray::flip(size_t index)
{
    bool curr = read(index);
    return set(index, !curr);
}
bool BitArray::read(size_t index) const
{
    if(index >= bitCount) return false;

    size_t byteIndex = index / BYTE_SIZE;
    size_t bitIndex  = index % BYTE_SIZE;

    return bytes[byteIndex].test(bitIndex);
}

#pragma endregion

#ifdef _MSC_VER
#include <intrin.h>
unsigned long getMSB(unsigned long mask)
{
    unsigned long msb;
    _BitScanReverse(&msb, mask);
    return msb;
}
#elif __GNUC__
unsigned long getMSB(unsigned long mask)
{
    constexpr unsigned long max = BYTE_SIZE * sizeof(uint32_t) - 1;
    return max - __builtin_clz(mask);
}
#else
uint8_t getMSB(unsigned long mask)
{
    uint8_t msb = 0;
    uint8_t max = BYTE_SIZE - 1;
    for (uint8_t bit = max; bit >= 0; --bit)
    {
        if (mask & (1 << bit)) { msb = bit; break; }
    }
    return msb;
}
#endif

BitArray& BitArray::trim()
{
    size_t newCount = 0;
    for (size_t i = bytes.size() - 1; i >= 0; --i) {
        if (bytes[i].any()) {
            unsigned long mask = bytes[i].to_ulong();
            unsigned long msb = getMSB(mask);
            newCount = i * BYTE_SIZE + msb + 1;
            break;
        }
    }

    bitCount = newCount;
    bytes.resize((bitCount + BYTE_SIZE - 1) / BYTE_SIZE);

    return *this;
}

size_t BitArray::size() const
{
    return bitCount;
}
};
