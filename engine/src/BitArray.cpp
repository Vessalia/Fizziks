#include <BitArray.h>

namespace Fizziks::internal
{
const BitArray BitArray::Zero = BitArray(0);

BitArray::BitArray(size_t val)
{
    uint8_t mask = 0b11111111;
    size_t numBits = 0;
    do
    {
        std::bitset<BYTE_SIZE> byte { ((val >> numBits) & mask) };
        bytes.push_back(byte);
        if(byte.any()) numBits += BYTE_SIZE;
    }
    while((val >> numBits) & mask);

    auto lastByte = bytes[bytes.size() - 1];
    lastByte <<= 1; // offset by 1
    while(lastByte.any())
    {
        lastByte <<= 1;
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
bool BitArray::operator[](size_t index) const
{
    size_t byteIndex = index / BYTE_SIZE;
    uint8_t bitIndex = index % BYTE_SIZE;

    if(byteIndex > bytes.size()) return false;

    return bytes[byteIndex].test(bitIndex);
}
std::bitset<BYTE_SIZE>::reference BitArray::operator[](size_t index)
{
    size_t byteIndex = index / BYTE_SIZE;
    uint8_t bitIndex = index % BYTE_SIZE;

    if(byteIndex > bytes.size()) 
        throw std::out_of_range("BitArray::operator[] index out of range");

    return bytes[byteIndex][bitIndex];
}

#pragma endregion

#pragma region binary assignment

BitArray& BitArray::operator|=(const BitArray& other)
{
    bytes.resize(std::max(bytes.size(), other.bytes.size()));
    bitCount = std::max(bitCount, other.bitCount);

    for(size_t i = 0; i < other.bytes.size(); ++i)
        bytes[i] |= other.bytes[i];

    return *this;
}
BitArray& BitArray::operator&=(const BitArray& other)
{
    bytes.resize(std::min(bytes.size(), other.bytes.size()));
    bitCount = std::min(bitCount, other.bitCount);

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

    bitCount = countBits();

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
        bitCount -= BYTE_SIZE * byteShift;
    }

    if (bitShift == 0) return *this;

    bitCount -= bitShift;
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

#ifdef _MSC_VER
#include <intrin.h>
unsigned long MSB(unsigned long mask)
{
    unsigned long msb;
    _BitScanReverse(&msb, mask);
    return msb;
}
unsigned long LSB(unsigned long mask)
{
    unsigned long lsb;
    _BitScanForward(&lsb, mask);
    return lsb;
}
#elif __GNUC__
unsigned long MSB(unsigned long mask)
{
    constexpr unsigned long max = BYTE_SIZE * sizeof(uint32_t) - 1;
    return max - __builtin_clzl(mask);
}
unsigned long LSB(unsigned long mask)
{
    return __builtin_ctzl(mask);
}
#else
uint8_t MSB(unsigned long mask)
{
    uint8_t msb = 0;
    uint8_t max = BYTE_SIZE - 1;
    for (uint8_t bit = max; bit >= 0; --bit)
    {
        if (mask & (1 << bit)) { msb = bit; break; }
    }
    return msb;
}
uint8_t LSB(unsigned long mask)
{
    uint8_t lsb = 0;
    for (uint8_t bit = 0; bit < BYTE_SIZE; ++bit)
    {
        if (mask & (1 << bit)) { lsb = bit; break; }
    }
    return lsb;
}
#endif

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
    else if (!val && index + 1 == bitCount)
        bitCount = countBits();

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

BitArray& BitArray::trim()
{
    bitCount = countBits();
    bytes.resize(std::max(1, static_cast<int>(bitCount + BYTE_SIZE - 1) / BYTE_SIZE));

    return *this;
}

unsigned long BitArray::getMSB() const
{
    return bitCount ? bitCount - 1 : 0;
}

unsigned long BitArray::getLSB() const
{
    for (size_t i = 0; i < bytes.size(); ++i) 
    {
        auto byte = bytes[i];
        if (byte.any())
            return i * BYTE_SIZE + LSB(byte.to_ulong());
    }

    return 0;
}

size_t BitArray::size() const
{
    return bitCount;
}

size_t BitArray::countBits() const
{
    size_t count = 0;
    for(size_t i = bytes.size(); i > 0; --i) 
    {
        auto byte = bytes[i - 1];
        if (byte.any()) 
        {
            unsigned long mask = byte.to_ulong();
            unsigned long msb = MSB(mask);
            count = (i - 1) * BYTE_SIZE + msb + 1;
            break;
        }
    }

    return count;
}
}
