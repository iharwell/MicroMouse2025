#pragma once
#include <cstdint>

using std::uint8_t;
using std::int32_t;

class QuadEncoder
{
public:
    QuadEncoder(uint8_t, uint8_t, uint8_t, uint8_t = 0U) {}
    void setInitConfig() {}
    void init() {}
    int32_t read() const { return 0; }
    void write(int32_t) {}
};
