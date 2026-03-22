#pragma once
#include <cstddef>
#include <cstdint>

using std::size_t;
using std::uint8_t;
using std::uint16_t;
using std::uint32_t;
using std::int16_t;
using std::int32_t;

static constexpr uint8_t OUTPUT = 1;
static constexpr uint8_t INPUT = 0;
static constexpr uint8_t LOW = 0;
static constexpr uint8_t HIGH = 1;
static constexpr uint8_t MSBFIRST = 1;
static constexpr uint8_t SPI_MODE3 = 3;

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline void analogWrite(uint8_t, int) {}
inline void analogWriteResolution(int) {}
inline void analogWriteFrequency(uint8_t, uint32_t) {}
inline void delay(unsigned long) {}
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
