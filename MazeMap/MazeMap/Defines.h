#ifndef DEFINES_H
#define DEFINES_H

constexpr float AVG_SPD_WEIGHT = 0.3f;
//#define SMALL_RDD
constexpr int PATH_SIZE = 256;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float HALF_PI_F = 0.5f * PI_F;
constexpr float TWO_PI_F = 2.0f * PI_F;
constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
constexpr float RAD_TO_DEG_F = 180.0f / PI_F;
constexpr float RT2 = 1.414213562f;
constexpr float HALF_RT2 = 0.707106781f;
constexpr float WALL_THICKNESS = 0.012f;
constexpr float MIN_CLEARANCE = 0.012f;
constexpr float GRAVITY_MPS2 = 9.80665f;

#include <assert.h>
#if defined(ARDUINO) || defined(CORE_TEENSY) || defined(ARDUINO_TEENSY41)
#include <arm_math.h>
#include <Arduino.h>
#include <QuadEncoder.h>
#include <new>

/*
*   void arm_sin_cos_f32(
  float32_t theta,
  float32_t * pSinVal,
  float32_t * pCosVal);
*/

inline void sin_cosf(float theta, float& sin, float& cos)
{
	theta = 180.0f * theta / PI_F;
    arm_sin_cos_f32(theta, &sin, &cos);
}

#ifndef EXPORT
#define EXPORT
#endif

#ifndef IMPORT
#define IMPORT
#endif

#else

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <istream>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <thread>
#include <new>

inline void sin_cosf(float theta, float& sin, float& cos)
{
    sin = std::sinf(theta);
    cos = std::cosf(theta);
}
// Host builds avoid defining Arduino's boolean alias to prevent Windows RPC type collisions.
using byte = std::uint8_t;
using word = std::uint16_t;

using std::int8_t;
using std::uint8_t;
using std::int16_t;
using std::uint16_t;
using std::int32_t;
using std::uint32_t;
using std::int64_t;
using std::uint64_t;
using std::size_t;

inline constexpr int HIGH = 0x1;
inline constexpr int LOW = 0x0;
#ifndef INPUT
#define INPUT 0x0
#endif

#ifndef OUTPUT
#define OUTPUT 0x1
#endif

#ifndef INPUT_PULLUP
#define INPUT_PULLUP 0x2
#endif

#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN 0x3
#endif
inline constexpr int CHANGE = 1;
inline constexpr int FALLING = 2;
inline constexpr int RISING = 3;
inline constexpr int DEFAULT = 1;
inline constexpr int EXTERNAL = 0;
inline constexpr int INTERNAL = 2;
inline constexpr int INTERNAL1V1 = 3;
inline constexpr int INTERNAL2V56 = 4;
inline constexpr int DEC = 10;
inline constexpr int HEX = 16;
inline constexpr int OCT = 8;
inline constexpr int BIN = 2;
inline constexpr int LSBFIRST = 0;
inline constexpr int MSBFIRST = 1;
inline constexpr double PI = 3.1415926535897932384626433832795;
inline constexpr double HALF_PI = 1.5707963267948966192313216916398;
inline constexpr double TWO_PI = 6.283185307179586476925286766559;
inline constexpr double DEG_TO_RAD = 0.017453292519943295769236907684886;
inline constexpr double RAD_TO_DEG = 57.295779513082320876798154814105;

#ifndef F_CPU
#define F_CPU 600000000UL
#endif

#ifndef BUILTIN_SDCARD
#define BUILTIN_SDCARD 254
#endif

#ifndef FLASHMEM
#define FLASHMEM
#endif

#ifndef FASTRUN
#define FASTRUN
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef DMAMEM
#define DMAMEM
#endif

template <typename TBit>
constexpr unsigned long bit(TBit bitIndex) noexcept
{
    return 1UL << bitIndex;
}

template <typename TValue, typename TBit>
constexpr unsigned long bitRead(TValue value, TBit bitIndex) noexcept
{
    return (static_cast<unsigned long>(value) >> bitIndex) & 0x1UL;
}

template <typename TValue, typename TBit>
constexpr TValue& bitSet(TValue& value, TBit bitIndex) noexcept
{
    value = static_cast<TValue>(value | static_cast<TValue>(1UL << bitIndex));
    return value;
}

template <typename TValue, typename TBit>
constexpr TValue& bitClear(TValue& value, TBit bitIndex) noexcept
{
    value = static_cast<TValue>(value & static_cast<TValue>(~(1UL << bitIndex)));
    return value;
}

template <typename TValue, typename TBit>
constexpr TValue& bitWrite(TValue& value, TBit bitIndex, bool bitValue) noexcept
{
    return bitValue ? bitSet(value, bitIndex) : bitClear(value, bitIndex);
}

template <typename TValue>
constexpr uint8_t lowByte(TValue value) noexcept
{
    return static_cast<uint8_t>(value & 0xFFU);
}

template <typename TValue>
constexpr uint8_t highByte(TValue value) noexcept
{
    return static_cast<uint8_t>((value >> 8) & 0xFFU);
}

template <typename TValue>
constexpr auto sq(TValue value) noexcept
{
    return value * value;
}

template <typename TValue>
constexpr TValue constrain(TValue amount, TValue low, TValue high) noexcept
{
    return (amount < low) ? low : ((amount > high) ? high : amount);
}

template <typename TValue>
constexpr auto radians(TValue degreesValue) noexcept
{
    return degreesValue * DEG_TO_RAD;
}

template <typename TValue>
constexpr auto degrees(TValue radiansValue) noexcept
{
    return radiansValue * RAD_TO_DEG;
}

inline void cli();
inline void sei();

inline void noInterrupts() noexcept
{
    cli();
}

inline void interrupts() noexcept
{
    sei();
}

inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (in_max == in_min)
        ? out_min
        : (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

namespace MazeMap::arduino_stub_detail
{
    constexpr size_t kHostDigitalPinCapacity = 256U;

    inline std::chrono::steady_clock::time_point start_time()
    {
        static const auto t0 = std::chrono::steady_clock::now();
        return t0;
    }

    struct HostDigitalPinState
    {
        uint8_t mode = INPUT;
        uint8_t outputValue = LOW;
        int inputValue = LOW;
        bool hasInputOverride = false;
    };

    inline auto& host_pin_states()
    {
        static std::array<HostDigitalPinState, kHostDigitalPinCapacity> states{};
        return states;
    }

    inline auto& host_pin_shorts()
    {
        static std::array<std::array<bool, kHostDigitalPinCapacity>, kHostDigitalPinCapacity> shorts{};
        return shorts;
    }

    inline bool is_valid_host_pin(uint8_t pin)
    {
        return static_cast<size_t>(pin) < kHostDigitalPinCapacity;
    }

    inline void reset_host_digital_pins()
    {
        auto& states = host_pin_states();
        for (auto& state : states)
        {
            state = HostDigitalPinState{};
        }

        auto& shorts = host_pin_shorts();
        for (auto& row : shorts)
        {
            row.fill(false);
        }
    }

    inline void set_host_pin_short(uint8_t pinA, uint8_t pinB, bool connected)
    {
        if (!is_valid_host_pin(pinA) || !is_valid_host_pin(pinB))
        {
            return;
        }

        auto& shorts = host_pin_shorts();
        shorts[pinA][pinB] = connected;
        shorts[pinB][pinA] = connected;
    }

    inline void set_host_digital_input(uint8_t pin, int value)
    {
        if (!is_valid_host_pin(pin))
        {
            return;
        }

        HostDigitalPinState& state = host_pin_states()[pin];
        state.inputValue = (value == LOW) ? LOW : HIGH;
        state.hasInputOverride = true;
    }

    inline void clear_host_digital_input(uint8_t pin)
    {
        if (!is_valid_host_pin(pin))
        {
            return;
        }

        host_pin_states()[pin].hasInputOverride = false;
    }

    inline int resolve_host_digital_read(uint8_t pin)
    {
        if (!is_valid_host_pin(pin))
        {
            return LOW;
        }

        const auto& states = host_pin_states();
        const HostDigitalPinState& state = states[pin];

        if (state.hasInputOverride)
        {
            return state.inputValue;
        }

        if (state.mode == OUTPUT)
        {
            return state.outputValue;
        }

        int resolved = (state.mode == INPUT_PULLUP) ? HIGH : LOW;
        const auto& shorts = host_pin_shorts();
        for (size_t otherPin = 0; otherPin < kHostDigitalPinCapacity; ++otherPin)
        {
            if (!shorts[pin][otherPin] || otherPin == static_cast<size_t>(pin))
            {
                continue;
            }

            const HostDigitalPinState& otherState = states[otherPin];
            int otherValue = LOW;
            if (otherState.hasInputOverride)
            {
                otherValue = otherState.inputValue;
            }
            else if (otherState.mode == OUTPUT)
            {
                otherValue = otherState.outputValue;
            }
            else if (otherState.mode == INPUT_PULLUP)
            {
                otherValue = HIGH;
            }
            else
            {
                otherValue = LOW;
            }

            if (otherValue == LOW)
            {
                return LOW;
            }
            resolved = HIGH;
        }

        return resolved;
    }
}

namespace arduino_stub_detail = MazeMap::arduino_stub_detail;

inline unsigned long millis()
{
    const auto now = std::chrono::steady_clock::now();
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - arduino_stub_detail::start_time()).count());
}

inline unsigned long micros()
{
    const auto now = std::chrono::steady_clock::now();
    return static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - arduino_stub_detail::start_time()).count());
}

inline void delay(unsigned long ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline void delayMicroseconds(unsigned int us)
{
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

class elapsedMillis
{
public:
    elapsedMillis() : start_(millis()) {}
    explicit elapsedMillis(unsigned long value) : start_(millis() - value) {}

    operator unsigned long() const
    {
        return millis() - start_;
    }

    elapsedMillis &operator=(unsigned long value)
    {
        start_ = millis() - value;
        return *this;
    }

private:
    unsigned long start_;
};

class elapsedMicros
{
public:
    elapsedMicros() : start_(micros()) {}
    explicit elapsedMicros(unsigned long value) : start_(micros() - value) {}

    operator unsigned long() const
    {
        return micros() - start_;
    }

    elapsedMicros &operator=(unsigned long value)
    {
        start_ = micros() - value;
        return *this;
    }

private:
    unsigned long start_;
};

using InterruptCallback = void (*)();

inline void yield() {}
inline void cli() {}
inline void sei() {}

inline void HostResetDigitalPins()
{
    arduino_stub_detail::reset_host_digital_pins();
}

inline void HostSetPinShort(uint8_t pinA, uint8_t pinB, bool connected = true)
{
    arduino_stub_detail::set_host_pin_short(pinA, pinB, connected);
}

inline void HostSetDigitalInput(uint8_t pin, int value)
{
    arduino_stub_detail::set_host_digital_input(pin, value);
}

inline void HostClearDigitalInput(uint8_t pin)
{
    arduino_stub_detail::clear_host_digital_input(pin);
}

inline void pinMode(uint8_t pin, uint8_t mode)
{
    if (!arduino_stub_detail::is_valid_host_pin(pin))
    {
        return;
    }

    arduino_stub_detail::host_pin_states()[pin].mode = mode;
}

inline void digitalWrite(uint8_t pin, uint8_t value)
{
    if (!arduino_stub_detail::is_valid_host_pin(pin))
    {
        return;
    }

    arduino_stub_detail::host_pin_states()[pin].outputValue = (value == LOW) ? LOW : HIGH;
}

inline int digitalRead(uint8_t pin)
{
    return arduino_stub_detail::resolve_host_digital_read(pin);
}
inline void digitalWriteFast(uint8_t pin, uint8_t value) { digitalWrite(pin, value); }
inline int digitalReadFast(uint8_t pin) { return digitalRead(pin); }

inline void analogWrite(uint8_t, int) {}
inline int analogRead(uint8_t) { return 0; }
inline void analogReference(uint8_t) {}
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}
inline void analogWriteFrequency(uint8_t, uint32_t) {}

inline void tone(uint8_t, unsigned int) {}
inline void tone(uint8_t, unsigned int, unsigned long) {}
inline void noTone(uint8_t) {}

inline void shiftOut(uint8_t, uint8_t, uint8_t, uint8_t) {}
inline uint8_t shiftIn(uint8_t, uint8_t, uint8_t) { return 0; }

inline unsigned long pulseIn(uint8_t, uint8_t, unsigned long = 1000000UL) { return 0; }
inline unsigned long pulseInLong(uint8_t, uint8_t, unsigned long = 1000000UL) { return 0; }

inline void attachInterrupt(uint8_t, InterruptCallback, int) {}
inline void detachInterrupt(uint8_t) {}
inline int digitalPinToInterrupt(uint8_t pin) { return static_cast<int>(pin); }

inline long random(long max)
{
    return (max <= 0) ? 0 : (std::rand() % max);
}

inline long random(long min_value, long max_value)
{
    return (max_value <= min_value) ? min_value : (min_value + (std::rand() % (max_value - min_value)));
}

inline void randomSeed(unsigned long seed)
{
    std::srand(static_cast<unsigned int>(seed));
}

class String
{
public:
    String() = default;
    String(const char *value) : storage_(value ? value : "") {}
    String(char value) : storage_(1, value) {}
    String(const std::string &value) : storage_(value) {}
    String(int value) : storage_(std::to_string(value)) {}
    String(unsigned int value) : storage_(std::to_string(value)) {}
    String(long value) : storage_(std::to_string(value)) {}
    String(unsigned long value) : storage_(std::to_string(value)) {}
    String(long long value) : storage_(std::to_string(value)) {}
    String(unsigned long long value) : storage_(std::to_string(value)) {}
    String(float value) : storage_(std::to_string(value)) {}
    String(double value) : storage_(std::to_string(value)) {}

    const char *c_str() const { return storage_.c_str(); }
    std::size_t length() const { return storage_.length(); }
    bool isEmpty() const { return storage_.empty(); }

    String &operator+=(const String &rhs)
    {
        storage_ += rhs.storage_;
        return *this;
    }

    friend String operator+(const String &lhs, const String &rhs)
    {
        return String(lhs.storage_ + rhs.storage_);
    }

    bool operator==(const String &rhs) const { return storage_ == rhs.storage_; }
    bool operator!=(const String &rhs) const { return storage_ != rhs.storage_; }

private:
    std::string storage_;
};

class Print
{
public:
    virtual ~Print() = default;

    virtual std::size_t write(std::uint8_t value)
    {
        (void)value;
        return 1;
    }

    virtual std::size_t write(const std::uint8_t *buffer, std::size_t size)
    {
        if (buffer == nullptr)
        {
            return 0;
        }

        for (std::size_t i = 0; i < size; ++i)
        {
            write(buffer[i]);
        }

        return size;
    }

    std::size_t write(const char *value)
    {
        if (value == nullptr)
        {
            return 0;
        }

        return write(reinterpret_cast<const std::uint8_t *>(value), std::strlen(value));
    }

    template <typename T>
    void print(const T &value)
    {
        std::ostringstream oss;
        oss << value;
        write(oss.str().c_str());
    }

    void print(const String &value)
    {
        write(value.c_str());
    }

    template <typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
    void print(T value, int base)
    {
        std::ostringstream oss;
        switch (base)
        {
        case HEX:
            oss << std::uppercase << std::hex << static_cast<unsigned long long>(value);
            break;
        case OCT:
            oss << std::oct << static_cast<unsigned long long>(value);
            break;
        case BIN:
        {
            const auto unsignedValue = static_cast<unsigned long long>(value);
            if (unsignedValue == 0ULL)
            {
                oss << '0';
                break;
            }

            bool wroteBit = false;
            for (int bit = static_cast<int>(sizeof(unsigned long long) * 8U) - 1; bit >= 0; --bit)
            {
                const bool set = ((unsignedValue >> bit) & 0x1ULL) != 0ULL;
                wroteBit |= set;
                if (wroteBit)
                {
                    oss << (set ? '1' : '0');
                }
            }
            break;
        }
        case DEC:
        default:
            oss << value;
            break;
        }

        write(oss.str().c_str());
    }

    void print(float value, int digits)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(digits) << value;
        write(oss.str().c_str());
    }

    void print(double value, int digits)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(digits) << value;
        write(oss.str().c_str());
    }

    template <typename T>
    void println(const T &value)
    {
        print(value);
        write("\r\n");
    }

    void println(float value, int digits)
    {
        print(value, digits);
        write("\r\n");
    }

    void println(double value, int digits)
    {
        print(value, digits);
        write("\r\n");
    }

    void println()
    {
        write("\r\n");
    }
};

class Stream : public Print
{
public:
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    virtual void flush() {}
};

template <typename T>
inline constexpr bool kSerialTextOutputIsBanned = false;

class HardwareSerial : public Stream
{
public:
    void begin(unsigned long) {}
    void end() {}

    template <typename T>
    void print(const T &)
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    template <typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
    void print(T, int)
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    template <typename T = float>
    void print(float, int)
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    template <typename T = double>
    void print(double, int)
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    template <typename T>
    void println(const T &)
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    template <typename T = void>
    void println()
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    template <typename T = float>
    void println(float, int)
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    template <typename T = double>
    void println(double, int)
    {
        static_assert(
            kSerialTextOutputIsBanned<T>,
            "Serial output cannot be captured, so use the SharedRobotRuntime methods instead.");
    }

    std::size_t write(std::uint8_t value) override
    {
        std::cout.put(static_cast<char>(value));
        return 1;
    }

    void flush() override
    {
        std::cout.flush();
    }

    using Print::write;
};

static HardwareSerial Serial;
static HardwareSerial Serial1;
static HardwareSerial Serial2;
static HardwareSerial Serial3;
static HardwareSerial Serial4;
static HardwareSerial Serial5;
static HardwareSerial Serial6;
static HardwareSerial Serial7;
static HardwareSerial Serial8;

class QuadEncoder
{
public:
    QuadEncoder(uint8_t channel = 0U, uint8_t pinA = 0U, uint8_t pinB = 0U, uint8_t pullups = 0U)
        : channel_(channel)
        , pin_a_(pinA)
        , pin_b_(pinB)
        , pullups_(pullups)
    {
    }

    void setInitConfig() {}
    void init() {}

    int32_t read() const
    {
        return count_;
    }

    void write(int32_t value)
    {
        count_ = value;
    }

private:
    uint8_t channel_;
    uint8_t pin_a_;
    uint8_t pin_b_;
    uint8_t pullups_;
    int32_t count_ = 0;
};

#ifndef IMXRT_FLEXPWM1_ADDRESS
#define IMXRT_FLEXPWM1_ADDRESS 0x403DC000u
#endif

#ifndef IMXRT_FLEXPWM2_ADDRESS
#define IMXRT_FLEXPWM2_ADDRESS 0x403E0000u
#endif

#ifndef IMXRT_FLEXPWM3_ADDRESS
#define IMXRT_FLEXPWM3_ADDRESS 0x403E4000u
#endif

#ifndef IMXRT_FLEXPWM4_ADDRESS
#define IMXRT_FLEXPWM4_ADDRESS 0x403E8000u
#endif

typedef struct
{
    volatile uint16_t CNT;
    volatile uint16_t INIT;
    volatile uint16_t CTRL2;
    volatile uint16_t CTRL;
    volatile uint16_t unused1;
    volatile uint16_t VAL0;
    volatile uint16_t FRACVAL1;
    volatile uint16_t VAL1;
    volatile uint16_t FRACVAL2;
    volatile uint16_t VAL2;
    volatile uint16_t FRACVAL3;
    volatile uint16_t VAL3;
    volatile uint16_t FRACVAL4;
    volatile uint16_t VAL4;
    volatile uint16_t FRACVAL5;
    volatile uint16_t VAL5;
    volatile uint16_t FRCTRL;
    volatile uint16_t OCTRL;
    volatile uint16_t STS;
    volatile uint16_t INTEN;
    volatile uint16_t DMAEN;
    volatile uint16_t TCTRL;
    volatile uint16_t DISMAP0;
    volatile uint16_t DISMAP1;
    volatile uint16_t DTCNT0;
    volatile uint16_t DTCNT1;
    volatile uint16_t CAPTCTRLA;
    volatile uint16_t CAPTCOMPA;
    volatile uint16_t CAPTCTRLB;
    volatile uint16_t CAPTCOMPB;
    volatile uint16_t CAPTCTRLX;
    volatile uint16_t CAPTCOMPX;
    volatile uint16_t CVAL0;
    volatile uint16_t CVAL0CYC;
    volatile uint16_t CVAL1;
    volatile uint16_t CVAL1CYC;
    volatile uint16_t CVAL2;
    volatile uint16_t CVAL2CYC;
    volatile uint16_t CVAL3;
    volatile uint16_t CVAL3CYC;
    volatile uint16_t CVAL4;
    volatile uint16_t CVAL4CYC;
    volatile uint16_t CVAL5;
    volatile uint16_t CVAL5CYC;
    volatile uint16_t unused2;
    volatile uint16_t unused3;
    volatile uint16_t unused4;
    volatile uint16_t unused5;
} IMXRT_FLEXPWM_SM_t;

typedef struct
{
    IMXRT_FLEXPWM_SM_t SM[4];
    volatile uint16_t OUTEN;
    volatile uint16_t MASK;
    volatile uint16_t SWCOUT;
    volatile uint16_t DTSRCSEL;
    volatile uint16_t MCTRL;
    volatile uint16_t MCTRL2;
    volatile uint16_t FCTRL0;
    volatile uint16_t FSTS0;
    volatile uint16_t FFILT0;
    volatile uint16_t FTST0;
    volatile uint16_t FCTRL20;
} IMXRT_FLEXPWM_t;

static IMXRT_FLEXPWM_t IMXRT_FLEXPWM1_StubStorage{};
static IMXRT_FLEXPWM_t IMXRT_FLEXPWM2_StubStorage{};
static IMXRT_FLEXPWM_t IMXRT_FLEXPWM3_StubStorage{};
static IMXRT_FLEXPWM_t IMXRT_FLEXPWM4_StubStorage{};

#define IMXRT_FLEXPWM1 (IMXRT_FLEXPWM1_StubStorage)
#define IMXRT_FLEXPWM2 (IMXRT_FLEXPWM2_StubStorage)
#define IMXRT_FLEXPWM3 (IMXRT_FLEXPWM3_StubStorage)
#define IMXRT_FLEXPWM4 (IMXRT_FLEXPWM4_StubStorage)

#define FLEXPWM_SM_REG(module_, sm_, reg_) ((module_).SM[(sm_)].reg_)
#define FLEXPWM_SM(module_, sm_) ((module_).SM[(sm_)])

#define FLEXPWM_SMCTRL2_DBGEN ((uint16_t)(1u << 15))
#define FLEXPWM_SMCTRL2_WAITEN ((uint16_t)(1u << 14))
#define FLEXPWM_SMCTRL2_INDEP ((uint16_t)(1u << 13))
#define FLEXPWM_SMCTRL2_PWM23_INIT ((uint16_t)(1u << 12))
#define FLEXPWM_SMCTRL2_PWM45_INIT ((uint16_t)(1u << 11))
#define FLEXPWM_SMCTRL2_PWMX_INIT ((uint16_t)(1u << 10))
#define FLEXPWM_SMCTRL2_INIT_SEL(n) ((uint16_t)(((n) & 0x03u) << 8))
#define FLEXPWM_SMCTRL2_FRCEN ((uint16_t)(1u << 7))
#define FLEXPWM_SMCTRL2_FORCE ((uint16_t)(1u << 6))
#define FLEXPWM_SMCTRL2_FORCE_SEL(n) ((uint16_t)(((n) & 0x07u) << 3))
#define FLEXPWM_SMCTRL2_RELOAD_SEL ((uint16_t)(1u << 2))
#define FLEXPWM_SMCTRL2_CLK_SEL(n) ((uint16_t)(((n) & 0x03u) << 0))

#define FLEXPWM_SMCTRL_LDFQ(n) ((uint16_t)(((n) & 0x0Fu) << 12))
#define FLEXPWM_SMCTRL_HALF ((uint16_t)(1u << 11))
#define FLEXPWM_SMCTRL_FULL ((uint16_t)(1u << 10))
#define FLEXPWM_SMCTRL_DT(n) ((uint16_t)(((n) & 0x03u) << 8))
#define FLEXPWM_SMCTRL_COMPMODE ((uint16_t)(1u << 7))
#define FLEXPWM_SMCTRL_PRSC(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_SMCTRL_SPLIT ((uint16_t)(1u << 3))
#define FLEXPWM_SMCTRL_LDMOD ((uint16_t)(1u << 2))
#define FLEXPWM_SMCTRL_DBLX ((uint16_t)(1u << 1))
#define FLEXPWM_SMCTRL_DBLEN ((uint16_t)(1u << 0))

#define FLEXPWM_SMFRCTRL_TEST ((uint16_t)(1u << 15))
#define FLEXPWM_SMFRCTRL_FRAC_PU ((uint16_t)(1u << 8))
#define FLEXPWM_SMFRCTRL_FRAC45_EN ((uint16_t)(1u << 4))
#define FLEXPWM_SMFRCTRL_FRAC23_EN ((uint16_t)(1u << 2))
#define FLEXPWM_SMFRCTRL_FRAC1_EN ((uint16_t)(1u << 1))

#define FLEXPWM_SMOCTRL_PWMA_IN ((uint16_t)(1u << 15))
#define FLEXPWM_SMOCTRL_PWMB_IN ((uint16_t)(1u << 14))
#define FLEXPWM_SMOCTRL_PWMX_IN ((uint16_t)(1u << 13))
#define FLEXPWM_SMOCTRL_POLA ((uint16_t)(1u << 10))
#define FLEXPWM_SMOCTRL_POLB ((uint16_t)(1u << 9))
#define FLEXPWM_SMOCTRL_POLX ((uint16_t)(1u << 8))
#define FLEXPWM_SMOCTRL_PWMAFS(n) ((uint16_t)(((n) & 0x03u) << 4))
#define FLEXPWM_SMOCTRL_PWMBFS(n) ((uint16_t)(((n) & 0x03u) << 2))
#define FLEXPWM_SMOCTRL_PWMXFS(n) ((uint16_t)(((n) & 0x03u) << 0))

#define FLEXPWM_SMSTS_RUF ((uint16_t)(1u << 14))
#define FLEXPWM_SMSTS_REF ((uint16_t)(1u << 13))
#define FLEXPWM_SMSTS_RF ((uint16_t)(1u << 12))
#define FLEXPWM_SMSTS_CFA1 ((uint16_t)(1u << 11))
#define FLEXPWM_SMSTS_CFA0 ((uint16_t)(1u << 10))
#define FLEXPWM_SMSTS_CFB1 ((uint16_t)(1u << 9))
#define FLEXPWM_SMSTS_CFB0 ((uint16_t)(1u << 8))
#define FLEXPWM_SMSTS_CFX1 ((uint16_t)(1u << 7))
#define FLEXPWM_SMSTS_CFX0 ((uint16_t)(1u << 6))
#define FLEXPWM_SMSTS_CMPF(n) ((uint16_t)(((n) & 0x3Fu) << 0))

#define FLEXPWM_SMINTEN_REIE ((uint16_t)(1u << 13))
#define FLEXPWM_SMINTEN_RIE ((uint16_t)(1u << 12))
#define FLEXPWM_SMINTEN_CA1IE ((uint16_t)(1u << 11))
#define FLEXPWM_SMINTEN_CA0IE ((uint16_t)(1u << 10))
#define FLEXPWM_SMINTEN_CB1IE ((uint16_t)(1u << 9))
#define FLEXPWM_SMINTEN_CB0IE ((uint16_t)(1u << 8))
#define FLEXPWM_SMINTEN_CX1IE ((uint16_t)(1u << 7))
#define FLEXPWM_SMINTEN_CX0IE ((uint16_t)(1u << 6))
#define FLEXPWM_SMINTEN_CMPIE(n) ((uint16_t)(((n) & 0x3Fu) << 0))

#define FLEXPWM_SMDMAEN_VALDE ((uint16_t)(1u << 9))
#define FLEXPWM_SMDMAEN_FAND ((uint16_t)(1u << 8))
#define FLEXPWM_SMDMAEN_CAPTDE(n) ((uint16_t)(((n) & 0x03u) << 6))
#define FLEXPWM_SMDMAEN_CA1DE ((uint16_t)(1u << 5))
#define FLEXPWM_SMDMAEN_CA0DE ((uint16_t)(1u << 4))
#define FLEXPWM_SMDMAEN_CB1DE ((uint16_t)(1u << 3))
#define FLEXPWM_SMDMAEN_CB0DE ((uint16_t)(1u << 2))
#define FLEXPWM_SMDMAEN_CX1DE ((uint16_t)(1u << 1))
#define FLEXPWM_SMDMAEN_CX0DE ((uint16_t)(1u << 0))

#define FLEXPWM_SMTCTRL_PWAOT0 ((uint16_t)(1u << 15))
#define FLEXPWM_SMTCTRL_PWBOT1 ((uint16_t)(1u << 14))
#define FLEXPWM_SMTCTRL_TRGFRQ ((uint16_t)(1u << 12))
#define FLEXPWM_SMTCTRL_OUT_TRIG_EN(n) ((uint16_t)(((n) & 0x3Fu) << 0))

#define FLEXPWM_SMDISMAP0_DIS0X(n) ((uint16_t)(((n) & 0x0Fu) << 8))
#define FLEXPWM_SMDISMAP0_DIS0B(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_SMDISMAP0_DIS0A(n) ((uint16_t)(((n) & 0x0Fu) << 0))

#define FLEXPWM_SMDISMAP1_DIS1X(n) ((uint16_t)(((n) & 0x0Fu) << 8))
#define FLEXPWM_SMDISMAP1_DIS1B(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_SMDISMAP1_DIS1A(n) ((uint16_t)(((n) & 0x0Fu) << 0))

#define FLEXPWM_SMCAPTCTRLA_CA1CNT(n) ((uint16_t)(((n) & 0x07u) << 13))
#define FLEXPWM_SMCAPTCTRLA_CA0CNT(n) ((uint16_t)(((n) & 0x07u) << 10))
#define FLEXPWM_SMCAPTCTRLA_CFAWM(n) ((uint16_t)(((n) & 0x03u) << 8))
#define FLEXPWM_SMCAPTCTRLA_EDGCNTA_EN ((uint16_t)(1u << 7))
#define FLEXPWM_SMCAPTCTRLA_INP_SELA ((uint16_t)(1u << 6))
#define FLEXPWM_SMCAPTCTRLA_EDGA1(n) ((uint16_t)(((n) & 0x03u) << 4))
#define FLEXPWM_SMCAPTCTRLA_EDGA0(n) ((uint16_t)(((n) & 0x03u) << 2))
#define FLEXPWM_SMCAPTCTRLA_ONESHOTA ((uint16_t)(1u << 1))
#define FLEXPWM_SMCAPTCTRLA_ARMA ((uint16_t)(1u << 0))

#define FLEXPWM_SMCAPTCOMPA_EDGCNTA(n) ((uint16_t)(((n) & 0xFFu) << 8))
#define FLEXPWM_SMCAPTCOMPA_EDGCMPA(n) ((uint16_t)(((n) & 0xFFu) << 0))

#define FLEXPWM_SMCAPTCTRLB_CB1CNT(n) ((uint16_t)(((n) & 0x07u) << 13))
#define FLEXPWM_SMCAPTCTRLB_CB0CNT(n) ((uint16_t)(((n) & 0x07u) << 10))
#define FLEXPWM_SMCAPTCTRLB_CFBWM(n) ((uint16_t)(((n) & 0x03u) << 8))
#define FLEXPWM_SMCAPTCTRLB_EDGCNTB_EN ((uint16_t)(1u << 7))
#define FLEXPWM_SMCAPTCTRLB_INP_SELB ((uint16_t)(1u << 6))
#define FLEXPWM_SMCAPTCTRLB_EDGB1(n) ((uint16_t)(((n) & 0x03u) << 4))
#define FLEXPWM_SMCAPTCTRLB_EDGB0(n) ((uint16_t)(((n) & 0x03u) << 2))
#define FLEXPWM_SMCAPTCTRLB_ONESHOTB ((uint16_t)(1u << 1))
#define FLEXPWM_SMCAPTCTRLB_ARMB ((uint16_t)(1u << 0))

#define FLEXPWM_SMCAPTCOMPB_EDGCNTB(n) ((uint16_t)(((n) & 0xFFu) << 8))
#define FLEXPWM_SMCAPTCOMPB_EDGCMPB(n) ((uint16_t)(((n) & 0xFFu) << 0))

#define FLEXPWM_SMCAPTCTRLX_CX1CNT(n) ((uint16_t)(((n) & 0x07u) << 13))
#define FLEXPWM_SMCAPTCTRLX_CX0CNT(n) ((uint16_t)(((n) & 0x07u) << 10))
#define FLEXPWM_SMCAPTCTRLX_CFXWM(n) ((uint16_t)(((n) & 0x03u) << 8))
#define FLEXPWM_SMCAPTCTRLX_EDGCNTX_EN ((uint16_t)(1u << 7))
#define FLEXPWM_SMCAPTCTRLX_INP_SELX ((uint16_t)(1u << 6))
#define FLEXPWM_SMCAPTCTRLX_EDGX1(n) ((uint16_t)(((n) & 0x03u) << 4))
#define FLEXPWM_SMCAPTCTRLX_EDGX0(n) ((uint16_t)(((n) & 0x03u) << 2))
#define FLEXPWM_SMCAPTCTRLX_ONESHOTX ((uint16_t)(1u << 1))
#define FLEXPWM_SMCAPTCTRLX_ARMX ((uint16_t)(1u << 0))

#define FLEXPWM_SMCAPTCOMPX_EDGCNTX(n) ((uint16_t)(((n) & 0xFFu) << 8))
#define FLEXPWM_SMCAPTCOMPX_EDGCMPX(n) ((uint16_t)(((n) & 0xFFu) << 0))

#define FLEXPWM_OUTEN_PWMA_EN(n) ((uint16_t)(((n) & 0x0Fu) << 8))
#define FLEXPWM_OUTEN_PWMB_EN(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_OUTEN_PWMX_EN(n) ((uint16_t)(((n) & 0x0Fu) << 0))

#define FLEXPWM_MASK_UPDATE_MASK(n) ((uint16_t)(((n) & 0x0Fu) << 12))
#define FLEXPWM_MASK_MASKA(n) ((uint16_t)(((n) & 0x0Fu) << 8))
#define FLEXPWM_MASK_MASKB(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_MASK_MASKX(n) ((uint16_t)(((n) & 0x0Fu) << 0))

#define FLEXPWM_SWCOUT_SM3OUT23 ((uint16_t)(1u << 7))
#define FLEXPWM_SWCOUT_SM3OUT45 ((uint16_t)(1u << 6))
#define FLEXPWM_SWCOUT_SM2OUT23 ((uint16_t)(1u << 5))
#define FLEXPWM_SWCOUT_SM2OUT45 ((uint16_t)(1u << 4))
#define FLEXPWM_SWCOUT_SM1OUT23 ((uint16_t)(1u << 3))
#define FLEXPWM_SWCOUT_SM1OUT45 ((uint16_t)(1u << 2))
#define FLEXPWM_SWCOUT_SM0OUT23 ((uint16_t)(1u << 1))
#define FLEXPWM_SWCOUT_SM0OUT45 ((uint16_t)(1u << 0))

#define FLEXPWM_DTSRCSEL_SM3SEL23(n) ((uint16_t)(((n) & 0x03u) << 14))
#define FLEXPWM_DTSRCSEL_SM3SEL45(n) ((uint16_t)(((n) & 0x03u) << 12))
#define FLEXPWM_DTSRCSEL_SM2SEL23(n) ((uint16_t)(((n) & 0x03u) << 10))
#define FLEXPWM_DTSRCSEL_SM2SEL45(n) ((uint16_t)(((n) & 0x03u) << 8))
#define FLEXPWM_DTSRCSEL_SM1SEL23(n) ((uint16_t)(((n) & 0x03u) << 6))
#define FLEXPWM_DTSRCSEL_SM1SEL45(n) ((uint16_t)(((n) & 0x03u) << 4))
#define FLEXPWM_DTSRCSEL_SM0SEL23(n) ((uint16_t)(((n) & 0x03u) << 2))
#define FLEXPWM_DTSRCSEL_SM0SEL45(n) ((uint16_t)(((n) & 0x03u) << 0))

#define FLEXPWM_MCTRL_IPOL(n) ((uint16_t)(((n) & 0x0Fu) << 12))
#define FLEXPWM_MCTRL_RUN(n) ((uint16_t)(((n) & 0x0Fu) << 8))
#define FLEXPWM_MCTRL_CLDOK(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_MCTRL_LDOK(n) ((uint16_t)(((n) & 0x0Fu) << 0))

#define FLEXPWM_MCTRL2_MONPLL(n) ((uint16_t)(((n) & 0x03u) << 0))

#define FLEXPWM_FCTRL0_FLVL(n) ((uint16_t)(((n) & 0x0Fu) << 12))
#define FLEXPWM_FCTRL0_FAUTO(n) ((uint16_t)(((n) & 0x0Fu) << 8))
#define FLEXPWM_FCTRL0_FSAFE(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_FCTRL0_FIE(n) ((uint16_t)(((n) & 0x0Fu) << 0))

#define FLEXPWM_FSTS0_FHALF(n) ((uint16_t)(((n) & 0x0Fu) << 12))
#define FLEXPWM_FSTS0_FFPIN(n) ((uint16_t)(((n) & 0x0Fu) << 8))
#define FLEXPWM_FSTS0_FFULL(n) ((uint16_t)(((n) & 0x0Fu) << 4))
#define FLEXPWM_FSTS0_FFLAG(n) ((uint16_t)(((n) & 0x0Fu) << 0))

#define FLEXPWM_FFILT0_GSTR ((uint16_t)(1u << 15))
#define FLEXPWM_FFILT0_FILT_CNT(n) ((uint16_t)(((n) & 0x07u) << 8))
#define FLEXPWM_FFILT0_FILT_PER(n) ((uint16_t)(((n) & 0xFFu) << 0))

#define FLEXPWM_FTST0_FTEST ((uint16_t)(1u << 0))

#define FLEXPWM_FCTRL20_NOCOMB(n) ((uint16_t)(((n) & 0x0Fu) << 0))

inline void init() {}
inline void initVariant() {}
#ifndef _MSC_VER
inline int atexit(void (*)()) { return 0; }
#endif

#if defined(_MSC_VER)
#ifdef MAZEMAP_EXPORTS
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __declspec(dllimport)
#endif
#define IMPORT __declspec(dllimport)
#else
#define EXPORT
#define IMPORT
#endif

#endif

#ifndef MAZEMAP_INLINE
#define MAZEMAP_INLINE inline
#endif

#include <cmath>

namespace MazeMap
{
    namespace Platform
    {
        constexpr uint8_t kInvalidPin = 0xFFU;
        constexpr uint8_t kInvalidEncoderChannel = 0xFFU;
        constexpr uint8_t kMaxEncoderChannels = 5U;

        constexpr uint32_t kMotorPwmFrequencyHz = 80000U;
        // Teensy 4 FlexPWM uses a 150 MHz source on these motor pins, so 1875 counts yields an 80 kHz period.
        constexpr uint16_t kMotorPwmCounts = 1875U;
        constexpr uint16_t kMotorPwmMaxCode = kMotorPwmCounts - 1U;
        constexpr uint8_t kMotorPwmBits = 12U;

        inline bool IsAssignedPin(uint8_t pin)
        {
            return pin != kInvalidPin;
        }

        inline void ConfigureMotorPwmPin(uint8_t pin)
        {
            if (!IsAssignedPin(pin))
            {
                return;
            }

            analogWriteResolution(kMotorPwmBits);
            pinMode(pin, OUTPUT);
            analogWriteFrequency(pin, kMotorPwmFrequencyHz);
            analogWrite(pin, 0);
        }

        inline void DrivePinLow(uint8_t pin)
        {
            if (!IsAssignedPin(pin))
            {
                return;
            }

            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }

        inline void DrivePinHigh(uint8_t pin)
        {
            if (!IsAssignedPin(pin))
            {
                return;
            }

            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
        }

#if defined(IMXRT_FLEXPWM1) && defined(IMXRT_FLEXPWM2) && defined(FLEXPWM_MCTRL_CLDOK) && defined(FLEXPWM_OUTEN_PWMX_EN)
        struct FlexPwmPinInfo
        {
            IMXRT_FLEXPWM_t* module;
            uint8_t submodule;
            uint8_t channel; // 0 = X, 1 = A, 2 = B
        };

        inline bool ResolveMotorFlexPwmPin(uint8_t pin, FlexPwmPinInfo& info)
        {
            switch (pin)
            {
            case 5:
                info.module = &IMXRT_FLEXPWM2;
                info.submodule = 1;
                info.channel = 1;
                return true;

            case 6:
                info.module = &IMXRT_FLEXPWM2;
                info.submodule = 2;
                info.channel = 1;
                return true;

            case 24:
                info.module = &IMXRT_FLEXPWM1;
                info.submodule = 2;
                info.channel = 0;
                return true;

            case 25:
                info.module = &IMXRT_FLEXPWM1;
                info.submodule = 3;
                info.channel = 0;
                return true;

            default:
                return false;
            }
        }
#endif

        inline void WriteMotorPwmCode(uint8_t pin, uint16_t code)
        {
            if (!IsAssignedPin(pin))
            {
                return;
            }

            if (code > kMotorPwmMaxCode)
            {
                code = kMotorPwmMaxCode;
            }

#if defined(IMXRT_FLEXPWM1) && defined(IMXRT_FLEXPWM2) && defined(FLEXPWM_MCTRL_CLDOK) && defined(FLEXPWM_OUTEN_PWMX_EN)
            FlexPwmPinInfo info{};

            if (ResolveMotorFlexPwmPin(pin, info))
            {
                analogWrite(pin, 0U);

                const uint16_t mask = static_cast<uint16_t>(1U << info.submodule);

                info.module->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);

                switch (info.channel)
                {
                case 0:
                    info.module->SM[info.submodule].VAL0 = kMotorPwmMaxCode - code;
                    info.module->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(mask);
                    break;

                case 1:
                    info.module->SM[info.submodule].VAL3 = code;
                    info.module->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
                    break;

                case 2:
                    info.module->SM[info.submodule].VAL5 = code;
                    info.module->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
                    break;

                default:
                    info.module->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
                    return;
                }

                info.module->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
                return;
            }
#endif

            analogWrite(pin, static_cast<int>(code));
        }

        struct EncoderSlot
        {
            bool initialized = false;
            uint8_t pinA = kInvalidPin;
            uint8_t pinB = kInvalidPin;
            alignas(QuadEncoder) unsigned char storage[sizeof(QuadEncoder)] = {};
        };

        inline EncoderSlot* EncoderSlots()
        {
            static EncoderSlot slots[kMaxEncoderChannels];
            return slots;
        }

        inline QuadEncoder& AccessEncoder(EncoderSlot& slot)
        {
            return *reinterpret_cast<QuadEncoder*>(slot.storage);
        }

        inline const QuadEncoder& AccessEncoder(const EncoderSlot& slot)
        {
            return *reinterpret_cast<const QuadEncoder*>(slot.storage);
        }

        inline bool IsAssignedEncoder(uint8_t channel, uint8_t pinA, uint8_t pinB)
        {
            return (channel != kInvalidEncoderChannel) &&
                   (channel < kMaxEncoderChannels) &&
                   IsAssignedPin(pinA) &&
                   IsAssignedPin(pinB);
        }

        inline bool ConfigureEncoder(uint8_t channel, uint8_t pinA, uint8_t pinB)
        {
            if (!IsAssignedEncoder(channel, pinA, pinB))
            {
                return false;
            }

            EncoderSlot& slot = EncoderSlots()[channel];

            if (slot.initialized && (slot.pinA != pinA || slot.pinB != pinB))
            {
                AccessEncoder(slot).~QuadEncoder();
                slot.initialized = false;
            }

            if (!slot.initialized)
            {
                // The encoder hardware drives these lines directly, so leave the library pull-ups disabled.
                new (slot.storage) QuadEncoder(channel, pinA, pinB, 0U);
                slot.initialized = true;
                slot.pinA = pinA;
                slot.pinB = pinB;
            }

            AccessEncoder(slot).setInitConfig();
            AccessEncoder(slot).init();
            return true;
        }

        inline int32_t ReadEncoderCount(uint8_t channel)
        {
            if (channel >= kMaxEncoderChannels)
            {
                return 0;
            }

            EncoderSlot& slot = EncoderSlots()[channel];
            return slot.initialized ? AccessEncoder(slot).read() : 0;
        }

        inline void WriteEncoderCount(uint8_t channel, int32_t value)
        {
            if (channel >= kMaxEncoderChannels)
            {
                return;
            }

            EncoderSlot& slot = EncoderSlots()[channel];

            if (!slot.initialized)
            {
                return;
            }

            AccessEncoder(slot).write(value);
        }
    }
    namespace Math
    {
        inline float Sqrtf(float value) noexcept
        {
#if (defined(ARDUINO) || defined(CORE_TEENSY) || defined(ARDUINO_TEENSY41)) && defined(__arm__) && (defined(__VFP_FP__) || defined(__ARM_FP)) && !defined(__SOFTFP__)
            float result;
            __asm__ volatile ("vsqrt.f32 %0, %1" : "=t" (result) : "t" (value));
            return result;
#else
            using std::sqrt;
            return sqrt(value);
#endif
        }

        inline float Absf(float value) noexcept
        {
            using std::fabs;
            return fabs(value);
        }
    }
}

#endif








