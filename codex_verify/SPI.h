#pragma once
#include "Arduino.h"

class SPISettings
{
public:
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass
{
public:
    void setMOSI(int) {}
    void setMISO(int) {}
    void setSCK(int) {}
    void begin() {}
    void beginTransaction(const SPISettings &) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t value) { return value; }
};

extern SPIClass SPI;
