/*
 * XPT2046_Touchscreen compatible local library for ESP32.
 * Based on the open-source Paul Stoffregen XPT2046_Touchscreen library.
 * Original project: https://github.com/PaulStoffregen/XPT2046_Touchscreen
 * License: MIT. Keep this notice with redistributed copies.
 */
#ifndef _XPT2046_Touchscreen_h_
#define _XPT2046_Touchscreen_h_

#include <Arduino.h>
#include <SPI.h>

class TS_Point {
public:
    TS_Point(void) : x(0), y(0), z(0) {}
    TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
    bool operator==(TS_Point p) const { return p.x == x && p.y == y && p.z == z; }
    bool operator!=(TS_Point p) const { return !(*this == p); }
    int16_t x, y, z;
};

class XPT2046_Touchscreen {
public:
    constexpr XPT2046_Touchscreen(uint8_t cspin, uint8_t tirq = 255)
        : csPin(cspin), tirqPin(tirq) {}
    bool begin(SPIClass &wspi = SPI);
    TS_Point getPoint();
    bool tirqTouched();
    bool touched();
    void readData(uint16_t *x, uint16_t *y, uint8_t *z);
    bool bufferEmpty();
    uint8_t bufferSize() { return 1; }
    void setRotation(uint8_t n) { rotation = n % 4; }

private:
    void update();
    uint8_t csPin, tirqPin, rotation = 1;
    int16_t xraw = 0, yraw = 0, zraw = 0;
    uint32_t msraw = 0x80000000UL;
    bool isrWake = true;
    SPIClass *_pspi = nullptr;
    static XPT2046_Touchscreen *isrPinptr;
    static void IRAM_ATTR isrPin();
};

#endif
