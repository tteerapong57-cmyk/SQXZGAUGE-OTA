/* MIT licensed implementation compatible with Paul Stoffregen's
 * XPT2046_Touchscreen API, kept local so PlatformIO does not require git. */
#include "XPT2046_Touchscreen.h"

#define Z_THRESHOLD 300
#define Z_THRESHOLD_INT 75
#define MSEC_THRESHOLD 3
#define SPI_SETTING SPISettings(2000000, MSBFIRST, SPI_MODE0)

XPT2046_Touchscreen *XPT2046_Touchscreen::isrPinptr = nullptr;

bool XPT2046_Touchscreen::begin(SPIClass &wspi) {
    _pspi = &wspi;
    _pspi->begin();
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    if (tirqPin != 255) {
        pinMode(tirqPin, INPUT);
        isrPinptr = this;
        attachInterrupt(digitalPinToInterrupt(tirqPin), isrPin, FALLING);
    }
    isrWake = true;
    return true;
}

void IRAM_ATTR XPT2046_Touchscreen::isrPin() {
    if (isrPinptr) isrPinptr->isrWake = true;
}

TS_Point XPT2046_Touchscreen::getPoint() {
    update();
    return TS_Point(xraw, yraw, zraw);
}

bool XPT2046_Touchscreen::tirqTouched() { return isrWake; }
bool XPT2046_Touchscreen::touched() { update(); return zraw >= Z_THRESHOLD; }

void XPT2046_Touchscreen::readData(uint16_t *x, uint16_t *y, uint8_t *z) {
    update();
    *x = (uint16_t)xraw;
    *y = (uint16_t)yraw;
    *z = (uint8_t)constrain(zraw, 0, 255);
}

bool XPT2046_Touchscreen::bufferEmpty() { return (millis() - msraw) < MSEC_THRESHOLD; }

static int16_t besttwoavg(int16_t x, int16_t y, int16_t z) {
    int16_t da = abs(x - y);
    int16_t db = abs(x - z);
    int16_t dc = abs(z - y);
    if (da <= db && da <= dc) return (x + y) >> 1;
    if (db <= da && db <= dc) return (x + z) >> 1;
    return (y + z) >> 1;
}

void XPT2046_Touchscreen::update() {
    if (!isrWake) return;
    uint32_t now = millis();
    if (now - msraw < MSEC_THRESHOLD) return;
    if (!_pspi) return;

    int16_t data[6];
    int z;
    _pspi->beginTransaction(SPI_SETTING);
    digitalWrite(csPin, LOW);

    _pspi->transfer(0xB1); // Z1
    int16_t z1 = _pspi->transfer16(0xC1) >> 3; // Z2
    z = z1 + 4095;
    int16_t z2 = _pspi->transfer16(0x91) >> 3; // X
    z -= z2;

    if (z >= Z_THRESHOLD) {
        _pspi->transfer16(0x91); // dummy X
        data[0] = _pspi->transfer16(0xD1) >> 3; // Y
        data[1] = _pspi->transfer16(0x91) >> 3; // X
        data[2] = _pspi->transfer16(0xD1) >> 3; // Y
        data[3] = _pspi->transfer16(0x91) >> 3; // X
    } else {
        data[0] = data[1] = data[2] = data[3] = 0;
    }
    data[4] = _pspi->transfer16(0xD0) >> 3; // last Y power down
    data[5] = _pspi->transfer16(0) >> 3;

    digitalWrite(csPin, HIGH);
    _pspi->endTransaction();

    if (z < 0) z = 0;
    if (z < Z_THRESHOLD) {
        zraw = 0;
        if (z < Z_THRESHOLD_INT && tirqPin != 255) isrWake = false;
        return;
    }

    zraw = z;
    int16_t x = besttwoavg(data[0], data[2], data[4]);
    int16_t y = besttwoavg(data[1], data[3], data[5]);

    msraw = now;
    switch (rotation) {
        case 0: xraw = 4095 - y; yraw = x; break;
        case 1: xraw = x; yraw = y; break;
        case 2: xraw = y; yraw = 4095 - x; break;
        default: xraw = 4095 - x; yraw = 4095 - y; break;
    }
}
