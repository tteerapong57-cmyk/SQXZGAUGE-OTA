#pragma once
#include <stdint.h>

class TFT_eSPI;

// Both OTA and WiFi-setup run from setup() BEFORE lv_init()/kline/gps start, so
// nothing else uses RAM, SPI or the UART while WiFi is on. They never return
// (always ESP.restart()).

// touch_is_down: true while a finger is on the screen (so the next boot does not
// re-enter OTA mode while the finger is still held).
[[noreturn]] void ota_run(TFT_eSPI &tft, bool (*touch_is_down)());

// touch_get: returns true while pressed and writes mapped screen coordinates (0..319, 0..239).
[[noreturn]] void ota_wifi_setup_run(TFT_eSPI &tft, bool (*touch_get)(int16_t *x, int16_t *y));

// ---- P08 button helpers ----------------------------------------------------
[[noreturn]] void ota_request_and_reboot();        // UPDATE button
[[noreturn]] void ota_wifi_setup_and_reboot();     // WIFI button
bool ota_take_request();                           // true once; clears the flag
bool ota_take_wifi_setup_request();                // true once; clears the flag

// ---- saved WiFi credentials (NVS; falls back to include/wifi_secrets.h) -----
const char *ota_wifi_ssid();                       // "" if none
const char *ota_wifi_pass();
void        ota_wifi_save(const char *ssid, const char *pass);
