#pragma once

// ============================================================================
// SQXZ-PRO application configuration
// Keep hardware pins and runtime timing in one place.
// Protocol-specific ECU constants remain inside kline.cpp.
// ============================================================================

// --- Display / touch -------------------------------------------------------
constexpr int SCREEN_WIDTH  = 320;
constexpr int SCREEN_HEIGHT = 240;

constexpr int TOUCH_IRQ  = 36;
constexpr int TOUCH_MOSI = 32;
constexpr int TOUCH_MISO = 39;
constexpr int TOUCH_CLK  = 25;
constexpr int CYD_TOUCH_CS = 33;

// XPT2046 calibration for the current CYD panel.
constexpr int TOUCH_RAW_X_MIN = 280;
constexpr int TOUCH_RAW_X_MAX = 3860;
constexpr int TOUCH_RAW_Y_MIN = 340;
constexpr int TOUCH_RAW_Y_MAX = 3860;

// --- Backlight -------------------------------------------------------------
constexpr int BACKLIGHT_PIN       = 21;
constexpr int BACKLIGHT_CHANNEL   = 0;
constexpr int BACKLIGHT_FREQUENCY = 5000;
constexpr int BACKLIGHT_RES_BITS  = 8;
constexpr uint8_t DEFAULT_BRIGHTNESS = 150;

// --- Runtime scheduling ----------------------------------------------------
// LVGL is kept responsive. Sensor/UI work is paced independently.
constexpr uint32_t UI_UPDATE_MS         = 33;   // ~30 FPS application update
constexpr uint32_t STARTUP_SPLASH_MS    = 2500; // show logo briefly, then enter dashboard immediately
constexpr uint32_t GPS_DEBUG_MS         = 2000;
constexpr uint32_t SYSTEM_DEBUG_MS      = 2000;
// GPS updates at ~1 Hz and 30 FPS UI is already smoother than a speedometer
// needs, so the loop doesn't need to spin as tight as 2ms. 5ms frees up CPU
// time for lv_timer_handler()/kline without any visible change on screen.
constexpr uint32_t LOOP_DELAY_MS        = 5;

// Stability supervision
constexpr uint32_t GPS_STALE_MS         = 5000;
constexpr uint32_t GPS_UART_RECOVER_MS   = 10000;
constexpr uint32_t KLINE_LINK_STALE_MS  = 4500;
constexpr uint32_t MEMORY_DEBUG_MS      = 5000;
constexpr uint32_t MIN_FREE_HEAP_WARN   = 45000;
constexpr uint32_t MIN_FREE_HEAP_CRIT   = 25000;

// --- Diagnostics -----------------------------------------------------------
// Stable build notes: high-volume K-Line frame dumps are debug-only; UI/theme
// updates are change-driven; PAGE 8 exposes live heap health information.
// Production build: serial diagnostics off. Flip back to true only when
// actively debugging with a USB cable connected.
constexpr bool ENABLE_RUNTIME_DEBUG = false;

// K-Line optional/experimental polling. Keep disabled for stable normal use.
// T10 speed decoding is based on an external CBR600RR table and is not needed
// by the dashboard (GPS remains the authoritative speed source).
constexpr bool ENABLE_KLINE_T10_SPEED = false;

// --- Stability / alarms ---------------------------------------------------
constexpr float ALARM_ECT_HIGH_C   = 105.0f;
constexpr float ALARM_BATT_LOW_V   = 11.5f;
constexpr float ALARM_RPM_HIGH     = 9000.0f;
constexpr float ALARM_SPEED_HIGH   = 160.0f;
constexpr uint32_t ALARM_HYST_MS   = 2000;
constexpr uint32_t LOGGER_SAMPLE_MS = 100;
constexpr uint32_t LOGGER_STOP_MS   = 3000;
constexpr size_t LOGGER_MAX_SAMPLES = 300;

