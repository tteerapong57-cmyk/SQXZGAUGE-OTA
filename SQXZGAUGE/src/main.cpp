#include <Arduino.h>
#include <stdarg.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <lvgl.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_idf_version.h>
#include "app_config.h"
#include "gauge_ui.h"
#include "kline.h"
#include "gps.h"
#include "ota_update.h"

// ── SQXZ-PRO splash: full-color RGB565 image (320x240) ─────────────────────
extern const uint16_t SQXZ_PRO_320x240[];

static void draw_sqxz_splash(TFT_eSPI &tft) {
    tft.fillScreen(TFT_BLACK);
    tft.pushImage(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SQXZ_PRO_320x240);
}

TFT_eSPI tft = TFT_eSPI();

static SPIClass touchSPI(VSPI);
static XPT2046_Touchscreen touch(CYD_TOUCH_CS, TOUCH_IRQ);

static lv_disp_draw_buf_t draw_buf;
// Keep a small LVGL line buffer to avoid large transient allocations.
static lv_color_t buf1[SCREEN_WIDTH * 20];

static volatile uint32_t g_watchdog_feed_count = 0;
static volatile uint32_t g_watchdog_last_feed_ms = 0;
static volatile bool g_watchdog_ready = false;
static volatile bool g_touch_ready = false;

// Same condition touch_read() uses: the IRQ pin must be low AND the pressure
// reading valid. (touched() alone can read "down" spuriously on some CYD panels.)
static bool touch_is_down() { return g_touch_ready && touch.tirqTouched() && touch.touched(); }

uint32_t app_watchdog_feed_count() { return g_watchdog_feed_count; }
uint32_t app_watchdog_last_feed_ms() { return g_watchdog_last_feed_ms; }
bool app_watchdog_ready() { return g_watchdog_ready; }

static void app_watchdog_init() {
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t cfg;
    cfg.timeout_ms = 8000;
    cfg.idle_core_mask = (1U << portNUM_PROCESSORS) - 1U;
    cfg.trigger_panic = true;
    esp_err_t err = esp_task_wdt_init(&cfg);
    if(err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_add(NULL);
        g_watchdog_ready = true;
    }
#else
    esp_err_t err = esp_task_wdt_init(8, true);
    if(err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_add(NULL);
        g_watchdog_ready = true;
    }
#endif
    Serial.printf("[WDT] ready=%d reset_reason=%d\n", g_watchdog_ready ? 1 : 0, (int)esp_reset_reason());
}

static void app_watchdog_feed() {
    if(!g_watchdog_ready) return;
    esp_task_wdt_reset();
    g_watchdog_feed_count = g_watchdog_feed_count + 1;
    g_watchdog_last_feed_ms = millis();
}

static void debug_print(const char *fmt, ...) {
    if (!ENABLE_RUNTIME_DEBUG) return;
    va_list ap;
    va_start(ap, fmt);
    char buffer[192];
    vsnprintf(buffer, sizeof(buffer), fmt, ap);
    va_end(ap);
    Serial.print(buffer);
}

void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    const uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
    const uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(disp);
}

// Smoothly ramps the backlight PWM duty from `from` to `to` over roughly
// `duration_ms`, so brightness changes read as a soft fade instead of an
// abrupt on/off snap. Used around screen transitions during boot.
static void backlight_fade(uint8_t from, uint8_t to, uint16_t duration_ms) {
    const uint8_t steps = 30;
    const uint16_t step_delay = duration_ms / steps;
    for (uint8_t i = 1; i <= steps; i++) {
        const int v = from + (((int)to - (int)from) * (int)i) / steps;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(BACKLIGHT_PIN, v);
#else
        ledcWrite(BACKLIGHT_CHANNEL, v);
#endif
        delay(step_delay);
    }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(BACKLIGHT_PIN, to);
#else
    ledcWrite(BACKLIGHT_CHANNEL, to); // land exactly on target, no rounding drift
#endif
}

static inline int32_t touch_map_x(int32_t raw) {
    return constrain(
        map((long)raw, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, 0, SCREEN_WIDTH - 1),
        0L, (long)SCREEN_WIDTH - 1L);
}

static inline int32_t touch_map_y(int32_t raw) {
    return constrain(
        map((long)raw, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, 0, SCREEN_HEIGHT - 1),
        0L, (long)SCREEN_HEIGHT - 1L);
}

// Mapped touch point for the boot-time WiFi setup screens (no LVGL running).
static bool touch_get_mapped(int16_t *x, int16_t *y) {
    if (!g_touch_ready) return false;
    if (!(touch.tirqTouched() && touch.touched())) return false;
    const TS_Point p = touch.getPoint();
    *x = (int16_t)touch_map_x(p.x);
    *y = (int16_t)touch_map_y(p.y);
    return true;
}

void touch_read(lv_indev_drv_t *indev, lv_indev_data_t *data) {
    (void)indev;

    const bool pressed = touch.tirqTouched() && touch.touched();
    static bool was_pressed = false;
    static bool touch_sequence_valid = false;
    static bool press_started_on_special_page = false;
    static int32_t last_x = 0;
    static int32_t last_y = 0;
    static int32_t last_raw_x = 0;
    static int32_t last_raw_y = 0;

    if (pressed) {
        const TS_Point p = touch.getPoint();
        const int32_t x = touch_map_x(p.x);
        const int32_t y = touch_map_y(p.y);

        last_x = x;
        last_y = y;
        last_raw_x = p.x;
        last_raw_y = p.y;
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = (lv_coord_t)x;
        data->point.y = (lv_coord_t)y;

        if (!was_pressed) {
            const uint8_t page = gauge_ui_get_page();
            // Pages with their own release handlers (0 = menu screen).
            // Page 16 (DISTANCE TEST) also needs precise button hit-testing
            // for -/+/START-STOP/RESET, same reason pages 1/6/7 are here.
            // Page 12 (ALARM) now cycles through a 2nd sub-view (PERFORMANCE)
            // before returning to the menu, so it also needs the dedicated
            // release handler instead of the generic "tap anywhere = menu".
            press_started_on_special_page =
                (page == 0 || page == 1 || page == 3 || page == 6 || page == 7 || page == 8 || page == 12 || page == 16);
            touch_sequence_valid = true;
        }

        was_pressed = true;
        return;
    }

    data->state = LV_INDEV_STATE_RELEASED;
    data->point.x = (lv_coord_t)last_x;
    data->point.y = (lv_coord_t)last_y;

    if (!was_pressed || !touch_sequence_valid) {
        was_pressed = false;
        touch_sequence_valid = false;
        press_started_on_special_page = false;
        return;
    }

    // One physical tap produces one logical action, processed on release.
    if (press_started_on_special_page) {
        gauge_ui_handle_touch_release_raw(
            last_raw_x, last_raw_y, last_x, last_y);
    } else {
        gauge_ui_touch_toggle_at(last_x, last_y);
    }

    was_pressed = false;
    touch_sequence_valid = false;
    press_started_on_special_page = false;
}

void setup() {
    Serial.begin(115200);
    delay(300);
    debug_print("### BOOT: setup start ###\n");

    app_watchdog_init();

    tft.init();
    debug_print("### tft.init() OK ###\n");
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    // Backlight is put under PWM control immediately, at full brightness,
    // so the splash below is shown exactly as before. TFT_eSPI's own
    // tft.init() already drove this pin HIGH via TFT_BL/TFT_BACKLIGHT_ON,
    // so attaching LEDC here and writing full brightness is a seamless
    // continuation rather than a glitchy re-configuration.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // Arduino-ESP32 Core 3.x: LEDC channel is attached to the pin.
    if(!ledcAttachChannel(BACKLIGHT_PIN, BACKLIGHT_FREQUENCY, BACKLIGHT_RES_BITS,
                          BACKLIGHT_CHANNEL)) {
        Serial.println("[BACKLIGHT] ledcAttachChannel failed");
    }
    ledcWrite(BACKLIGHT_PIN, (1 << BACKLIGHT_RES_BITS) - 1);
#else
    // Arduino-ESP32 Core 2.x compatibility.
    ledcSetup(BACKLIGHT_CHANNEL, BACKLIGHT_FREQUENCY, BACKLIGHT_RES_BITS);
    ledcAttachPin(BACKLIGHT_PIN, BACKLIGHT_CHANNEL);
    ledcWrite(BACKLIGHT_CHANNEL, (1 << BACKLIGHT_RES_BITS) - 1);
#endif

    // Splash is drawn directly by TFT before LVGL starts.
    draw_sqxz_splash(tft);
    delay(STARTUP_SPLASH_MS);

    // Fade the splash out gently instead of cutting it to black. This also
    // doubles as the "screen is now dark, safe to redraw" cue below.
    backlight_fade((1 << BACKLIGHT_RES_BITS) - 1, 0, 280);
    tft.fillScreen(TFT_BLACK);
    debug_print("### splash done ###\n");

    // Backlight stays at 0 while the UI is built. gauge_ui_init() below
    // creates every screen (14 pages worth of widgets) and only performs
    // its first real screen flush at the very end, right before it
    // returns. Without this, the backlight would stay lit the whole time,
    // so the panel's top-to-bottom redraw of that first flush would be
    // visible as a "double image"/ghosting wipe. Keeping the backlight off
    // until page 1 is fully drawn, then fading it in below, makes the
    // splash -> page 1 switch a soft, deliberate reveal instead.

    // Touch uses the separate VSPI bus.
    touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, CYD_TOUCH_CS);
    g_touch_ready = touch.begin(touchSPI);
    touch.setRotation(1);

    // OTA update mode: hold a finger on the screen while powering on.
    // Runs before LVGL/K-Line/GPS start, so WiFi+TLS has the whole heap and
    // nothing else touches SPI/UART. Never returns (always reboots).
    // Also a recovery path if a bad build crashes later in setup().
    // The P08 UPDATE button also gets here (flag in NVS + reboot).
    // The P08 WIFI / UPDATE buttons set a one-shot flag in NVS and reboot.
    // Holding a finger on the screen while powering on also enters OTA mode.
    const bool want_wifi_setup = ota_take_wifi_setup_request();
    const bool want_ota_flag   = ota_take_request();
    bool want_ota_touch = false;
    if (touch_is_down()) { delay(80); want_ota_touch = touch_is_down(); }   // must stay down
    Serial.printf("[BOOT] wifi_setup=%d ota_flag=%d ota_touch=%d\n",
                  (int)want_wifi_setup, (int)want_ota_flag, (int)want_ota_touch);

    if (want_wifi_setup || want_ota_flag || want_ota_touch) {
        // The backlight was faded to 0 after the splash. Without this the OTA /
        // WiFi-setup screens run but the panel looks dead (black).
        backlight_fade(0, DEFAULT_BRIGHTNESS, 150);
        if (want_wifi_setup) ota_wifi_setup_run(tft, touch_get_mapped);
        ota_run(tft, touch_is_down);
    }

    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, SCREEN_WIDTH * 20);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = SCREEN_WIDTH;
    disp_drv.ver_res  = SCREEN_HEIGHT;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register(&indev_drv);

    debug_print("### LVGL ready, calling gauge_ui_init ###\n");
    gauge_ui_init();
    debug_print("### gauge_ui_init DONE ###\n");

    // Page 1 is now fully rendered into the panel's memory (gauge_ui_init
    // ends with its own lv_scr_load + lv_refr_now). Fade the backlight up
    // from here, so the reveal reads as a smooth, deliberate fade-in
    // instead of a jarring full-brightness snap.
    backlight_fade(0, DEFAULT_BRIGHTNESS, 380);
    debug_print("### backlight faded in, page 1 revealed ###\n");

    // Start the ECU engine on its own FreeRTOS task so blocking ECU reads
    // never stall the LVGL/UI loop.
    kline_init();
    kline_start_background();
    debug_print("### kline background task started ###\n");

    gps_init();
    debug_print("### gps_init DONE ###\n");

}

void loop() {
    static uint32_t last_ui_ms = 0;
    static uint32_t last_gps_dbg_ms = 0;
    static uint32_t last_system_dbg_ms = 0;
    static uint32_t last_memory_check_ms = 0;

    const uint32_t now = millis();
    // Serial parsers can run continuously without touching LVGL objects.
    gps_update();

    // Application-level UI updates are paced independently from LVGL's own
    // timer servicing. This keeps animation/input responsive without wasting
    // CPU updating dozens of labels every few milliseconds.
    if (now - last_ui_ms >= UI_UPDATE_MS) {
        last_ui_ms = now;
        gauge_ui_update();
    }

    // LVGL remains responsive even between application updates.
    lv_timer_handler();

    if (ENABLE_RUNTIME_DEBUG && now - last_gps_dbg_ms >= GPS_DEBUG_MS) {
        last_gps_dbg_ms = now;
        Serial.printf("[GPS] chars=%lu sat=%d fix=%d speed=%.1f\n",
            (unsigned long)gps_chars_processed(), gps_satellites(),
            gps_has_fix() ? 1 : 0, gps_speed_kmh());
    }

    if (ENABLE_RUNTIME_DEBUG && now - last_system_dbg_ms >= SYSTEM_DEBUG_MS) {
        last_system_dbg_ms = now;
        Serial.printf(
            "[SYSTEM] uptime=%lus heap=%u minheap=%u gps_age=%lums gps_recover=%lu ecu=%d kline_age=%lums reconnects=%lu page=%u stack_free=%uB\n",
            (unsigned long)(now / 1000),
            ESP.getFreeHeap(),
            (unsigned)esp_get_minimum_free_heap_size(),
            (unsigned long)(now - gps_last_data_ms()),
            (unsigned long)gps_uart_recovery_count(),
            kline_is_connected() ? 1 : 0,
            (unsigned long)(kline_last_good_data_ms() ? now - kline_last_good_data_ms() : 0),
            (unsigned long)kline_reconnect_count(),
            gauge_ui_get_page(),
            (unsigned)kline_task_stack_high_water_mark());
    }

    if (now - last_memory_check_ms >= MEMORY_DEBUG_MS) {
        last_memory_check_ms = now;
        const uint32_t heap = ESP.getFreeHeap();
        if (ENABLE_RUNTIME_DEBUG && heap < MIN_FREE_HEAP_WARN) {
            Serial.printf("[MEMORY] %s free heap=%luB\n",
                heap < MIN_FREE_HEAP_CRIT ? "CRITICAL" : "WARNING",
                (unsigned long)heap);
        }
    }

    app_watchdog_feed();

    delay(LOOP_DELAY_MS);
}
