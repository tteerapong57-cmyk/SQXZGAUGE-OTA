#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <esp_task_wdt.h>
#include "app_config.h"
#include "ota_update.h"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"       // optional fallback network (first boot / before WIFI setup)
#endif
#ifndef OTA_WIFI_SSID
#define OTA_WIFI_SSID ""
#define OTA_WIFI_PASS ""
#endif

// ── saved credentials ───────────────────────────────────────────────────────
static char g_ssid[33], g_pass[65];

static void load_creds() {
    Preferences p; p.begin("ota", true);
    String s = p.getString("ssid", "");
    String w = p.getString("pass", "");
    p.end();
    if (s.length() == 0) { s = OTA_WIFI_SSID; w = OTA_WIFI_PASS; }
    strlcpy(g_ssid, s.c_str(), sizeof(g_ssid));
    strlcpy(g_pass, w.c_str(), sizeof(g_pass));
}
const char *ota_wifi_ssid() { load_creds(); return g_ssid; }
const char *ota_wifi_pass() { load_creds(); return g_pass; }
void ota_wifi_save(const char *ssid, const char *pass) {
    Preferences p; p.begin("ota", false);
    p.putString("ssid", ssid);
    p.putString("pass", pass);
    p.end();
}

// ── one-shot boot flags ─────────────────────────────────────────────────────
static bool take_flag(const char *key) {
    Preferences p; p.begin("ota", false);
    const bool v = p.getBool(key, false);
    if (v) p.putBool(key, false);          // clear first: never loop on a failure
    p.end();
    return v;
}
static void set_flag_and_reboot(const char *key) {
    Preferences p; p.begin("ota", false);
    p.putBool(key, true); p.end();
    delay(200);
    ESP.restart();
    while (true) delay(1000);
}
bool ota_take_request()            { return take_flag("go"); }
bool ota_take_wifi_setup_request() { return take_flag("ws"); }
[[noreturn]] void ota_request_and_reboot()        { set_flag_and_reboot("go"); while (true) {} }
[[noreturn]] void ota_wifi_setup_and_reboot()     { set_flag_and_reboot("ws"); while (true) {} }

// Both files are assets of the newest (non-prerelease) GitHub Release.
#define OTA_BASE_URL "https://github.com/" OTA_GITHUB_OWNER "/" OTA_GITHUB_REPO "/releases/latest/download/"
#define OTA_VERSION_URL  OTA_BASE_URL "version.txt"
#define OTA_FIRMWARE_URL OTA_BASE_URL "firmware.bin"

static TFT_eSPI *g_tft = nullptr;

// ── polished boot-time status UI (direct TFT, LVGL is not running) ────────
static constexpr uint16_t UI_BG      = 0x0841;  // deep navy
static constexpr uint16_t UI_PANEL   = 0x10A2;  // blue-black panel
static constexpr uint16_t UI_EDGE    = 0x29E5;
static constexpr uint16_t UI_CYAN    = 0x07FF;
static constexpr uint16_t UI_GREEN   = 0x07E0;
static constexpr uint16_t UI_YELLOW  = 0xFFE0;
static constexpr uint16_t UI_RED     = 0xF800;
static constexpr uint16_t UI_MUTED   = 0x8C92;

static void ui_badge(const char *text, uint16_t color) {
    const int w = g_tft->textWidth(text, 2) + 14;
    const int x = 320 - w - 10;
    g_tft->fillRoundRect(x, 8, w, 20, 8, UI_PANEL);
    g_tft->drawRoundRect(x, 8, w, 20, 8, color);
    g_tft->setTextDatum(MC_DATUM);
    g_tft->setTextColor(color, UI_PANEL);
    g_tft->drawString(text, x + w/2, 18, 2);
}

static void ui_header(const char *title) {
    g_tft->fillScreen(UI_BG);
    g_tft->fillRect(0, 0, 320, 3, UI_CYAN);
    g_tft->setTextDatum(ML_DATUM);
    g_tft->setTextColor(UI_CYAN, UI_BG);
    g_tft->drawString("SQXZ GAUGE", 10, 18, 2);
    ui_badge("OTA", UI_CYAN);

    g_tft->setTextDatum(TC_DATUM);
    g_tft->setTextColor(TFT_WHITE, UI_BG);
    g_tft->drawString(title, 160, 48, 4);

    g_tft->setTextColor(UI_MUTED, UI_BG);
    g_tft->drawString("FIRMWARE  v" APP_VERSION, 160, 73, 2);
    g_tft->drawFastHLine(14, 84, 292, UI_EDGE);
}

static void ui_card_line(const char *text, uint16_t color, int y = 122, int font = 2) {
    g_tft->fillRoundRect(14, 94, 292, 78, 12, UI_PANEL);
    g_tft->drawRoundRect(14, 94, 292, 78, 12, UI_EDGE);
    g_tft->setTextDatum(TC_DATUM);
    g_tft->setTextColor(color, UI_PANEL);
    g_tft->drawString(text, 160, y, font);
}

static void ui_screen(const char *title) {
    ui_header(title);
    ui_card_line("Preparing...", UI_MUTED, 126, 2);
}

static void ui_line(const char *text, uint16_t color, int y = 150) {
    const bool bottom = y >= 180;
    const uint16_t bg = bottom ? UI_BG : UI_PANEL;
    const int x = bottom ? 0 : 24;
    const int w = bottom ? 320 : 272;
    g_tft->fillRect(x, y - 12, w, 24, bg);
    g_tft->setTextDatum(TC_DATUM);
    g_tft->setTextColor(color, bg);
    g_tft->drawString(text, 160, y, 2);
}

static void ui_progress(int pct) {
    pct = constrain(pct, 0, 100);
    const int x = 22, y = 184, w = 276, h = 18;
    g_tft->fillRoundRect(x, y, w, h, 8, UI_PANEL);
    g_tft->drawRoundRect(x, y, w, h, 8, UI_EDGE);
    if (pct > 0) {
        const int fillW = max(2, ((w - 4) * pct) / 100);
        g_tft->fillRoundRect(x + 2, y + 2, fillW, h - 4, 6, UI_GREEN);
    }
    char b[16];
    snprintf(b, sizeof(b), "%d%%", pct);
    g_tft->fillRect(0, 207, 320, 28, UI_BG);
    g_tft->setTextDatum(TC_DATUM);
    g_tft->setTextColor(TFT_WHITE, UI_BG);
    g_tft->drawString(b, 160, 216, 2);
}

[[noreturn]] static void finish(bool (*touch_is_down)(), const char *l1, uint16_t c) {
    ui_line(l1, c);
    delay(2500);
    // Wait for the finger to lift, otherwise the next boot would re-enter OTA.
    ui_line("RELEASE TOUCH TO REBOOT", UI_YELLOW, 215);
    const uint32_t tw = millis();
    while (touch_is_down && touch_is_down() && millis() - tw < 5000UL) delay(50);   // bounded: never hang here
    ESP.restart();
    while (true) delay(1000);
}

// version.txt = "<version> <md5-of-firmware.bin>"   e.g. "7.11 9e107d9d372bb6826bd81d3542a419d6"
static int g_http_code = 0;   // last HTTP status (or negative HTTPClient error) for the on-screen message

static bool fetch_version(String &ver, String &md5) {
    WiFiClientSecure client;
    client.setInsecure();               // see README: no cert pinning
    HTTPClient http;
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.setTimeout(OTA_HTTP_TIMEOUT_MS);
    if (!http.begin(client, OTA_VERSION_URL)) return false;
    const int code = http.GET();
    g_http_code = code;
    if (code != HTTP_CODE_OK) {
        Serial.printf("[OTA] version.txt HTTP %d\n", code);
        http.end();
        return false;
    }
    String body = http.getString();
    http.end();
    body.trim();
    const int sp = body.indexOf(' ');
    if (sp < 1) return false;
    ver = body.substring(0, sp);
    md5 = body.substring(sp + 1);
    md5.trim();
    ver.trim();
    return ver.length() > 0 && md5.length() == 32;
}

[[noreturn]] void ota_run(TFT_eSPI &tft, bool (*touch_is_down)()) {
    g_tft = &tft;

    // The main task is on the 8 s task watchdog (app_watchdog_init). A download
    // takes much longer and we always reboot at the end, so drop it from the WDT.
    esp_task_wdt_delete(NULL);

    tft.setRotation(1);
    ui_screen("CONNECTING WIFI");

    const char *ssid = ota_wifi_ssid();
    const char *pass = ota_wifi_pass();
    if (ssid[0] == 0) {
        finish(touch_is_down, "NO WIFI: SET IT ON P08", UI_RED);
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    ui_line(ssid, UI_CYAN, 142);
    const uint32_t t0 = millis();
    uint32_t lastAnim = 0;
    int dots = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < OTA_WIFI_TIMEOUT_MS) {
        delay(120);
        if (millis() - lastAnim >= 320) {
            lastAnim = millis();
            dots = (dots + 1) % 4;
            char anim[16];
            snprintf(anim, sizeof(anim), "CONNECTING%.*s", dots, "...");
            ui_line(anim, UI_YELLOW, 112);
        }
    }
    if (WiFi.status() != WL_CONNECTED) {
        finish(touch_is_down, "WIFI FAILED", UI_RED);
    }

    ui_screen("CHECKING UPDATE");
    ui_line("CHECKING GITHUB RELEASE", UI_CYAN, 126);
    String newVer, md5;
    if (!fetch_version(newVer, md5)) {
        char m[48];
        if (g_http_code == 404)      snprintf(m, sizeof(m), "NO RELEASE YET (HTTP 404)");
        else if (g_http_code == 200) snprintf(m, sizeof(m), "BAD version.txt FORMAT");
        else                         snprintf(m, sizeof(m), "CANNOT READ version.txt (%d)", g_http_code);
        finish(touch_is_down, m, UI_RED);
    }
    Serial.printf("[OTA] current=%s remote=%s\n", APP_VERSION, newVer.c_str());

    // "!=" (not ">") on purpose: publishing an older release rolls back.
    if (newVer == APP_VERSION) {
        finish(touch_is_down, "ALREADY UP TO DATE", UI_GREEN);
    }

    ui_screen("DOWNLOADING");
    char nv[40];
    snprintf(nv, sizeof(nv), "NEW FIRMWARE  v%s", newVer.c_str());
    ui_line(nv, UI_YELLOW, 130);

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.setTimeout(OTA_HTTP_TIMEOUT_MS);
    if (!http.begin(client, OTA_FIRMWARE_URL)) {
        finish(touch_is_down, "HTTP BEGIN FAILED", UI_RED);
    }
    const int code = http.GET();
    const int total = http.getSize();
    if (code != HTTP_CODE_OK || total <= 0) {
        Serial.printf("[OTA] firmware HTTP %d size %d\n", code, total);
        http.end();
        finish(touch_is_down, "DOWNLOAD FAILED", UI_RED);
    }
    if (!Update.begin((size_t)total)) {
        http.end();
        finish(touch_is_down, "NOT ENOUGH FLASH SPACE", UI_RED);
    }
    Update.setMD5(md5.c_str());

    WiFiClient *stream = http.getStreamPtr();
    uint8_t buf[1024];
    int written = 0, lastPct = -1;
    uint32_t lastData = millis();
    while (written < total) {
        const size_t avail = stream->available();
        if (avail) {
            const int n = stream->readBytes(buf, min(avail, sizeof(buf)));
            if (n > 0) {
                if (Update.write(buf, n) != (size_t)n) break;
                written += n;
                lastData = millis();
                const int pct = (int)((int64_t)written * 100 / total);
                if (pct != lastPct) { lastPct = pct; ui_progress(pct); }
            }
        } else {
            if (!http.connected() || millis() - lastData > OTA_HTTP_TIMEOUT_MS) break;
            delay(2);
        }
    }
    http.end();

    // end(true) verifies the MD5 and size, then switches the boot partition.
    if (written != total || !Update.end(true) || !Update.isFinished()) {
        Serial.printf("[OTA] failed written=%d/%d err=%s\n", written, total, Update.errorString());
        Update.abort();
        finish(touch_is_down, "UPDATE FAILED (OLD FW KEPT)", UI_RED);
    }

    ui_screen("UPDATE OK");
    finish(touch_is_down, "REBOOTING TO NEW FIRMWARE", UI_GREEN);
}
