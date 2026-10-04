// On-device WiFi setup: scan -> pick network -> touch keyboard -> test -> save.
// Drawn directly with TFT_eSPI (LVGL is not running yet), so it costs no LVGL
// memory and needs no LVGL keyboard widget.
#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <esp_task_wdt.h>
#include "ota_update.h"

typedef bool (*TouchGet)(int16_t *, int16_t *);
static TFT_eSPI *T;
static TouchGet getTouch;

static const uint16_t C_BG   = TFT_BLACK;
static const uint16_t C_KEY  = 0x2104;   // dark grey
static const uint16_t C_EDGE = 0x5AEB;   // mid grey
static const uint16_t C_ACC  = TFT_CYAN;
static const uint16_t C_OK   = TFT_GREEN;
static const uint16_t C_WARN = TFT_YELLOW;
static const uint16_t C_ERR  = TFT_RED;

struct Rect {
    int16_t x, y, w, h;
    bool hit(int16_t px, int16_t py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

static void draw_btn(const Rect &r, const char *label, uint16_t border, uint16_t fg, bool pressed = false) {
    const uint16_t bg = pressed ? border : C_KEY;
    T->fillRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 5, bg);
    T->drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 5, border);
    T->setTextDatum(MC_DATUM);
    T->setTextColor(pressed ? TFT_BLACK : fg, bg);
    T->drawString(label, r.x + r.w / 2, r.y + r.h / 2, 2);
}

static void wait_release() {
    int16_t a, b;
    const uint32_t t0 = millis();
    while (getTouch(&a, &b) && millis() - t0 < 2000UL) delay(10);   // bounded
    delay(40);
}

[[noreturn]] static void reboot() {
    wait_release();                // otherwise a held finger would enter OTA mode on the next boot
    ESP.restart();
    while (true) delay(1000);
}

// Blocks until a (debounced) press. Gives up after idle_ms and reboots.
static void wait_tap(int16_t &x, int16_t &y) {
    const uint32_t t0 = millis();
    for (;;) {
        int16_t px, py;
        if (getTouch(&px, &py)) {
            delay(15);
            if (getTouch(&px, &py)) { x = px; y = py; return; }
        }
        if (millis() - t0 > 120000UL) reboot();   // idle 2 min -> back to normal boot
        delay(10);
    }
}

static void header(const char *title, const char *sub = nullptr) {
    T->fillScreen(C_BG);
    T->setTextDatum(TC_DATUM);
    T->setTextColor(C_ACC, C_BG);
    T->drawString(title, 160, 4, 4);
    if (sub) {
        T->setTextColor(TFT_DARKGREY, C_BG);
        T->drawString(sub, 160, 34, 2);
    }
}

static void center_msg(const char *l1, uint16_t c1, const char *l2 = nullptr, uint16_t c2 = TFT_WHITE) {
    T->setTextDatum(MC_DATUM);
    T->fillRect(0, 90, 320, 70, C_BG);
    T->setTextColor(c1, C_BG);
    T->drawString(l1, 160, l2 ? 105 : 120, 4);
    if (l2) { T->setTextColor(c2, C_BG); T->drawString(l2, 160, 140, 2); }
}

// ── network list ────────────────────────────────────────────────────────────
struct Ap { char ssid[33]; int8_t rssi; bool open; };
static Ap g_aps[24];
static int g_apn = 0;

static void scan() {
    header("WIFI SETUP");
    center_msg("SCANNING...", C_WARN);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    const int n = WiFi.scanNetworks();
    g_apn = 0;
    for (int i = 0; i < n && g_apn < 24; i++) {
        String s = WiFi.SSID(i);
        if (s.length() == 0 || s.length() > 32) continue;
        bool dup = false;
        for (int k = 0; k < g_apn; k++) if (s == g_aps[k].ssid) { dup = true; break; }
        if (dup) continue;
        strlcpy(g_aps[g_apn].ssid, s.c_str(), sizeof(g_aps[0].ssid));
        g_aps[g_apn].rssi = (int8_t)WiFi.RSSI(i);
        g_aps[g_apn].open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
        g_apn++;
    }
    WiFi.scanDelete();
    for (int i = 1; i < g_apn; i++) {                 // strongest first
        Ap t = g_aps[i]; int j = i - 1;
        while (j >= 0 && g_aps[j].rssi < t.rssi) { g_aps[j + 1] = g_aps[j]; j--; }
        g_aps[j + 1] = t;
    }
}

// returns index >= 0 (chosen), -1 cancel, -2 rescan
static int list_screen() {
    const int PER = 5;
    int page = 0;
    const Rect bCancel = {4, 192, 100, 44}, bScan = {110, 192, 100, 44}, bMore = {216, 192, 100, 44};
    for (;;) {
        header("SELECT WIFI");
        const int pages = g_apn ? (g_apn + PER - 1) / PER : 1;
        char pg[12]; snprintf(pg, sizeof(pg), "%d/%d", page + 1, pages);
        T->setTextDatum(TR_DATUM); T->setTextColor(TFT_DARKGREY, C_BG);
        T->drawString(pg, 316, 8, 2);

        Rect rows[PER];
        for (int i = 0; i < PER; i++) {
            rows[i] = {4, (int16_t)(34 + i * 31), 312, 29};
            const int idx = page * PER + i;
            if (idx >= g_apn) continue;
            draw_btn(rows[i], "", C_EDGE, TFT_WHITE);
            T->setTextDatum(ML_DATUM);
            T->setTextColor(TFT_WHITE, C_KEY);
            char nm[27]; strlcpy(nm, g_aps[idx].ssid, sizeof(nm));
            T->drawString(nm, 12, rows[i].y + 15, 2);
            char r[12];
            T->setTextDatum(MR_DATUM);
            if (g_aps[idx].open) { T->setTextColor(C_OK, C_KEY); T->drawString("OPEN", 308, rows[i].y + 15, 2); }
            else { snprintf(r, sizeof(r), "%d", g_aps[idx].rssi); T->setTextColor(TFT_DARKGREY, C_KEY); T->drawString(r, 308, rows[i].y + 15, 2); }
        }
        if (g_apn == 0) {
            T->setTextDatum(MC_DATUM); T->setTextColor(C_WARN, C_BG);
            T->drawString("NO NETWORKS FOUND", 160, 100, 4);
        }
        draw_btn(bCancel, "CANCEL", C_ERR, TFT_WHITE);
        draw_btn(bScan, "RESCAN", C_ACC, TFT_WHITE);
        draw_btn(bMore, "MORE >", C_EDGE, TFT_WHITE);

        for (;;) {
            int16_t x, y; wait_tap(x, y); wait_release();
            if (bCancel.hit(x, y)) return -1;
            if (bScan.hit(x, y))   return -2;
            if (bMore.hit(x, y))   { page = (page + 1) % pages; break; }
            for (int i = 0; i < PER; i++) {
                const int idx = page * PER + i;
                if (idx < g_apn && rows[i].hit(x, y)) return idx;
            }
        }
    }
}

// ── keyboard ────────────────────────────────────────────────────────────────
static const char *KEYS[3][4] = {
    {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"},
    {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"},
    {"!@#$%^&*()", "-_=+[]{}\\|", ";:'\",.<>/", "?~`    "},
};
static const int ROW_Y[4] = {64, 98, 132, 166};
static const int ROW_X[4] = {0, 0, 16, 48};
static const int KH = 34;

static const Rect RC_BACK  = {0, 0, 64, 28};
static const Rect RC_SHIFT = {0, 166, 48, 34};
static const Rect RC_BKSP  = {272, 166, 48, 34};
static const Rect RC_MODE  = {0, 200, 60, 38};
static const Rect RC_SPACE = {60, 200, 140, 38};
static const Rect RC_SHOW  = {200, 200, 56, 38};
static const Rect RC_OK    = {256, 200, 64, 38};

static void draw_field(const char *pass, bool show, bool open) {
    T->fillRect(0, 30, 320, 32, C_BG);
    T->drawRoundRect(4, 31, 312, 30, 5, C_ACC);
    T->setTextDatum(ML_DATUM);
    T->setTextColor(TFT_WHITE, C_BG);
    const int len = strlen(pass);
    char buf[48];
    const int maxc = 34;                              // fits the field width at font 2
    const int start = len > maxc ? len - maxc : 0;
    int n = 0;
    for (int i = start; i < len && n < 46; i++) buf[n++] = show ? pass[i] : '*';
    buf[n] = 0;
    if (len == 0 && open) T->drawString("(open network - press OK)", 12, 46, 2);
    else { T->drawString(buf, 12, 46, 2); }
    const int cx = 12 + T->textWidth(buf, 2);               // text cursor
    T->drawFastVLine(cx + 1, 38, 16, C_WARN);
}

static void draw_keys(int layer, bool show) {
    for (int r = 0; r < 4; r++) {
        const char *row = KEYS[layer][r];
        for (int i = 0; row[i]; i++) {
            if (row[i] == ' ') continue;
            const Rect k = {(int16_t)(ROW_X[r] + i * 32), (int16_t)ROW_Y[r], 32, (int16_t)KH};
            char s[2] = {row[i], 0};
            draw_btn(k, s, C_EDGE, TFT_WHITE);
        }
    }
    draw_btn(RC_SHIFT, layer == 1 ? "SHIFT" : "shift", layer == 2 ? C_EDGE : C_WARN, layer == 2 ? C_EDGE : TFT_WHITE, layer == 1);
    draw_btn(RC_BKSP, "DEL", C_ERR, TFT_WHITE);
    draw_btn(RC_MODE, layer == 2 ? "ABC" : "?123", C_ACC, TFT_WHITE);
    draw_btn(RC_SPACE, "SPACE", C_EDGE, TFT_WHITE);
    draw_btn(RC_SHOW, show ? "HIDE" : "SHOW", C_EDGE, TFT_WHITE);
    draw_btn(RC_OK, "OK", C_OK, TFT_WHITE);
}

// returns true = OK pressed (pass filled in), false = BACK
static bool password_screen(const char *ssid, bool open, char *pass, size_t cap) {
    int layer = 0;
    bool show = true;
    T->fillScreen(C_BG);
    draw_btn(RC_BACK, "< BACK", C_ERR, TFT_WHITE);
    T->setTextDatum(ML_DATUM); T->setTextColor(C_ACC, C_BG);
    char nm[28]; strlcpy(nm, ssid, sizeof(nm));
    T->drawString(nm, 72, 14, 2);
    draw_field(pass, show, open);
    draw_keys(layer, show);

    for (;;) {
        int16_t x, y; wait_tap(x, y);
        const size_t len = strlen(pass);

        if (RC_BACK.hit(x, y)) { wait_release(); return false; }
        if (RC_OK.hit(x, y))   { draw_btn(RC_OK, "OK", C_OK, TFT_WHITE, true); wait_release(); return true; }

        bool redrawKeys = false;
        if (RC_SHIFT.hit(x, y)) { if (layer != 2) { layer = layer ? 0 : 1; redrawKeys = true; } }
        else if (RC_MODE.hit(x, y)) { layer = (layer == 2) ? 0 : 2; redrawKeys = true; }
        else if (RC_SHOW.hit(x, y)) { show = !show; redrawKeys = true; }
        else if (RC_BKSP.hit(x, y)) { if (len) pass[len - 1] = 0; }
        else if (RC_SPACE.hit(x, y)) { if (len + 1 < cap) { pass[len] = ' '; pass[len + 1] = 0; } }
        else {
            for (int r = 0; r < 4; r++) {
                const char *row = KEYS[layer][r];
                for (int i = 0; row[i]; i++) {
                    if (row[i] == ' ') continue;
                    const Rect k = {(int16_t)(ROW_X[r] + i * 32), (int16_t)ROW_Y[r], 32, (int16_t)KH};
                    if (!k.hit(x, y)) continue;
                    char s[2] = {row[i], 0};
                    draw_btn(k, s, C_EDGE, TFT_WHITE, true);        // press feedback
                    if (len + 1 < cap) { pass[len] = row[i]; pass[len + 1] = 0; }
                    if (layer == 1) { layer = 0; redrawKeys = true; } // one-shot shift
                    r = 4; break;
                }
            }
        }
        wait_release();
        (void)redrawKeys;
        draw_keys(layer, show);                                        // also clears the pressed-key highlight
        draw_field(pass, show, open);
    }
}

// ── connection test ─────────────────────────────────────────────────────────
static bool try_connect(const char *ssid, const char *pass) {
    header("CONNECTING");
    T->setTextDatum(TC_DATUM); T->setTextColor(TFT_WHITE, C_BG);
    T->drawString(ssid, 160, 60, 2);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, pass);
    const uint32_t t0 = millis();
    int dots = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) {
        delay(300);
        char d[8]; dots = (dots % 6) + 1;
        for (int i = 0; i < dots; i++) d[i] = '.';
        d[dots] = 0;
        center_msg(d, C_WARN);
    }
    return WiFi.status() == WL_CONNECTED;
}

[[noreturn]] void ota_wifi_setup_run(TFT_eSPI &tft, bool (*touch_get)(int16_t *, int16_t *)) {
    esp_task_wdt_delete(NULL);          // long interactive screen; we always reboot at the end
    T = &tft;
    getTouch = touch_get;
    tft.setRotation(1);
    wait_release();

    for (;;) {
        scan();
        const int sel = list_screen();
        if (sel == -1) reboot();
        if (sel == -2) continue;

        char ssid[33];
        strlcpy(ssid, g_aps[sel].ssid, sizeof(ssid));
        const bool open = g_aps[sel].open;
        char pass[65] = "";

        for (;;) {
            if (!password_screen(ssid, open, pass, sizeof(pass))) break;      // BACK -> list
            if (try_connect(ssid, pass)) {
                ota_wifi_save(ssid, pass);
                header("WIFI SAVED");
                center_msg("CONNECTED", C_OK, ssid, TFT_WHITE);
                delay(2000);
                reboot();
            }
            center_msg("CONNECT FAILED", C_ERR, "CHECK THE PASSWORD", TFT_WHITE);
            delay(2200);
        }
    }
}
