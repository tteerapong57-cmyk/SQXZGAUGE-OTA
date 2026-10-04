#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include "sqxzgauge_page.h"
#include "kline.h"
#include "gps.h"
#include "gauge_ui.h"
#include "app_config.h"

extern const lv_font_t dseg7_24;

namespace {

#define NTP_BG       lv_color_hex(0x0f1419)
#define NTP_NAVY     lv_color_hex(0x1a1f3a)
#define NTP_NAVY2    lv_color_hex(0x2c3e50)
#define NTP_WHITE    lv_color_hex(0xf8f9fc)
#define NTP_LGRAY    lv_color_hex(0xe8eef5)
#define NTP_OK       lv_color_hex(0x2ee59d)
#define NTP_WARN     lv_color_hex(0xffb700)
#define NTP_ERR      lv_color_hex(0xff4757)
#define NTP_COLD     lv_color_hex(0x4a90e2)
#define NTP_ECT_HIGH_BG lv_color_hex(0xffe6e6)
#define NTP_AFR_INVALID lv_color_hex(0x8fa3bd)
// (แก้ไข) ฟอนต์ AFR หน้า 15: โหมดกลางวัน = ดำสนิท, โหมดกลางคืน = ขาวสนิท
#define NTP_AFR_DAY     lv_color_hex(0x000000)
#define NTP_AFR_NIGHT   lv_color_hex(0xffffff)
#define NTP_UNIT      lv_color_hex(0x6688bb)
// (แก้ไข) ตอนยังไม่มีข้อมูล AFR ให้แสดง 0.0 แทน --.- ให้ตรงกับหน้า GAUGE/กราฟ
// ใช้เฉพาะช่อง AFR เท่านั้น ช่องอื่น (SPEED/RPM/TPS/ECT/BATT/INJ) ใช้ "0" อยู่แล้ว
#define NTP_PLACEHOLDER "0.0"
#define NTP_TIME_PLACEHOLDER "--:--:--"

// PAGE 15 keeps the same layout in both modes, but swaps the surface/text
// palette so it visually follows the global dashboard DAY/NIGHT setting.
#define NTP_DAY_BG       lv_color_hex(0xf2f3f5)
#define NTP_DAY_BOX      lv_color_hex(0xffffff)
#define NTP_DAY_BOTTOM   lv_color_hex(0xe8eef5)
#define NTP_DAY_BORDER   lv_color_hex(0xc2c7cf)
#define NTP_DAY_PRIMARY  lv_color_hex(0x111318)
#define NTP_DAY_SECONDARY lv_color_hex(0x33404e)
#define NTP_DAY_UNIT     lv_color_hex(0x59749b)
#define NTP_DAY_HEADER   lv_color_hex(0x1a1f3a)
// (แก้ไข) พื้นหลัง BAR RPM หน้า 15 โหมดกลางวัน เปลี่ยนเป็นสีเทาเข้ม เท่ากับสีกล่อง
// หัวข้อ "HONDA -PEERAPOL-" ด้านบน (เท่ากับ NTP_DAY_HEADER)
#define NTP_DAY_TRACK    NTP_DAY_HEADER

#define NTP_NIGHT_BG      lv_color_hex(0x080b10)
#define NTP_NIGHT_BOX     lv_color_hex(0x121722)
#define NTP_NIGHT_BOTTOM  lv_color_hex(0x0d1218)
#define NTP_NIGHT_BORDER  lv_color_hex(0x2a3140)
#define NTP_NIGHT_PRIMARY lv_color_hex(0xf4f7fb)
#define NTP_NIGHT_SECONDARY lv_color_hex(0xc6cfdb)
#define NTP_NIGHT_UNIT    lv_color_hex(0x8ea9c7)
#define NTP_NIGHT_HEADER  lv_color_hex(0x111827)
// (แก้ไข) เข้มขึ้นจากเดิม 0x081026 ให้แถบ RPM ที่ยังไม่ติดไฟแยกออกจากกล่อง
// (NTP_NIGHT_BOX 0x121722) ได้ชัดเจนขึ้นทั้งสองโหมด
#define NTP_NIGHT_TRACK   lv_color_hex(0x040611)

constexpr int NUM_TICKS = 20;
constexpr int SCREEN_W = 320;

static lv_obj_t *g_scr = nullptr;
static lv_obj_t *g_ticks[NUM_TICKS] = {};
static lv_obj_t *g_lbl_speed = nullptr;
static lv_obj_t *g_lbl_rpm = nullptr;
static lv_obj_t *g_lbl_afr = nullptr;
static lv_obj_t *g_lbl_tps = nullptr;
static lv_obj_t *g_lbl_ect = nullptr;
static lv_obj_t *g_lbl_batt = nullptr;
static lv_obj_t *g_lbl_inj = nullptr;
static lv_obj_t *g_box_ect = nullptr;
static lv_obj_t *g_lbl_rpm_max = nullptr;
static lv_obj_t *g_lbl_speed_max = nullptr;
static lv_obj_t *g_ecu_status_led = nullptr;
static lv_obj_t *g_gps_status_led = nullptr;
static lv_obj_t *g_gps_status_label = nullptr;
static lv_obj_t *g_bottom_bar = nullptr;
static lv_obj_t *g_header = nullptr;
static lv_obj_t *g_rpm_track = nullptr;
static bool g_day_mode_applied = false;
static bool g_theme_applied_once = false;

static float g_rpm_max = 0.0f;
static float g_speed_max = 0.0f;
static float g_rpm_bar = 0.0f;
static uint32_t g_rpm_bar_ms = 0;
static float g_speed_display = 0.0f;
static bool g_ready = false;

// Persistent text storage. lv_label_set_text_static() points directly to these
// buffers, so LVGL does not allocate/free a new heap block on each update.
static char g_txt_speed[12]     = "0";
static char g_txt_rpm[12]       = "0";
static char g_txt_afr[12]       = NTP_PLACEHOLDER;
static char g_txt_tps[12]       = "0";
static char g_txt_ect[12]       = "0";
static char g_txt_batt[12]      = "0";
static char g_txt_inj[12]       = "0";
static char g_txt_time[12]      = NTP_TIME_PLACEHOLDER;
static char g_txt_rpm_max[24]   = "RPM MAX : 0";
static char g_txt_speed_max[24] = "SPEED MAX : 0";

static bool g_ecu_led_state = false;
static bool g_gps_led_state = false;
static int g_afr_state = -1;
static int g_ect_state = -1;
static bool g_ect_hot_bg = false;
static bool g_batt_low = false;
static int g_last_tick_active = -1;
static bool g_last_tick_redline = false;
static bool g_last_tick_blink = true;

struct HeapStats {
    uint32_t free_heap;
    uint32_t min_free_heap;
    uint32_t largest_internal;
};

static HeapStats heap_stats_now() {
    HeapStats h;
    h.free_heap = ESP.getFreeHeap();
    h.min_free_heap = ESP.getMinFreeHeap();
    h.largest_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return h;
}

static uint32_t g_heap_init_free = 0;
static uint32_t g_heap_init_largest = 0;
static uint32_t g_heap_diag_ms = 0;
static uint32_t g_heap_sample_ms = 0;

static bool set_label_static_if_changed(lv_obj_t *obj, char *storage, size_t storage_size, const char *text) {
    if (!obj || !storage || storage_size == 0 || !text) return false;
    if (strncmp(storage, text, storage_size) == 0) return false;
    snprintf(storage, storage_size, "%s", text);
    lv_label_set_text_static(obj, storage);
    return true;
}

static bool set_label_fmt_if_changed(lv_obj_t *obj, char *storage, size_t storage_size, const char *fmt, double value) {
    if (!obj || !storage || storage_size == 0 || !fmt) return false;
    char next[32];
    const int n = snprintf(next, sizeof(next), fmt, value);
    if (n < 0) return false;
    if (strncmp(storage, next, storage_size) == 0) return false;
    snprintf(storage, storage_size, "%s", next);
    lv_label_set_text_static(obj, storage);
    return true;
}

static void heap_diag(const HeapStats &before, const HeapStats &after, uint32_t now) {
    const int32_t free_delta = (int32_t)after.free_heap - (int32_t)before.free_heap;
    const int32_t largest_delta = (int32_t)after.largest_internal - (int32_t)before.largest_internal;
    const bool suspicious = (free_delta < -128) || (largest_delta < -128);

    if (!suspicious && (now - g_heap_diag_ms < 5000U)) return;
    if (now - g_heap_diag_ms < 5000U && !suspicious) return;
    g_heap_diag_ms = now;

    if (!ENABLE_RUNTIME_DEBUG) return;
    Serial.printf(
        "[NTP HEAP] before=%lu after=%lu delta=%ld min=%lu largest=%lu ld=%ld base=%lu/%lu\n",
        (unsigned long)before.free_heap,
        (unsigned long)after.free_heap,
        (long)free_delta,
        (unsigned long)after.min_free_heap,
        (unsigned long)after.largest_internal,
        (long)largest_delta,
        (unsigned long)g_heap_init_free,
        (unsigned long)g_heap_init_largest
    );
}

static bool color_eq(lv_color_t a, lv_color_t b) {
#if LV_COLOR_DEPTH == 32
    return a.full == b.full;
#else
    return lv_color_to32(a) == lv_color_to32(b);
#endif
}

static void ntp_apply_text_theme(lv_obj_t *obj, bool day) {
    if (!obj) return;
    const lv_color_t c = lv_obj_get_style_text_color(obj, LV_PART_MAIN);
    if (day) {
        if (color_eq(c, NTP_NAVY) || color_eq(c, NTP_DAY_PRIMARY) || color_eq(c, NTP_NIGHT_PRIMARY))
            lv_obj_set_style_text_color(obj, NTP_DAY_PRIMARY, 0);
        else if (color_eq(c, NTP_NAVY2) || color_eq(c, NTP_DAY_SECONDARY) || color_eq(c, NTP_NIGHT_SECONDARY))
            lv_obj_set_style_text_color(obj, NTP_DAY_SECONDARY, 0);
        else if (color_eq(c, NTP_UNIT) || color_eq(c, NTP_DAY_UNIT) || color_eq(c, NTP_NIGHT_UNIT))
            lv_obj_set_style_text_color(obj, NTP_DAY_UNIT, 0);
    } else {
        if (color_eq(c, NTP_NAVY) || color_eq(c, NTP_DAY_PRIMARY) || color_eq(c, NTP_NIGHT_PRIMARY))
            lv_obj_set_style_text_color(obj, NTP_NIGHT_PRIMARY, 0);
        else if (color_eq(c, NTP_NAVY2) || color_eq(c, NTP_DAY_SECONDARY) || color_eq(c, NTP_NIGHT_SECONDARY))
            lv_obj_set_style_text_color(obj, NTP_NIGHT_SECONDARY, 0);
        else if (color_eq(c, NTP_UNIT) || color_eq(c, NTP_DAY_UNIT) || color_eq(c, NTP_NIGHT_UNIT))
            lv_obj_set_style_text_color(obj, NTP_NIGHT_UNIT, 0);
    }
}

static void ntp_apply_tree_theme(lv_obj_t *obj, bool day) {
    if (!obj) return;

    const lv_color_t bg = lv_obj_get_style_bg_color(obj, LV_PART_MAIN);
    if (day) {
        if (color_eq(bg, NTP_BG) || color_eq(bg, NTP_NIGHT_BG))
            lv_obj_set_style_bg_color(obj, NTP_DAY_BG, 0);
        else if (color_eq(bg, NTP_WHITE) || color_eq(bg, NTP_DAY_BOX) || color_eq(bg, NTP_NIGHT_BOX))
            lv_obj_set_style_bg_color(obj, NTP_DAY_BOX, 0);
        else if (color_eq(bg, NTP_LGRAY) || color_eq(bg, NTP_DAY_BOTTOM) || color_eq(bg, NTP_NIGHT_BOTTOM))
            lv_obj_set_style_bg_color(obj, NTP_DAY_BOTTOM, 0);
        else if (color_eq(bg, NTP_NAVY) || color_eq(bg, NTP_DAY_HEADER) || color_eq(bg, NTP_NIGHT_HEADER))
            lv_obj_set_style_bg_color(obj, NTP_DAY_HEADER, 0);
    } else {
        if (color_eq(bg, NTP_BG) || color_eq(bg, NTP_DAY_BG))
            lv_obj_set_style_bg_color(obj, NTP_NIGHT_BG, 0);
        else if (color_eq(bg, NTP_WHITE) || color_eq(bg, NTP_DAY_BOX) || color_eq(bg, NTP_NIGHT_BOX))
            lv_obj_set_style_bg_color(obj, NTP_NIGHT_BOX, 0);
        else if (color_eq(bg, NTP_LGRAY) || color_eq(bg, NTP_DAY_BOTTOM) || color_eq(bg, NTP_NIGHT_BOTTOM))
            lv_obj_set_style_bg_color(obj, NTP_NIGHT_BOTTOM, 0);
        else if (color_eq(bg, NTP_NAVY) || color_eq(bg, NTP_DAY_HEADER) || color_eq(bg, NTP_NIGHT_HEADER))
            lv_obj_set_style_bg_color(obj, NTP_NIGHT_HEADER, 0);
    }

    const lv_color_t bc = lv_obj_get_style_border_color(obj, LV_PART_MAIN);
    if (day) {
        if (color_eq(bc, NTP_NAVY2) || color_eq(bc, NTP_DAY_BORDER) || color_eq(bc, NTP_NIGHT_BORDER))
            lv_obj_set_style_border_color(obj, NTP_DAY_BORDER, 0);
        else if (color_eq(bc, NTP_NAVY))
            lv_obj_set_style_border_color(obj, NTP_DAY_HEADER, 0);
    } else {
        if (color_eq(bc, NTP_NAVY2) || color_eq(bc, NTP_DAY_BORDER) || color_eq(bc, NTP_NIGHT_BORDER))
            lv_obj_set_style_border_color(obj, NTP_NIGHT_BORDER, 0);
        else if (color_eq(bc, NTP_NAVY))
            lv_obj_set_style_border_color(obj, NTP_NIGHT_HEADER, 0);
    }

    ntp_apply_text_theme(obj, day);

    uint32_t i = 0; lv_obj_t *child = nullptr;
    while ((child = lv_obj_get_child(obj, i++)) != nullptr) {
        ntp_apply_tree_theme(child, day);
    }
}

static void ntp_apply_runtime_theme_colors(bool day) {
    if (!g_scr) return;

    lv_obj_set_style_bg_color(g_scr, day ? NTP_DAY_BG : NTP_NIGHT_BG, 0);
    if (g_header) {
        lv_obj_set_style_bg_color(g_header, day ? NTP_DAY_HEADER : NTP_NIGHT_HEADER, 0);
    }
    if (g_rpm_track) {
        lv_obj_set_style_bg_color(g_rpm_track, day ? NTP_DAY_TRACK : NTP_NIGHT_TRACK, 0);
    }

    // Dynamic labels keep their status/accent colors, while normal values
    // follow the current surface theme.
    lv_obj_set_style_text_color(g_lbl_speed, day ? NTP_DAY_PRIMARY : NTP_NIGHT_PRIMARY, 0);
    lv_obj_set_style_text_color(g_lbl_rpm, day ? NTP_DAY_PRIMARY : NTP_NIGHT_PRIMARY, 0);
    if (g_lbl_afr) {
        lv_obj_set_style_text_color(g_lbl_afr, day ? NTP_AFR_DAY : NTP_AFR_NIGHT, 0);
    }
    if (g_lbl_ect) {
        const lv_color_t c = (g_ect_state == 2) ? NTP_ERR :
                             ((g_ect_state == 1) ? NTP_COLD : (day ? NTP_DAY_SECONDARY : NTP_NIGHT_SECONDARY));
        lv_obj_set_style_text_color(g_lbl_ect, c, 0);
        lv_obj_set_style_bg_color(g_box_ect, g_ect_hot_bg ? NTP_ECT_HIGH_BG : (day ? NTP_DAY_BOX : NTP_NIGHT_BOX), 0);
    }
    if (g_lbl_batt)
        lv_obj_set_style_text_color(g_lbl_batt, g_batt_low ? NTP_ERR : (day ? NTP_DAY_SECONDARY : NTP_NIGHT_SECONDARY), 0);
    if (g_lbl_tps)
        lv_obj_set_style_text_color(g_lbl_tps, day ? NTP_DAY_SECONDARY : NTP_NIGHT_SECONDARY, 0);
    if (g_lbl_inj)
        lv_obj_set_style_text_color(g_lbl_inj, day ? NTP_DAY_SECONDARY : NTP_NIGHT_SECONDARY, 0);
    if (g_lbl_rpm_max)
        lv_obj_set_style_text_color(g_lbl_rpm_max, day ? NTP_DAY_PRIMARY : NTP_NIGHT_PRIMARY, 0);
    if (g_lbl_speed_max)
        lv_obj_set_style_text_color(g_lbl_speed_max, day ? NTP_DAY_PRIMARY : NTP_NIGHT_PRIMARY, 0);

    if (g_ecu_status_led) lv_obj_set_style_border_color(g_ecu_status_led, NTP_WHITE, 0);
    if (g_gps_status_led) lv_obj_set_style_border_color(g_gps_status_led, NTP_WHITE, 0);
    if (g_gps_status_label) lv_obj_set_style_text_color(g_gps_status_label, NTP_WHITE, 0);

    // Re-apply tick colors from the current bar state after the generic tree pass.
    int active = g_last_tick_active;
    if (active < 0) active = 0;
    if (active > NUM_TICKS) active = NUM_TICKS;
    const bool redline = g_last_tick_redline;
    const bool rl_on = g_last_tick_blink;
    for (int i = 0; i < NUM_TICKS; ++i) {
        lv_color_t col;
        // (แก้ไข) เดิมใช้ NTP_NAVY2 (เทาอมฟ้าอ่อน) ซึ่งคนละสีกับที่ลูปอัปเดตค่า
        // RPM จริงใช้ (NTP_NIGHT_TRACK สีเข้ม) พอมีการ apply ธีมใหม่ (เช่น
        // สลับ DAY/NIGHT หรือปรับความสว่าง) ระหว่างที่ RPM นิ่ง/คงที่
        // (ขับจริงที่รอบเครื่องคงที่) ช่องที่ยังไม่ติดไฟจะค้างเป็นสีเทาอ่อนผิดสี
        // ไปจนกว่ารอบเครื่องจะขยับข้ามช่อง — ตอนนี้ใช้สีเดียวกับลูปอัปเดตจริง
        if (i >= active) col = day ? NTP_DAY_TRACK : NTP_NIGHT_TRACK;
        else if (i < (int)(NUM_TICKS * 0.60f)) col = NTP_OK;
        else if (i < (int)(NUM_TICKS * 0.85f)) col = NTP_WARN;
        else if (redline && !rl_on) col = lv_color_hex(0x331111);
        else col = NTP_ERR;
        if (g_ticks[i]) lv_obj_set_style_bg_color(g_ticks[i], col, 0);
    }
}

static void ntp_apply_theme(bool day) {
    if (!g_scr) return;
    if (g_theme_applied_once && g_day_mode_applied == day) return;
    ntp_apply_tree_theme(g_scr, day);
    ntp_apply_runtime_theme_colors(day);
    g_day_mode_applied = day;
    g_theme_applied_once = true;
}

static float smooth_rpm_bar(float target) {
    if (target < 0.0f) target = 0.0f;
    const uint32_t now = millis();
    if (g_rpm_bar_ms == 0) {
        g_rpm_bar_ms = now;
        g_rpm_bar = target;
        return g_rpm_bar;
    }
    float dt = (float)(now - g_rpm_bar_ms) * 0.001f;
    g_rpm_bar_ms = now;
    if (dt <= 0.0f) return g_rpm_bar;
    if (dt > 0.10f) dt = 0.10f;
    const float tau = (target >= g_rpm_bar) ? 0.055f : 0.085f;
    float alpha = 1.0f - expf(-dt / tau);
    if (fabsf(target - g_rpm_bar) > 2200.0f) alpha = fmaxf(alpha, 0.40f);
    g_rpm_bar += (target - g_rpm_bar) * alpha;
    if (fabsf(target - g_rpm_bar) < 3.0f) g_rpm_bar = target;
    return g_rpm_bar;
}

static lv_obj_t* mk_box(lv_obj_t *s, int x, int y, int w, int h, int border = 2) {
    lv_obj_t *b = lv_obj_create(s);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(b, NTP_WHITE, 0);
    lv_obj_set_style_border_color(b, NTP_NAVY2, 0);
    lv_obj_set_style_border_width(b, border, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 2, 0);
    lv_obj_set_style_pad_all(b, 3, 0);
    return b;
}

static void mk_title(lv_obj_t *box, const char *txt) {
    lv_obj_t *l = lv_label_create(box);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, NTP_NAVY, 0);
    lv_label_set_text_static(l, txt);
    lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 0);
}

static lv_obj_t* mk_val(lv_obj_t *box, const lv_font_t *font) {
    lv_obj_t *l = lv_label_create(box);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, NTP_NAVY2, 0);
    lv_label_set_text_static(l, "0");
    lv_obj_align(l, LV_ALIGN_CENTER, 0, 10);
    return l;
}

static lv_obj_t* mk_stat_box(lv_obj_t *scr, int x, int y, int w, int h,
                             const char *title, lv_obj_t **val_out) {
    lv_obj_t *box = mk_box(scr, x, y, w, h, 3);
    mk_title(box, title);
    *val_out = mk_val(box, &dseg7_24);
    return box;
}

} // namespace

void sqxzgauge_page_init() {
    if (g_scr) return;

    g_scr = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr, NTP_BG, 0);
    lv_obj_set_style_bg_opa(g_scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_scr, 0, 0);
    lv_obj_set_style_pad_all(g_scr, 0, 0);

    lv_obj_t *hdr = lv_obj_create(g_scr);
    g_header = hdr;
    lv_obj_set_pos(hdr, 2, 2);
    lv_obj_set_size(hdr, 316, 18);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(hdr, NTP_NAVY, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 3, 0);
    lv_obj_set_style_pad_all(hdr, 2, 0);

    lv_obj_t *hdr_l = lv_label_create(hdr);
    lv_obj_set_style_text_font(hdr_l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hdr_l, NTP_WHITE, 0);
    lv_label_set_text_static(hdr_l, "HONDA");
    lv_obj_align(hdr_l, LV_ALIGN_LEFT_MID, 2, 0);

    lv_obj_t *hdr_c = lv_label_create(hdr);
    lv_obj_set_style_text_font(hdr_c, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hdr_c, NTP_WHITE, 0);
    lv_label_set_text_static(hdr_c, "-SQXZGAUGE-");
    lv_obj_align(hdr_c, LV_ALIGN_CENTER, 0, 0);

    g_ecu_status_led = lv_obj_create(hdr);
    lv_obj_set_size(g_ecu_status_led, 10, 10);
    lv_obj_align(g_ecu_status_led, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_clear_flag(g_ecu_status_led, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(g_ecu_status_led, 1, 0);
    lv_obj_set_style_border_color(g_ecu_status_led, NTP_WHITE, 0);
    lv_obj_set_style_radius(g_ecu_status_led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_ecu_status_led, NTP_ERR, 0);

    g_gps_status_led = lv_obj_create(hdr);
    lv_obj_set_size(g_gps_status_led, 10, 10);
    lv_obj_align(g_gps_status_led, LV_ALIGN_RIGHT_MID, -18, 0);
    lv_obj_clear_flag(g_gps_status_led, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(g_gps_status_led, 1, 0);
    lv_obj_set_style_border_color(g_gps_status_led, NTP_WHITE, 0);
    lv_obj_set_style_radius(g_gps_status_led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_gps_status_led, NTP_ERR, 0);

    g_gps_status_label = lv_label_create(hdr);
    lv_obj_set_style_text_font(g_gps_status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_gps_status_label, NTP_WHITE, 0);
    lv_label_set_text_static(g_gps_status_label, g_txt_time);
    lv_obj_set_width(g_gps_status_label, 62);
    lv_obj_set_style_text_align(g_gps_status_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_long_mode(g_gps_status_label, LV_LABEL_LONG_CLIP);
    lv_obj_align(g_gps_status_label, LV_ALIGN_RIGHT_MID, -31, 0);

    const int Y0 = 22, TOPH = 100, BOXW = 157, X1 = 2, X2 = 2 + BOXW + 2;
    const int SUBW = (BOXW - 2) / 2;
    const int SUBW2 = BOXW - 2 - SUBW;
    const int XS1 = X1, XS2 = X1 + SUBW + 2;

    lv_obj_t *bx_spd = mk_box(g_scr, XS1, Y0, SUBW, TOPH, 3);
    mk_title(bx_spd, "SPEED");
    lv_obj_t *unit_spd = lv_label_create(bx_spd);
    lv_obj_set_style_text_font(unit_spd, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(unit_spd, NTP_UNIT, 0);
    lv_label_set_text_static(unit_spd, "km/h");
    lv_obj_align(unit_spd, LV_ALIGN_BOTTOM_LEFT, 4, -2);
    g_lbl_speed = lv_label_create(bx_spd);
    lv_obj_set_style_text_font(g_lbl_speed, &dseg7_24, 0);
    lv_obj_set_style_text_color(g_lbl_speed, NTP_NAVY, 0);
    lv_label_set_text_static(g_lbl_speed, g_txt_speed);
    lv_obj_align(g_lbl_speed, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *bx_afr = mk_box(g_scr, XS2, Y0, SUBW2, TOPH, 3);
    mk_title(bx_afr, "AFR");
    g_lbl_afr = lv_label_create(bx_afr);
    lv_obj_set_style_text_font(g_lbl_afr, &dseg7_24, 0);
    lv_obj_set_style_text_color(g_lbl_afr, NTP_AFR_NIGHT, 0);
    lv_label_set_text_static(g_lbl_afr, g_txt_afr);
    lv_obj_align(g_lbl_afr, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *bx_rpm = mk_box(g_scr, X2, Y0, BOXW, TOPH, 3);
    const int BORDER = 3, PAD = 3, TICK_MARGIN = 5;
    const int TRACK_Y = 9, TRACK_H = 25;
    const int inner_x0 = X2 + BORDER + PAD + TICK_MARGIN;
    const int inner_w = BOXW - 2 * (BORDER + PAD) - 2 * TICK_MARGIN;
    const int TG = 2;
    const int TW = (inner_w - (NUM_TICKS - 1) * TG) / NUM_TICKS;
    const int ticks_total_w = NUM_TICKS * TW + (NUM_TICKS - 1) * TG;
    const int ticks_x0 = inner_x0 + (inner_w - ticks_total_w) / 2;

    lv_obj_t *track = lv_obj_create(g_scr);
    g_rpm_track = track;
    lv_obj_set_pos(track, inner_x0 - 3, Y0 + TRACK_Y - 3);
    lv_obj_set_size(track, inner_w + 6, TRACK_H + 6);
    lv_obj_clear_flag(track, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(track, lv_color_hex(0x040611), 0);
    lv_obj_set_style_border_width(track, 0, 0);
    lv_obj_set_style_radius(track, 5, 0);

    for (int i = 0; i < NUM_TICKS; ++i) {
        lv_obj_t *tk = lv_obj_create(g_scr);
        lv_obj_set_pos(tk, ticks_x0 + i * (TW + TG), Y0 + TRACK_Y);
        lv_obj_set_size(tk, TW, TRACK_H);
        lv_obj_clear_flag(tk, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(tk, 0, 0);
        lv_obj_set_style_radius(tk, 2, 0);
        lv_obj_set_style_bg_color(tk, NTP_NAVY2, 0);
        g_ticks[i] = tk;
    }

    lv_obj_t *unit_rpm = lv_label_create(bx_rpm);
    lv_obj_set_style_text_font(unit_rpm, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(unit_rpm, NTP_UNIT, 0);
    lv_label_set_text_static(unit_rpm, "r/min");
    lv_obj_align(unit_rpm, LV_ALIGN_BOTTOM_LEFT, 4, -2);
    g_lbl_rpm = lv_label_create(bx_rpm);
    lv_obj_set_style_text_font(g_lbl_rpm, &dseg7_24, 0);
    lv_obj_set_style_text_color(g_lbl_rpm, NTP_NAVY, 0);
    lv_label_set_text_static(g_lbl_rpm, g_txt_rpm);
    lv_obj_align(g_lbl_rpm, LV_ALIGN_CENTER, 0, 10);

    const int FY = Y0 + TOPH + 2, FH = 80, SW = 77;
    const int SX0 = X1;
    const int SX1 = SX0 + (SW + 1) + 2;
    const int SX2 = SX1 + SW + 2;
    const int SX3 = SX2 + SW + 2;

    mk_stat_box(g_scr, SX0, FY, SW + 1, FH, "TPS %", &g_lbl_tps);
    g_box_ect = mk_stat_box(g_scr, SX1, FY, SW, FH, "ECT degC", &g_lbl_ect);
    mk_stat_box(g_scr, SX2, FY, SW, FH, "VOLT", &g_lbl_batt);
    mk_stat_box(g_scr, SX3, FY, SW + 1, FH, "INJ ms", &g_lbl_inj);
    lv_label_set_text_static(g_lbl_tps, g_txt_tps);
    lv_label_set_text_static(g_lbl_ect, g_txt_ect);
    lv_label_set_text_static(g_lbl_batt, g_txt_batt);
    lv_label_set_text_static(g_lbl_inj, g_txt_inj);

    const int MAX_Y = FY + FH + 2, MAX_H = 28;
    g_bottom_bar = lv_obj_create(g_scr);
    lv_obj_set_pos(g_bottom_bar, 0, MAX_Y);
    lv_obj_set_size(g_bottom_bar, SCREEN_W, MAX_H);
    lv_obj_clear_flag(g_bottom_bar, LV_OBJ_FLAG_SCROLLABLE);
    // (แก้ไข) เดิมกล่องล่าง (RPM MAX / SPEED MAX) ใช้สี/ขอบ/มุมคนละชุดกับกล่องบน
    // (SPEED/AFR/RPM, TPS/ECT/VOLT/INJ ที่สร้างจาก mk_box) ทำให้ดูไม่เข้าชุดกัน
    // ตอนนี้ใช้ค่าเดียวกับ mk_box ทุกอย่าง: พื้น NTP_WHITE (ตาม BOX ธีม),
    // ขอบ NTP_NAVY2 หนา 3px, มุมโค้ง 6, เงา 2px, padding 3px
    lv_obj_set_style_bg_color(g_bottom_bar, NTP_WHITE, 0);
    lv_obj_set_style_border_color(g_bottom_bar, NTP_NAVY2, 0);
    lv_obj_set_style_border_width(g_bottom_bar, 3, 0);
    lv_obj_set_style_radius(g_bottom_bar, 6, 0);
    lv_obj_set_style_shadow_width(g_bottom_bar, 2, 0);
    lv_obj_set_style_pad_all(g_bottom_bar, 3, 0);

    g_lbl_rpm_max = lv_label_create(g_bottom_bar);
    lv_obj_set_style_text_font(g_lbl_rpm_max, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_lbl_rpm_max, NTP_NAVY, 0);
    lv_label_set_text_static(g_lbl_rpm_max, g_txt_rpm_max);
    lv_obj_align(g_lbl_rpm_max, LV_ALIGN_LEFT_MID, 8, 0);

    g_lbl_speed_max = lv_label_create(g_bottom_bar);
    lv_obj_set_style_text_font(g_lbl_speed_max, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_lbl_speed_max, NTP_NAVY, 0);
    lv_label_set_text_static(g_lbl_speed_max, g_txt_speed_max);
    lv_obj_align(g_lbl_speed_max, LV_ALIGN_RIGHT_MID, -8, 0);

    // Apply the current global DAY/NIGHT mode once the complete PAGE 15 tree exists.
    ntp_apply_theme(gauge_ui_is_day_mode());

    g_ready = true;
    const HeapStats heap_init = heap_stats_now();
    g_heap_init_free = heap_init.free_heap;
    g_heap_init_largest = heap_init.largest_internal;
    g_heap_diag_ms = millis();
    if (ENABLE_RUNTIME_DEBUG) {
        Serial.printf("[NTP HEAP] init free=%lu min=%lu largest=%lu\n",
                      (unsigned long)heap_init.free_heap,
                      (unsigned long)heap_init.min_free_heap,
                      (unsigned long)heap_init.largest_internal);
    }
    sqxzgauge_page_update();
}

void sqxzgauge_page_set_day_mode(bool day) {
    ntp_apply_theme(day);
}

void sqxzgauge_page_update() {
    if (!g_ready || !g_scr) return;
    static uint32_t last = 0;
    const uint32_t now = millis();
    if ((uint32_t)(now - last) < 50U) return;
    last = now;

    ntp_apply_theme(gauge_ui_is_day_mode());
    const bool sample_heap = (g_heap_sample_ms == 0) || ((uint32_t)(now - g_heap_sample_ms) >= 1000U);
    HeapStats heap_before{};
    if (sample_heap) heap_before = heap_stats_now();

    KlineSnapshot kl{};
    kline_get_snapshot(&kl);
    const bool ecu_connected = kl.connected;
    const float rpm = ecu_connected && kline_snapshot_sensor_valid(&kl, KLINE_SENSOR_RPM) ? kl.rpm : 0.0f;
    const float tps = ecu_connected && kline_snapshot_sensor_valid(&kl, KLINE_SENSOR_TPS) ? kl.tps : 0.0f;
    const float ect = ecu_connected && kline_snapshot_sensor_valid(&kl, KLINE_SENSOR_ECT) ? kl.ect_c : 0.0f;
    const float batt = ecu_connected && kline_snapshot_sensor_valid(&kl, KLINE_SENSOR_BATT) ? kl.batt_v : 0.0f;
    const float inj = ecu_connected && kline_snapshot_sensor_valid(&kl, KLINE_SENSOR_INJ) ? kl.inj_ms : 0.0f;
    const bool afr_valid = ecu_connected && gauge_ui_is_afr_valid();

    const bool gps_fix = gps_has_fix();
    const float target_speed = gps_speed_valid() ? gps_speed_kmh() : 0.0f;
    const float delta = target_speed - g_speed_display;
    if (fabsf(delta) <= 0.6f) {
        g_speed_display = target_speed;
    } else {
        const float max_step = 1.8f;
        g_speed_display += (delta > 0.0f) ? fminf(delta, max_step) : fmaxf(delta, -max_step);
    }
    const float speed = fmaxf(0.0f, g_speed_display);

    if (rpm > g_rpm_max) g_rpm_max = rpm;
    if (speed > g_speed_max) g_speed_max = speed;

    // Only touch a widget when its visible state actually changed.
    if (g_ecu_led_state != ecu_connected) {
        g_ecu_led_state = ecu_connected;
        lv_obj_set_style_bg_color(g_ecu_status_led, ecu_connected ? NTP_OK : NTP_ERR, 0);
    }
    if (g_gps_led_state != gps_fix) {
        g_gps_led_state = gps_fix;
        lv_obj_set_style_bg_color(g_gps_status_led, gps_fix ? NTP_OK : NTP_ERR, 0);
    }

    char timebuf[12];
    gps_time_str(timebuf, sizeof(timebuf));
    if (timebuf[0] == '\0' || strlen(timebuf) < 8) {
        snprintf(timebuf, sizeof(timebuf), "%s", NTP_TIME_PLACEHOLDER);
    }
    set_label_static_if_changed(g_gps_status_label, g_txt_time, sizeof(g_txt_time), timebuf);

    set_label_fmt_if_changed(g_lbl_speed, g_txt_speed, sizeof(g_txt_speed), "%.0f", speed);
    set_label_fmt_if_changed(g_lbl_rpm, g_txt_rpm, sizeof(g_txt_rpm), "%.0f", rpm);
    set_label_fmt_if_changed(g_lbl_tps, g_txt_tps, sizeof(g_txt_tps), "%.0f", tps);
    set_label_fmt_if_changed(g_lbl_ect, g_txt_ect, sizeof(g_txt_ect), "%.0f", ect);
    set_label_fmt_if_changed(g_lbl_batt, g_txt_batt, sizeof(g_txt_batt), "%.1f", batt);
    set_label_fmt_if_changed(g_lbl_inj, g_txt_inj, sizeof(g_txt_inj), "%.1f", inj);

    if (afr_valid) {
        set_label_fmt_if_changed(g_lbl_afr, g_txt_afr, sizeof(g_txt_afr), "%.1f", gauge_ui_get_afr_est());
        g_afr_state = 1;
    } else {
        set_label_static_if_changed(g_lbl_afr, g_txt_afr, sizeof(g_txt_afr), NTP_PLACEHOLDER);
        g_afr_state = 0;
    }
    // (แก้ไข) เดิมสีตัวเลข AFR จะถูกตั้งเฉพาะตอน "เปลี่ยนสถานะ" (valid<->invalid)
    // เท่านั้น ทำให้ค่าตอนเริ่มต้น/รอสัญญาณ ECU อาจค้างเป็นสีที่ยังไม่ตรงกับ
    // ธีมปัจจุบัน (ดูเหมือนสีเทาไม่ชัดในโหมด NIGHT) — ตอนนี้บังคับสีให้ตรงกับ
    // โหมด DAY/NIGHT ปัจจุบันทุกครั้งที่อัปเดต ทำให้โหมด NIGHT เป็นสีขาวสนิท
    // (NTP_AFR_NIGHT) เสมอ ไม่ว่าจะอยู่สถานะไหนก็ตาม
    lv_obj_set_style_text_color(g_lbl_afr, g_day_mode_applied ? NTP_AFR_DAY : NTP_AFR_NIGHT, 0);

    const int new_ect_state = (ect > 110.0f) ? 2 : ((ect < 50.0f) ? 1 : 0);
    if (new_ect_state != g_ect_state) {
        g_ect_state = new_ect_state;
        const bool day = gauge_ui_is_day_mode();
        const lv_color_t ect_col = (new_ect_state == 2) ? NTP_ERR : ((new_ect_state == 1) ? NTP_COLD : (day ? NTP_DAY_SECONDARY : NTP_NIGHT_SECONDARY));
        lv_obj_set_style_text_color(g_lbl_ect, ect_col, 0);
    }
    const bool ect_hot_bg = (ect > 110.0f);
    if (ect_hot_bg != g_ect_hot_bg) {
        g_ect_hot_bg = ect_hot_bg;
        lv_obj_set_style_bg_color(g_box_ect, ect_hot_bg ? NTP_ECT_HIGH_BG : (gauge_ui_is_day_mode() ? NTP_DAY_BOX : NTP_NIGHT_BOX), 0);
    }

    const bool batt_low = (batt > 0.0f && batt < 12.0f);
    if (batt_low != g_batt_low) {
        g_batt_low = batt_low;
        lv_obj_set_style_text_color(g_lbl_batt, batt_low ? NTP_ERR : (gauge_ui_is_day_mode() ? NTP_DAY_SECONDARY : NTP_NIGHT_SECONDARY), 0);
    }

    set_label_fmt_if_changed(g_lbl_rpm_max, g_txt_rpm_max, sizeof(g_txt_rpm_max), "RPM MAX : %.0f", g_rpm_max);
    set_label_fmt_if_changed(g_lbl_speed_max, g_txt_speed_max, sizeof(g_txt_speed_max), "SPEED MAX : %.0f", g_speed_max);

    const float rpm_bar = smooth_rpm_bar(rpm);
    int active = (int)lroundf((rpm_bar / 10000.0f) * NUM_TICKS);
    if (active < 0) active = 0;
    if (active > NUM_TICKS) active = NUM_TICKS;
    const bool redline = active >= (int)(NUM_TICKS * 0.90f);
    static bool rl_on = true;
    static uint32_t rl_t = 0;
    if (redline && (uint32_t)(now - rl_t) > 110U) {
        rl_t = now;
        rl_on = !rl_on;
    }

    if (active != g_last_tick_active || redline != g_last_tick_redline || rl_on != g_last_tick_blink) {
        for (int i = 0; i < NUM_TICKS; ++i) {
            lv_color_t col;
            if (i >= active) col = gauge_ui_is_day_mode() ? NTP_DAY_TRACK : NTP_NIGHT_TRACK;
            else if (i < (int)(NUM_TICKS * 0.60f)) col = NTP_OK;
            else if (i < (int)(NUM_TICKS * 0.85f)) col = NTP_WARN;
            else if (redline && !rl_on) col = lv_color_hex(0x331111);
            else col = NTP_ERR;
            lv_obj_set_style_bg_color(g_ticks[i], col, 0);
        }
        g_last_tick_active = active;
        g_last_tick_redline = redline;
        g_last_tick_blink = rl_on;
    }

    if (sample_heap) {
        const HeapStats heap_after = heap_stats_now();
        g_heap_sample_ms = now;
        heap_diag(heap_before, heap_after, now);
    }
}

bool sqxzgauge_page_ready() { return g_ready && g_scr; }

void sqxzgauge_page_get_screen(void **out_screen) {
    if (!out_screen) return;
    *out_screen = (void*)g_scr;
}
