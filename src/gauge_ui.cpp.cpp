#include <Arduino.h>
#include <stdarg.h>
#include <lvgl.h>
#include <LittleFS.h>
#include <esp_system.h>
#include <Preferences.h>
#include "gauge_ui.h"
#include "app_config.h"
#include "kline.h"
#include "gps.h"
#include "sqxzgauge_page.h"

// Serial I/O is surprisingly expensive on tap/release paths. Keep all UI
// diagnostics available during debugging, but completely remove them from
// the release build so they cannot add touch latency.
#define UI_TRACE(...) do { if(ENABLE_RUNTIME_DEBUG) Serial.printf(__VA_ARGS__); } while(0)
#define UI_TRACE_LINE(msg) do { if(ENABLE_RUNTIME_DEBUG) Serial.println(msg); } while(0)

// ── หน้า 6: LOG ───────────────────────────────────────────────
// เก็บข้อมูลใน RAM เพื่อไม่เขียน Flash ทุก snapshot (ถนอมอายุ Flash)
static lv_obj_t *g_scr_log = NULL;
static lv_obj_t *g_log_rows[3] = {NULL,NULL,NULL};
static char g_log_text[5][120] = {{0}};
static uint8_t g_log_head = 0;
static uint8_t g_log_count = 0;
static char g_last_dtc_event[100] = "DTC: --";
static lv_obj_t *g_log_event_label = NULL;
static lv_obj_t *g_lbl_rpm_max_log = NULL;
static lv_obj_t *g_lbl_speed_max_log = NULL;
static lv_obj_t *g_dist_run_rows[3] = {NULL,NULL,NULL};
static lv_obj_t *g_lbl_ecm_id = NULL; // ECM ID - แสดงใต้ DIST RUNS หน้า 7

struct DistRunHistory {
    float target_m;
    uint32_t time_ms;
    float avg_speed_kmh;
    float max_speed_kmh;
    float max_rpm;
};
static DistRunHistory g_dist_run_history[3] = {};
static uint8_t g_dist_run_count = 0;

// ── หน้า 7: SETTINGS (DAY/NIGHT + BRIGHTNESS) ──────────────────
static lv_obj_t *g_scr_settings = NULL;
static lv_obj_t *g_lbl_settings_mode = NULL;
static lv_obj_t *g_lbl_brightness_value = NULL;
static lv_obj_t *g_btn_night = NULL;
static lv_obj_t *g_btn_day = NULL;
static lv_obj_t *g_btn_brightness_minus = NULL;
static lv_obj_t *g_btn_brightness_plus = NULL;
static uint8_t g_brightness = DEFAULT_BRIGHTNESS;
static bool g_day_mode = false;

// ── Extended pages 8..13 (appended after the existing LOG page) ────────────
static lv_obj_t *g_scr_health = NULL;
static lv_obj_t *g_scr_watchdog = NULL;
static lv_obj_t *g_scr_sensors = NULL;
static lv_obj_t *g_scr_data_logger = NULL;
static lv_obj_t *g_scr_alarm = NULL;
static lv_obj_t *g_scr_performance = NULL;
static lv_obj_t *g_scr_fueltrim = NULL;
// PAGE 13 (เดิมคือ PERFORMANCE ซึ่งย้ายไปเป็นหน้าย่อยของ PAGE 12 แล้ว):
// ตอนนี้เป็นหน้า K-LINE TX / RX LOG ล้วน ๆ (แสดงเฉพาะค่า log ไม่มีปุ่ม/ค่าอื่น)
// ชื่อตัวแปร g_scr_fuel_table คงไว้เพื่อไม่ให้กระทบโค้ดส่วนอื่นที่อ้างถึง
static lv_obj_t *g_scr_fuel_table = NULL;
static lv_obj_t *g_fuel_table_label = NULL;
static lv_obj_t *g_fuel_table_segments[8] = {NULL};
// PAGE 14 is now the second main instrument cluster, styled after the
// reference motorcycle dashboard: top tachometer arc, large GPS speed,
// fuel-style segment row, and three live status fields at the bottom.
// PAGE 14 RPM BAR -- same lightweight rectangular tick style as PAGE 15.
// The bar is horizontal and intentionally longer on PAGE 14; geometry is fixed
// at init, and runtime only changes tick colors.
constexpr int P14_RPM_TICKS = 20;
static lv_obj_t *g_cluster_rpm_ticks[P14_RPM_TICKS] = {NULL};
static lv_obj_t *g_cluster_rpm_track = NULL;
static float g_cluster_rpm_visual = 0.0f;
static uint32_t g_cluster_rpm_visual_ms = 0;
static int g_cluster_rpm_last_full = -1;
static int g_cluster_rpm_last_partial = -1;
static bool g_cluster_rpm_last_redline = false;
static bool g_cluster_rpm_last_blink = true;
static lv_obj_t *g_cluster_speed = NULL;
static lv_obj_t *g_cluster_speed_unit = NULL;
// PAGE 14 side readouts: live numeric RPM (left) and TPS (right).
static lv_obj_t *g_cluster_rpm_value = NULL;
static lv_obj_t *g_cluster_tps_value = NULL;
static lv_obj_t *g_cluster_trip = NULL;
static lv_obj_t *g_cluster_batt = NULL;
static lv_obj_t *g_cluster_ect = NULL;
static lv_obj_t *g_cluster_link = NULL;
static lv_obj_t *g_cluster_afr_label = NULL;
static lv_obj_t *g_cluster_afr_val = NULL;
static lv_obj_t *g_cluster_tacho_labels[9] = {NULL};
static lv_obj_t *g_cluster_status_line = NULL;
static lv_obj_t *g_cluster_bezel = NULL;
// Legacy PAGE 14 fuel-trim controls are retained only for source/API
// compatibility; the screen itself is repurposed as the cluster above.
static lv_obj_t *g_fueltrim_value = NULL;
static lv_obj_t *g_fueltrim_status = NULL;
// PAGE 13: label เดียวแสดง K-LINE TX/RX LOG ทั้งหมด (ใช้ buffer static + lv_label_set_text_static
// เพื่อไม่ให้เกิด heap alloc/free ซ้ำ ๆ ใน LVGL pool ทุกครั้งที่ log เปลี่ยน)
static lv_obj_t *g_kline_log_label = NULL;
#define KLINE_LOG_TEXT_LEN 1900   // 8 รายการ x (TX<=24B + RX<=40B) พร้อมหัวบรรทัด
static char g_kline_log_text[KLINE_LOG_TEXT_LEN] = "--";

// ── PAGE 16: DISTANCE TEST — ตั้งระยะทาง (เมตร) แล้ววัดเวลา/ความเร็วเฉลี่ย
// ที่ใช้วิ่งครบระยะนั้นจริง โดยอ้างอิงระยะทางสะสมจาก GPS (วิธีเดียวกับ TRIP)
static lv_obj_t *g_scr_disttest = NULL;
static lv_obj_t *g_lbl_dist_target = NULL;   // ค่าระยะทางเป้าหมายที่เลือกไว้ เช่น "400 m"
static lv_obj_t *g_lbl_dist_status = NULL;   // READY / RUNNING.. / DONE
static lv_obj_t *g_lbl_dist_time = NULL;     // เวลาที่ใช้ (วิ่งอยู่ = เรียลไทม์, จบแล้ว = ค่าสุดท้าย)
static lv_obj_t *g_lbl_dist_speed = NULL;    // ความเร็วเฉลี่ยตลอดระยะทางนั้น (คำนวณเมื่อจบ)
static lv_obj_t *g_btn_dist_startstop_lbl = NULL; // ป้ายข้อความบนปุ่ม AUTO/ABORT ตามสถานะ
enum DistTestState : uint8_t { DIST_READY = 0, DIST_RUNNING = 1, DIST_DONE = 2 };
static DistTestState g_dist_state = DIST_READY;
static const float DIST_TEST_PRESETS_M[] = {100.0f, 200.0f, 300.0f, 402.0f, 500.0f, 1000.0f, 1609.0f, 2000.0f};
static const uint8_t DIST_TEST_PRESET_COUNT = sizeof(DIST_TEST_PRESETS_M)/sizeof(DIST_TEST_PRESETS_M[0]);
static uint8_t g_dist_preset_idx = 3; // เริ่มต้นที่ 402 m (ระยะควอเตอร์ไมล์)
static bool g_dist_have_last = false;
static double g_dist_last_lat = 0.0, g_dist_last_lon = 0.0;
static float g_dist_covered_m = 0.0f;
static uint32_t g_dist_start_ms = 0;
static uint32_t g_dist_result_ms = 0;
static float g_dist_result_speed_kmh = 0.0f;
static float g_dist_run_max_speed_kmh = 0.0f;
static float g_dist_run_max_rpm = 0.0f;
static Preferences g_disttest_prefs;
static bool g_disttest_prefs_ready = false;

// ── หน้า 0: MENU — หน้าแรกสุดตอนบูท ──────────────────────────────
// หน้า 9, 11, 12, 13 ถูกผนวกรวมเป็นชุดย่อยของ PAGE 8 แล้ว
// เมนูหลักจึงเหลือ 12 รายการ และจัดเป็นกริด 3 x 4 ให้สมดุลกับจอ 320x240
// แตะแผ่นไหนในเมนู = ไปหน้านั้น, จากหน้า 8 ชุดย่อยจะไล่ต่อด้วยการแตะครั้งละ 1 หน้า
static lv_obj_t *g_scr_menu = NULL;
static lv_obj_t *g_menu_header = NULL;
static lv_obj_t *g_menu_header_title = NULL;
static lv_obj_t *g_menu_header_hint = NULL;
static lv_obj_t *g_menu_tiles[12] = {NULL};
static const uint8_t MENU_PAGE_TARGET[12] = {
    1, 2, 3, 4, 5, 6, 7, 8, 10, 14, 15, 16
};
static const char* MENU_TILE_LABEL[12] = {
    "SETTINGS", "GAUGE-1", "SPEED-1", "GRAPH-1", "GRAPH-2", "DTC",
    "LOG", "HEALTH", "SENSOR", "SPEED-2", "GAUGE-2", "DIST"
};
static int8_t g_fuel_trim_pct = 0;
static Preferences g_fueltrim_prefs;
static bool g_fueltrim_prefs_ready = false;
// Hard safety gate: keep ECU write disabled until the exact ECU write protocol
// for this model is positively identified and implemented.
static constexpr bool KLINE_WRITE_UNLOCKED = false;

static lv_obj_t *g_health_values[6] = {NULL};
static lv_obj_t *g_health_heap_min = NULL;
static lv_obj_t *g_health_heap_block = NULL;
static lv_obj_t *g_health_reset = NULL;
static lv_obj_t *g_wdt_values[5] = {NULL};
static lv_obj_t *g_sensor_values[12] = {NULL};
static lv_obj_t *g_logger_values[6] = {NULL};
static lv_obj_t *g_alarm_values[5] = {NULL};
static lv_obj_t *g_perf_values[6] = {NULL};

struct LoggerSample {
    uint32_t ms;
    float rpm, speed, tps, ect, batt, inj, ign, afr;
};
static LoggerSample g_logger_samples[LOGGER_MAX_SAMPLES];
static uint16_t g_logger_sample_count = 0;
static bool g_logger_recording = false;
static uint32_t g_logger_last_sample_ms = 0;
static uint32_t g_logger_last_motion_ms = 0;
static uint32_t g_logger_session_count = 0;
static char g_logger_last_file[32] = "--";
static bool g_logger_fs_ready = false;
static uint32_t g_logger_session_start_ms = 0;
static float g_session_max_rpm = 0.0f, g_session_min_rpm = 0.0f;
static float g_session_max_speed = 0.0f, g_session_min_speed = 0.0f;
static float g_session_max_ect = 0.0f, g_session_min_ect = 0.0f;
static float g_session_max_tps = 0.0f, g_session_min_tps = 0.0f;
static float g_session_min_batt = 99.0f, g_session_max_batt = 0.0f;
static float g_session_max_afr = 0.0f, g_session_min_afr = 99.0f;
static float g_session_start_trip_km = 0.0f;

static bool g_event_prev_ect = false;
static bool g_event_prev_batt = false;
static bool g_event_prev_rpm = false;
static bool g_event_prev_speed = false;
static bool g_event_prev_ecu = false;
static bool g_event_prev_gps = false;

static bool g_alarm_ect = false;
static bool g_alarm_batt = false;
static bool g_alarm_rpm = false;
static bool g_alarm_speed = false;
static uint32_t g_alarm_clear_candidate_ms = 0;

static float g_tps_peak = 0.0f;
static float g_ect_peak = 20.0f;


extern const lv_font_t dseg7_24;
extern const lv_font_t dseg7_32;
extern const lv_font_t dseg7_48;
extern const lv_font_t dseg7_60;
extern const lv_font_t A4SPEED_26;
extern const lv_font_t A4SPEED_36;
extern const lv_font_t A4SPEED_50;
extern const lv_font_t A4SPEED_14;
extern const lv_font_t A4SPEED_16;

// ── ค่าที่ Honda PGM-FI ส่งจริงผ่าน K-line ────────────────────
static float g_rpm=0, g_ect=20, g_tps=0;
static float g_inj=0, g_iat=25, g_batt=12.6, g_speed=0;
static float g_rpm_peak=0;   // ── ค่า RPM สูงสุดที่เคยขึ้นถึง (peak-hold) ──
static char  g_dtc[128]=""; static bool g_dtc_active=false;
static uint32_t g_dtc_scan_ms=0;

// ── ค่าที่ผ่านการกรอง (smoothed) ใช้แสดงผลจริง กัน noise/spike
//    จาก K-line ทำให้ตัวเลข/บาร์กระตุก ──────────────────────────
static float f_rpm=0, f_ect=20, f_tps=0, f_batt=12.6, f_inj=0, f_ign=0, f_afr=17, f_co2=0;
static bool g_t17_valid=false;
static bool g_afr_valid=false;

// ── O2 median-of-3 pre-filter: ตัด spike 1 ตัวอย่างจาก noise บน K-line
//    ก่อนเข้า EMA (คนละหน้าที่กับ EMA - median ตัด outlier แบบไม่หน่วง
//    เวลา ส่วน EMA ทำให้ค่าไหลลื่น) ──────────────────────────────────
static inline float median3(float a, float b, float c){
    if(a>b){ float t=a; a=b; b=t; }
    if(b>c){ float t=b; b=c; c=t; }
    if(a>b){ float t=a; a=b; b=t; }
    return b;
}

// ── EMA low-pass filter: prev + alpha*(raw-prev)
//    alpha สูง = ตอบสนองไว/สมูทน้อย, alpha ต่ำ = สมูทมาก/ตอบสนองช้า ──
static inline float ema(float prev, float raw, float alpha){
    return prev + alpha*(raw-prev);
}

// ── Widgets ───────────────────────────────────────────────────
// Forward declarations for DTC clear actions.
static void clear_dtc_btn_cb(lv_event_t *e);
static void page4_confirm_yes_cb(lv_event_t *e);
static void page4_confirm_no_cb(lv_event_t *e);
static void page4_verify_ok_cb(lv_event_t *e);
static void page4_open_confirm();
static void dtc_close_confirm_overlay();
static void finalize_clear_dtc_if_ready();
static void page2_open_trip_confirm();
static void page2_trip_confirm_yes_cb(lv_event_t *e);
static void page2_trip_confirm_no_cb(lv_event_t *e);
static void settings_apply_theme();
static void settings_set_day_mode(bool day);
static void settings_set_brightness(int delta);
static void settings_back_to_main();
// (หน้า 13 ไม่มีปุ่ม FUEL TRIM แล้ว — คง callback เดิมไว้เผื่อใช้ภายหลัง จึงติด unused กัน warning)
static void fueltrim_minus_cb(lv_event_t *e) __attribute__((unused));
static void fueltrim_plus_cb(lv_event_t *e) __attribute__((unused));
static void fueltrim_apply_cb(lv_event_t *e) __attribute__((unused));

#define NUM_TICKS 24
static lv_obj_t *g_ticks[NUM_TICKS];
static lv_obj_t *g_lbl_rpm, *g_lbl_rpm_peak, *g_lbl_tps;
static lv_obj_t *g_lbl_batt, *g_lbl_inj, *g_lbl_ect, *g_lbl_inc, *g_lbl_tps_stat;
// ── AFR gauge ล่างสุด (ค่าประมาณจากแรงดัน O2 - ไม่ใช่ AFR จริง) ──
// แสดงเฉพาะตัวเลข AFR/CO2 (ไม่มีแถบบาร์แล้ว)
static lv_obj_t *g_lbl_afr_label, *g_lbl_afr_val;   // label + ตัวเลข AFR มุมขวาแถว AFR
static lv_obj_t *g_dtc_bar, *g_dtc_label;
static lv_obj_t *g_link_dot;   // ── LINK LED: สถานะเชื่อมต่อ ECU แบบกะพริบ ──

// ── Menu ─────────────────────────────────────────────────────
static lv_obj_t *g_scr_main = NULL;
static lv_obj_t *g_scr_black = NULL;
static lv_obj_t *g_lbl_speed_page = NULL;
static lv_obj_t *g_lbl_speed_unit = NULL;
static lv_obj_t *g_lbl_speed_max = NULL;
static lv_obj_t *g_lbl_speed_iat = NULL;
static lv_obj_t *g_lbl_speed_status = NULL;
static lv_obj_t *g_lbl_speed_time = NULL;   // นาฬิกา GPS มุมขวาบนหน้า 2
static lv_obj_t *g_lbl_gps_speed = NULL;    // ความเร็วจาก GPS (กล่องที่ 3 หน้า 2)
static lv_obj_t *g_rpm_bar2[16];             // แถบ RPM หน้า 3 (อยู่บน)
static lv_obj_t *g_lbl_rpm_bar2_tag = NULL;  // ป้ายกำกับ "RPM" หน้าแถบ RPM
static lv_obj_t *g_spd_bar2[16];             // แถบความเร็วหน้า 2 (อยู่ล่าง, เพิ่มกลับเข้ามา)
static lv_obj_t *g_lbl_spd_bar2_tag = NULL;  // ป้ายกำกับ "SPD" หน้าแถบความเร็ว
static lv_obj_t *g_speed_link_dot = NULL;   // จุดไฟสถานะ ECU มุมขวาแถบสถานะหน้า 2
static float g_speed_max = 0;
static float f_speed = 0;
static KlineSnapshot g_kline_snapshot{};
static float g_trip_km = 0.0f;
static bool g_trip_have_last = false;
static double g_trip_last_lat = 0.0;
static double g_trip_last_lon = 0.0;
static lv_obj_t *g_lbl_trip = NULL;
static lv_obj_t *g_btn_reset_trip = NULL;
static bool g_reset_trip_hit = false;
static lv_obj_t *g_trip_confirm_overlay = NULL;
static bool g_trip_confirm_open = false;
static bool g_show_black = false;

// ── หน้า 5: DTC / ERROR CODE ───────────────────────────────────
static lv_obj_t *g_scr_dtc = NULL;
static lv_obj_t *g_lbl_dtc_title = NULL;
static lv_obj_t *g_lbl_dtc_code = NULL;
static lv_obj_t *g_lbl_dtc_desc = NULL;
static lv_obj_t *g_lbl_dtc_conn = NULL;
static lv_obj_t *g_lbl_dtc_result = NULL;
// เก็บ object จริงของปุ่มบนหน้า 6 ไว้ เพื่อให้ raw touch hit-test ใช้พิกัดจริง
// (แก้ไข: เดิม hardcode กรอบ y=148..205 ซึ่งไม่ตรงกับปุ่ม CLEAR DTC ที่อยู่ y=130..170
//  ทำให้แตะครึ่งบนของปุ่มแล้วหลุดไปเงื่อนไข fallback -> go_to_menu())
static lv_obj_t *g_btn_clear_dtc = NULL;
static lv_obj_t *g_btn_dtc_back  = NULL;
static lv_obj_t *g_dtc_page4_confirm_overlay = NULL;
static bool g_dtc_page4_confirm_open = false;
static lv_obj_t *g_dtc_page4_verify_overlay = NULL;
static lv_obj_t *g_dtc_page4_verify_label = NULL;
static bool g_dtc_page4_verify_open = false;

static void dist_run_history_refresh(){
    for(int i=0;i<3;i++){
        if(!g_dist_run_rows[i]) continue;
        if(i >= g_dist_run_count){
            lv_label_set_text(g_dist_run_rows[i], "--");
            continue;
        }
        const DistRunHistory &r = g_dist_run_history[i];
        char line[96];
        snprintf(line, sizeof(line), "R%d %.0fm %.2fs\nA%.1f M%.0f R%.0f",
                 i+1, r.target_m, r.time_ms / 1000.0f, r.avg_speed_kmh,
                 r.max_speed_kmh, r.max_rpm);
        lv_label_set_text(g_dist_run_rows[i], line);
    }
}

static void dist_run_history_push(float target_m, uint32_t time_ms, float avg_speed_kmh, float max_speed_kmh, float max_rpm){
    if(g_dist_run_count < 3){
        for(int i=(int)g_dist_run_count; i>0; --i){
            g_dist_run_history[i] = g_dist_run_history[i-1];
        }
        g_dist_run_count++;
    } else {
        for(int i=2; i>0; --i){
            g_dist_run_history[i] = g_dist_run_history[i-1];
        }
    }
    g_dist_run_history[0] = {target_m, time_ms, avg_speed_kmh, max_speed_kmh, max_rpm};
    dist_run_history_refresh();
}

static void log_refresh_rows(){
    if(!g_scr_log) return;
    for(int row=0; row<3; ++row){
        if(!g_log_rows[row]) continue;
        if(row >= g_log_count){
            lv_label_set_text(g_log_rows[row], "--");
            continue;
        }
        int idx = (int)g_log_head - 1 - row;
        while(idx < 0) idx += 5;
        lv_label_set_text(g_log_rows[row], g_log_text[idx]);
    }
}

static void log_push(const char *line){
    if(!line) return;
    strncpy(g_log_text[g_log_head], line, sizeof(g_log_text[g_log_head])-1);
    g_log_text[g_log_head][sizeof(g_log_text[g_log_head])-1] = '\0';
    g_log_head = (uint8_t)((g_log_head + 1) % 5);
    if(g_log_count < 5) g_log_count++;
    log_refresh_rows();
}

static void log_clear_event(const char *result, const char *before_summary){
    char t[12]; gps_time_str(t, sizeof(t));
    const char *res = result ? result : "UNKNOWN";

    char firstDtc[8] = "";
    if(before_summary){
        const char *p = strstr(before_summary, "DTCs:");
        if(p){
            p += 5;
            while(*p == ' ') ++p;
            if(strlen(p) >= 5 && p[0] >= '0' && p[0] <= '9' &&
               p[1] >= '0' && p[1] <= '9' && p[2] >= '0' && p[2] <= '9' && p[3] == '-' &&
               p[4] >= '0' && p[4] <= '9'){
                memcpy(firstDtc, p, 5);
                firstDtc[5] = '\0';
                strncpy(g_dtc, firstDtc, sizeof(g_dtc)-1);
                g_dtc[sizeof(g_dtc)-1] = '\0';
            }
        }
    }

    // หน้า 6 บันทึกเฉพาะประวัติการ CLEAR DTC เท่านั้น
    if(firstDtc[0]) snprintf(g_last_dtc_event, sizeof(g_last_dtc_event),
                             "%s  CLEAR DTC : %s  | %s", t, res, firstDtc);
    else snprintf(g_last_dtc_event, sizeof(g_last_dtc_event),
                   "%s  CLEAR DTC : %s", t, res);
    if(g_log_event_label) lv_label_set_text(g_log_event_label, g_last_dtc_event);

    char line[120];
    if(firstDtc[0]) snprintf(line, sizeof(line), "%s  CLEAR DTC : %s | %s", t, res, firstDtc);
    else snprintf(line, sizeof(line), "%s  CLEAR DTC : %s", t, res);
    log_push(line);
}

// ── หน้า 3: กราฟเปรียบเทียบ RPM / SPEED ───────────────────────
static lv_obj_t *g_scr_graph = NULL;
static lv_obj_t *g_graph_rpm = NULL;
static lv_obj_t *g_graph_speed = NULL;
static lv_chart_series_t *g_series_rpm = NULL;
static lv_chart_series_t *g_series_speed = NULL;
static lv_obj_t *g_lbl_graph_rpm = NULL;
static lv_obj_t *g_lbl_graph_speed = NULL;
static lv_obj_t *g_lbl_graph_status = NULL;
static lv_obj_t *g_lbl_graph_gps_status = NULL;

// ── หน้า 5: กราฟ AFR เทียบกับ RPM ───────────────────────────
static lv_obj_t *g_scr_afr_rpm = NULL;
static lv_obj_t *g_afr_rpm_line = NULL;
static lv_obj_t *g_afr_rpm_afr_line = NULL;
static lv_obj_t *g_lbl_afr_rpm_value = NULL;
static lv_obj_t *g_lbl_rpm_value = NULL;
static lv_obj_t *g_lbl_afr_value = NULL;
static lv_obj_t *g_lbl_afr_rpm_now = NULL;
// หน้า 5 ใช้การรวมข้อมูลตามช่วง RPM เพื่อให้เส้นกราฟแสดง AFR เทียบ RPM จริง
// และไม่เกิดเส้นซิกแซกจากการที่ RPM ขึ้น/ลงตามลำดับเวลา
static lv_point_t g_afr_rpm_points[48];
static lv_point_t g_afr_rpm_afr_points[48];
static uint8_t g_afr_rpm_count = 0;
static uint32_t g_afr_rpm_sample_ms = 0;

// ── หน้า 3: จับเวลา "รถเคลื่อนที่จนหยุด" + รอบเครื่องสูงสุด (peak) ระหว่างวิ่ง ──
enum MoveTimerState { MOVE_STOPPED, MOVE_RUNNING };
static MoveTimerState g_move_state      = MOVE_STOPPED;
static uint32_t       g_move_start_ms   = 0;
static uint32_t       g_move_result_ms  = 0;
static float          g_move_peak_rpm   = 0.0f;   // รอบเครื่องสูงสุดระหว่างรอบที่กำลังวิ่ง
static float          g_move_result_rpm = 0.0f;   // รอบสูงสุดของรอบล่าสุดที่จบไปแล้ว
static bool           g_move_has_result = false;
static const float    MOVE_START_KMH    = 1.0f;   // ต่ำกว่านี้ถือว่า "หยุดนิ่ง"

static uint8_t g_page = 0;
// หน้า 12 (ALARM) ตอนนี้แยกเป็น 2 หน้าย่อยแตะสลับได้: 0=ALARM, 1=PERFORMANCE
// (PERFORMANCE ย้ายมาจากหน้า 13 เดิม) แตะครั้งที่ 3 กลับไปเมนู
static uint8_t g_page12_view = 0;

void gauge_ui_update();

// ── Color Scheme (Night Dash) ───────────────────────────────
// อ้างอิงจากดีไซน์ dashboard สไตล์ dark theme ตัวเลขใหญ่ตัดกับพื้นดำสนิท
#define BG_BLACK   lv_color_hex(0x000000)      // พื้นหลังดำสนิท
#define PANEL_DARK lv_color_hex(0x0a0a0d)      // แผงรองพื้น (แทบไม่ต่างจากดำ)
#define PANEL_CARD lv_color_hex(0x121319)      // การ์ด/กล่องลอย (เข้มกว่าพื้นหลังเล็กน้อย ให้แยกชั้นชัด)
#define BTN_NEUTRAL lv_color_hex(0x2a2d35)     // ปุ่มเมนูสีกลาง (ยกเลิก/ปิด/ตกลง)
#define WHITE      lv_color_hex(0xffffff)      // ตัวเลขหลัก/label หลัก
#define GRAY_LBL   lv_color_hex(0x8a8f9a)      // label รอง (หรี่กว่าตัวเลข)
#define GRAY_LINE  BTN_NEUTRAL      // เส้นแบ่ง/แทร็กพื้นหลัง
#define BORDER_SUBTLE lv_color_hex(0x22252c)   // ขอบการ์ดแบบบางเบา แยกชั้นแต่ไม่แย่งซีน
#define GRAPH_GRID    lv_color_hex(0x3a3e48)   // เส้นกริดกราฟให้เห็นชัดบนพื้นดำ
#define ACCENT_OK  lv_color_hex(0x8ce62b)      // เขียวมะนาว (โซนปกติ) เหมือนภาพต้นแบบ
#define ACCENT_WARN lv_color_hex(0xffc400)    // เหลืองอำพัน (ระวัง)
#define ACCENT_ERR lv_color_hex(0xff3b30)     // แดง (แดงไลน์/แจ้งเตือน)
#define ACCENT_BATT lv_color_hex(0xff5a3c)    // ส้ม-แดง (ไอคอนแบตเตอรี่)
#define ACCENT_TEMP lv_color_hex(0x3ba7ff)    // ฟ้า (ไอคอนอุณหภูมิ)
#define ACCENT_BLUE lv_color_hex(0x5aa9ff)    // ฟ้าอ่อน (เส้นตกแต่ง)
#define ACCENT_INJ  lv_color_hex(0x8a7cff)    // ม่วงฟ้า (ไอคอนหัวฉีด)
#define ACCENT_SPARK lv_color_hex(0xffd23f)   // เหลืองประกาย (ไอคอนไฟจุดระเบิด)
#define ACCENT_RICH lv_color_hex(0xff8a3c)    // ส้ม (AFR โซนรวย/Rich)
#define ACCENT_LEAN lv_color_hex(0x3cd6ff)    // ฟ้าสด (AFR โซนบาง/Lean)
#define ACCENT_TPS  lv_color_hex(0x6be36b)    // เขียวสด (ไอคอน TPS)
#define ACCENT_MAGENTA lv_color_hex(0xff2f7e)  // ชมพูบานเย็น (แถบหัวข้อหน้ากราฟ)
#define ACCENT_PURPLE  lv_color_hex(0x7a3bd8)  // ม่วง (ไล่เฉดคู่กับ MAGENTA)

// ── Day/Night theme palette ────────────────────────────────────
#define DAY_BG        lv_color_hex(0xf2f3f5)
#define DAY_PANEL     lv_color_hex(0xffffff)
#define DAY_CARD      lv_color_hex(0xffffff)
#define DAY_BTN       lv_color_hex(0xd9dde3)
#define DAY_TEXT      lv_color_hex(0x111318)
#define DAY_SUBTEXT   lv_color_hex(0x4e5560)
#define DAY_LINE      lv_color_hex(0xc2c7cf)
#define DAY_BORDER    lv_color_hex(0xb8bec8)
#define DAY_GRID      lv_color_hex(0xd2d6dd)

static bool color_eq(lv_color_t a, lv_color_t b){
#if LV_COLOR_DEPTH == 32
    return a.full == b.full;
#else
    return lv_color_to32(a) == lv_color_to32(b);
#endif
}

static lv_color_t theme_bg(){ return g_day_mode ? DAY_BG : BG_BLACK; }
static lv_color_t theme_panel(){ return g_day_mode ? DAY_PANEL : PANEL_DARK; }
static lv_color_t theme_card(){ return g_day_mode ? DAY_CARD : PANEL_CARD; }
static lv_color_t theme_btn(){ return g_day_mode ? DAY_BTN : BTN_NEUTRAL; }
static lv_color_t theme_primary_text(){ return g_day_mode ? DAY_TEXT : WHITE; }
// PAGE 14 live numeric readouts stay high-contrast: WHITE in NIGHT mode,
// and the dark DAY_TEXT equivalent in DAY mode so the white DAY background
// does not make RPM/TPS disappear. This color is re-applied on every theme
// switch and on every live-value refresh.
static lv_color_t page14_value_text(){ return g_day_mode ? DAY_TEXT : WHITE; }
static lv_color_t theme_subtext(){ return g_day_mode ? DAY_SUBTEXT : GRAY_LBL; }
static lv_color_t theme_line(){ return g_day_mode ? DAY_LINE : GRAY_LINE; }
static lv_color_t theme_border(){ return g_day_mode ? DAY_BORDER : BORDER_SUBTLE; }
static lv_color_t theme_grid(){ return g_day_mode ? DAY_GRID : GRAPH_GRID; }

static inline void set_text_color_if_changed(lv_obj_t *obj, lv_color_t color){
    if(!obj) return;
    if(!color_eq(lv_obj_get_style_text_color(obj, 0), color)){
        lv_obj_set_style_text_color(obj, color, 0);
    }
}

static inline void set_bg_color_if_changed(lv_obj_t *obj, lv_color_t color){
    if(!obj) return;
    if(!color_eq(lv_obj_get_style_bg_color(obj, 0), color)){
        lv_obj_set_style_bg_color(obj, color, 0);
    }
}

static inline void set_bg_opa_if_changed(lv_obj_t *obj, lv_opa_t opa){
    if(!obj) return;
    if(lv_obj_get_style_bg_opa(obj, 0) != opa){
        lv_obj_set_style_bg_opa(obj, opa, 0);
    }
}

static void theme_apply_obj(lv_obj_t *obj){
    if(!obj) return;

    lv_color_t bg = lv_obj_get_style_bg_color(obj, 0);
    if(g_day_mode){
        if(color_eq(bg, BG_BLACK))       lv_obj_set_style_bg_color(obj, DAY_BG, 0);
        else if(color_eq(bg, PANEL_DARK))lv_obj_set_style_bg_color(obj, DAY_PANEL, 0);
        else if(color_eq(bg, PANEL_CARD))lv_obj_set_style_bg_color(obj, DAY_CARD, 0);
        else if(color_eq(bg, BTN_NEUTRAL))lv_obj_set_style_bg_color(obj, DAY_BTN, 0);
    }else{
        if(color_eq(bg, DAY_BG))         lv_obj_set_style_bg_color(obj, BG_BLACK, 0);
        else if(color_eq(bg, DAY_PANEL)) lv_obj_set_style_bg_color(obj, PANEL_DARK, 0);
        else if(color_eq(bg, DAY_CARD))  lv_obj_set_style_bg_color(obj, PANEL_CARD, 0);
        else if(color_eq(bg, DAY_BTN))   lv_obj_set_style_bg_color(obj, BTN_NEUTRAL, 0);
    }

    lv_color_t bc = lv_obj_get_style_border_color(obj, 0);
    if(g_day_mode){
        if(color_eq(bc, BORDER_SUBTLE)) lv_obj_set_style_border_color(obj, DAY_BORDER, 0);
        else if(color_eq(bc, GRAY_LINE)) lv_obj_set_style_border_color(obj, DAY_LINE, 0);
    }else{
        if(color_eq(bc, DAY_BORDER)) lv_obj_set_style_border_color(obj, BORDER_SUBTLE, 0);
        else if(color_eq(bc, DAY_LINE)) lv_obj_set_style_border_color(obj, GRAY_LINE, 0);
    }

    lv_color_t tc = lv_obj_get_style_text_color(obj, 0);
    if(g_day_mode){
        if(color_eq(tc, WHITE)) lv_obj_set_style_text_color(obj, DAY_TEXT, 0);
        else if(color_eq(tc, GRAY_LBL)) lv_obj_set_style_text_color(obj, DAY_SUBTEXT, 0);
    }else{
        if(color_eq(tc, DAY_TEXT)) lv_obj_set_style_text_color(obj, WHITE, 0);
        else if(color_eq(tc, DAY_SUBTEXT)) lv_obj_set_style_text_color(obj, GRAY_LBL, 0);
    }

    if(obj == g_graph_rpm || obj == g_graph_speed){
        lv_obj_set_style_bg_color(obj, theme_card(), 0);
        lv_obj_set_style_border_color(obj, theme_border(), 0);
        lv_obj_set_style_line_color(obj, theme_grid(), LV_PART_MAIN);
    }

    uint32_t i = 0; lv_obj_t *child;
    while((child = lv_obj_get_child(obj, i++)) != NULL) theme_apply_obj(child);
}

static void settings_apply_theme(){
    lv_obj_t *roots[] = {g_scr_main, g_scr_black, g_scr_graph, g_scr_afr_rpm, g_scr_dtc, g_scr_log, g_scr_settings,
                         g_scr_health, g_scr_watchdog, g_scr_sensors, g_scr_data_logger, g_scr_alarm, g_scr_performance, g_scr_fueltrim,
                         g_scr_fuel_table, g_scr_disttest};
    for(auto r : roots) if(r) theme_apply_obj(r);

    // PAGE 14 RPM bar uses a dark track/tick surface in both DAY and NIGHT.
    // The PAGE 14 screen itself still follows DAY/NIGHT; only this bar remains
    // dark so active colors stay clear and unlit ticks never become washed out.
    if(g_cluster_rpm_track){
        lv_obj_set_style_bg_color(g_cluster_rpm_track, BG_BLACK, 0);
        lv_obj_set_style_bg_opa(g_cluster_rpm_track, LV_OPA_COVER, 0);
    }
    for(int i=0;i<P14_RPM_TICKS;i++) if(g_cluster_rpm_ticks[i]){
        // Do not overwrite active colors here; cluster_rpm_update() owns those.
        if(g_cluster_rpm_last_full <= i){
            lv_obj_set_style_bg_color(g_cluster_rpm_ticks[i], BG_BLACK, 0);
        }
        lv_obj_set_style_bg_opa(g_cluster_rpm_ticks[i], LV_OPA_COVER, 0);
    }

    if(g_cluster_speed) lv_obj_set_style_text_color(g_cluster_speed, theme_primary_text(), 0);
    if(g_cluster_speed_unit) lv_obj_set_style_text_color(g_cluster_speed_unit, theme_primary_text(), 0);
    // PAGE 14 numeric RPM/TPS values follow the DAY/NIGHT display mode.
    // NIGHT = pure white; DAY = dark primary text for legibility on light surfaces.
    if(g_cluster_rpm_value) lv_obj_set_style_text_color(g_cluster_rpm_value, page14_value_text(), 0);
    if(g_cluster_tps_value) lv_obj_set_style_text_color(g_cluster_tps_value, page14_value_text(), 0);
    if(g_cluster_trip) lv_obj_set_style_text_color(g_cluster_trip, ACCENT_OK, 0);
    if(g_cluster_batt) lv_obj_set_style_text_color(g_cluster_batt, ACCENT_BLUE, 0);
    if(g_cluster_ect) lv_obj_set_style_text_color(g_cluster_ect, ACCENT_TEMP, 0);
    if(g_cluster_afr_label) lv_obj_set_style_text_color(g_cluster_afr_label, theme_subtext(), 0);
    if(g_cluster_status_line) lv_obj_set_style_text_color(g_cluster_status_line, kline_is_connected() ? ACCENT_OK : ACCENT_WARN, 0);
    // AFR value color is re-derived every tick from the live reading (see
    // gauge_ui_update), so no theme-generic override is applied to it here.
    if(g_fuel_table_label) lv_obj_set_style_text_color(g_fuel_table_label, theme_subtext(), 0);
    for(int i=0;i<8;i++) if(g_fuel_table_segments[i]){
        lv_obj_set_style_bg_color(g_fuel_table_segments[i], GRAY_LINE, 0);
        lv_obj_set_style_border_color(g_fuel_table_segments[i], theme_border(), 0);
    }
    for(int i=0;i<=8;i++) if(g_cluster_tacho_labels[i]){
        lv_obj_set_style_text_color(g_cluster_tacho_labels[i], (i>=7) ? ACCENT_ERR : theme_subtext(), 0);
    }

    // PAGE 14 follows the selected display mode for the main background:
    // NIGHT = deep black, DAY = clean white.  The RPM track itself remains
    // black so the inactive area never becomes a washed-out color.
    const lv_color_t p14_bg = g_day_mode ? DAY_BG : BG_BLACK;
    if(g_scr_fueltrim) lv_obj_set_style_bg_color(g_scr_fueltrim, p14_bg, 0);
    if(g_cluster_bezel){
        lv_obj_set_style_bg_color(g_cluster_bezel, p14_bg, 0);
        lv_obj_set_style_bg_opa(g_cluster_bezel, LV_OPA_COVER, 0);
    }

    // Re-apply accent colors on settings controls after the generic pass.
    if(g_btn_night) lv_obj_set_style_bg_color(g_btn_night, g_day_mode ? DAY_BTN : ACCENT_BLUE, 0);
    if(g_btn_day)   lv_obj_set_style_bg_color(g_btn_day,   g_day_mode ? ACCENT_OK : DAY_BTN, 0);
    if(g_lbl_settings_mode){
        lv_label_set_text(g_lbl_settings_mode, g_day_mode ? "DAY MODE" : "NIGHT MODE");
        lv_obj_set_style_text_color(g_lbl_settings_mode, g_day_mode ? ACCENT_OK : ACCENT_BLUE, 0);
    }
    if(g_lbl_brightness_value){
        char b[24]; snprintf(b, sizeof(b), "%u%%", (unsigned)((uint16_t)g_brightness * 100u / 255u));
        lv_label_set_text(g_lbl_brightness_value, b);
        lv_obj_set_style_text_color(g_lbl_brightness_value, theme_primary_text(), 0);
    }

    // SELECT PAGE is intentionally themed here instead of passing through
    // theme_apply_obj(), because its header is a dedicated gradient control.
    // NTP GAUGE (PAGE 15) uses its own compact palette but follows the same
    // global DAY/NIGHT selection as every other page. The setter is a no-op
    // until PAGE 15 has been created.
    sqxzgauge_page_set_day_mode(g_day_mode);

    if(g_scr_menu){
        lv_obj_set_style_bg_color(g_scr_menu, theme_bg(), 0);
        if(g_menu_header){
            lv_obj_set_style_bg_color(g_menu_header, ACCENT_BLUE, 0);
            lv_obj_set_style_bg_grad_color(g_menu_header, ACCENT_PURPLE, 0);
            lv_obj_set_style_text_color(g_menu_header_title, WHITE, 0);
            lv_obj_set_style_text_color(g_menu_header_hint, WHITE, 0);
        }
        for(int i = 0; i < 12; ++i){
            lv_obj_t *tile = g_menu_tiles[i];
            if(!tile) continue;
            const bool locked = false;
            lv_obj_set_style_bg_color(tile, locked ? ACCENT_ERR : theme_card(), 0);
            // SELECT PAGE ใช้กรอบสีเขียวเป็นเอกลักษณ์ทั้ง 12 ช่อง
            lv_obj_set_style_border_color(tile, ACCENT_OK, 0);
            lv_obj_set_style_border_width(tile, locked ? 2 : 1, 0);
            lv_obj_t *num = lv_obj_get_child(tile, 0);
            lv_obj_t *word = lv_obj_get_child(tile, 1);
            // DAY mode ใช้พื้นขาว จึงต้องเปลี่ยนข้อความจาก WHITE เป็นสีเข้ม
            // เพื่อป้องกันข้อความหาย/มองไม่เห็น
            if(num) lv_obj_set_style_text_color(num, locked ? WHITE : theme_subtext(), 0);
            if(word) lv_obj_set_style_text_color(word, locked ? WHITE : theme_primary_text(), 0);
        }
    }
}

// ── Helper: label ทั่วไป ─────────────────────────────────────
static lv_obj_t* mk_label(lv_obj_t *parent,const lv_font_t*f,lv_color_t col,
                           const char*txt,lv_align_t al,int ox,int oy){
    lv_obj_t *l=lv_label_create(parent);
    lv_obj_set_style_text_font(l,f,0);
    lv_obj_set_style_text_color(l,col,0);
    lv_label_set_text(l,txt);
    lv_obj_align(l,al,ox,oy);
    return l;
}

// ── Helper: คอลัมน์สถิติแบบสมมาตร (การ์ดกรอบสีเทา) ─────
// ใช้ container การ์ดขนาดเท่ากันทุกคอลัมน์ มีเฉพาะกรอบสีเทา ไม่มีแถบสี
// เพื่อให้ BATT / INJ / ECT / INC / TPS เรียงเป็นแถวเดียวสมมาตรกันจริงๆ
// ── Helper: ปุ่มเมนู/ปุ่มยืนยันแบบมาตรฐาน ─────────────────────
static lv_obj_t* mk_menu_btn(lv_obj_t *parent, int x, int y, int w, int h,
                              const char *txt, lv_color_t bg, lv_event_cb_t cb){
    lv_obj_t *b=lv_obj_create(parent);
    lv_obj_set_pos(b,x,y);
    lv_obj_set_size(b,w,h);
    lv_obj_clear_flag(b,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(b,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(b,bg,0);
    lv_obj_set_style_bg_opa(b,LV_OPA_COVER,0);
    lv_obj_set_style_border_width(b,0,0);
    lv_obj_set_style_radius(b,6,0);
    if(cb) lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,NULL);
    lv_obj_t *l=lv_label_create(b);
    lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(l,WHITE,0);
    lv_label_set_text(l,txt);
    lv_obj_center(l);
    return b;
}

static lv_obj_t* mk_stat_col(lv_obj_t *parent, int x, int y, int w, int h,
                              const char *caption, lv_color_t /*accent*/,
                              lv_obj_t **out_value){
    const int GAP=3;   // ช่องไฟภายในกริด ลดเล็กน้อยเพื่อเพิ่มพื้นที่ข้อความ
    lv_obj_t *col=lv_obj_create(parent);
    lv_obj_set_pos(col,x,y);
    lv_obj_set_size(col,w,h);
    lv_obj_clear_flag(col,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(col,PANEL_CARD,0);
    lv_obj_set_style_bg_opa(col,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(col,BORDER_SUBTLE,0);
    lv_obj_set_style_border_width(col,1,0);
    lv_obj_set_style_radius(col,8,0);
    lv_obj_set_style_pad_all(col,0,0);
    lv_obj_set_style_shadow_width(col,8,0);
    lv_obj_set_style_shadow_color(col,lv_color_hex(0x000000),0);
    lv_obj_set_style_shadow_opa(col,55,0);

    // ไม่มีแถบสีด้านบน: ใช้เฉพาะกรอบสีเทาเพื่อให้ UI สะอาดและไม่แย่งสายตา

    lv_obj_t *val=lv_label_create(col);
    lv_obj_set_style_text_font(val,&A4SPEED_16,0);
    lv_obj_set_style_text_color(val,WHITE,0);
    lv_obj_set_style_text_align(val,LV_TEXT_ALIGN_CENTER,0);
    lv_label_set_long_mode(val,LV_LABEL_LONG_CLIP);
    lv_label_set_text(val,"0");
    lv_obj_set_width(val,w-4);
    lv_obj_set_height(val,20);
    lv_obj_align(val,LV_ALIGN_TOP_MID,0,8);
    if(out_value) *out_value=val;

    lv_obj_t *cap=lv_label_create(col);
    lv_obj_set_style_text_font(cap,&A4SPEED_14,0);
    lv_obj_set_style_text_color(cap,GRAY_LBL,0);
    lv_obj_set_style_text_align(cap,LV_TEXT_ALIGN_CENTER,0);
    lv_label_set_long_mode(cap,LV_LABEL_LONG_CLIP);
    lv_label_set_text(cap,caption);
    lv_obj_set_width(cap,w-4);
    lv_obj_set_height(cap,16);
    lv_obj_align(cap,LV_ALIGN_TOP_MID,0,35);
    return col;
}



// ── Extended page helpers ──────────────────────────────────────────────────
static lv_obj_t* ext_screen(const char *title, int page_no){
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    char t[40]; snprintf(t, sizeof(t), "PAGE %d  %s", page_no, title);
    mk_label(scr, &A4SPEED_16, ACCENT_BLUE, t, LV_ALIGN_TOP_MID, 0, 5);
    return scr;
}

static lv_obj_t* ext_metric(lv_obj_t *parent, int x, int y, int w, int h, const char *name, lv_obj_t **out){
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_pos(box, x, y); lv_obj_set_size(box, w, h);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(box, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(box, 1, 0);
    lv_obj_set_style_radius(box, 7, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_t *val = lv_label_create(box);
    lv_obj_set_style_text_font(val, &A4SPEED_16, 0);
    lv_obj_set_style_text_color(val, WHITE, 0);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(val, "--");
    lv_obj_set_width(val, w-8);
    // จองพื้นที่สูงสุดให้พอดีเหนือ caption เสมอ (caption สูง 14 + ขอบล่าง 4 = 18)
    // เผื่อกรณีข้อความยาว (เช่นสถานะ GPS) ที่ต้องขึ้น 2 บรรทัด จะไม่ล้นไปทับ caption
    const int val_h = h - 22;
    lv_obj_set_height(val, val_h > 18 ? val_h : 18);
    lv_label_set_long_mode(val, LV_LABEL_LONG_WRAP);
    lv_obj_align(val, LV_ALIGN_TOP_MID, 0, 4);
    if(out) *out = val;

    lv_obj_t *cap = lv_label_create(box);
    // หน้า 10 ใช้กล่องแคบ จึงลด caption เป็น 12 px เพื่อให้ชื่อไม่ล้นขอบ
    lv_obj_set_style_text_font(cap, (w <= 90) ? &lv_font_montserrat_12 : &A4SPEED_14, 0);
    lv_obj_set_style_text_color(cap, GRAY_LBL, 0);
    lv_obj_set_style_text_align(cap, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(cap, LV_LABEL_LONG_CLIP);
    lv_label_set_text(cap, name);
    lv_obj_set_width(cap, w-8);
    lv_obj_set_height(cap, 14);
    lv_obj_align(cap, LV_ALIGN_BOTTOM_MID, 0, -4);
    return box;
}

static const char* reset_reason_name(){
    switch(esp_reset_reason()){
        case ESP_RST_POWERON: return "POWER ON";
        case ESP_RST_EXT: return "EXTERNAL";
        case ESP_RST_SW: return "SOFTWARE";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT WDT";
        case ESP_RST_TASK_WDT: return "TASK WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        default: return "OTHER";
    }
}

static void logger_append_event(const char *type, const char *detail){
    if(!g_logger_fs_ready || !type || !detail) return;
    File f = LittleFS.open("/events.csv", FILE_APPEND);
    if(!f) return;
    if(f.size() == 0) f.println("ms,session,event,detail");
    char safe[96];
    snprintf(safe, sizeof(safe), "%s", detail);
    for(char *p=safe; *p; ++p) if(*p==',' || *p=='\n' || *p=='\r') *p=';';
    f.printf("%lu,%lu,%s,%s\n", (unsigned long)millis(),
             (unsigned long)g_logger_session_count, type, safe);
    f.close();
    strncpy(g_last_dtc_event, detail, sizeof(g_last_dtc_event)-1);
    g_last_dtc_event[sizeof(g_last_dtc_event)-1] = '\0';
}

static void logger_load_session_counter(){
    g_logger_session_count = 0;
    if(!g_logger_fs_ready) return;
    File f = LittleFS.open("/session.dat", FILE_READ);
    if(f){
        uint32_t v=0;
        if(f.available()) v=(uint32_t)f.parseInt();
        f.close();
        g_logger_session_count=v;
    }
}

static void logger_save_session_counter(){
    if(!g_logger_fs_ready) return;
    File f = LittleFS.open("/session.dat", FILE_WRITE);
    if(!f) return;
    f.printf("%lu\n", (unsigned long)g_logger_session_count);
    f.close();
}

static void logger_reset_session_stats(){
    g_session_max_rpm=0; g_session_min_rpm=0;
    g_session_max_speed=0; g_session_min_speed=0;
    g_session_max_ect=0; g_session_min_ect=0;
    g_session_max_tps=0; g_session_min_tps=0;
    g_session_min_batt=99; g_session_max_batt=0;
    g_session_max_afr=0; g_session_min_afr=99;
    g_session_start_trip_km=g_trip_km;
}

static void logger_update_session_stats(){
    const bool rpm_ok = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_RPM);
    const bool speed_ok = gps_speed_valid();
    const bool ect_ok = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_ECT);
    const bool tps_ok = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_TPS);
    const bool batt_ok = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_BATT);
    const bool afr_ok = g_afr_valid;
    if(rpm_ok){ if(g_session_min_rpm==0 || f_rpm<g_session_min_rpm) g_session_min_rpm=f_rpm; if(f_rpm>g_session_max_rpm) g_session_max_rpm=f_rpm; }
    if(speed_ok){ if(g_session_min_speed==0 || f_speed<g_session_min_speed) g_session_min_speed=f_speed; if(f_speed>g_session_max_speed) g_session_max_speed=f_speed; }
    if(ect_ok){ if(g_session_min_ect==0 || f_ect<g_session_min_ect) g_session_min_ect=f_ect; if(f_ect>g_session_max_ect) g_session_max_ect=f_ect; }
    if(tps_ok){ if(g_session_min_tps==0 || f_tps<g_session_min_tps) g_session_min_tps=f_tps; if(f_tps>g_session_max_tps) g_session_max_tps=f_tps; }
    if(batt_ok){ if(f_batt<g_session_min_batt) g_session_min_batt=f_batt; if(f_batt>g_session_max_batt) g_session_max_batt=f_batt; }
    if(afr_ok){ if(g_session_min_afr==99 || f_afr<g_session_min_afr) g_session_min_afr=f_afr; if(f_afr>g_session_max_afr) g_session_max_afr=f_afr; }
}

static void logger_write_session(){
    if(!g_logger_fs_ready || g_logger_sample_count == 0) return;
    char path[40];
    snprintf(path, sizeof(path), "/run_%03lu.csv", (unsigned long)(g_logger_session_count % 1000));
    File f = LittleFS.open(path, FILE_WRITE);
    if(!f) return;
    f.println("ms,session,rpm,speed_kmh,tps_pct,ect_c,battery_v,inj_ms,ign_deg,afr_est,trip_km,gps_quality");
    for(uint16_t i=0;i<g_logger_sample_count;i++){
        const LoggerSample &s = g_logger_samples[i];
        const char *gq = "NODATA";
        switch(gps_quality()){
            case GPS_QUALITY_EXCELLENT: gq="EXCELLENT"; break;
            case GPS_QUALITY_GOOD: gq="GOOD"; break;
            case GPS_QUALITY_FAIR: gq="FAIR"; break;
            case GPS_QUALITY_WEAK: gq="WEAK"; break;
            case GPS_QUALITY_NO_FIX: gq="NOFIX"; break;
            default: break;
        }
        f.printf("%lu,%lu,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.2f,%.3f,%s\n",
                 (unsigned long)s.ms, (unsigned long)g_logger_session_count,
                 s.rpm, s.speed, s.tps, s.ect, s.batt, s.inj, s.ign, s.afr,
                 g_trip_km, gq);
        if((i % 32)==0) delay(0);
    }
    f.close();
    strncpy(g_logger_last_file, path+1, sizeof(g_logger_last_file)-1);
    g_logger_last_file[sizeof(g_logger_last_file)-1] = '\0';

    File sf = LittleFS.open("/run_summary.csv", FILE_APPEND);
    if(sf){
        if(sf.size()==0) sf.println("session,duration_s,trip_km,max_rpm,min_rpm,max_speed,min_speed,max_ect,min_ect,max_tps,min_tps,min_batt,max_batt,min_afr,max_afr");
        const uint32_t dur = g_logger_session_start_ms ? (millis()-g_logger_session_start_ms)/1000UL : 0;
        sf.printf("%lu,%lu,%.3f,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f\n",
                  (unsigned long)g_logger_session_count, (unsigned long)dur,
                  fmaxf(0.0f, g_trip_km-g_session_start_trip_km),
                  g_session_max_rpm, g_session_min_rpm,
                  g_session_max_speed, g_session_min_speed,
                  g_session_max_ect, g_session_min_ect,
                  g_session_max_tps, g_session_min_tps,
                  g_session_min_batt < 98.0f ? g_session_min_batt : 0.0f,
                  g_session_max_batt,
                  g_session_min_afr < 98.0f ? g_session_min_afr : 0.0f,
                  g_session_max_afr);
        sf.close();
    }
}

static lv_obj_t* health_metric(lv_obj_t *parent, int x, int y, int w, int h,
                                  const char *name, lv_obj_t **out){
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_pos(box, x, y);
    lv_obj_set_size(box, w, h);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(box, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(box, 1, 0);
    lv_obj_set_style_radius(box, 7, 0);
    lv_obj_set_style_pad_all(box, 0, 0);

    lv_obj_t *val = lv_label_create(box);
    lv_obj_set_style_text_font(val, &A4SPEED_16, 0);
    lv_obj_set_style_text_color(val, WHITE, 0);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(val, LV_LABEL_LONG_CLIP);
    lv_label_set_text(val, "--");
    lv_obj_set_width(val, w - 4);
    lv_obj_set_height(val, 22);
    lv_obj_align(val, LV_ALIGN_TOP_MID, 0, 4);
    if(out) *out = val;

    lv_obj_t *cap = lv_label_create(box);
    lv_obj_set_style_text_font(cap, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(cap, GRAY_LBL, 0);
    lv_obj_set_style_text_align(cap, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(cap, LV_LABEL_LONG_CLIP);
    lv_label_set_text(cap, name);
    lv_obj_set_width(cap, w - 4);
    lv_obj_set_height(cap, 14);
    lv_obj_align(cap, LV_ALIGN_BOTTOM_MID, 0, -3);
    return box;
}

static void extended_pages_init(){
    // PAGE 8: SYSTEM HEALTH CENTER
    // One compact dashboard: 4 live status cards, 3 heap figures, uptime and
    // the reset reason. Values stay short so they remain readable on 320x240.
    g_scr_health = ext_screen("SYSTEM HEALTH", 8);

    const int STATUS_Y = 38;
    const int STATUS_H = 44;

    // PAGE 8: wider ECU/GPS cards to prevent status text overflow.
    // K-LINE stays compact on the right; TOUCH is intentionally removed.
    health_metric(g_scr_health, 6,   STATUS_Y, 112, STATUS_H, "ECU",    &g_health_values[0]);
    health_metric(g_scr_health, 126, STATUS_Y, 112, STATUS_H, "GPS",    &g_health_values[1]);
    health_metric(g_scr_health, 246, STATUS_Y, 68,  STATUS_H, "K-LINE", &g_health_values[5]);

    // Heap Monitor: three equal columns, intentionally compact to leave room
    // for uptime/reset without making the page feel crowded.
    lv_obj_t *heap_card = lv_obj_create(g_scr_health);
    lv_obj_set_pos(heap_card, 7, 88);
    lv_obj_set_size(heap_card, 306, 64);
    lv_obj_clear_flag(heap_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(heap_card, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(heap_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(heap_card, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(heap_card, 1, 0);
    lv_obj_set_style_radius(heap_card, 8, 0);
    lv_obj_set_style_pad_all(heap_card, 0, 0);

    mk_label(heap_card, &A4SPEED_14, ACCENT_OK, "HEAP MONITOR", LV_ALIGN_TOP_MID, 0, 2);

    g_health_values[3] = lv_label_create(heap_card);
    lv_obj_set_style_text_font(g_health_values[3], &A4SPEED_16, 0);
    lv_obj_set_style_text_color(g_health_values[3], WHITE, 0);
    lv_obj_set_style_text_align(g_health_values[3], LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(g_health_values[3], "-- K");
    lv_obj_set_width(g_health_values[3], 96);
    lv_obj_set_height(g_health_values[3], 22);
    lv_obj_set_pos(g_health_values[3], 4, 22);

    g_health_heap_min = lv_label_create(heap_card);
    lv_obj_set_style_text_font(g_health_heap_min, &A4SPEED_16, 0);
    lv_obj_set_style_text_color(g_health_heap_min, WHITE, 0);
    lv_obj_set_style_text_align(g_health_heap_min, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(g_health_heap_min, "-- K");
    lv_obj_set_width(g_health_heap_min, 96);
    lv_obj_set_height(g_health_heap_min, 22);
    lv_obj_set_pos(g_health_heap_min, 105, 22);

    g_health_heap_block = lv_label_create(heap_card);
    lv_obj_set_style_text_font(g_health_heap_block, &A4SPEED_16, 0);
    lv_obj_set_style_text_color(g_health_heap_block, WHITE, 0);
    lv_obj_set_style_text_align(g_health_heap_block, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(g_health_heap_block, "-- K");
    lv_obj_set_width(g_health_heap_block, 96);
    lv_obj_set_height(g_health_heap_block, 22);
    lv_obj_set_pos(g_health_heap_block, 205, 22);

    mk_label(heap_card, &lv_font_montserrat_10, GRAY_LBL, "FREE", LV_ALIGN_BOTTOM_LEFT, 8, -3);
    mk_label(heap_card, &lv_font_montserrat_10, GRAY_LBL, "MIN FREE", LV_ALIGN_BOTTOM_MID, 0, -3);
    mk_label(heap_card, &lv_font_montserrat_10, GRAY_LBL, "LARGEST", LV_ALIGN_BOTTOM_RIGHT, -8, -3);

    // Bottom diagnostics: human-readable uptime and the reset reason captured
    // at boot. Keeping them separate prevents long reset strings from colliding
    // with the uptime value.
    ext_metric(g_scr_health, 7, 160, 148, 50, "UPTIME", &g_health_values[4]);
    ext_metric(g_scr_health, 165, 160, 148, 50, "RESET", &g_health_reset);


    // PAGE 9: WATCHDOG / RESET
    g_scr_watchdog = ext_screen("WATCHDOG", 9);
    ext_metric(g_scr_watchdog,8,38,148,52,"WDT",&g_wdt_values[0]);
    ext_metric(g_scr_watchdog,164,38,148,52,"FEEDS",&g_wdt_values[1]);
    ext_metric(g_scr_watchdog,8,98,148,52,"LAST FEED",&g_wdt_values[2]);
    ext_metric(g_scr_watchdog,164,98,148,52,"RESET",&g_wdt_values[3]);
    ext_metric(g_scr_watchdog,8,158,304,52,"RECOVERY",&g_wdt_values[4]);

    // PAGE 10: SENSOR STATUS / FRAMEWORK
    // กล่องเดิมแคบเกินไปสำหรับ caption เช่น SPEED GPS / STFT REAL
    // ใช้กล่องกว้างขึ้นและลดระยะระหว่าง value กับ caption เพื่อไม่ให้ตัวหนังสือแตะกรอบ
    g_scr_sensors = ext_screen("SENSOR STATUS", 10);
    const char *sn[12] = {"RPM REAL","SPEED GPS","TPS REAL","ECT REAL","IAT REAL","MAP CALC","O2 REAL","AFR EST","STFT REAL","BATT REAL","INJ CALC","IGN CALC"};
    for(int i=0;i<12;i++){
        int col=i%4, row=i/4;
        ext_metric(g_scr_sensors, 4+col*78, 38+row*67, 74, 60, sn[i], &g_sensor_values[i]);
    }

    // PAGE 11: DATA LOGGER (automatic drive-session capture)
    g_scr_data_logger = ext_screen("DATA LOGGER", 11);
    ext_metric(g_scr_data_logger,8,38,148,52,"MODE",&g_logger_values[0]);
    ext_metric(g_scr_data_logger,164,38,148,52,"SAMPLES",&g_logger_values[1]);
    ext_metric(g_scr_data_logger,8,98,148,52,"SESSIONS",&g_logger_values[2]);
    ext_metric(g_scr_data_logger,164,98,148,52,"BUFFER",&g_logger_values[3]);
    ext_metric(g_scr_data_logger,8,158,148,52,"LAST FILE",&g_logger_values[4]);
    ext_metric(g_scr_data_logger,164,158,148,52,"STORAGE",&g_logger_values[5]);

    // PAGE 12: ALARM
    g_scr_alarm = ext_screen("ALARMS", 12);
    ext_metric(g_scr_alarm,8,38,148,52,"TEMP",&g_alarm_values[0]);
    ext_metric(g_scr_alarm,164,38,148,52,"BATTERY",&g_alarm_values[1]);
    ext_metric(g_scr_alarm,8,98,148,52,"RPM",&g_alarm_values[2]);
    ext_metric(g_scr_alarm,164,98,148,52,"SPEED",&g_alarm_values[3]);
    ext_metric(g_scr_alarm,8,158,304,52,"SYSTEM",&g_alarm_values[4]);

    // PAGE 13: PERFORMANCE
    // ปรับเป็น 2 คอลัมน์เพื่อเพิ่มความกว้างของกรอบ ลดการเบียดกันของฟอนต์
    g_scr_performance = ext_screen("PERFORMANCE", 13);
    ext_metric(g_scr_performance,   7, 36, 148, 54, "MAX SPEED", &g_perf_values[0]);
    ext_metric(g_scr_performance, 165, 36, 148, 54, "MAX RPM",   &g_perf_values[1]);
    ext_metric(g_scr_performance,   7, 94, 148, 54, "MAX TPS",   &g_perf_values[2]);
    ext_metric(g_scr_performance, 165, 94, 148, 54, "MAX ECT",   &g_perf_values[3]);
    ext_metric(g_scr_performance,   7,152, 148, 54, "LAST RUN",  &g_perf_values[4]);
    ext_metric(g_scr_performance, 165,152, 148, 54, "TRIP",      &g_perf_values[5]);
    mk_label(g_scr_performance,&A4SPEED_14,GRAY_LBL,"Live drive statistics",LV_ALIGN_TOP_MID,0,216);

    // PAGE 13: K-LINE TX / RX LOG (แสดงเฉพาะค่า log)
    // เดิมหน้านี้มีกล่อง TARGET FUEL TRIM, ปุ่ม -/+, สถานะ ECU WRITE, ปุ่ม APPLY
    // และ log แค่ 3 บรรทัด — ตอนนี้เอาออกทั้งหมด เหลือ log เต็มหน้าจอ
    // (ใหม่สุดอยู่บนสุด: บรรทัดแรก = TX, ตามด้วย RX ที่ตัดบรรทัดอัตโนมัติ)
    g_scr_fuel_table = ext_screen("K-LINE TX / RX LOG", 13);

    g_kline_log_label = lv_label_create(g_scr_fuel_table);
    lv_obj_set_style_text_font(g_kline_log_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(g_kline_log_label, WHITE, 0);
    lv_obj_set_style_text_line_space(g_kline_log_label, 1, 0);
    lv_label_set_long_mode(g_kline_log_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(g_kline_log_label, 6, 27);
    lv_obj_set_size(g_kline_log_label, 308, 209);   // ข้อความที่เกินความสูงนี้ถูกตัดทิ้งเอง
    lv_label_set_text_static(g_kline_log_label, g_kline_log_text);

    // PAGE 14: SECOND INSTRUMENT CLUSTER — motorsport / OEM-style dash
    // Visual-only redesign. Data flow and live update logic remain unchanged.
    // The layout uses a layered bezel, segmented tach arc, framed speed panel,
    // AFR capsule and three compact telemetry cards for a more professional
    // instrument-cluster appearance on the 320x240 display.
    g_scr_fueltrim = ext_screen("CLUSTER", 14);
    if(lv_obj_get_child(g_scr_fueltrim, 0)){
        lv_obj_add_flag(lv_obj_get_child(g_scr_fueltrim, 0), LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_style_bg_color(g_scr_fueltrim, g_day_mode ? DAY_BG : BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_fueltrim, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(g_scr_fueltrim, 0, 0);

    // ── Outer bezel / depth frame ─────────────────────────────
    g_cluster_bezel = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(g_cluster_bezel, 1, 1);
    lv_obj_set_size(g_cluster_bezel, 318, 238);
    lv_obj_clear_flag(g_cluster_bezel, LV_OBJ_FLAG_SCROLLABLE);
    // P14 main background follows DAY/NIGHT; RPM tracks stay black for contrast.
    lv_obj_set_style_bg_color(g_cluster_bezel, g_day_mode ? DAY_BG : BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_cluster_bezel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_cluster_bezel, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(g_cluster_bezel, 1, 0);
    lv_obj_set_style_radius(g_cluster_bezel, 8, 0);
    lv_obj_set_style_pad_all(g_cluster_bezel, 0, 0);

    // ── Header rail ────────────────────────────────────────────
    lv_obj_t *cluster_hdr = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(cluster_hdr, 10, 8);
    lv_obj_set_size(cluster_hdr, 300, 19);
    lv_obj_clear_flag(cluster_hdr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_hdr, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(cluster_hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cluster_hdr, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(cluster_hdr, 1, 0);
    lv_obj_set_style_radius(cluster_hdr, 6, 0);
    lv_obj_set_style_pad_all(cluster_hdr, 0, 0);

    lv_obj_t *cluster_hdr_accent = lv_obj_create(cluster_hdr);
    lv_obj_set_pos(cluster_hdr_accent, 1, 1);
    lv_obj_set_size(cluster_hdr_accent, 4, 15);
    lv_obj_clear_flag(cluster_hdr_accent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_hdr_accent, ACCENT_OK, 0);
    lv_obj_set_style_bg_opa(cluster_hdr_accent, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cluster_hdr_accent, 0, 0);
    lv_obj_set_style_radius(cluster_hdr_accent, 2, 0);

    mk_label(cluster_hdr, &A4SPEED_14, WHITE,
             "SQXZ  /  CLUSTER-2", LV_ALIGN_LEFT_MID, 10, 0);
    g_cluster_status_line = mk_label(cluster_hdr, &A4SPEED_14, ACCENT_OK,
                                     "ECU LIVE", LV_ALIGN_RIGHT_MID, -12, 0);

    // ── PAGE 14 RPM BAR — PAGE 15 STYLE, HORIZONTAL & LONG ─────────────
    // ใช้ขีดสี่เหลี่ยมเรียงตรงแบบ PAGE 15 เพื่อลดภาระ LVGL ให้ต่ำที่สุด
    // และทำให้ตอบสนอง/ไหลของ RPM ได้ใกล้เคียงกับหน้า 15
    const int P14_BAR_X = 18;
    const int P14_BAR_W = 284;   // ยาวขึ้นเพื่อใช้พื้นที่หน้า 14 เกือบเต็ม
    const int P14_BAR_Y = 37;
    const int P14_BAR_H = 31;
    const int P14_BORDER = 3;
    const int P14_PAD = 3;
    const int P14_GAP = 3;
    const int P14_TICK_H = 21;
    const int P14_INNER_W = P14_BAR_W - 2 * (P14_BORDER + P14_PAD);
    const int P14_TICK_W = (P14_INNER_W - (P14_RPM_TICKS - 1) * P14_GAP) / P14_RPM_TICKS;
    const int P14_TICKS_TOTAL_W = P14_RPM_TICKS * P14_TICK_W + (P14_RPM_TICKS - 1) * P14_GAP;
    const int P14_TICKS_X0 = P14_BAR_X + P14_BORDER + P14_PAD +
                             (P14_INNER_W - P14_TICKS_TOTAL_W) / 2;
    const int P14_TICKS_Y = P14_BAR_Y + P14_BORDER + P14_PAD;

    g_cluster_rpm_track = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(g_cluster_rpm_track, P14_BAR_X, P14_BAR_Y);
    lv_obj_set_size(g_cluster_rpm_track, P14_BAR_W, P14_BAR_H);
    lv_obj_clear_flag(g_cluster_rpm_track, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(g_cluster_rpm_track, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(g_cluster_rpm_track, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_cluster_rpm_track, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_cluster_rpm_track, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(g_cluster_rpm_track, 1, 0);
    lv_obj_set_style_radius(g_cluster_rpm_track, 5, 0);
    lv_obj_set_style_pad_all(g_cluster_rpm_track, 0, 0);

    for(int i = 0; i < P14_RPM_TICKS; ++i){
        lv_obj_t *tick = lv_obj_create(g_scr_fueltrim);
        g_cluster_rpm_ticks[i] = tick;
        lv_obj_set_pos(tick, P14_TICKS_X0 + i * (P14_TICK_W + P14_GAP), P14_TICKS_Y);
        lv_obj_set_size(tick, P14_TICK_W, P14_TICK_H);
        lv_obj_clear_flag(tick, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(tick, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_border_width(tick, 0, 0);
        lv_obj_set_style_radius(tick, 2, 0);
        // Inactive ticks stay true black; no faded zone colors in the background.
        lv_obj_set_style_bg_color(tick, BG_BLACK, 0);
        lv_obj_set_style_bg_opa(tick, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(tick, 0, 0);
    }


    // ── PAGE 14 side numeric panels — 2 stacked boxes per side ─────
    // กล่องบน = ชื่อค่า / กล่องล่าง = ตัวเลขค่า
    // ใช้ฟอนต์ A4SPEED_14 แบบเดียวกับ BATT / TRIP / TEMP
    const int SIDE_Y = 84;
    const int SIDE_W = 86;
    const int SIDE_TOP_H = 25;
    const int SIDE_GAP = 4;
    const int SIDE_VAL_Y = SIDE_Y + SIDE_TOP_H + SIDE_GAP;
    const int SIDE_VAL_H = 40;
    const int SIDE_L_X = 8;
    const int SIDE_R_X = 226;

    // ── RPM : กล่องบน + กล่องล่าง ─────────────────────────────
    lv_obj_t *cluster_rpm_cap_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(cluster_rpm_cap_box, SIDE_L_X, SIDE_Y);
    lv_obj_set_size(cluster_rpm_cap_box, SIDE_W, SIDE_TOP_H);
    lv_obj_clear_flag(cluster_rpm_cap_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_rpm_cap_box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(cluster_rpm_cap_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cluster_rpm_cap_box, ACCENT_OK, 0);
    lv_obj_set_style_border_width(cluster_rpm_cap_box, 1, 0);
    lv_obj_set_style_radius(cluster_rpm_cap_box, 7, 0);
    lv_obj_set_style_pad_all(cluster_rpm_cap_box, 0, 0);

    lv_obj_t *cluster_rpm_cap = mk_label(cluster_rpm_cap_box, &A4SPEED_14, theme_subtext(),
                                         "RPM", LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_width(cluster_rpm_cap, SIDE_W - 4);
    lv_obj_set_style_text_align(cluster_rpm_cap, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *cluster_rpm_val_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(cluster_rpm_val_box, SIDE_L_X, SIDE_VAL_Y);
    lv_obj_set_size(cluster_rpm_val_box, SIDE_W, SIDE_VAL_H);
    lv_obj_clear_flag(cluster_rpm_val_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_rpm_val_box, PANEL_DARK, 0);
    lv_obj_set_style_bg_opa(cluster_rpm_val_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cluster_rpm_val_box, ACCENT_OK, 0);
    lv_obj_set_style_border_width(cluster_rpm_val_box, 1, 0);
    lv_obj_set_style_radius(cluster_rpm_val_box, 7, 0);
    lv_obj_set_style_pad_all(cluster_rpm_val_box, 0, 0);

    g_cluster_rpm_value = mk_label(cluster_rpm_val_box, &A4SPEED_16, page14_value_text(),
                                   "0", LV_ALIGN_CENTER, 0, 6);
    lv_obj_set_width(g_cluster_rpm_value, SIDE_W - 6);
    lv_obj_set_height(g_cluster_rpm_value, 24);
    lv_label_set_long_mode(g_cluster_rpm_value, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(g_cluster_rpm_value, LV_TEXT_ALIGN_CENTER, 0);

    // ── TPS : กล่องบน + กล่องล่าง ─────────────────────────────
    lv_obj_t *cluster_tps_cap_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(cluster_tps_cap_box, SIDE_R_X, SIDE_Y);
    lv_obj_set_size(cluster_tps_cap_box, SIDE_W, SIDE_TOP_H);
    lv_obj_clear_flag(cluster_tps_cap_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_tps_cap_box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(cluster_tps_cap_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cluster_tps_cap_box, ACCENT_BLUE, 0);
    lv_obj_set_style_border_width(cluster_tps_cap_box, 1, 0);
    lv_obj_set_style_radius(cluster_tps_cap_box, 7, 0);
    lv_obj_set_style_pad_all(cluster_tps_cap_box, 0, 0);

    lv_obj_t *cluster_tps_cap = mk_label(cluster_tps_cap_box, &A4SPEED_14, theme_subtext(),
                                         "TPS", LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_width(cluster_tps_cap, SIDE_W - 4);
    lv_obj_set_style_text_align(cluster_tps_cap, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *cluster_tps_val_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(cluster_tps_val_box, SIDE_R_X, SIDE_VAL_Y);
    lv_obj_set_size(cluster_tps_val_box, SIDE_W, SIDE_VAL_H);
    lv_obj_clear_flag(cluster_tps_val_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_tps_val_box, PANEL_DARK, 0);
    lv_obj_set_style_bg_opa(cluster_tps_val_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cluster_tps_val_box, ACCENT_BLUE, 0);
    lv_obj_set_style_border_width(cluster_tps_val_box, 1, 0);
    lv_obj_set_style_radius(cluster_tps_val_box, 7, 0);
    lv_obj_set_style_pad_all(cluster_tps_val_box, 0, 0);

    g_cluster_tps_value = mk_label(cluster_tps_val_box, &A4SPEED_16, page14_value_text(),
                                   "0%", LV_ALIGN_CENTER, 0, 6);
    lv_obj_set_width(g_cluster_tps_value, SIDE_W - 6);
    lv_obj_set_height(g_cluster_tps_value, 24);
    lv_label_set_long_mode(g_cluster_tps_value, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(g_cluster_tps_value, LV_TEXT_ALIGN_CENTER, 0);

    // ── Main speed panel: layered frame + subtle shadow ────────
    lv_obj_t *cluster_speed_panel = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(cluster_speed_panel, 102, 84);
    lv_obj_set_size(cluster_speed_panel, 116, 69);
    lv_obj_clear_flag(cluster_speed_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_speed_panel, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(cluster_speed_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cluster_speed_panel, ACCENT_BLUE, 0);
    lv_obj_set_style_border_width(cluster_speed_panel, 1, 0);
    lv_obj_set_style_radius(cluster_speed_panel, 10, 0);
    lv_obj_set_style_pad_all(cluster_speed_panel, 0, 0);
    lv_obj_set_style_shadow_width(cluster_speed_panel, 7, 0);
    lv_obj_set_style_shadow_color(cluster_speed_panel, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(cluster_speed_panel, 80, 0);

    lv_obj_t *cluster_speed_glow = lv_obj_create(cluster_speed_panel);
    lv_obj_set_pos(cluster_speed_glow, 5, 3);
    lv_obj_set_size(cluster_speed_glow, 105, 3);
    lv_obj_clear_flag(cluster_speed_glow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_speed_glow, ACCENT_BLUE, 0);
    lv_obj_set_style_bg_grad_color(cluster_speed_glow, ACCENT_INJ, 0);
    lv_obj_set_style_bg_grad_dir(cluster_speed_glow, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(cluster_speed_glow, LV_OPA_80, 0);
    lv_obj_set_style_border_width(cluster_speed_glow, 0, 0);
    lv_obj_set_style_radius(cluster_speed_glow, 2, 0);

    g_cluster_speed = mk_label(cluster_speed_panel, &dseg7_32, theme_primary_text(),
                               "0", LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_width(g_cluster_speed, 160);
    lv_obj_set_height(g_cluster_speed, 33);
    lv_label_set_long_mode(g_cluster_speed, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(g_cluster_speed, LV_TEXT_ALIGN_CENTER, 0);

    g_cluster_speed_unit = mk_label(cluster_speed_panel, &A4SPEED_16, theme_primary_text(),
                                     "KM/H", LV_ALIGN_TOP_MID, 0, 47);
    lv_obj_set_width(g_cluster_speed_unit, 72);
    lv_obj_set_style_text_align(g_cluster_speed_unit, LV_TEXT_ALIGN_CENTER, 0);

    // ── AFR capsule ────────────────────────────────────────────
    lv_obj_t *cluster_afr_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(cluster_afr_box, 12, 160);
    lv_obj_set_size(cluster_afr_box, 296, 21);
    lv_obj_clear_flag(cluster_afr_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_afr_box, PANEL_DARK, 0);
    lv_obj_set_style_bg_opa(cluster_afr_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(cluster_afr_box, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(cluster_afr_box, 1, 0);
    lv_obj_set_style_radius(cluster_afr_box, 6, 0);
    lv_obj_set_style_pad_all(cluster_afr_box, 0, 0);

    lv_obj_t *cluster_afr_mark = lv_obj_create(cluster_afr_box);
    lv_obj_set_pos(cluster_afr_mark, 3, 3);
    lv_obj_set_size(cluster_afr_mark, 3, 13);
    lv_obj_clear_flag(cluster_afr_mark, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(cluster_afr_mark, ACCENT_RICH, 0);
    lv_obj_set_style_bg_opa(cluster_afr_mark, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cluster_afr_mark, 0, 0);
    lv_obj_set_style_radius(cluster_afr_mark, 1, 0);

    // Match AFR / EST typography to the battery/VOLTAGE field: same A4SPEED_14
    // segmented font for both the caption and the live numeric value.
    g_cluster_afr_label = mk_label(cluster_afr_box, &A4SPEED_14, theme_subtext(),
                                   "AFR  /  EST.", LV_ALIGN_LEFT_MID, 12, 0);
    g_cluster_afr_val = mk_label(cluster_afr_box, &A4SPEED_14, ACCENT_RICH,
                                  "--.-", LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_width(g_cluster_afr_val, 70);
    lv_label_set_long_mode(g_cluster_afr_val, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(g_cluster_afr_val, LV_TEXT_ALIGN_RIGHT, 0);

    // ── Bottom telemetry cards ─────────────────────────────────
    const int STAT_Y = 188;
    const int STAT_W = 96;
    const int STAT_H = 45;
    const int STAT_G = 8;
    const int STAT_X0 = 8;

    lv_obj_t *trip_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(trip_box, STAT_X0, STAT_Y);
    lv_obj_set_size(trip_box, STAT_W, STAT_H);
    lv_obj_clear_flag(trip_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(trip_box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(trip_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(trip_box, ACCENT_OK, 0);
    lv_obj_set_style_border_width(trip_box, 1, 0);
    lv_obj_set_style_radius(trip_box, 7, 0);
    lv_obj_set_style_pad_all(trip_box, 0, 0);
    lv_obj_set_style_shadow_width(trip_box, 2, 0);
    lv_obj_set_style_shadow_color(trip_box, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(trip_box, 90, 0);

    lv_obj_t *trip_cap = mk_label(trip_box, &A4SPEED_14, theme_subtext(),
                                  "TRIP", LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_width(trip_cap, STAT_W-4);
    lv_obj_set_style_text_align(trip_cap, LV_TEXT_ALIGN_CENTER, 0);
    g_cluster_trip = mk_label(trip_box, &A4SPEED_14, ACCENT_OK,
                              "0.00 km", LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_width(g_cluster_trip, STAT_W-4);
    lv_obj_set_height(g_cluster_trip, 20);
    lv_obj_set_style_text_align(g_cluster_trip, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(g_cluster_trip, LV_LABEL_LONG_CLIP);

    lv_obj_t *volt_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(volt_box, STAT_X0 + STAT_W + STAT_G, STAT_Y);
    lv_obj_set_size(volt_box, STAT_W, STAT_H);
    lv_obj_clear_flag(volt_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(volt_box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(volt_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(volt_box, ACCENT_BLUE, 0);
    lv_obj_set_style_border_width(volt_box, 1, 0);
    lv_obj_set_style_radius(volt_box, 7, 0);
    lv_obj_set_style_pad_all(volt_box, 0, 0);
    lv_obj_set_style_shadow_width(volt_box, 2, 0);
    lv_obj_set_style_shadow_color(volt_box, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(volt_box, 90, 0);

    lv_obj_t *volt_cap = mk_label(volt_box, &A4SPEED_14, theme_subtext(),
                                  "VOLTAGE", LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_width(volt_cap, STAT_W-4);
    lv_obj_set_style_text_align(volt_cap, LV_TEXT_ALIGN_CENTER, 0);
    g_cluster_batt = mk_label(volt_box, &A4SPEED_14, ACCENT_BLUE,
                              "0.0 V", LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_width(g_cluster_batt, STAT_W-4);
    lv_obj_set_height(g_cluster_batt, 20);
    lv_obj_set_style_text_align(g_cluster_batt, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(g_cluster_batt, LV_LABEL_LONG_CLIP);

    lv_obj_t *ect_box = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(ect_box, STAT_X0 + 2*(STAT_W + STAT_G), STAT_Y);
    lv_obj_set_size(ect_box, STAT_W, STAT_H);
    lv_obj_clear_flag(ect_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ect_box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(ect_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(ect_box, ACCENT_TEMP, 0);
    lv_obj_set_style_border_width(ect_box, 1, 0);
    lv_obj_set_style_radius(ect_box, 7, 0);
    lv_obj_set_style_pad_all(ect_box, 0, 0);
    lv_obj_set_style_shadow_width(ect_box, 2, 0);
    lv_obj_set_style_shadow_color(ect_box, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(ect_box, 90, 0);

    lv_obj_t *ect_cap = mk_label(ect_box, &A4SPEED_14, theme_subtext(),
                                 "TEMP", LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_width(ect_cap, STAT_W-4);
    lv_obj_set_style_text_align(ect_cap, LV_TEXT_ALIGN_CENTER, 0);
    g_cluster_ect = mk_label(ect_box, &A4SPEED_14, ACCENT_TEMP,
                             "-- C", LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_width(g_cluster_ect, STAT_W-4);
    lv_obj_set_height(g_cluster_ect, 20);
    lv_obj_set_style_text_align(g_cluster_ect, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(g_cluster_ect, LV_LABEL_LONG_CLIP);

    // Small live-link indicator integrated into the header.
    g_cluster_link = lv_obj_create(g_scr_fueltrim);
    lv_obj_set_pos(g_cluster_link, 300, 14);
    lv_obj_set_size(g_cluster_link, 6, 6);
    lv_obj_clear_flag(g_cluster_link, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(g_cluster_link, 3, 0);
    lv_obj_set_style_bg_color(g_cluster_link, ACCENT_ERR, 0);
    lv_obj_set_style_bg_opa(g_cluster_link, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_cluster_link, 0, 0);

    // PAGE 15: SQXZGAUGE GAUGE — ดึงเฉพาะหน้า Gauge UI จากโปรเจกต์ SQXZGAUGE
    sqxzgauge_page_init();

    // PAGE 16: DISTANCE TEST — ตั้งระยะทาง (ปุ่ม -/+) แล้ววัดเวลา/ความเร็วเฉลี่ย
    // จากระยะทางจริงที่สะสมจาก GPS ระหว่างกดเริ่มจนครบระยะที่ตั้งไว้
    g_scr_disttest = ext_screen("DISTANCE TEST", 16);

    // แผงเลือกระยะทางเป้าหมาย: ปุ่ม [-] ค่า [+]
    lv_obj_t *dist_box = lv_obj_create(g_scr_disttest);
    lv_obj_set_pos(dist_box, 8, 34); lv_obj_set_size(dist_box, 304, 58);
    lv_obj_clear_flag(dist_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(dist_box, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(dist_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(dist_box, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(dist_box, 1, 0);
    lv_obj_set_style_radius(dist_box, 8, 0);
    lv_obj_set_style_pad_all(dist_box, 0, 0);

    mk_menu_btn(dist_box, 6, 10, 50, 38, "-", ACCENT_WARN, NULL);
    mk_menu_btn(dist_box, 248, 10, 50, 38, "+", ACCENT_OK, NULL);
    g_lbl_dist_target = mk_label(dist_box, &A4SPEED_26, WHITE, "402 m", LV_ALIGN_TOP_MID, 0, 12);
    mk_label(dist_box, &lv_font_montserrat_10, GRAY_LBL, "TARGET DISTANCE", LV_ALIGN_BOTTOM_MID, 0, -2);

    g_lbl_dist_status = mk_label(g_scr_disttest, &A4SPEED_16, ACCENT_BLUE, "READY", LV_ALIGN_TOP_MID, 0, 98);

    ext_metric(g_scr_disttest,   8, 122, 148, 58, "TIME (s)",         &g_lbl_dist_time);
    ext_metric(g_scr_disttest, 164, 122, 148, 58, "AVG SPEED (km/h)", &g_lbl_dist_speed);

    lv_obj_t *dist_startstop_btn = mk_menu_btn(g_scr_disttest, 8, 190, 148, 40, "AUTO START", ACCENT_OK, NULL);
    g_btn_dist_startstop_lbl = lv_obj_get_child(dist_startstop_btn, 0);
    mk_menu_btn(g_scr_disttest, 164, 190, 148, 40, "RESET", BTN_NEUTRAL, NULL);
}

// Text cache is keyed by the actual LVGL object, not by source-code call
// site. The old per-call-site cache was incorrect for loops/shared macros:
// one cache entry could be reused for several different labels, causing
// unnecessary redraws and also suppressing legitimate text changes.
struct LabelTextCache {
    lv_obj_t *obj;
    char text[120];
};
static LabelTextCache g_label_text_cache[96] = {};

static inline void label_set_if_changed(lv_obj_t *obj, const char *txt){
    if(!obj || !txt) return;

    LabelTextCache *slot = nullptr;
    LabelTextCache *free_slot = nullptr;
    for(size_t i = 0; i < sizeof(g_label_text_cache)/sizeof(g_label_text_cache[0]); ++i){
        LabelTextCache &c = g_label_text_cache[i];
        if(c.obj == obj){ slot = &c; break; }
        if(!c.obj && !free_slot) free_slot = &c;
    }
    if(!slot) slot = free_slot;

    // All screens are intentionally persistent for fast navigation, so a
    // small fixed cache is enough and avoids heap churn on every UI update.
    if(!slot){
        lv_label_set_text(obj, txt);
        return;
    }

    if(slot->obj == obj && strncmp(slot->text, txt, sizeof(slot->text)) == 0) return;

    slot->obj = obj;
    strncpy(slot->text, txt, sizeof(slot->text) - 1);
    slot->text[sizeof(slot->text) - 1] = '\0';
    lv_label_set_text(obj, txt);
}

#define LABEL_SET_IF_CHANGED(obj, txt) label_set_if_changed((obj), (txt))

static void fueltrim_save(){
    if(!g_fueltrim_prefs_ready) return;
    g_fueltrim_prefs.putChar("trim", g_fuel_trim_pct);
}

static void fueltrim_set(int delta){
    int v = (int)g_fuel_trim_pct + delta;
    if(v < -20) v = -20;
    if(v > 20) v = 20;
    g_fuel_trim_pct = (int8_t)v;
    fueltrim_save();
    if(g_fueltrim_value){
        char t[16]; snprintf(t,sizeof(t),"%+d%%",(int)g_fuel_trim_pct);
        lv_label_set_text(g_fueltrim_value,t);
    }
    UI_TRACE("[FUEL TRIM] target=%+d%% (UI/NVS only; ECU WRITE LOCKED)\n", (int)g_fuel_trim_pct);
}

static void fueltrim_minus_cb(lv_event_t *e){ (void)e; fueltrim_set(-1); }
static void fueltrim_plus_cb(lv_event_t *e){ (void)e; fueltrim_set(+1); }

static void fueltrim_apply_cb(lv_event_t *e){
    (void)e;
    UI_TRACE("[FUEL TRIM] APPLY requested target=%+d%%; ECU WRITE LOCKED\n", (int)g_fuel_trim_pct);
    if(g_fueltrim_status){
        lv_label_set_text(g_fueltrim_status, KLINE_WRITE_UNLOCKED ? "ECU WRITE: ENABLED" : "ECU WRITE: LOCKED");
        lv_obj_set_style_text_color(g_fueltrim_status, KLINE_WRITE_UNLOCKED ? ACCENT_OK : ACCENT_WARN, 0);
    }
}

// ต่อข้อความลง buffer แบบปลอดภัย (ไม่เกิน cap-1) คืนค่าความยาวใหม่
static size_t klog_append(char *buf, size_t cap, size_t used, const char *fmt, ...){
    if(used + 1 >= cap) return used;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(buf + used, cap - used, fmt, ap);
    va_end(ap);
    if(r < 0) return used;
    size_t nu = used + (size_t)r;
    return (nu >= cap) ? (cap - 1) : nu;
}

// PAGE 13: สร้างข้อความ K-LINE TX/RX LOG (ใหม่สุดก่อน) แล้วอัปเดต label
// รูปแบบต่อ 1 รายการ:
//   > 123.456s TX 72 05 71 17 01
//   RX 02 1F 71 17 ....            (ยาวเกินความกว้างจะตัดบรรทัดเอง)
// '>' = ได้ตอบกลับจาก ECU, '!' = ไม่มี RX (ไม่ตอบ/หมดเวลา)
static void kline_log_page_update(){
    if(!g_kline_log_label) return;

    // จำกัดความถี่ ~6-7 Hz: log ใน ring buffer หมุนเร็วกว่าที่ตาอ่านทัน
    // และลดงาน redraw ของ LVGL
    static uint32_t s_last_ms = 0;
    const uint32_t now = millis();
    if(now - s_last_ms < 150UL) return;
    s_last_ms = now;

    // ลายเซ็นของ log = จำนวนรายการ + เวลาของรายการใหม่สุด
    // ถ้าไม่เปลี่ยนก็ไม่ต้อง rebuild / ไม่ต้อง redraw
    static bool     s_sig_valid = false;
    static uint8_t  s_sig_n = 0;
    static uint32_t s_sig_ms = 0;

    const uint8_t n = kline_txrx_log_count();
    KlineTxRxLogEntry e{};
    uint32_t newest_ms = 0;
    if(n > 0 && kline_get_txrx_log(0, &e)) newest_ms = e.ms;

    if(s_sig_valid && n == s_sig_n && newest_ms == s_sig_ms) return;
    s_sig_valid = true; s_sig_n = n; s_sig_ms = newest_ms;

    size_t used = 0;
    g_kline_log_text[0] = '\0';

    for(uint8_t i = 0; i < n; ++i){
        if(!kline_get_txrx_log(i, &e)) break;
        used = klog_append(g_kline_log_text, sizeof(g_kline_log_text), used,
                           "%s%c %lu.%03lus TX", (i ? "\n" : ""), e.ok ? '>' : '!',
                           (unsigned long)(e.ms / 1000UL), (unsigned long)(e.ms % 1000UL));
        for(uint8_t j = 0; j < e.tx_len; ++j)
            used = klog_append(g_kline_log_text, sizeof(g_kline_log_text), used, " %02X", e.tx[j]);
        used = klog_append(g_kline_log_text, sizeof(g_kline_log_text), used, "\nRX");
        if(e.rx_len == 0){
            used = klog_append(g_kline_log_text, sizeof(g_kline_log_text), used, " --");
        }else{
            for(uint8_t j = 0; j < e.rx_len; ++j)
                used = klog_append(g_kline_log_text, sizeof(g_kline_log_text), used, " %02X", e.rx[j]);
        }
    }
    if(used == 0) klog_append(g_kline_log_text, sizeof(g_kline_log_text), 0, "--");

    // เรียกซ้ำด้วย pointer เดิมเพื่อให้ LVGL อ่านเนื้อหา buffer ใหม่และ refresh
    lv_label_set_text_static(g_kline_log_label, g_kline_log_text);
}

// ── PAGE 16: DISTANCE TEST — ควบคุมการเลือกระยะทาง/เริ่ม/หยุด/รีเซ็ต ──────
static void disttest_save_preset(){
    if(!g_disttest_prefs_ready) return;
    g_disttest_prefs.putUChar("presetidx", g_dist_preset_idx);
}

static void disttest_refresh_target_label(){
    if(!g_lbl_dist_target) return;
    char t[16]; snprintf(t, sizeof(t), "%.0f m", DIST_TEST_PRESETS_M[g_dist_preset_idx]);
    lv_label_set_text(g_lbl_dist_target, t);
}

static void disttest_preset_adjust(int delta){
    // ปรับระยะทางเป้าหมายได้เฉพาะตอนยังไม่เริ่มจับเวลา (READY) หรือดูผลจบแล้ว (DONE)
    if(g_dist_state == DIST_RUNNING) return;
    int idx = (int)g_dist_preset_idx + delta;
    if(idx < 0) idx = 0;
    if(idx >= (int)DIST_TEST_PRESET_COUNT) idx = DIST_TEST_PRESET_COUNT - 1;
    g_dist_preset_idx = (uint8_t)idx;
    disttest_save_preset();
    disttest_refresh_target_label();
    UI_TRACE("[DIST TEST] TARGET -> %.0f m\n", DIST_TEST_PRESETS_M[g_dist_preset_idx]);
}

static void disttest_set_button_label(const char *txt){
    if(g_btn_dist_startstop_lbl) lv_label_set_text(g_btn_dist_startstop_lbl, txt);
}

static void disttest_start(){
    g_dist_state = DIST_RUNNING;
    g_dist_covered_m = 0.0f;
    g_dist_have_last = false;
    g_dist_start_ms = millis();
    g_dist_run_max_speed_kmh = gps_speed_valid() ? gps_speed_kmh() : 0.0f;
    g_dist_run_max_rpm = f_rpm;

    // ใช้ตำแหน่ง GPS ปัจจุบันเป็นจุดเริ่มต้นทันทีที่รถเริ่มเคลื่อนที่
    // เพื่อไม่ให้นับระยะย้อนหลัง/นับ jump ก่อนเริ่มเวลา
    if(gps_has_fix()){
        g_dist_last_lat = gps_latitude();
        g_dist_last_lon = gps_longitude();
        g_dist_have_last = true;
    }

    disttest_set_button_label("RUNNING");
    UI_TRACE("[DIST TEST] AUTO START target=%.0f m\n", DIST_TEST_PRESETS_M[g_dist_preset_idx]);
}

static void disttest_reset(){
    g_dist_state = DIST_READY;
    g_dist_covered_m = 0.0f;
    g_dist_have_last = false;
    g_dist_result_ms = 0;
    g_dist_result_speed_kmh = 0.0f;
    disttest_set_button_label("AUTO START");
    UI_TRACE_LINE("[DIST TEST] RESET -> READY (AUTO START)");
}

// หน้า 16 เป็น AUTO START เต็มรูปแบบ: ปุ่มซ้ายเป็นเพียงตัวบอกโหมด
// การเริ่ม/หยุดรอบทำโดย GPS อัตโนมัติ จึงไม่ใช้ปุ่มนี้เป็นคำสั่ง START/STOP
static void disttest_startstop_pressed(){
    UI_TRACE_LINE("[DIST TEST] AUTO START mode - manual START ignored");
}

// Keep the lightweight P14 RPM tick bar synchronized even while another page is visible.
static inline lv_color_t cluster_rpm_zone_color(int tick){
    if(tick < 4)  return lv_color_hex(0x29A9FF); // blue
    if(tick < 12) return lv_color_hex(0x35D06F); // green
    if(tick < 17) return lv_color_hex(0xFFD43B); // yellow
    if(tick < 19) return lv_color_hex(0xFF8A32); // orange
    return lv_color_hex(0xFF3B30);               // red
}

static float cluster_rpm_smooth_value(float target){
    const uint32_t now = millis();
    target = constrain(target, 0.0f, 8000.0f);

    if(g_cluster_rpm_visual_ms == 0){
        g_cluster_rpm_visual = target;
        g_cluster_rpm_visual_ms = now;
        return g_cluster_rpm_visual;
    }

    float dt = (float)(now - g_cluster_rpm_visual_ms) * 0.001f;
    g_cluster_rpm_visual_ms = now;
    if(dt <= 0.0f) return g_cluster_rpm_visual;
    if(dt > 0.100f) dt = 0.100f;

    // Match the responsive feel used by PAGE 15 without filtering the real RPM.
    const float tau = (target >= g_cluster_rpm_visual) ? 0.055f : 0.085f;
    float alpha = 1.0f - expf(-dt / tau);
    if(fabsf(target - g_cluster_rpm_visual) > 2200.0f)
        alpha = fmaxf(alpha, 0.40f);

    g_cluster_rpm_visual += (target - g_cluster_rpm_visual) * alpha;
    if(fabsf(target - g_cluster_rpm_visual) < 3.0f)
        g_cluster_rpm_visual = target;
    return g_cluster_rpm_visual;
}

static void cluster_rpm_update(){
    if(!g_scr_fueltrim) return;

    const float rpm = cluster_rpm_smooth_value(f_rpm);
    const int active = (int)lroundf(constrain(rpm / 8000.0f, 0.0f, 1.0f) * P14_RPM_TICKS);
    const bool redline = active >= (int)(P14_RPM_TICKS * 0.90f);

    static uint32_t redline_ms = 0;
    static bool redline_on = true;
    const uint32_t now = millis();
    if(redline && (uint32_t)(now - redline_ms) >= 110U){
        redline_ms = now;
        redline_on = !redline_on;
    }

    if(active == g_cluster_rpm_last_full &&
       redline == g_cluster_rpm_last_redline &&
       redline_on == g_cluster_rpm_last_blink){
        return;
    }

    // Exactly the same lightweight rendering model as PAGE 15:
    // rectangular ticks, no arc, no line geometry, no transforms.
    for(int i = 0; i < P14_RPM_TICKS; ++i){
        lv_obj_t *tick = g_cluster_rpm_ticks[i];
        if(!tick) continue;

        lv_color_t col = BG_BLACK;
        if(i < active){
            col = cluster_rpm_zone_color(i);
            if(redline && !redline_on && i >= (int)(P14_RPM_TICKS * 0.90f))
                col = lv_color_hex(0x401010);
        }
        // One opaque background color change only; no opacity animation and
        // no per-frame geometry work, which keeps the bar responsive like P15.
        set_bg_color_if_changed(tick, col);
        set_bg_opa_if_changed(tick, LV_OPA_COVER);
    }

    g_cluster_rpm_last_full = active;
    g_cluster_rpm_last_partial = -1;
    g_cluster_rpm_last_redline = redline;
    g_cluster_rpm_last_blink = redline_on;
}

static void cluster_update(){
    cluster_rpm_update();
    if(!g_scr_fueltrim || g_page != 14) return;

    if(g_cluster_rpm_value){
        char t[20];
        snprintf(t, sizeof(t), "%.0f", f_rpm);
        LABEL_SET_IF_CHANGED(g_cluster_rpm_value, t);
        // Keep the PAGE 14 RPM number theme-aware instead of recoloring it
        // green/yellow/red every refresh. The RPM bar already communicates
        // the live zone colors; the numeric readout remains monochrome.
        lv_obj_set_style_text_color(g_cluster_rpm_value, page14_value_text(), 0);
    }
    if(g_cluster_tps_value){
        char t[20];
        snprintf(t, sizeof(t), "%.0f%%", f_tps);
        LABEL_SET_IF_CHANGED(g_cluster_tps_value, t);
        lv_obj_set_style_text_color(g_cluster_tps_value, page14_value_text(), 0);
    }
    if(g_cluster_speed){
        char t[20];
        snprintf(t, sizeof(t), "%.0f", f_speed);
        LABEL_SET_IF_CHANGED(g_cluster_speed, t);
        lv_obj_set_style_text_color(g_cluster_speed,
                                    (f_speed >= 120.0f) ? ACCENT_ERR : theme_primary_text(), 0);
    }
    if(g_cluster_trip){
        char t[24]; snprintf(t, sizeof(t), "%.2f km", g_trip_km);
        LABEL_SET_IF_CHANGED(g_cluster_trip, t);
        lv_obj_set_style_text_color(g_cluster_trip, ACCENT_OK, 0);
    }
    if(g_cluster_batt){
        char t[24]; snprintf(t, sizeof(t), "%.1f V", f_batt);
        LABEL_SET_IF_CHANGED(g_cluster_batt, t);
        lv_obj_set_style_text_color(g_cluster_batt, ACCENT_BLUE, 0);
    }
    if(g_cluster_ect){
        char t[24]; snprintf(t, sizeof(t), "%.0f C", f_ect);
        LABEL_SET_IF_CHANGED(g_cluster_ect, t);
        lv_obj_set_style_text_color(g_cluster_ect,
                                    (f_ect >= ALARM_ECT_HIGH_C) ? ACCENT_ERR : ACCENT_TEMP, 0);
    }
    if(g_cluster_status_line){
        LABEL_SET_IF_CHANGED(g_cluster_status_line, kline_is_connected() ? "ECU LIVE" : "ECU");
        lv_obj_set_style_text_color(g_cluster_status_line, kline_is_connected() ? ACCENT_OK : ACCENT_WARN, 0);
    }
    if(g_cluster_link){
        static bool blink=true; static uint32_t last_blink=0;
        const uint32_t now = millis();
        if(now-last_blink >= 400){ last_blink=now; blink=!blink; }
        lv_obj_set_style_bg_color(g_cluster_link, kline_is_connected() ? ACCENT_OK : ACCENT_WARN, 0);
        lv_obj_set_style_bg_opa(g_cluster_link, blink ? LV_OPA_COVER : LV_OPA_30, 0);
    }
}

static void extended_pages_update(){
    const uint32_t now = millis();
    char b[48];
    // Update heavyweight secondary screens only while they are visible.
    // Their widgets are persistent, so there is no need to redraw hidden pages.
    cluster_update();

    // PAGE 13: K-LINE TX / RX LOG — อัปเดตเฉพาะตอนหน้านี้แสดงอยู่
    if(g_page == 13) kline_log_page_update();

    // PAGE 8 health data is intentionally sampled once per second. This is
    // diagnostic UI, not a real-time gauge, so faster writes only add LVGL work.
    static uint32_t last_health_ui_ms = 0;
    if(g_page == 8 && now - last_health_ui_ms >= 1000UL){
        last_health_ui_ms = now;

        // ECU/K-Line: distinguish a live link from a connected-but-stale link.
        const bool ecu_connected = kline_is_connected();
        const uint32_t kline_last = kline_last_good_data_ms();
        const bool kline_live = ecu_connected && kline_last != 0 && (now - kline_last) < KLINE_LINK_STALE_MS;
        LABEL_SET_IF_CHANGED(g_health_values[0], ecu_connected ? "ONLINE" : "OFFLINE");
        set_text_color_if_changed(g_health_values[0], ecu_connected ? ACCENT_OK : ACCENT_WARN);

        // GPS status uses the existing quality state rather than a simple
        // boolean so the page can distinguish fixed, weak, searching and lost.
        const GpsQuality q = gps_quality();
        const char *gps_state = "LOST";
        lv_color_t gps_col = ACCENT_ERR;
        switch(q){
            case GPS_QUALITY_EXCELLENT:
            case GPS_QUALITY_GOOD:      gps_state = "FIX";    gps_col = ACCENT_OK;   break;
            case GPS_QUALITY_FAIR:
            case GPS_QUALITY_WEAK:      gps_state = "WEAK";   gps_col = ACCENT_WARN; break;
            case GPS_QUALITY_NO_FIX:    gps_state = "SEARCH"; gps_col = ACCENT_WARN; break;
            default:                    gps_state = "LOST";   gps_col = ACCENT_ERR;  break;
        }
        LABEL_SET_IF_CHANGED(g_health_values[1], gps_state);
        set_text_color_if_changed(g_health_values[1], gps_col);

        LABEL_SET_IF_CHANGED(g_health_values[5], kline_live ? "LIVE" : (ecu_connected ? "STALE" : "OFF"));
        set_text_color_if_changed(g_health_values[5], kline_live ? ACCENT_OK : (ecu_connected ? ACCENT_WARN : ACCENT_ERR));

        const uint32_t heap_free_kb  = ESP.getFreeHeap() / 1024UL;
        const uint32_t heap_min_kb   = ESP.getMinFreeHeap() / 1024UL;
        const uint32_t heap_block_kb = ESP.getMaxAllocHeap() / 1024UL;
        if(g_health_values[3]){
            snprintf(b,sizeof(b),"%u K",(unsigned)heap_free_kb);
            LABEL_SET_IF_CHANGED(g_health_values[3],b);
        }
        if(g_health_heap_min){
            snprintf(b,sizeof(b),"%u K",(unsigned)heap_min_kb);
            LABEL_SET_IF_CHANGED(g_health_heap_min,b);
        }
        if(g_health_heap_block){
            snprintf(b,sizeof(b),"%u K",(unsigned)heap_block_kb);
            LABEL_SET_IF_CHANGED(g_health_heap_block,b);
        }
        if(g_health_values[4]){
            const uint32_t total_s = now / 1000UL;
            const uint32_t hh = total_s / 3600UL;
            const uint32_t mm = (total_s / 60UL) % 60UL;
            const uint32_t ss = total_s % 60UL;
            snprintf(b,sizeof(b),"%02lu:%02lu:%02lu", (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
            LABEL_SET_IF_CHANGED(g_health_values[4],b);
        }
        if(g_health_reset){
            const char *rr = reset_reason_name();
            LABEL_SET_IF_CHANGED(g_health_reset, rr);
            const bool normal = !strcmp(rr, "POWER ON");
            set_text_color_if_changed(g_health_reset, normal ? ACCENT_OK : ACCENT_WARN);
        }
    }

    if(g_page == 9){
        if(g_wdt_values[0]){ LABEL_SET_IF_CHANGED(g_wdt_values[0], app_watchdog_ready()?"ACTIVE":"OFF"); set_text_color_if_changed(g_wdt_values[0],app_watchdog_ready()?ACCENT_OK:ACCENT_WARN); }
        if(g_wdt_values[1]){ snprintf(b,sizeof(b),"%lu",(unsigned long)app_watchdog_feed_count()); LABEL_SET_IF_CHANGED(g_wdt_values[1],b); }
        if(g_wdt_values[2]){ uint32_t last=app_watchdog_last_feed_ms(); snprintf(b,sizeof(b),"%lums",(unsigned long)(last?now-last:0)); LABEL_SET_IF_CHANGED(g_wdt_values[2],b); }
        if(g_wdt_values[3]){ LABEL_SET_IF_CHANGED(g_wdt_values[3],reset_reason_name()); }
        if(g_wdt_values[4]){ LABEL_SET_IF_CHANGED(g_wdt_values[4],"AUTO RECOVERY READY"); set_text_color_if_changed(g_wdt_values[4],ACCENT_OK); }
    }

    const float afr_est_page = f_afr;
    const float sv[12] = {f_rpm,f_speed,f_tps,f_ect,g_iat,g_kline_snapshot.map_v,g_kline_snapshot.o2_v,afr_est_page,g_kline_snapshot.stft_pct,f_batt,f_inj,f_ign};
    const char *fmt[12] = {"%.0f","%.0f","%.1f","%.0f","%.0f","%.2f","%.2f","%.1f","%.1f","%.2f","%.2f","%.1f"};
    const bool valid[12] = {
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_RPM),
        gps_speed_valid(),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_TPS),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_ECT),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_IAT),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_MAP),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_O2),
        g_afr_valid,
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_STFT),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_BATT),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_INJ),
        kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_IGN)
    };
    if(g_page == 10) for(int i=0;i<12;i++){
        if(g_sensor_values[i]){
            if(valid[i]) snprintf(b,sizeof(b),fmt[i],sv[i]);
            else strncpy(b,"--",sizeof(b));
            b[sizeof(b)-1]='\0';
            LABEL_SET_IF_CHANGED(g_sensor_values[i],b);
            set_text_color_if_changed(g_sensor_values[i], valid[i] ? theme_primary_text() : theme_subtext());
        }
    }

    const bool moving = kline_is_connected() && (g_kline_snapshot.engine_running || f_speed > 2.0f || f_rpm > 400.0f);
    if(moving) g_logger_last_motion_ms = now;
    if(!g_logger_recording && moving){
        g_logger_recording=true;
        g_logger_sample_count=0;
        g_logger_last_sample_ms=0;
        g_logger_session_count++;
        g_logger_session_start_ms=now;
        logger_reset_session_stats();
        logger_save_session_counter();
        logger_append_event("SESSION_START", "Drive session started");
    }
    if(g_logger_recording && now-g_logger_last_sample_ms >= LOGGER_SAMPLE_MS){
        g_logger_last_sample_ms=now;
        logger_update_session_stats();
        if(g_logger_sample_count < LOGGER_MAX_SAMPLES){
            LoggerSample &s=g_logger_samples[g_logger_sample_count++];
            s.ms=now; s.rpm=f_rpm; s.speed=f_speed; s.tps=f_tps; s.ect=f_ect; s.batt=f_batt; s.inj=f_inj; s.ign=f_ign; s.afr=f_afr;
        }
        if(now-g_logger_last_motion_ms >= LOGGER_STOP_MS || g_logger_sample_count >= LOGGER_MAX_SAMPLES){
            g_logger_recording=false;
            logger_write_session();
            logger_append_event("SESSION_END", "Drive session saved");
            g_logger_sample_count=0;
        }
    }
    if(g_page == 11){
        if(g_logger_values[0]){ LABEL_SET_IF_CHANGED(g_logger_values[0],g_logger_recording?"RECORD":"STANDBY"); set_text_color_if_changed(g_logger_values[0],g_logger_recording?ACCENT_ERR:ACCENT_OK); }
        if(g_logger_values[1]){ snprintf(b,sizeof(b),"%u",(unsigned)g_logger_sample_count); LABEL_SET_IF_CHANGED(g_logger_values[1],b); }
        if(g_logger_values[2]){ snprintf(b,sizeof(b),"%lu",(unsigned long)g_logger_session_count); LABEL_SET_IF_CHANGED(g_logger_values[2],b); }
        if(g_logger_values[3]){ snprintf(b,sizeof(b),"%u/%u",(unsigned)g_logger_sample_count,(unsigned)LOGGER_MAX_SAMPLES); LABEL_SET_IF_CHANGED(g_logger_values[3],b); }
        if(g_logger_values[4]) LABEL_SET_IF_CHANGED(g_logger_values[4],g_logger_last_file);
        if(g_logger_values[5]) LABEL_SET_IF_CHANGED(g_logger_values[5],g_logger_fs_ready?"LITTLEFS OK":"RAM ONLY");
    }

    g_alarm_ect = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_ECT) && (f_ect >= ALARM_ECT_HIGH_C);
    g_alarm_batt = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_BATT) && (f_batt > 0.1f && f_batt <= ALARM_BATT_LOW_V);
    g_alarm_rpm = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_RPM) && (f_rpm >= ALARM_RPM_HIGH);
    g_alarm_speed = gps_speed_valid() && (f_speed >= ALARM_SPEED_HIGH);
    if(g_alarm_ect != g_event_prev_ect){ logger_append_event(g_alarm_ect?"ALARM_ON":"ALARM_OFF", g_alarm_ect?"ECT HIGH":"ECT normal"); g_event_prev_ect=g_alarm_ect; }
    if(g_alarm_batt != g_event_prev_batt){ logger_append_event(g_alarm_batt?"ALARM_ON":"ALARM_OFF", g_alarm_batt?"BATTERY LOW":"Battery normal"); g_event_prev_batt=g_alarm_batt; }
    if(g_alarm_rpm != g_event_prev_rpm){ logger_append_event(g_alarm_rpm?"ALARM_ON":"ALARM_OFF", g_alarm_rpm?"RPM HIGH":"RPM normal"); g_event_prev_rpm=g_alarm_rpm; }
    if(g_alarm_speed != g_event_prev_speed){ logger_append_event(g_alarm_speed?"ALARM_ON":"ALARM_OFF", g_alarm_speed?"SPEED HIGH":"Speed normal"); g_event_prev_speed=g_alarm_speed; }
    const bool ecu_now = kline_is_connected();
    const bool gps_now = gps_speed_valid();
    if(ecu_now != g_event_prev_ecu){ logger_append_event(ecu_now?"ECU_ON":"ECU_LOST", ecu_now?"ECU connected":"ECU disconnected"); g_event_prev_ecu=ecu_now; }
    if(gps_now != g_event_prev_gps){ logger_append_event(gps_now?"GPS_OK":"GPS_LOST", gps_now?"GPS speed valid":"GPS speed invalid"); g_event_prev_gps=gps_now; }
    const int active = (g_alarm_ect?1:0)+(g_alarm_batt?1:0)+(g_alarm_rpm?1:0)+(g_alarm_speed?1:0);
    if(active==0){ if(!g_alarm_clear_candidate_ms) g_alarm_clear_candidate_ms=now; }
    else g_alarm_clear_candidate_ms=0;
    if(g_page == 12){
        if(g_alarm_values[0]){ LABEL_SET_IF_CHANGED(g_alarm_values[0],g_alarm_ect?"HIGH":"OK"); set_text_color_if_changed(g_alarm_values[0],g_alarm_ect?ACCENT_ERR:ACCENT_OK); }
        if(g_alarm_values[1]){ LABEL_SET_IF_CHANGED(g_alarm_values[1],g_alarm_batt?"LOW":"OK"); set_text_color_if_changed(g_alarm_values[1],g_alarm_batt?ACCENT_ERR:ACCENT_OK); }
        if(g_alarm_values[2]){ LABEL_SET_IF_CHANGED(g_alarm_values[2],g_alarm_rpm?"HIGH":"OK"); set_text_color_if_changed(g_alarm_values[2],g_alarm_rpm?ACCENT_ERR:ACCENT_OK); }
        if(g_alarm_values[3]){ LABEL_SET_IF_CHANGED(g_alarm_values[3],g_alarm_speed?"HIGH":"OK"); set_text_color_if_changed(g_alarm_values[3],g_alarm_speed?ACCENT_WARN:ACCENT_OK); }
        if(g_alarm_values[4]){ snprintf(b,sizeof(b),"%d ACTIVE",active); LABEL_SET_IF_CHANGED(g_alarm_values[4],b); set_text_color_if_changed(g_alarm_values[4],active?ACCENT_ERR:ACCENT_OK); }
    }

    if(f_rpm > g_rpm_peak) g_rpm_peak=f_rpm;
    if(f_tps > g_tps_peak) g_tps_peak=f_tps;
    if(f_ect > g_ect_peak) g_ect_peak=f_ect;
    if(g_page == 12 && g_page12_view == 1){
        if(g_perf_values[0]){ snprintf(b,sizeof(b),"%.0f",g_speed_max); LABEL_SET_IF_CHANGED(g_perf_values[0],b); }
        if(g_perf_values[1]){ snprintf(b,sizeof(b),"%.0f",g_rpm_peak); LABEL_SET_IF_CHANGED(g_perf_values[1],b); }
        if(g_perf_values[2]){ snprintf(b,sizeof(b),"%.0f%%",g_tps_peak); LABEL_SET_IF_CHANGED(g_perf_values[2],b); }
        if(g_perf_values[3]){ snprintf(b,sizeof(b),"%.0fC",g_ect_peak); LABEL_SET_IF_CHANGED(g_perf_values[3],b); }
        if(g_perf_values[4]){ snprintf(b,sizeof(b),"%lus",(unsigned long)(g_move_has_result?g_move_result_ms/1000:0)); LABEL_SET_IF_CHANGED(g_perf_values[4],b); }
        if(g_perf_values[5]){ snprintf(b,sizeof(b),"%.2f",g_trip_km); LABEL_SET_IF_CHANGED(g_perf_values[5],b); }
    }
}

// ── Init ──────────────────────────────────────────────────────
void gauge_ui_init(){
    // สร้างหน้า Dashboard เป็น screen แยก ไม่ใช้ active screen เดิมของ LVGL
    // ป้องกันภาพหน้า 2 กระพริบระหว่างช่วงเริ่มต้นระบบ
    lv_obj_t *scr=lv_obj_create(NULL);
    g_scr_main = scr;
    lv_obj_set_style_bg_color(scr,BG_BLACK,0);

    // ══════════════════════════════════════════════════════════
    // หน้า 2: SPEED DASHBOARD — ใช้ A4SPEED50 เป็นพระเอก
    // ดีไซน์ให้เข้าชุดกับหน้า 3: กรอบรอบจอ + แถบหัวข้อไล่เฉดสี
    // + พาเนลตัวเลขความเร็วแบบมีกรอบ/เงา + กล่อง MAX/IAT มีเลขกำกับ
    // + แถบสถานะ ECU ด้านล่างแบบมีกรอบ
    // ══════════════════════════════════════════════════════════
    g_scr_black = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr_black, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr_black, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_black, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(g_scr_black, 0, 0);
    // กรอบรอบทั้งหน้าจอ ให้เข้าชุดกับหน้า 3
    // หมายเหตุ: ไม่ใส่ radius ที่ตัวจอเต็มจอ เพราะมุมโค้งบนอ็อบเจ็กต์ที่กว้าง/สูง
    // เท่าจอพอดี จะทำให้พิกเซลตรงมุมนอกส่วนโค้งไม่ถูกวาดทับ เห็นเป็นกรอบขาวเล็กๆ
    // ที่มุมจอ (หลุดออกมาจากเฟรมบัฟเฟอร์เดิม) — เอาออกเพื่อตัดปัญหานี้
    lv_obj_set_style_border_color(g_scr_black, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(g_scr_black, 2, 0);
    lv_obj_set_style_radius(g_scr_black, 0, 0);

    // ── จัดกึ่งกลางแนวนอนให้สมมาตร: ขอบซ้าย = ขอบขวา (3px เท่ากันทั้งคู่) ──
    // จอกว้าง 320px, องค์ประกอบกว้าง 314px → เหลือ 6px แบ่งข้างละ 3px พอดี
    // เปลี่ยนค่าเดียวนี้เพื่อขยับทุกองค์ประกอบในหน้านี้พร้อมกัน
    const int SCR2_DX = -2;   // ลบ = ขยับซ้าย, บวก = ขยับขวา (0 = สมมาตรกึ่งกลาง)

    // ── แถบหัวข้อไล่เฉดสีฟ้า→ม่วง "SPEED DASHBOARD" ─────────────
    lv_obj_t *g_speed_title_bar = lv_obj_create(g_scr_black);
    lv_obj_set_pos(g_speed_title_bar, 3+SCR2_DX, 3);
    lv_obj_set_size(g_speed_title_bar, 314, 22);
    lv_obj_clear_flag(g_speed_title_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_speed_title_bar, ACCENT_BLUE, 0);
    lv_obj_set_style_bg_grad_color(g_speed_title_bar, ACCENT_INJ, 0);
    lv_obj_set_style_bg_grad_dir(g_speed_title_bar, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(g_speed_title_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_speed_title_bar, 0, 0);
    lv_obj_set_style_radius(g_speed_title_bar, 5, 0);
    lv_obj_set_style_pad_all(g_speed_title_bar, 0, 0);

    mk_label(g_speed_title_bar, &A4SPEED_16, WHITE,
             "SPEED DASHBOARD", LV_ALIGN_LEFT_MID, 8, 0);
    g_lbl_speed_unit = mk_label(g_speed_title_bar, &A4SPEED_16, WHITE,
                                "--:--:--", LV_ALIGN_RIGHT_MID, -8, 0);
    g_lbl_speed_time = g_lbl_speed_unit;

    // ── พาเนลตัวเลขความเร็วหลัก มีกรอบ+เงา เหมือนหน้า 3 ──────────
    const int SPD_PANEL_Y=27, SPD_PANEL_H=147;
    lv_obj_t *g_speed_panel = lv_obj_create(g_scr_black);
    lv_obj_set_pos(g_speed_panel, 3+SCR2_DX, SPD_PANEL_Y);
    lv_obj_set_size(g_speed_panel, 314, SPD_PANEL_H);
    lv_obj_clear_flag(g_speed_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_speed_panel, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(g_speed_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_speed_panel, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(g_speed_panel, 1, 0);
    lv_obj_set_style_radius(g_speed_panel, 8, 0);
    lv_obj_set_style_pad_all(g_speed_panel, 0, 0);
    lv_obj_set_style_shadow_width(g_speed_panel, 10, 0);
    lv_obj_set_style_shadow_color(g_speed_panel, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(g_speed_panel, 60, 0);

    // ตัวเลขความเร็วขนาดใหญ่ (อยู่ในพาเนล) — ลดความสูงกล่องลงจากเดิม
    // (dseg7_60 สูงจริงแค่ 60px) เพื่อเปิดพื้นที่ด้านล่างไว้ใส่แถบ RPM + SPEED
    g_lbl_speed_page = lv_label_create(g_speed_panel);
    lv_obj_set_style_text_font(g_lbl_speed_page, &dseg7_60, 0);
    lv_obj_set_style_text_color(g_lbl_speed_page, WHITE, 0);
    lv_label_set_text(g_lbl_speed_page, "0");
    lv_obj_set_width(g_lbl_speed_page, 306);
    lv_obj_set_height(g_lbl_speed_page, 70);
    lv_obj_set_style_text_align(g_lbl_speed_page, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(g_lbl_speed_page, LV_ALIGN_TOP_MID, 0, 15);

    // ══════════════════════════════════════════════════════════
    // แถบ RPM (แถวบน) + แถบความเร็ว (แถวล่าง) อยู่ในพาเนลใต้ตัวเลข
    // แต่ละแถวมีป้ายข้อความ "RPM" / "SPD" อยู่หน้าแถบให้ชัดเจน
    // ── จัดให้สมมาตร: ระยะขอบซ้าย-ขวาของทั้งแถว (ป้าย+บาร์) เท่ากัน ──
    //   ป้าย 44px + ช่องไฟ 6px + บาร์ 16 ช่อง (กว้าง10 ห่าง3) = 205px
    //   รวมทั้งแถว = 44+6+205 = 255 → เหลือ 314-255=59 แบ่งข้างละ ~29
    // ══════════════════════════════════════════════════════════
    const int BAR_LBL_W=44, BAR_LBL_GAP=6;
    const int BAR_N=16, BAR_W=10, BAR_G=3, BAR_H=18;
    const int BAR_SPAN = BAR_N*BAR_W + (BAR_N-1)*BAR_G;         // 205
    const int BAR_ROW_SPAN = BAR_LBL_W + BAR_LBL_GAP + BAR_SPAN; // 255
    const int BAR_ROW_X = (314 - BAR_ROW_SPAN) / 2;              // 29 (สมมาตรซ้าย=ขวา)
    const int BAR_X = BAR_ROW_X + BAR_LBL_W + BAR_LBL_GAP;       // จุดเริ่มบาร์ (หลังป้าย)
    const int RPM_ROW_Y = 89, SPD_ROW_Y = 111;   // ระยะห่างแถวบน/ล่างเท่ากัน (6px)

    // ── แถว RPM (บน) ──────────────────────────────────────────
    g_lbl_rpm_bar2_tag = lv_label_create(g_speed_panel);
    lv_obj_set_style_text_font(g_lbl_rpm_bar2_tag, &A4SPEED_14, 0);
    lv_obj_set_style_text_color(g_lbl_rpm_bar2_tag, WHITE, 0);
    lv_label_set_text(g_lbl_rpm_bar2_tag, "RPM");
    lv_obj_set_pos(g_lbl_rpm_bar2_tag, BAR_ROW_X, RPM_ROW_Y);
    lv_obj_set_size(g_lbl_rpm_bar2_tag, BAR_LBL_W, BAR_H);
    lv_obj_set_style_text_align(g_lbl_rpm_bar2_tag, LV_TEXT_ALIGN_LEFT, 0);
    for(int i=0;i<BAR_N;i++){
        lv_obj_t *b=lv_obj_create(g_speed_panel);
        lv_obj_set_pos(b, BAR_X+i*(BAR_W+BAR_G), RPM_ROW_Y+1);
        lv_obj_set_size(b, BAR_W, BAR_H-2);
        lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(b, BORDER_SUBTLE, 0);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_set_style_radius(b, 2, 0);
        // ใช้บาร์ตรงแบบช่องเล็ก ๆ เพื่อให้แสดงผลเสถียรบน LVGL 8.3
        // ความสมูทจะทำที่ค่า f_rpm ก่อนเข้าชุดบาร์แทนการหมุนวัตถุ
        g_rpm_bar2[i]=b;
    }

    // ── แถว SPEED (ล่าง) ──────────────────────────────────────
    g_lbl_spd_bar2_tag = lv_label_create(g_speed_panel);
    lv_obj_set_style_text_font(g_lbl_spd_bar2_tag, &A4SPEED_14, 0);
    lv_obj_set_style_text_color(g_lbl_spd_bar2_tag, WHITE, 0);
    lv_label_set_text(g_lbl_spd_bar2_tag, "SPD");
    lv_obj_set_pos(g_lbl_spd_bar2_tag, BAR_ROW_X, SPD_ROW_Y);
    lv_obj_set_size(g_lbl_spd_bar2_tag, BAR_LBL_W, BAR_H);
    lv_obj_set_style_text_align(g_lbl_spd_bar2_tag, LV_TEXT_ALIGN_LEFT, 0);
    for(int i=0;i<BAR_N;i++){
        lv_obj_t *b=lv_obj_create(g_speed_panel);
        lv_obj_set_pos(b, BAR_X+i*(BAR_W+BAR_G), SPD_ROW_Y+1);
        lv_obj_set_size(b, BAR_W, BAR_H-2);
        lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(b, BORDER_SUBTLE, 0);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_set_style_radius(b, 2, 0);
        // ใช้บาร์ตรงแบบช่องเล็ก ๆ เพื่อให้แสดงผลเสถียรบน LVGL 8.3
        // ความสมูทจะทำที่ค่า f_speed ก่อนเข้าชุดบาร์แทนการหมุนวัตถุ
        g_spd_bar2[i]=b;
    }

    // ── กล่อง MAX / IAT / GPS / TRIP ───────────────────────────
    // ตัด badge หมายเลข 1/2/3 ออกตามที่ต้องการ และเพิ่ม TRIP
    // 3 กล่อง * 100px + 2 ช่องไฟ * 7px = 314px พอดี เท่ากับความกว้างแถบหัวข้อ/
    // พาเนลความเร็ว/แถบสถานะด้านบน-ล่าง ทำให้ขอบซ้าย-ขวาของแถวกล่องนี้ตรงกับ
    // องค์ประกอบอื่นทั้งหมดในหน้า (ของเดิม 101px*3+5px*2=313 เหลื่อมไป 1px)
    const int SBOX_Y=SPD_PANEL_Y+SPD_PANEL_H+4, SBOX_H=30, SBOX_W=100, SBOX_GAP=7;

    auto make_speed_box = [&](int x, const char *caption, lv_color_t border_col, lv_obj_t **out_value)->lv_obj_t* {
        lv_obj_t *box = lv_obj_create(g_scr_black);
        lv_obj_set_pos(box, x, SBOX_Y);
        lv_obj_set_size(box, SBOX_W, SBOX_H);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(box, PANEL_CARD, 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(box, border_col, 0);
        lv_obj_set_style_border_width(box, 1, 0);
        lv_obj_set_style_radius(box, 6, 0);
        lv_obj_set_style_pad_all(box, 0, 0);

        lv_obj_t *val = lv_label_create(box);
        lv_obj_set_style_text_font(val, &A4SPEED_16, 0);
        lv_obj_set_style_text_color(val, WHITE, 0);
        lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_long_mode(val, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(val, SBOX_W-7);
        lv_obj_set_height(val, 18);
        lv_obj_align(val, LV_ALIGN_TOP_RIGHT, -4, 3);
        lv_label_set_text(val, "0");
        if(out_value) *out_value = val;

        lv_obj_t *cap = lv_label_create(box);
        lv_obj_set_style_text_font(cap, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(cap, GRAY_LBL, 0);
        lv_obj_set_style_text_align(cap, LV_TEXT_ALIGN_LEFT, 0);
        lv_label_set_long_mode(cap, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(cap, SBOX_W-8);
        lv_obj_set_height(cap, 12);
        lv_obj_align(cap, LV_ALIGN_BOTTOM_LEFT, 4, -2);
        lv_label_set_text(cap, caption);
        return box;
    };

    make_speed_box(3+SCR2_DX, "MAX", ACCENT_WARN, &g_lbl_speed_max);
    make_speed_box(3+SCR2_DX+(SBOX_W+SBOX_GAP), "GPS", ACCENT_BLUE, &g_lbl_gps_speed);
    make_speed_box(3+SCR2_DX+2*(SBOX_W+SBOX_GAP), "TRIP km", ACCENT_OK, &g_lbl_trip);

    // ── แถบสถานะ GPS + เวลา + ปุ่ม RESET TRIP ─────────────────────
    const int SBAR_Y=SBOX_Y+SBOX_H+1, SBAR_H=26;
    lv_obj_t *g_status_bar = lv_obj_create(g_scr_black);
    lv_obj_set_pos(g_status_bar, 3+SCR2_DX, SBAR_Y);
    lv_obj_set_size(g_status_bar, 314, SBAR_H);
    lv_obj_clear_flag(g_status_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_status_bar, BTN_NEUTRAL, 0);
    lv_obj_set_style_bg_opa(g_status_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_status_bar, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(g_status_bar, 1, 0);
    lv_obj_set_style_radius(g_status_bar, 6, 0);
    lv_obj_set_style_pad_all(g_status_bar, 0, 0);

    // จุดสถานะ (สี/กระพริบบอกว่า GPS ล็อกหรือยัง) ย้ายมาไว้ด้านหน้า
    // ข้อความแทน (เดิมอยู่ต่อท้ายคำ)
    g_speed_link_dot = lv_obj_create(g_status_bar);
    lv_obj_set_size(g_speed_link_dot, 7, 7);
    lv_obj_clear_flag(g_speed_link_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(g_speed_link_dot, 4, 0);
    lv_obj_set_style_border_width(g_speed_link_dot, 0, 0);
    lv_obj_set_style_bg_color(g_speed_link_dot, ACCENT_WARN, 0);
    lv_obj_set_style_bg_opa(g_speed_link_dot, LV_OPA_COVER, 0);
    lv_obj_align(g_speed_link_dot, LV_ALIGN_LEFT_MID, 7, 0);

    g_lbl_speed_status = mk_label(g_status_bar, &A4SPEED_14, ACCENT_OK,
                                  "SEARCHING GPS", LV_ALIGN_LEFT_MID, 7, 0);
    // ทำให้กรอบของ label เท่ากับความกว้างข้อความจริง
    lv_obj_set_width(g_lbl_speed_status, LV_SIZE_CONTENT);
    // ให้ข้อความตามหลังจุดสถานะ (จุดอยู่หน้า ข้อความอยู่หลัง)
    lv_obj_align_to(g_lbl_speed_status, g_speed_link_dot, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    // เวลา GPS ย้ายไปมุมขวาบนของแถบหัวข้อแล้ว
    // ตำแหน่งเดิมในแถบสถานะถูกเว้นไว้เพื่อไม่ให้แสดงซ้ำ

    g_btn_reset_trip = lv_obj_create(g_status_bar);
    lv_obj_set_pos(g_btn_reset_trip, 230, 0);
    lv_obj_set_size(g_btn_reset_trip, 79, 24);
    lv_obj_add_flag(g_btn_reset_trip, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(g_btn_reset_trip, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_btn_reset_trip, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(g_btn_reset_trip, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_btn_reset_trip, ACCENT_WARN, 0);
    lv_obj_set_style_border_width(g_btn_reset_trip, 1, 0);
    lv_obj_set_style_radius(g_btn_reset_trip, 5, 0);
    lv_obj_t *reset_lbl = lv_label_create(g_btn_reset_trip);
    lv_obj_set_style_text_font(reset_lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(reset_lbl, ACCENT_WARN, 0);
    lv_label_set_text(reset_lbl, "RESET TRIP");
    lv_obj_center(reset_lbl);
    // Touch on PAGE 2 is handled by the dedicated XPT2046 release handler.
    // Do not register a second LVGL CLICKED handler here; two independent
    // paths can consume the same release and make the dialog unreliable.

    // ── ยืนยัน RESET TRIP ก่อนล้างระยะทาง ─────────────────────────
    // Modal อยู่บนหน้า 2 และกินพื้นที่เฉพาะ dialog เพื่อไม่ให้กดผ่านไปยังปุ่มอื่น
    g_trip_confirm_overlay = lv_obj_create(g_scr_black);
    lv_obj_set_pos(g_trip_confirm_overlay, 20, 62);
    lv_obj_set_size(g_trip_confirm_overlay, 280, 116);
    lv_obj_clear_flag(g_trip_confirm_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_trip_confirm_overlay, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(g_trip_confirm_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_trip_confirm_overlay, ACCENT_WARN, 0);
    lv_obj_set_style_border_width(g_trip_confirm_overlay, 2, 0);
    lv_obj_set_style_radius(g_trip_confirm_overlay, 10, 0);
    lv_obj_set_style_pad_all(g_trip_confirm_overlay, 6, 0);
    lv_obj_set_style_shadow_width(g_trip_confirm_overlay, 18, 0);
    lv_obj_set_style_shadow_color(g_trip_confirm_overlay, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(g_trip_confirm_overlay, 140, 0);
    lv_obj_add_flag(g_trip_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_trip_confirm_overlay, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *trip_confirm_title = lv_label_create(g_trip_confirm_overlay);
    lv_obj_set_style_text_font(trip_confirm_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(trip_confirm_title, ACCENT_WARN, 0);
    lv_label_set_text(trip_confirm_title, "RESET TRIP?");
    lv_obj_set_width(trip_confirm_title, 260);
    lv_obj_set_style_text_align(trip_confirm_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(trip_confirm_title, LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t *trip_confirm_msg = lv_label_create(g_trip_confirm_overlay);
    lv_obj_set_style_text_font(trip_confirm_msg, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(trip_confirm_msg, GRAY_LBL, 0);
    lv_label_set_text(trip_confirm_msg, "Reset trip distance to 0.00 km?");
    lv_obj_set_width(trip_confirm_msg, 260);
    lv_obj_set_style_text_align(trip_confirm_msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(trip_confirm_msg, LV_ALIGN_TOP_MID, 0, 34);

    lv_obj_t *trip_cancel_btn = mk_menu_btn(g_trip_confirm_overlay, 0, 55, 125, 40,
                                            "CANCEL", BTN_NEUTRAL, page2_trip_confirm_no_cb);
    lv_obj_t *trip_yes_btn = mk_menu_btn(g_trip_confirm_overlay, 140, 55, 125, 40,
                                         "YES", ACCENT_WARN, page2_trip_confirm_yes_cb);
    lv_obj_set_style_border_width(trip_cancel_btn, 1, 0);
    lv_obj_set_style_border_width(trip_yes_btn, 2, 0);
    lv_obj_set_style_border_color(trip_cancel_btn, lv_color_hex(0x6b7280), 0);
    lv_obj_set_style_border_color(trip_yes_btn, lv_color_hex(0xffd166), 0);

    // ══════════════════════════════════════════════════════════
    // หน้า 3: กราฟเปรียบเทียบ RPM / SPEED แบบเรียลไทม์
    // ดีไซน์อ้างอิงหน้าจอเครื่องมือวัดสไตล์ "Real Time Chart":
    // แถบหัวข้อไล่เฉดสีชมพู-ม่วง + กล่องค่าปัจจุบัน 2 กล่อง (มีเลขกำกับ)
    // + กราฟกรอบชัดเจน + แถวเคอร์เซอร์เวลา + แถบเมนูล่าง 4 ช่อง
    // ใช้ chart ซ้อน 2 ชั้น เพื่อให้แต่ละเส้นมีสเกล Y ของตัวเอง
    // ══════════════════════════════════════════════════════════
    g_scr_graph = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr_graph, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr_graph, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_graph, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(g_scr_graph, 0, 0);
    // กรอบรอบทั้งหน้าจอ ให้ความรู้สึกเหมือนกรอบเครื่องมือวัดจริง
    // หมายเหตุ: ไม่ใส่ radius ที่ตัวจอเต็มจอ (เหตุผลเดียวกับหน้า 2) — มุมโค้ง
    // บนอ็อบเจ็กต์ที่กว้าง/สูงเท่าจอพอดี ทำให้เห็นกรอบขาวเล็กๆหลุดที่มุมจอ
    lv_obj_set_style_border_color(g_scr_graph, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(g_scr_graph, 2, 0);
    lv_obj_set_style_radius(g_scr_graph, 0, 0);

    // ── แถบหัวข้อไล่เฉดสีชมพู→ม่วง "REAL TIME CHART" ─────────────
    lv_obj_t *g_title_bar = lv_obj_create(g_scr_graph);
    lv_obj_set_pos(g_title_bar, 3+SCR2_DX, 3);
    lv_obj_set_size(g_title_bar, 314, 22);
    lv_obj_clear_flag(g_title_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_title_bar, ACCENT_MAGENTA, 0);
    lv_obj_set_style_bg_grad_color(g_title_bar, ACCENT_PURPLE, 0);
    lv_obj_set_style_bg_grad_dir(g_title_bar, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(g_title_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_title_bar, 0, 0);
    lv_obj_set_style_radius(g_title_bar, 5, 0);
    lv_obj_set_style_pad_all(g_title_bar, 0, 0);

    mk_label(g_title_bar, &A4SPEED_16, WHITE,
             "REAL TIME CHART", LV_ALIGN_LEFT_MID, 8, 0);
    g_lbl_graph_status = mk_label(g_title_bar, &lv_font_montserrat_14, WHITE,
                                  "LIVE", LV_ALIGN_RIGHT_MID, -8, 0);

    // ── กล่องค่าปัจจุบัน 2 กล่อง: 1=TACHO(RPM) สีฟ้า, 2=SPEED สีเขียว ──
    const int BOX_Y=29, BOX_H=26, BOX_W=154;
    lv_obj_t *g_box_rpm = lv_obj_create(g_scr_graph);
    lv_obj_set_pos(g_box_rpm, 3+SCR2_DX, BOX_Y);
    lv_obj_set_size(g_box_rpm, BOX_W, BOX_H);
    lv_obj_clear_flag(g_box_rpm, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_box_rpm, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(g_box_rpm, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_box_rpm, ACCENT_BLUE, 0);
    lv_obj_set_style_border_width(g_box_rpm, 1, 0);
    lv_obj_set_style_radius(g_box_rpm, 6, 0);
    lv_obj_set_style_pad_all(g_box_rpm, 0, 0);

    lv_obj_t *g_badge_rpm = lv_obj_create(g_box_rpm);
    lv_obj_set_size(g_badge_rpm, 18, 18);
    lv_obj_align(g_badge_rpm, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_clear_flag(g_badge_rpm, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_badge_rpm, ACCENT_BLUE, 0);
    lv_obj_set_style_bg_opa(g_badge_rpm, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_badge_rpm, 0, 0);
    lv_obj_set_style_radius(g_badge_rpm, 9, 0);
    mk_label(g_badge_rpm, &A4SPEED_14, BG_BLACK, "1", LV_ALIGN_CENTER, 0, 0);
    mk_label(g_box_rpm, &A4SPEED_14, WHITE, "", LV_ALIGN_LEFT_MID, 27, 0);
    g_lbl_graph_rpm = mk_label(g_box_rpm, &A4SPEED_16, ACCENT_BLUE,
                               "0 RPM", LV_ALIGN_RIGHT_MID, -8, 0);

    lv_obj_t *g_box_speed = lv_obj_create(g_scr_graph);
    lv_obj_set_pos(g_box_speed, 3+SCR2_DX+BOX_W+4, BOX_Y);
    lv_obj_set_size(g_box_speed, BOX_W, BOX_H);
    lv_obj_clear_flag(g_box_speed, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_box_speed, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(g_box_speed, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_box_speed, ACCENT_OK, 0);
    lv_obj_set_style_border_width(g_box_speed, 1, 0);
    lv_obj_set_style_radius(g_box_speed, 6, 0);
    lv_obj_set_style_pad_all(g_box_speed, 0, 0);

    lv_obj_t *g_badge_speed = lv_obj_create(g_box_speed);
    lv_obj_set_size(g_badge_speed, 18, 18);
    lv_obj_align(g_badge_speed, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_clear_flag(g_badge_speed, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_badge_speed, ACCENT_OK, 0);
    lv_obj_set_style_bg_opa(g_badge_speed, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_badge_speed, 0, 0);
    lv_obj_set_style_radius(g_badge_speed, 9, 0);
    mk_label(g_badge_speed, &A4SPEED_14, BG_BLACK, "2", LV_ALIGN_CENTER, 0, 0);
    mk_label(g_box_speed, &A4SPEED_14, WHITE, "", LV_ALIGN_LEFT_MID, 27, 0);
    g_lbl_graph_speed = mk_label(g_box_speed, &A4SPEED_16, ACCENT_OK,
                                 "0 KM/H", LV_ALIGN_RIGHT_MID, -8, 0);

    // ── กรอบพาเนลของกราฟ ให้ดูเป็นจอวัดจริง มีขอบและเงาเบาๆ ──────
    const int FRAME_X=3+SCR2_DX, FRAME_Y=59, FRAME_W=314, FRAME_H=154;
    lv_obj_t *g_graph_frame = lv_obj_create(g_scr_graph);
    lv_obj_set_pos(g_graph_frame, FRAME_X, FRAME_Y);
    lv_obj_set_size(g_graph_frame, FRAME_W, FRAME_H);
    lv_obj_clear_flag(g_graph_frame, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_graph_frame, PANEL_CARD, 0);
    lv_obj_set_style_bg_opa(g_graph_frame, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_graph_frame, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(g_graph_frame, 1, 0);
    lv_obj_set_style_radius(g_graph_frame, 8, 0);
    lv_obj_set_style_pad_all(g_graph_frame, 0, 0);
    lv_obj_set_style_shadow_width(g_graph_frame, 10, 0);
    lv_obj_set_style_shadow_color(g_graph_frame, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(g_graph_frame, 60, 0);

    // เว้นพื้นที่ซ้าย/ขวาสำหรับตัวเลขสเกล เพื่อไม่ให้ label ชนกับเส้นกราฟ
    const int GX=FRAME_X+32, GY=FRAME_Y+6, GW=FRAME_W-64, GH=FRAME_H-14;
    g_graph_rpm = lv_chart_create(g_scr_graph);
    lv_obj_set_pos(g_graph_rpm, GX, GY);
    lv_obj_set_size(g_graph_rpm, GW, GH);
    lv_chart_set_type(g_graph_rpm, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(g_graph_rpm, 60);
    lv_chart_set_range(g_graph_rpm, LV_CHART_AXIS_PRIMARY_Y, 0, 10000);
    lv_chart_set_div_line_count(g_graph_rpm, 5, 6);
    lv_obj_set_style_bg_color(g_graph_rpm, PANEL_DARK, 0);
    lv_obj_set_style_bg_opa(g_graph_rpm, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_graph_rpm, GRAPH_GRID, 0);
    lv_obj_set_style_border_width(g_graph_rpm, 1, 0);
    lv_obj_set_style_radius(g_graph_rpm, 4, 0);
    lv_obj_set_style_line_color(g_graph_rpm, GRAPH_GRID, LV_PART_MAIN);
    lv_obj_set_style_line_width(g_graph_rpm, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_graph_rpm, 2, 0);
    // ── เส้นกราฟ RPM: เส้นเรียบต่อเนื่อง ไม่มีจุดกลมคั่นแบบในภาพตัวอย่าง ──
    lv_obj_set_style_line_width(g_graph_rpm, 2, LV_PART_ITEMS);
    lv_obj_set_style_line_rounded(g_graph_rpm, true, LV_PART_ITEMS);
    lv_obj_set_style_size(g_graph_rpm, 0, LV_PART_INDICATOR);   // ซ่อนจุดมาร์กเกอร์
    g_series_rpm = lv_chart_add_series(g_graph_rpm, ACCENT_BLUE, LV_CHART_AXIS_PRIMARY_Y);

    // จุดเริ่มต้นของกราฟจะเป็น 0 จนกว่าจะได้รับข้อมูลจริง
    lv_chart_refresh(g_graph_rpm);

    g_graph_speed = lv_chart_create(g_scr_graph);
    lv_obj_set_pos(g_graph_speed, GX, GY);
    lv_obj_set_size(g_graph_speed, GW, GH);
    lv_chart_set_type(g_graph_speed, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(g_graph_speed, 60);
    lv_chart_set_range(g_graph_speed, LV_CHART_AXIS_PRIMARY_Y, 0, 160);
    lv_chart_set_div_line_count(g_graph_speed, 0, 0);
    lv_obj_set_style_bg_opa(g_graph_speed, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_graph_speed, 0, 0);
    lv_obj_set_style_pad_all(g_graph_speed, 2, 0);
    // ── เส้นกราฟ SPEED: เส้นเรียบต่อเนื่องเช่นกัน ไม่มีจุดมาร์กเกอร์ ────
    lv_obj_set_style_line_width(g_graph_speed, 2, LV_PART_ITEMS);
    lv_obj_set_style_line_rounded(g_graph_speed, true, LV_PART_ITEMS);
    lv_obj_set_style_size(g_graph_speed, 0, LV_PART_INDICATOR);  // ซ่อนจุดมาร์กเกอร์
    g_series_speed = lv_chart_add_series(g_graph_speed, ACCENT_OK, LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_refresh(g_graph_speed);

    // ตัวเลขกำกับตาราง: แกนซ้าย = RPM, แกนขวา = SPEED
    // ใช้ 6 ระดับที่มีช่วงเท่ากัน เพื่ออ่านค่าได้ง่ายและละเอียดขึ้น
    const int gy0 = GY + GH - 8;
    const int gy1 = GY + 4*GH/5 - 8;
    const int gy2 = GY + 3*GH/5 - 8;
    const int gy3 = GY + 2*GH/5 - 8;
    const int gy4 = GY + GH/5 - 8;
    const int gy5 = GY - 3;

    // จัด label ให้อยู่กึ่งกลางเส้นกริดและอ่านง่ายกว่าค่าชุดเดิม
    const int LX=FRAME_X+3, RX=3;
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_BLUE, "0",      LV_ALIGN_TOP_LEFT,  LX, gy0);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_BLUE, "2k",  LV_ALIGN_TOP_LEFT,  LX, gy1);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_BLUE, "4k",  LV_ALIGN_TOP_LEFT,  LX, gy2);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_BLUE, "6k",  LV_ALIGN_TOP_LEFT,  LX, gy3);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_BLUE, "8k",  LV_ALIGN_TOP_LEFT,  LX, gy4);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_BLUE, "10k", LV_ALIGN_TOP_LEFT,  LX, gy5);

    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_OK, "0",     LV_ALIGN_TOP_RIGHT, -RX+SCR2_DX, gy0);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_OK, "32",    LV_ALIGN_TOP_RIGHT, -RX+SCR2_DX, gy1);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_OK, "64",    LV_ALIGN_TOP_RIGHT, -RX+SCR2_DX, gy2);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_OK, "96",    LV_ALIGN_TOP_RIGHT, -RX+SCR2_DX, gy3);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_OK, "128",   LV_ALIGN_TOP_RIGHT, -RX+SCR2_DX, gy4);
    mk_label(g_scr_graph, &lv_font_montserrat_12, ACCENT_OK, "160",   LV_ALIGN_TOP_RIGHT, -RX+SCR2_DX, gy5);

    // 60 จุดที่เก็บทุก 100 ms = หน้าต่างข้อมูลประมาณ 6 วินาที
    mk_label(g_scr_graph, &lv_font_montserrat_10, GRAY_LBL, "- ~6 s ago",
             LV_ALIGN_TOP_LEFT, GX+2, FRAME_Y+FRAME_H-13);
    mk_label(g_scr_graph, &lv_font_montserrat_10, GRAY_LBL, "NOW -",
             LV_ALIGN_TOP_RIGHT, -19+SCR2_DX, FRAME_Y+FRAME_H-13);

    // ── แทนที่แถว Cursor / Gauge ด้านล่างด้วยตัวจับเวลา "เคลื่อนที่จนหยุด" ──
    // ลบข้อความเดิม (-3.0 / Cursor / 0.0 Sec.) และแถบ GaugeNo./TimeRec./Run,Stop/Cursor
    // แล้วใช้พื้นที่ด้านล่างแสดงเวลาที่รถเคลื่อนที่จนหยุด พร้อมรอบเครื่อง ณ ขณะหยุด
    const int CUR_Y = FRAME_Y+FRAME_H+4;
    g_lbl_graph_gps_status = mk_label(g_scr_graph, &A4SPEED_16, GRAY_LBL,
                                      "READY", LV_ALIGN_TOP_MID, 0+SCR2_DX, CUR_Y+0);

    // ══════════════════════════════════════════════════════════
    //  หน้า 6: LIVE GRAPH — RPM / AFR
    //  ค่าทั้ง 2 เป็นสเกลแนวตั้ง แยกซ้าย/ขวา และเส้นกราฟวิ่งแนวนอน
    // ══════════════════════════════════════════════════════════
    g_scr_afr_rpm = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr_afr_rpm, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr_afr_rpm, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_afr_rpm, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_scr_afr_rpm, 0, 0);
    lv_obj_set_style_pad_all(g_scr_afr_rpm, 0, 0);

    // ── แถบหัวข้อหน้า 6: ไล่เฉดสีส้ม→เหลือง ให้สื่อถึงการวิเคราะห์การเผาไหม้ ──
    lv_obj_t *afr_title = lv_obj_create(g_scr_afr_rpm);
    lv_obj_set_pos(afr_title, 3, 3);
    lv_obj_set_size(afr_title, 314, 24);
    lv_obj_clear_flag(afr_title, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(afr_title, lv_color_hex(0xc0392b), 0);      // แดงเข้ม
    lv_obj_set_style_bg_grad_color(afr_title, lv_color_hex(0xf39c12), 0); // ส้มทอง
    lv_obj_set_style_bg_grad_dir(afr_title, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(afr_title, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(afr_title, 0, 0);
    lv_obj_set_style_radius(afr_title, 5, 0);
    lv_obj_set_style_pad_all(afr_title, 0, 0);
    mk_label(afr_title, &A4SPEED_16, WHITE, "RPM / AFR LIVE GRAPH", LV_ALIGN_LEFT_MID, 8, 0);
    g_lbl_afr_rpm_now = mk_label(afr_title, &A4SPEED_14, WHITE, "LIVE", LV_ALIGN_RIGHT_MID, -8, 0);

    // แถบค่าด้านบน: แยก RPM / AFR เป็นคนละช่อง เพื่อให้อ่านค่าได้ทันที
    lv_obj_t *rpm_box = lv_obj_create(g_scr_afr_rpm);
    lv_obj_set_pos(rpm_box, 3, 30);
    lv_obj_set_size(rpm_box, 154, 32);
    lv_obj_clear_flag(rpm_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(rpm_box, PANEL_CARD, 0);
    lv_obj_set_style_border_color(rpm_box, ACCENT_BLUE, 0);
    lv_obj_set_style_border_width(rpm_box, 1, 0);
    lv_obj_set_style_radius(rpm_box, 6, 0);
    lv_obj_set_style_pad_all(rpm_box, 0, 0);
    mk_label(rpm_box, &A4SPEED_14, ACCENT_BLUE, "RPM", LV_ALIGN_LEFT_MID, 7, 0);
    g_lbl_rpm_value = mk_label(rpm_box, &A4SPEED_16, ACCENT_BLUE, "0",
                               LV_ALIGN_RIGHT_MID, -7, 0);

    lv_obj_t *afr_box = lv_obj_create(g_scr_afr_rpm);
    lv_obj_set_pos(afr_box, 163, 30);
    lv_obj_set_size(afr_box, 154, 32);
    lv_obj_clear_flag(afr_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(afr_box, PANEL_CARD, 0);
    lv_obj_set_style_border_color(afr_box, ACCENT_WARN, 0);
    lv_obj_set_style_border_width(afr_box, 1, 0);
    lv_obj_set_style_radius(afr_box, 6, 0);
    lv_obj_set_style_pad_all(afr_box, 0, 0);
    mk_label(afr_box, &A4SPEED_14, ACCENT_WARN, "AFR", LV_ALIGN_LEFT_MID, 7, 0);
    g_lbl_afr_value = mk_label(afr_box, &A4SPEED_16, ACCENT_WARN, "0.0",
                               LV_ALIGN_RIGHT_MID, -7, 0);
    g_lbl_afr_rpm_value = g_lbl_afr_value;

    // ── ขนาดพื้นที่กราฟ: คำนวณครั้งเดียว ใช้ทั้ง init และ update ──────
    // AF_X/AF_Y คือตำแหน่ง frame, PLOT_* คือพื้นที่วาดเส้นจริง
    // (เว้นซ้าย 34px สำหรับ label แกน Y, ขวา 34px, บน 9px, ล่าง 10px)
    const int AF_X = 3, AF_Y = 65, AF_W = 314, AF_H = 151;
    const int AFR_PLOT_X = AF_X + 34;
    const int AFR_PLOT_Y = AF_Y + 9;
    const int AFR_PLOT_W = AF_W - 68;   // 246
    const int AFR_PLOT_H = AF_H - 19;   // 132

    // กรอบนอกชั้นที่ 1: เพิ่มเส้นอีกชั้นเพื่อให้กรอบกราฟหนาและชัดขึ้น
    lv_obj_t *afr_frame_outer = lv_obj_create(g_scr_afr_rpm);
    lv_obj_set_pos(afr_frame_outer, AF_X-2, AF_Y-2);
    lv_obj_set_size(afr_frame_outer, AF_W+4, AF_H+4);
    lv_obj_clear_flag(afr_frame_outer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(afr_frame_outer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(afr_frame_outer, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(afr_frame_outer, 1, 0);
    lv_obj_set_style_radius(afr_frame_outer, 10, 0);
    lv_obj_set_style_pad_all(afr_frame_outer, 0, 0);

    // กรอบชั้นในเดิม
    lv_obj_t *afr_frame = lv_obj_create(g_scr_afr_rpm);
    lv_obj_set_pos(afr_frame, AF_X, AF_Y);
    lv_obj_set_size(afr_frame, AF_W, AF_H);
    lv_obj_clear_flag(afr_frame, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(afr_frame, PANEL_CARD, 0);
    lv_obj_set_style_border_color(afr_frame, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(afr_frame, 1, 0);
    lv_obj_set_style_radius(afr_frame, 8, 0);
    lv_obj_set_style_shadow_width(afr_frame, 10, 0);
    lv_obj_set_style_shadow_color(afr_frame, BG_BLACK, 0);
    lv_obj_set_style_shadow_opa(afr_frame, 60, 0);
    lv_obj_set_style_pad_all(afr_frame, 0, 0);

    // พื้นหลัง plot area สีเข้มกว่า frame เล็กน้อย
    lv_obj_t *plot_bg = lv_obj_create(g_scr_afr_rpm);
    lv_obj_set_pos(plot_bg, AFR_PLOT_X, AFR_PLOT_Y);
    lv_obj_set_size(plot_bg, AFR_PLOT_W, AFR_PLOT_H);
    lv_obj_clear_flag(plot_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(plot_bg, PANEL_DARK, 0);
    lv_obj_set_style_bg_opa(plot_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(plot_bg, GRAPH_GRID, 0);
    lv_obj_set_style_border_width(plot_bg, 1, 0);
    lv_obj_set_style_radius(plot_bg, 3, 0);
    lv_obj_set_style_pad_all(plot_bg, 0, 0);

    // เส้นกริดแนวนอน 5 เส้น (0%, 25%, 50%, 75%, 100% ของ plot height)
    for(int i=0;i<5;i++){
        int yy = AFR_PLOT_Y + (AFR_PLOT_H-1) - (AFR_PLOT_H-1)*i/4;
        lv_obj_t *hline = lv_obj_create(g_scr_afr_rpm);
        lv_obj_set_pos(hline, AFR_PLOT_X, yy);
        lv_obj_set_size(hline, AFR_PLOT_W, 1);
        lv_obj_clear_flag(hline, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(hline, GRAPH_GRID, 0);
        lv_obj_set_style_bg_opa(hline, (i==0||i==4)?LV_OPA_COVER:LV_OPA_50, 0);
        lv_obj_set_style_border_width(hline, 0, 0);
        lv_obj_set_style_pad_all(hline, 0, 0);
    }

    // เส้นกริดแนวตั้ง 7 เส้น (แบ่ง timeline เท่าๆ กัน)
    for(int i=0;i<7;i++){
        int xx = AFR_PLOT_X + (AFR_PLOT_W-1)*i/6;
        lv_obj_t *vline = lv_obj_create(g_scr_afr_rpm);
        lv_obj_set_pos(vline, xx, AFR_PLOT_Y);
        lv_obj_set_size(vline, 1, AFR_PLOT_H);
        lv_obj_clear_flag(vline, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(vline, GRAPH_GRID, 0);
        lv_obj_set_style_bg_opa(vline, LV_OPA_40, 0);
        lv_obj_set_style_border_width(vline, 0, 0);
        lv_obj_set_style_pad_all(vline, 0, 0);
    }

    // เส้นอ้างอิง stoichiometric AFR=14.7 (สีเขียวบางๆ บน AFR axis)
    {
        const float AFR_MIN_G = 12.0f, AFR_MAX_G = 18.0f;
        int stoich_y = AFR_PLOT_Y + (int)roundf(((AFR_MAX_G - 14.7f)/(AFR_MAX_G - AFR_MIN_G)) * (AFR_PLOT_H - 1));
        lv_obj_t *stoich_line = lv_obj_create(g_scr_afr_rpm);
        lv_obj_set_pos(stoich_line, AFR_PLOT_X, stoich_y);
        lv_obj_set_size(stoich_line, AFR_PLOT_W, 1);
        lv_obj_clear_flag(stoich_line, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(stoich_line, ACCENT_OK, 0);
        lv_obj_set_style_bg_opa(stoich_line, LV_OPA_60, 0);
        lv_obj_set_style_border_width(stoich_line, 0, 0);
        lv_obj_set_style_pad_all(stoich_line, 0, 0);
    }

    // ── แกนซ้าย = RPM (สีฟ้า) ──────────────────────────────────
    const int AXIS_Y_OFF = -5;   // offset ให้ตัวเลขอยู่กึ่งกลางเส้น
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_BLUE, "10k",
             LV_ALIGN_TOP_LEFT, AF_X+2, AFR_PLOT_Y + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_BLUE, "7.5k",
             LV_ALIGN_TOP_LEFT, AF_X+2, AFR_PLOT_Y + AFR_PLOT_H/4 + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_BLUE, "5k",
             LV_ALIGN_TOP_LEFT, AF_X+2, AFR_PLOT_Y + AFR_PLOT_H/2 + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_BLUE, "2.5k",
             LV_ALIGN_TOP_LEFT, AF_X+2, AFR_PLOT_Y + 3*AFR_PLOT_H/4 + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_BLUE, "0",
             LV_ALIGN_TOP_LEFT, AF_X+2, AFR_PLOT_Y + AFR_PLOT_H + AXIS_Y_OFF);

    // ── แกนขวา = AFR (สีเหลืองอำพัน) ───────────────────────────
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_WARN, "18",
             LV_ALIGN_TOP_RIGHT, -(AF_X+2), AFR_PLOT_Y + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_WARN, "16.5",
             LV_ALIGN_TOP_RIGHT, -(AF_X+2), AFR_PLOT_Y + AFR_PLOT_H/4 + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_WARN, "15",
             LV_ALIGN_TOP_RIGHT, -(AF_X+2), AFR_PLOT_Y + AFR_PLOT_H/2 + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_WARN, "13.5",
             LV_ALIGN_TOP_RIGHT, -(AF_X+2), AFR_PLOT_Y + 3*AFR_PLOT_H/4 + AXIS_Y_OFF);
    mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_WARN, "12",
             LV_ALIGN_TOP_RIGHT, -(AF_X+2), AFR_PLOT_Y + AFR_PLOT_H + AXIS_Y_OFF);

    // ── Legend แสดงสีเส้นกราฟ (มุมล่างขวา frame) ────────────────
    // จุดสีฟ้า + "RPM" และจุดสีเหลือง + "AFR" เรียงแนวนอน
    {
        const int LEG_Y = AF_Y + AF_H + 3;
        // dot RPM
        lv_obj_t *dot_rpm = lv_obj_create(g_scr_afr_rpm);
        lv_obj_set_pos(dot_rpm, AF_X + 6, LEG_Y + 2);
        lv_obj_set_size(dot_rpm, 8, 8);
        lv_obj_clear_flag(dot_rpm, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(dot_rpm, ACCENT_BLUE, 0);
        lv_obj_set_style_bg_opa(dot_rpm, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(dot_rpm, 4, 0);
        lv_obj_set_style_border_width(dot_rpm, 0, 0);
        mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_BLUE, "RPM",
                 LV_ALIGN_TOP_LEFT, AF_X + 18, LEG_Y);
        // dot AFR
        lv_obj_t *dot_afr = lv_obj_create(g_scr_afr_rpm);
        lv_obj_set_pos(dot_afr, AF_X + 56, LEG_Y + 2);
        lv_obj_set_size(dot_afr, 8, 8);
        lv_obj_clear_flag(dot_afr, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(dot_afr, ACCENT_WARN, 0);
        lv_obj_set_style_bg_opa(dot_afr, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(dot_afr, 4, 0);
        lv_obj_set_style_border_width(dot_afr, 0, 0);
        mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_WARN, "AFR (EST.)",
                 LV_ALIGN_TOP_LEFT, AF_X + 68, LEG_Y);
        // เส้น stoich ──
        lv_obj_t *dot_stoich = lv_obj_create(g_scr_afr_rpm);
        lv_obj_set_pos(dot_stoich, AF_X + 138, LEG_Y + 5);
        lv_obj_set_size(dot_stoich, 20, 2);
        lv_obj_clear_flag(dot_stoich, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(dot_stoich, ACCENT_OK, 0);
        lv_obj_set_style_bg_opa(dot_stoich, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dot_stoich, 0, 0);
        mk_label(g_scr_afr_rpm, &lv_font_montserrat_14, ACCENT_OK, "Stoich 14.7",
                 LV_ALIGN_TOP_LEFT, AF_X + 162, LEG_Y);
    }

    // เส้นกราฟ 2 ค่า วิ่งจากซ้ายไปขวา
    g_afr_rpm_line = lv_line_create(g_scr_afr_rpm);
    lv_obj_set_pos(g_afr_rpm_line, AFR_PLOT_X, AFR_PLOT_Y);
    lv_obj_set_size(g_afr_rpm_line, AFR_PLOT_W, AFR_PLOT_H);
    lv_obj_set_style_line_color(g_afr_rpm_line, ACCENT_BLUE, LV_PART_MAIN);
    lv_obj_set_style_line_width(g_afr_rpm_line, 3, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(g_afr_rpm_line, true, LV_PART_MAIN);

    g_afr_rpm_afr_line = lv_line_create(g_scr_afr_rpm);
    lv_obj_set_pos(g_afr_rpm_afr_line, AFR_PLOT_X, AFR_PLOT_Y);
    lv_obj_set_size(g_afr_rpm_afr_line, AFR_PLOT_W, AFR_PLOT_H);
    lv_obj_set_style_line_color(g_afr_rpm_afr_line, ACCENT_WARN, LV_PART_MAIN);
    lv_obj_set_style_line_width(g_afr_rpm_afr_line, 3, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(g_afr_rpm_afr_line, true, LV_PART_MAIN);

    g_afr_rpm_count = 1;
    g_afr_rpm_points[0].x = 0;
    g_afr_rpm_points[0].y = AFR_PLOT_H-1;
    g_afr_rpm_afr_points[0].x = 0;
    // stoich 14.7 อยู่ที่ (18-14.7)/(18-12)*(PLOT_H-1) = 3.3/6*131 ≈ 72 px จากบน
    g_afr_rpm_afr_points[0].y = (int)roundf(((18.0f - 14.7f)/(18.0f - 12.0f)) * (AFR_PLOT_H - 1));
    lv_line_set_points(g_afr_rpm_line, g_afr_rpm_points, 1);
    lv_line_set_points(g_afr_rpm_afr_line, g_afr_rpm_afr_points, 1);

    // ══════════════════════════════════════════════════════════
    g_scr_dtc = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr_dtc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr_dtc, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_dtc, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_scr_dtc, 0, 0);
    lv_obj_set_style_pad_all(g_scr_dtc, 0, 0);

    // Compact, balanced layout for 320x240: keep every element inside its own zone.
    g_lbl_dtc_title = mk_label(g_scr_dtc, &A4SPEED_16, ACCENT_ERR,
                                "DTC / ERROR CODE", LV_ALIGN_TOP_MID, 0, 7);
    g_lbl_dtc_conn = mk_label(g_scr_dtc, &A4SPEED_14, GRAY_LBL,
                              "ECU STATUS", LV_ALIGN_TOP_MID, 0, 29);

    lv_obj_t *dtc_card = lv_obj_create(g_scr_dtc);
    lv_obj_set_pos(dtc_card, 18, 50);
    lv_obj_set_size(dtc_card, 284, 72);
    lv_obj_clear_flag(dtc_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(dtc_card, PANEL_CARD, 0);
    lv_obj_set_style_border_color(dtc_card, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(dtc_card, 2, 0);
    lv_obj_set_style_radius(dtc_card, 10, 0);
    lv_obj_set_style_pad_all(dtc_card, 0, 0);

    g_lbl_dtc_code = lv_label_create(dtc_card);
    // Use the compact A4SPEED_16 font for the normal NO ERROR state.
    // This keeps the DTC card balanced and leaves enough room for the status line.
    lv_obj_set_style_text_font(g_lbl_dtc_code, &A4SPEED_16, 0);
    lv_obj_set_style_text_color(g_lbl_dtc_code, ACCENT_OK, 0);
    lv_obj_set_style_text_align(g_lbl_dtc_code, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(g_lbl_dtc_code, "NO ERROR");
    lv_obj_set_width(g_lbl_dtc_code, 268);
    lv_obj_set_height(g_lbl_dtc_code, 20);
    lv_obj_align(g_lbl_dtc_code, LV_ALIGN_TOP_MID, 0, 12);

    g_lbl_dtc_desc = lv_label_create(dtc_card);
    lv_obj_set_style_text_font(g_lbl_dtc_desc, &A4SPEED_14, 0);
    lv_obj_set_style_text_color(g_lbl_dtc_desc, GRAY_LBL, 0);
    lv_obj_set_style_text_align(g_lbl_dtc_desc, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(g_lbl_dtc_desc, "SYSTEM OK - NO ACTIVE DTC");
    lv_obj_set_width(g_lbl_dtc_desc, 268);
    lv_obj_set_height(g_lbl_dtc_desc, 16);
    lv_obj_align(g_lbl_dtc_desc, LV_ALIGN_BOTTOM_MID, 0, -5);

    lv_obj_t *clear_dtc_btn = lv_obj_create(g_scr_dtc);
    g_btn_clear_dtc = clear_dtc_btn;
    lv_obj_set_pos(clear_dtc_btn, 50, 130);
    lv_obj_set_size(clear_dtc_btn, 220, 40);
    lv_obj_clear_flag(clear_dtc_btn, LV_OBJ_FLAG_SCROLLABLE);
    // Make this a genuine LVGL clickable object, not just a visual panel.
    lv_obj_add_flag(clear_dtc_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(clear_dtc_btn, clear_dtc_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(clear_dtc_btn, ACCENT_ERR, 0);
    lv_obj_set_style_border_color(clear_dtc_btn, lv_color_hex(0xff756d), 0);
    lv_obj_set_style_border_width(clear_dtc_btn, 2, 0);
    lv_obj_set_style_radius(clear_dtc_btn, 8, 0);
    lv_obj_t *clear_lbl = lv_label_create(clear_dtc_btn);
    lv_obj_set_style_text_font(clear_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(clear_lbl, WHITE, 0);
    lv_label_set_text(clear_lbl, "CLEAR DTC");
    lv_obj_center(clear_lbl);

    // Status is centered directly below the CLEAR DTC button.
    g_lbl_dtc_result = mk_label(g_scr_dtc, &A4SPEED_16, GRAY_LBL,
                                "READY", LV_ALIGN_TOP_MID, 0, 174);
    lv_obj_set_width(g_lbl_dtc_result, 300);
    lv_obj_set_height(g_lbl_dtc_result, 20);
    lv_obj_set_style_text_align(g_lbl_dtc_result, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(g_lbl_dtc_result, LV_LABEL_LONG_CLIP);

    // ปุ่ม/พื้นที่สำหรับไปหน้า 1 โดยเฉพาะ — หน้า 6 จะไม่เปลี่ยนหน้าจากการแตะทั่วไป
    lv_obj_t *next_page_btn = lv_obj_create(g_scr_dtc);
    g_btn_dtc_back = next_page_btn;
    // Keep the status/result text fully above the BACK TO MENU button so
    // connection/clear-result messages never render underneath the button.
    lv_obj_set_pos(next_page_btn, 50, 210);
    lv_obj_set_size(next_page_btn, 220, 26);
    lv_obj_clear_flag(next_page_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(next_page_btn, lv_color_hex(0x151515), 0);
    lv_obj_set_style_bg_opa(next_page_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(next_page_btn, ACCENT_WARN, 0);
    lv_obj_set_style_border_width(next_page_btn, 1, 0);
    lv_obj_set_style_radius(next_page_btn, 6, 0);
    lv_obj_set_style_pad_all(next_page_btn, 0, 0);
    lv_obj_t *next_page_lbl = lv_label_create(next_page_btn);
    lv_obj_set_style_text_font(next_page_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(next_page_lbl, ACCENT_WARN, 0);
    lv_obj_set_style_text_align(next_page_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(next_page_lbl, "BACK TO MENU");
    lv_obj_set_width(next_page_lbl, 200);
    lv_obj_set_height(next_page_lbl, 24);
    lv_obj_center(next_page_lbl);

    // ── Confirmation dialog for CLEAR DTC belongs to page 5. ──
    // This prevents the confirmation from appearing on page 1.
    g_dtc_page4_confirm_overlay = lv_obj_create(g_scr_dtc);
    lv_obj_set_pos(g_dtc_page4_confirm_overlay,20,46);
    lv_obj_set_size(g_dtc_page4_confirm_overlay,280,148);
    lv_obj_clear_flag(g_dtc_page4_confirm_overlay,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_dtc_page4_confirm_overlay,PANEL_CARD,0);
    lv_obj_set_style_border_color(g_dtc_page4_confirm_overlay,ACCENT_ERR,0);
    lv_obj_set_style_border_width(g_dtc_page4_confirm_overlay,2,0);
    lv_obj_set_style_radius(g_dtc_page4_confirm_overlay,10,0);
    lv_obj_set_style_shadow_width(g_dtc_page4_confirm_overlay,24,0);
    lv_obj_set_style_shadow_opa(g_dtc_page4_confirm_overlay,120,0);
    lv_obj_add_flag(g_dtc_page4_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_dtc_page4_confirm_overlay, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *page4_confirm_txt=lv_label_create(g_dtc_page4_confirm_overlay);
    lv_obj_set_style_text_font(page4_confirm_txt,&lv_font_montserrat_16,0);
    lv_obj_set_style_text_color(page4_confirm_txt,ACCENT_ERR,0);
    lv_label_set_text(page4_confirm_txt,"CONFIRM CLEAR DTC");
    lv_obj_set_width(page4_confirm_txt,250);
    lv_obj_set_style_text_align(page4_confirm_txt,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_align(page4_confirm_txt,LV_ALIGN_TOP_MID,0,10);

    lv_obj_t *page4_confirm_msg=lv_label_create(g_dtc_page4_confirm_overlay);
    lv_obj_set_style_text_font(page4_confirm_msg,&lv_font_montserrat_12,0);
    lv_obj_set_style_text_color(page4_confirm_msg,GRAY_LBL,0);
    lv_label_set_text(page4_confirm_msg,"Clear all stored fault codes?");
    lv_obj_set_width(page4_confirm_msg,250);
    lv_obj_set_style_text_align(page4_confirm_msg,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_align(page4_confirm_msg,LV_ALIGN_TOP_MID,0,38);

    // Two confirmation buttons: same Y, same size, symmetric margins, and a fixed 10px gap.
    // This keeps the visual buttons perfectly level and also makes their touch targets
    // easy to calibrate consistently on the 320x240 panel.
    lv_obj_t *page4_cancel_btn = mk_menu_btn(g_dtc_page4_confirm_overlay,0,0,120,42,"CANCEL",BTN_NEUTRAL,page4_confirm_no_cb);
    lv_obj_t *page4_yes_btn    = mk_menu_btn(g_dtc_page4_confirm_overlay,0,0,120,42,"YES",ACCENT_ERR,page4_confirm_yes_cb);

    // Force perfect visual symmetry inside the 280x148 dialog.
    // Both buttons use the same size, same Y, equal 15px side margins and
    // a fixed 10px center gap. Alignment is relative to the parent so it
    // remains correct even if the dialog position changes.
    lv_obj_set_size(page4_cancel_btn,120,42);
    lv_obj_set_size(page4_yes_btn,120,42);
    lv_obj_set_style_pad_all(page4_cancel_btn,0,0);
    lv_obj_set_style_pad_all(page4_yes_btn,0,0);
    // Use explicit pixel positions instead of TOP_LEFT/TOP_RIGHT alignment.
    // LVGL parent padding reduces the content width, which can make the two
    // 120px buttons overlap when aligned to opposite edges.
    lv_obj_set_pos(page4_cancel_btn, 5, 80);
    lv_obj_set_pos(page4_yes_btn, 130, 80);
    lv_obj_set_style_border_width(page4_cancel_btn,2,0);
    lv_obj_set_style_border_width(page4_yes_btn,2,0);
    lv_obj_set_style_border_color(page4_cancel_btn,lv_color_hex(0x6b7280),0);
    lv_obj_set_style_border_color(page4_yes_btn,lv_color_hex(0xff756d),0);


    // ── Verification result overlay: remains on page 4 and reports the
    // actual DTC memory before clear, clear result, and post-clear rescan.
    g_dtc_page4_verify_overlay = lv_obj_create(g_scr_dtc);
    lv_obj_set_pos(g_dtc_page4_verify_overlay,10,34);
    lv_obj_set_size(g_dtc_page4_verify_overlay,300,172);
    lv_obj_clear_flag(g_dtc_page4_verify_overlay,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_dtc_page4_verify_overlay,PANEL_CARD,0);
    lv_obj_set_style_border_color(g_dtc_page4_verify_overlay,ACCENT_BLUE,0);
    lv_obj_set_style_border_width(g_dtc_page4_verify_overlay,2,0);
    lv_obj_set_style_radius(g_dtc_page4_verify_overlay,10,0);
    lv_obj_set_style_pad_all(g_dtc_page4_verify_overlay,6,0);
    lv_obj_add_flag(g_dtc_page4_verify_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_dtc_page4_verify_overlay, LV_OBJ_FLAG_CLICKABLE);

    g_dtc_page4_verify_label = lv_label_create(g_dtc_page4_verify_overlay);
    lv_obj_set_style_text_font(g_dtc_page4_verify_label,&lv_font_montserrat_12,0);
    lv_obj_set_style_text_color(g_dtc_page4_verify_label,WHITE,0);
    lv_obj_set_style_text_align(g_dtc_page4_verify_label,LV_TEXT_ALIGN_CENTER,0);
    lv_label_set_long_mode(g_dtc_page4_verify_label,LV_LABEL_LONG_WRAP);
    lv_obj_set_width(g_dtc_page4_verify_label,286);
    lv_obj_set_height(g_dtc_page4_verify_label,108);
    lv_obj_align(g_dtc_page4_verify_label,LV_ALIGN_TOP_MID,0,8);

    mk_menu_btn(g_dtc_page4_verify_overlay,100,122,100,38,"OK",BTN_NEUTRAL,page4_verify_ok_cb);

    // ══════════════════════════════════════════════════════════
    //  หน้า 1: SETTINGS — DAY/NIGHT + BRIGHTNESS
    // ══════════════════════════════════════════════════════════
    g_scr_settings = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr_settings, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr_settings, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_settings, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_scr_settings, 0, 0);
    lv_obj_set_style_pad_all(g_scr_settings, 0, 0);

    mk_label(g_scr_settings, &A4SPEED_16, ACCENT_BLUE, "PAGE 1  SETTINGS", LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t *mode_card = lv_obj_create(g_scr_settings);
    lv_obj_set_pos(mode_card, 10, 38);
    lv_obj_set_size(mode_card, 300, 78);
    lv_obj_clear_flag(mode_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(mode_card, PANEL_CARD, 0);
    lv_obj_set_style_border_color(mode_card, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(mode_card, 1, 0);
    lv_obj_set_style_radius(mode_card, 9, 0);
    lv_obj_set_style_pad_all(mode_card, 0, 0);

    mk_label(mode_card, &A4SPEED_14, GRAY_LBL, "DISPLAY MODE", LV_ALIGN_TOP_LEFT, 10, 9);
    g_lbl_settings_mode = mk_label(mode_card, &A4SPEED_16, ACCENT_BLUE, "NIGHT MODE", LV_ALIGN_TOP_LEFT, 10, 33);

    g_btn_night = mk_menu_btn(mode_card, 160, 13, 60, 46, "N", ACCENT_BLUE, NULL);
    g_btn_day   = mk_menu_btn(mode_card, 228, 13, 60, 46, "D", BTN_NEUTRAL, NULL);

    lv_obj_t *br_card = lv_obj_create(g_scr_settings);
    lv_obj_set_pos(br_card, 10, 124);
    lv_obj_set_size(br_card, 300, 70);
    lv_obj_clear_flag(br_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(br_card, PANEL_CARD, 0);
    lv_obj_set_style_border_color(br_card, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(br_card, 1, 0);
    lv_obj_set_style_radius(br_card, 9, 0);
    lv_obj_set_style_pad_all(br_card, 0, 0);

    mk_label(br_card, &A4SPEED_14, GRAY_LBL, "BRIGHTNESS", LV_ALIGN_TOP_LEFT, 10, 7);
    g_lbl_brightness_value = mk_label(br_card, &A4SPEED_16, WHITE, "59%", LV_ALIGN_TOP_MID, 0, 28);
    g_btn_brightness_minus = mk_menu_btn(br_card, 10, 22, 62, 40, "-", BTN_NEUTRAL, NULL);
    g_btn_brightness_plus  = mk_menu_btn(br_card, 228, 22, 62, 40, "+", BTN_NEUTRAL, NULL);

    lv_obj_t *back_area = lv_obj_create(g_scr_settings);
    lv_obj_set_pos(back_area, 8, 198);
    lv_obj_set_size(back_area, 304, 30);
    lv_obj_clear_flag(back_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(back_area, lv_color_hex(0x151515), 0);
    lv_obj_set_style_bg_opa(back_area, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(back_area, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(back_area, 1, 0);
    lv_obj_set_style_radius(back_area, 7, 0);
    mk_label(back_area, &A4SPEED_16, ACCENT_WARN, "BACK TO MENU", LV_ALIGN_CENTER, 0, 0);

    // ══════════════════════════════════════════════════════════
    //  หน้า 7: LOG — แสดงเมื่อกด NEXT PAGE จากหน้า 6 เท่านั้น
    // ══════════════════════════════════════════════════════════
    g_scr_log = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr_log, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr_log, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_log, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_scr_log, 0, 0);
    lv_obj_set_style_pad_all(g_scr_log, 0, 0);

    mk_label(g_scr_log, &A4SPEED_16, ACCENT_BLUE, "PAGE 7  LOG", LV_ALIGN_TOP_MID, 0, 5);
    mk_label(g_scr_log, &A4SPEED_14, GRAY_LBL,
             "CLEAR DTC HISTORY", LV_ALIGN_TOP_MID, 0, 27);

    lv_obj_t *log_event = lv_obj_create(g_scr_log);
    lv_obj_set_pos(log_event, 8, 45);
    lv_obj_set_size(log_event, 304, 34);
    lv_obj_clear_flag(log_event, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(log_event, PANEL_CARD, 0);
    lv_obj_set_style_border_color(log_event, ACCENT_ERR, 0);
    lv_obj_set_style_border_width(log_event, 1, 0);
    lv_obj_set_style_radius(log_event, 6, 0);
    lv_obj_set_style_pad_all(log_event, 3, 0);
    g_log_event_label = lv_label_create(log_event);
    lv_obj_set_style_text_font(g_log_event_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_log_event_label, GRAY_LBL, 0);
    lv_label_set_text(g_log_event_label, g_last_dtc_event);
    lv_obj_set_width(g_log_event_label, 296);
    lv_obj_align(g_log_event_label, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *log_card = lv_obj_create(g_scr_log);
    lv_obj_set_pos(log_card, 8, 83);
    lv_obj_set_size(log_card, 304, 141);
    lv_obj_clear_flag(log_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(log_card, PANEL_CARD, 0);
    lv_obj_set_style_border_color(log_card, BORDER_SUBTLE, 0);
    lv_obj_set_style_border_width(log_card, 1, 0);
    lv_obj_set_style_radius(log_card, 6, 0);
    lv_obj_set_style_pad_all(log_card, 3, 0);

    // ซ้าย: DTC HISTORY 2 รายการ
    mk_label(log_card, &lv_font_montserrat_10, ACCENT_BLUE,
             "DTC HISTORY", LV_ALIGN_TOP_LEFT, 0, 0);
    for(int i=0;i<2;i++){
        g_log_rows[i] = lv_label_create(log_card);
        lv_obj_set_style_text_font(g_log_rows[i], &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(g_log_rows[i], WHITE, 0);
        lv_label_set_text(g_log_rows[i], "--");
        lv_obj_set_width(g_log_rows[i], 145);
        lv_obj_set_height(g_log_rows[i], 20);
        // เว้นระยะ DTC HISTORY 2 บรรทัดให้ชัดเจน เพื่อไม่ให้ข้อความทับกัน
        // โดยยังคงพื้นที่ด้านล่างสำหรับ MAX PERFORMANCE
        lv_obj_set_pos(g_log_rows[i], 0, 15 + i*25);
    }

    // ขวา: DISTANCE TEST - 3 RUN ล่าสุด
    mk_label(log_card, &lv_font_montserrat_10, ACCENT_OK,
             "DIST RUNS (3)", LV_ALIGN_TOP_LEFT, 153, 0);
    for(int i=0;i<3;i++){
        g_dist_run_rows[i] = lv_label_create(log_card);
        lv_obj_set_style_text_font(g_dist_run_rows[i], &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(g_dist_run_rows[i], WHITE, 0);
        lv_label_set_text(g_dist_run_rows[i], "--");
        lv_obj_set_width(g_dist_run_rows[i], 145);
        lv_obj_set_height(g_dist_run_rows[i], 34);
        lv_obj_set_pos(g_dist_run_rows[i], 153, 15 + i*36);
    }

    // ECM ID - อ่านครั้งเดียวตอนเชื่อมต่อ ECU แสดงใต้ DIST RUNS (3) ฝั่งขวา
    g_lbl_ecm_id = lv_label_create(log_card);
    lv_obj_set_style_text_font(g_lbl_ecm_id, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(g_lbl_ecm_id, ACCENT_BLUE, 0);
    lv_label_set_text(g_lbl_ecm_id, "ECM ID: --");
    lv_obj_set_width(g_lbl_ecm_id, 145);
    lv_obj_set_pos(g_lbl_ecm_id, 153, 124);

    // MAX PERFORMANCE อยู่ด้านล่างเต็มความกว้าง
    mk_label(log_card, &lv_font_montserrat_10, ACCENT_WARN,
             "MAX PERFORMANCE", LV_ALIGN_TOP_LEFT, 0, 72);

    g_lbl_rpm_max_log = lv_label_create(log_card);
    lv_obj_set_style_text_font(g_lbl_rpm_max_log, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_lbl_rpm_max_log, WHITE, 0);
    lv_label_set_text(g_lbl_rpm_max_log, "RPM MAX : 0 RPM");
    lv_obj_set_width(g_lbl_rpm_max_log, 145);
    lv_obj_set_pos(g_lbl_rpm_max_log, 0, 91);

    g_lbl_speed_max_log = lv_label_create(log_card);
    lv_obj_set_style_text_font(g_lbl_speed_max_log, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_lbl_speed_max_log, WHITE, 0);
    lv_label_set_text(g_lbl_speed_max_log, "SPEED MAX : 0 KM/H");
    lv_obj_set_width(g_lbl_speed_max_log, 145);
    lv_obj_set_pos(g_lbl_speed_max_log, 0, 113);

    dist_run_history_refresh();
    log_refresh_rows();

    const int MX=8;             // margin ซ้าย-ขวา
    const int W = 320-2*MX;     // ความกว้างใช้งานจริง

    // ══════════════════════════════════════════════════════════
    //  แถบสถานะบางๆ ด้านบนสุด (แทนที่ header เดิม) - แตะเพื่อเปิดเมนู
    // ══════════════════════════════════════════════════════════
    g_dtc_bar=lv_obj_create(scr);
    lv_obj_set_pos(g_dtc_bar,0,0);
    // เพิ่มพื้นที่แถบสถานะ ECU ให้ข้อความ NOT CONNECTED มีพื้นที่หายใจมากขึ้น
    const int DTC_H=26;
    lv_obj_set_size(g_dtc_bar,320,DTC_H);
    lv_obj_clear_flag(g_dtc_bar,LV_OBJ_FLAG_SCROLLABLE);
    // ใช้พื้นหลังเดียวกับหน้าจอหลัก: ไม่มี panel/กรอบแยกสำหรับสถานะ ECU
    lv_obj_set_style_bg_opa(g_dtc_bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_dtc_bar, 0, 0);
    lv_obj_set_style_radius(g_dtc_bar, 0, 0);
    lv_obj_set_style_pad_all(g_dtc_bar,0,0);
    // แถบสถานะนี้เป็นข้อมูลแสดงผลเท่านั้น ไม่ใช่ปุ่ม/เมนูบนหน้า 1

    g_dtc_label=lv_label_create(g_dtc_bar);
    lv_obj_set_style_text_font(g_dtc_label,&lv_font_montserrat_14,0);
    lv_obj_set_style_text_color(g_dtc_label,theme_subtext(),0);
    lv_label_set_text(g_dtc_label,"SYSTEM OK - NO DTC");
    lv_obj_set_width(g_dtc_label,260);
    lv_obj_set_style_text_align(g_dtc_label,LV_TEXT_ALIGN_LEFT,0);
    lv_obj_align(g_dtc_label,LV_ALIGN_LEFT_MID,MX,0);

    // ── LINK LED: จุดไฟกะพริบ + ไอคอนสัญญาณ แสดงสถานะเชื่อมต่อ ECU ──
    g_link_dot=lv_obj_create(g_dtc_bar);
    lv_obj_set_size(g_link_dot,6,6);
    lv_obj_clear_flag(g_link_dot,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(g_link_dot,3,0);
    lv_obj_set_style_border_width(g_link_dot,0,0);
    lv_obj_set_style_bg_color(g_link_dot,ACCENT_OK,0);
    lv_obj_set_style_bg_opa(g_link_dot,LV_OPA_COVER,0);
    lv_obj_align(g_link_dot,LV_ALIGN_RIGHT_MID,-MX,0);

    // ══════════════════════════════════════════════════════════
    //  RPM CARD: ใช้รูปแบบกรอบเดียวกับ BATT / INJ / ECT / INC / TPS
    //  โดยยังคงตัวเลข RPM ขนาดใหญ่ + RPM segment bar ด้านล่าง
    // ══════════════════════════════════════════════════════════
    const int RPM_Y=28;
    const int RPM_CARD_H=48;

    lv_obj_t *rpm_card=lv_obj_create(scr);
    lv_obj_set_pos(rpm_card,MX,RPM_Y);
    lv_obj_set_size(rpm_card,W,RPM_CARD_H);
    lv_obj_clear_flag(rpm_card,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(rpm_card,PANEL_CARD,0);
    lv_obj_set_style_bg_opa(rpm_card,LV_OPA_COVER,0);
    lv_obj_set_style_border_color(rpm_card,BORDER_SUBTLE,0);
    lv_obj_set_style_border_width(rpm_card,1,0);
    lv_obj_set_style_radius(rpm_card,8,0);
    lv_obj_set_style_pad_all(rpm_card,0,0);
    lv_obj_set_style_shadow_width(rpm_card,8,0);
    lv_obj_set_style_shadow_color(rpm_card,lv_color_hex(0x000000),0);
    lv_obj_set_style_shadow_opa(rpm_card,55,0);

    g_lbl_rpm=lv_label_create(rpm_card);
    lv_obj_set_style_text_font(g_lbl_rpm,&A4SPEED_36,0);
    lv_obj_set_style_text_color(g_lbl_rpm,WHITE,0);
    lv_obj_set_style_text_align(g_lbl_rpm,LV_TEXT_ALIGN_CENTER,0);
    lv_label_set_long_mode(g_lbl_rpm,LV_LABEL_LONG_CLIP);
    lv_label_set_text(g_lbl_rpm,"0");
    lv_obj_set_width(g_lbl_rpm,W-4);
    lv_obj_set_height(g_lbl_rpm,34);
    // เอาป้าย RPM ใต้ตัวเลขออก และจัดตัวเลขให้อยู่ส่วนบนของกรอบ
    // เว้นพื้นที่ด้านล่างให้แถบ RPM โดยไม่ให้สองส่วนซ้อนกัน
    lv_obj_align(g_lbl_rpm,LV_ALIGN_TOP_MID,0,1);

    // ── แถบไฟ RPM สไตล์ A4SPEED racing (24 segments) ───────────
    // ย้ายแถบเข้าไปอยู่ "ภายใน" RPM card เพื่อกันล้นจอ/หลุดตำแหน่ง
    // และคุมระยะซ้าย-ขวาให้สมดุลแบบคงที่บนจอ 320x240
    const int TACHO_BAR_H=8;
    const int TG=2;                    // ช่องว่างระหว่าง segment
    const int TW=10;                   // ความกว้างแต่ละ segment
    const int TICKS_TOTAL_W = NUM_TICKS*TW + (NUM_TICKS-1)*TG;
    const int TICKS_X0 = (W-TICKS_TOTAL_W)/2;
    const int BAR_Y = 37;              // ตำแหน่ง Y ภายใน rpm_card; อยู่ใต้ตัวเลขพอดี
    for(int i=0;i<NUM_TICKS;i++){
        lv_obj_t *tk=lv_obj_create(rpm_card);
        lv_obj_set_pos(tk, TICKS_X0+i*(TW+TG), BAR_Y);
        lv_obj_set_size(tk, TW, TACHO_BAR_H);
        lv_obj_clear_flag(tk,LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(tk,0,0);
        lv_obj_set_style_radius(tk,1,0);
        lv_obj_set_style_bg_color(tk,GRAY_LINE,0);
        lv_obj_set_style_bg_opa(tk,LV_OPA_COVER,0);
        g_ticks[i]=tk;
    }

    // ══════════════════════════════════════════════════════════
    //  แถวรอง: RPM สูงสุด (peak-hold) | TPS %
    // ══════════════════════════════════════════════════════════
    const int ROW2_Y=RPM_Y+RPM_CARD_H+8;
    g_lbl_rpm_peak=mk_label(scr,&A4SPEED_26,GRAY_LBL,"0",LV_ALIGN_TOP_LEFT,MX+6,ROW2_Y);
    mk_label(scr,&A4SPEED_14,GRAY_LBL,"RPM max",LV_ALIGN_TOP_LEFT,MX+150,ROW2_Y+8);

    g_lbl_tps=mk_label(scr,&A4SPEED_26,WHITE,"0",LV_ALIGN_TOP_RIGHT,-MX-22,ROW2_Y);
    mk_label(scr,&A4SPEED_14,GRAY_LBL,"%",LV_ALIGN_TOP_RIGHT,-MX,ROW2_Y+8);

    // ── เส้นแบ่งบางๆ ──
    lv_obj_t *div1=lv_obj_create(scr);
    lv_obj_set_pos(div1,MX,ROW2_Y+28); lv_obj_set_size(div1,W,1);
    lv_obj_set_style_bg_color(div1,GRAY_LINE,0);
    lv_obj_set_style_border_width(div1,0,0);
    lv_obj_clear_flag(div1,LV_OBJ_FLAG_SCROLLABLE);

    // ══════════════════════════════════════════════════════════
    //  แถวสถิติเดียว สมมาตร 5 ช่อง: BATT | INJ | ECT | INC | TPS
    //  (เพิ่ม TPS เข้ามาในแถวเดียวกับ BATT ตามที่ขอ)
    // ══════════════════════════════════════════════════════════
    const int ROW3_Y=ROW2_Y+32;
    // กริด 5 ช่องแบบสมมาตรจริง: กล่องเท่ากัน + gap เท่ากัน + ระยะซ้าย/ขวาเท่ากัน
    const int STAT_GAP=3;
    const int STAT_W=57;
    const int COL_H=58;
    const int STAT_TOTAL_W=(STAT_W*5)+(STAT_GAP*4);
    const int STAT_X0=MX+(W-STAT_TOTAL_W)/2;
    const int C0=STAT_X0+0*(STAT_W+STAT_GAP);
    const int C1=STAT_X0+1*(STAT_W+STAT_GAP);
    const int C2=STAT_X0+2*(STAT_W+STAT_GAP);
    const int C3=STAT_X0+3*(STAT_W+STAT_GAP);
    const int C4=STAT_X0+4*(STAT_W+STAT_GAP);

    mk_stat_col(scr,C0,ROW3_Y,STAT_W,COL_H,"BATT",ACCENT_BATT,&g_lbl_batt);
    mk_stat_col(scr,C1,ROW3_Y,STAT_W,COL_H,"INJ ms",ACCENT_INJ,&g_lbl_inj);
    mk_stat_col(scr,C2,ROW3_Y,STAT_W,COL_H,"ECT C",ACCENT_TEMP,&g_lbl_ect);
    mk_stat_col(scr,C3,ROW3_Y,STAT_W,COL_H,"INC .",ACCENT_SPARK,&g_lbl_inc);
    mk_stat_col(scr,C4,ROW3_Y,STAT_W,COL_H,"TPS",ACCENT_TPS,&g_lbl_tps_stat);

    // ── เส้นแบ่งก่อนแถบ AFR ล่างสุด ──
    const int DIV2_Y=ROW3_Y+COL_H+9;
    lv_obj_t *div2=lv_obj_create(scr);
    lv_obj_set_pos(div2,MX,DIV2_Y); lv_obj_set_size(div2,W,1);
    lv_obj_set_style_bg_color(div2,GRAY_LINE,0);
    lv_obj_set_style_border_width(div2,0,0);
    lv_obj_clear_flag(div2,LV_OBJ_FLAG_SCROLLABLE);

    // ══════════════════════════════════════════════════════════
    //  แถวล่างสุด: AFR (ประมาณจากแรงดัน O2 sensor) แสดงเป็นตัวเลขล้วน
    //  (เอาแถบบาร์ segment ออกแล้ว)
    //  หมายเหตุความถูกต้อง: เซนเซอร์ O2 แบบ narrowband (Wave 110i/125i)
    //  ให้แรงดันจริงในช่วง ~0-1V เท่านั้น (สวิตช์เร็วรอบจุด stoich
    //  ~0.45V) ส่วนการถอดรหัส T20 offset 0x04 ยังไม่ได้ยืนยันกับรถจริง
    //  เหมือน T17 (RPM/TPS/ECT ฯลฯ) เหตุนี้ค่า AFR ที่แสดงจึงเป็น
    //  "ค่าประมาณ" สำหรับดูแนวโน้มเท่านั้น ไม่ใช่ AFR วัดจริง
    // ══════════════════════════════════════════════════════════
    const int AFR_Y=DIV2_Y+16;
    g_lbl_afr_label=mk_label(scr,&A4SPEED_16,WHITE,"AFR (EST.)",LV_ALIGN_TOP_LEFT,MX,AFR_Y);
    g_lbl_afr_val=mk_label(scr,&A4SPEED_16,ACCENT_RICH,"0.0 | CO2 --%",
                            LV_ALIGN_TOP_RIGHT,-MX,AFR_Y);

    // หน้า 1 ไม่มีเมนู CLEAR DTC แล้ว การลบโค้ดทำเฉพาะบนหน้า 6

    extended_pages_init();
    if(!g_fueltrim_prefs_ready){
        g_fueltrim_prefs_ready = g_fueltrim_prefs.begin("fueltrim", false);
        if(g_fueltrim_prefs_ready){
            int v = (int)g_fueltrim_prefs.getChar("trim", 0);
            if(v < -20 || v > 20) v = 0;
            g_fuel_trim_pct = (int8_t)v;
        }
        if(g_fueltrim_value){
            char t[16]; snprintf(t,sizeof(t),"%+d%%",(int)g_fuel_trim_pct);
            lv_label_set_text(g_fueltrim_value,t);
        }
    }
    if(!g_disttest_prefs_ready){
        g_disttest_prefs_ready = g_disttest_prefs.begin("disttest", false);
        if(g_disttest_prefs_ready){
            int v = (int)g_disttest_prefs.getUChar("presetidx", g_dist_preset_idx);
            if(v < 0 || v >= (int)DIST_TEST_PRESET_COUNT) v = g_dist_preset_idx;
            g_dist_preset_idx = (uint8_t)v;
        }
        if(g_lbl_dist_target){
            char t[16]; snprintf(t,sizeof(t),"%.0f m",DIST_TEST_PRESETS_M[g_dist_preset_idx]);
            lv_label_set_text(g_lbl_dist_target,t);
        }
    }
    g_logger_fs_ready = LittleFS.begin(false);
    logger_load_session_counter();
    logger_reset_session_stats();

    // ค่าเริ่มต้น: NIGHT + ความสว่าง 150/255 (~59%)
    // หมายเหตุ: ไม่เปิดแบ็คไลท์ตรงนี้ — ตัวแปร g_brightness ใช้แค่กำหนดค่าเริ่มต้น
    // ของสถานะซอฟต์แวร์ (label, ธีม) ส่วนแบ็คไลท์จริงจะถูกเปิดใน main.cpp
    // หลังจากฟังก์ชันนี้ทำงานเสร็จสมบูรณ์แล้วเท่านั้น (หน้า 1 ถูกวาดจบแล้ว)
    // เพื่อไม่ให้เห็นภาพซ้อน/ภาพวาดค้างระหว่างที่ยังสร้าง UI อยู่
    g_brightness = DEFAULT_BRIGHTNESS;
    g_day_mode = false;
    settings_apply_theme();

    gauge_ui_update();

    // ══════════════════════════════════════════════════════════
    // หน้า 0: MENU — สร้างทีหลังสุด (หน้าอื่นทั้ง 15 หน้าถูกสร้างพร้อม
    // ตัวชี้ g_scr_* ครบแล้ว) ใช้เป็นหน้าแรกที่เจอตอนบูทเครื่อง
    // แตะแผ่นไหน = ไปหน้านั้น, แตะอีกทีในหน้าปลายทาง = กลับมาที่นี่
    // ป้ายข้อความในแต่ละแผ่นใช้แค่ "คำแรก" ของชื่อหน้า กันข้อความล้นกล่อง
    // ══════════════════════════════════════════════════════════
    g_scr_menu = lv_obj_create(NULL);
    lv_obj_clear_flag(g_scr_menu, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_scr_menu, BG_BLACK, 0);
    lv_obj_set_style_bg_opa(g_scr_menu, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_scr_menu, 0, 0);
    lv_obj_set_style_pad_all(g_scr_menu, 0, 0);

    // ── SELECT PAGE: balanced 3x4 launcher ──────────────────────
    // หน้า 9/11/12/13 ถูกย้ายไปอยู่ในชุดย่อยของ PAGE 8 แล้ว
    // จึงใช้ 12 tile ที่จัดกึ่งกลางพอดีกับจอ 320x240
    g_menu_header = lv_obj_create(g_scr_menu);
    lv_obj_set_pos(g_menu_header, 4, 2);
    lv_obj_set_size(g_menu_header, 312, 21);
    lv_obj_clear_flag(g_menu_header, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(g_menu_header, ACCENT_BLUE, 0);
    lv_obj_set_style_bg_grad_color(g_menu_header, ACCENT_PURPLE, 0);
    lv_obj_set_style_bg_grad_dir(g_menu_header, LV_GRAD_DIR_HOR, 0);
    lv_obj_set_style_bg_opa(g_menu_header, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_menu_header, 0, 0);
    lv_obj_set_style_radius(g_menu_header, 6, 0);
    lv_obj_set_style_pad_all(g_menu_header, 0, 0);

    g_menu_header_title = mk_label(g_menu_header, &A4SPEED_16, WHITE, "SELECT PAGE", LV_ALIGN_LEFT_MID, 8, 0);
    g_menu_header_hint = mk_label(g_menu_header, &lv_font_montserrat_10, WHITE, "TOUCH TO OPEN", LV_ALIGN_RIGHT_MID, -8, 0);

    {
        const int MCOLS = 3;
        const int MGAP  = 5, MX0 = 4, MY0 = 27;
        const int MCW   = 101, MRH = 47;
        for(int i = 0; i < 12; i++){
            const int r = i / MCOLS, c = i % MCOLS;
            const int x = MX0 + c * (MCW + MGAP);
            const int y = MY0 + r * (MRH + MGAP);
            const bool locked = false; // PAGE 14 is now the second live instrument cluster

            lv_obj_t *tile = lv_obj_create(g_scr_menu);
            lv_obj_set_pos(tile, x, y);
            lv_obj_set_size(tile, MCW, MRH);
            lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_bg_color(tile, locked ? ACCENT_ERR : PANEL_CARD, 0);
            lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
            lv_obj_set_style_border_color(tile, ACCENT_OK, 0);
            lv_obj_set_style_border_width(tile, locked ? 2 : 1, 0);
            lv_obj_set_style_radius(tile, 8, 0);
            lv_obj_set_style_pad_all(tile, 0, 0);

            // หมายเลขหน้า: จัดกึ่งกลางแนวนอนด้านบน เพื่อให้เป็นลำดับชัดเจน
            lv_obj_t *num = lv_label_create(tile);
            lv_obj_set_style_text_font(num, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(num, locked ? WHITE : GRAY_LBL, 0);
            lv_obj_set_style_text_align(num, LV_TEXT_ALIGN_CENTER, 0);
            char numtxt[5]; snprintf(numtxt, sizeof(numtxt), "P%02d", MENU_PAGE_TARGET[i]);
            lv_label_set_text(num, numtxt);
            lv_obj_set_width(num, MCW - 8);
            lv_obj_align(num, LV_ALIGN_TOP_MID, 0, 3);

            // ชื่อหน้า: ลดฟอนต์ลง 1 ระดับเพื่อให้ข้อความยาวอ่านง่ายขึ้น
            // และจัดกึ่งกลางในพื้นที่หลักของ tile แทนการชิดด้านล่าง
            lv_obj_t *word = lv_label_create(tile);
            lv_obj_set_style_text_font(word, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(word, WHITE, 0);
            lv_label_set_long_mode(word, LV_LABEL_LONG_CLIP);
            lv_obj_set_width(word, MCW - 8);
            lv_obj_set_style_text_align(word, LV_TEXT_ALIGN_CENTER, 0);
            lv_label_set_text(word, MENU_TILE_LABEL[i]);
            lv_obj_align(word, LV_ALIGN_CENTER, 0, 6);

            g_menu_tiles[i] = tile;
        }
    }

    // ให้ SELECT PAGE เปลี่ยนตาม DAY/NIGHT เช่นเดียวกับหน้าอื่นๆ
    settings_apply_theme();

    // เริ่มต้นโปรแกรมที่หน้า 0 = MENU (เดิมเริ่มที่หน้า 1 SETTINGS)
    g_page = 0;
    g_show_black = false;
    if(g_scr_menu){ lv_scr_load(g_scr_menu); }
}

// ── ตาราง DTC Honda PGM-FI ────────────────────────────────────
static const char* honda_dtc_desc(const char*c){
    if(!c || !*c) return "Unknown";
    // Accept both the legacy base code ("7") and the readable code form
    // used by the DTC memory decoder ("007-2").
    char base[8] = {0};
    size_t i = 0;
    while(c[i] && c[i] != '-' && i < sizeof(base)-1){
        base[i] = c[i];
        ++i;
    }
    base[i] = '\0';
    if(!strcmp(base,"1"))return"MAP Sensor";
    if(!strcmp(base,"7"))return"ECT Sensor";
    if(!strcmp(base,"8"))return"TPS Sensor";
    if(!strcmp(base,"9"))return"IAT Sensor";
    if(!strcmp(base,"12"))return"Injector";
    if(!strcmp(base,"21"))return"O2 Sensor";
    if(!strcmp(base,"29"))return"IACV/ISC";
    if(!strcmp(base,"33"))return"ECU Memory";
    if(!strcmp(base,"54"))return"Bank Angle";
    return"Unknown";
}

// ============================================================
//  ใช้ค่าจริงจาก ECU เมื่อเชื่อมต่อ K-line สำเร็จ
//  ถ้ายังไม่ได้ต่อ/หลุดการเชื่อมต่อ แสดงค่า 0 จริง ๆ (ไม่มีค่าจำลอง)
//  และไม่มี DTC ปลอมใด ๆ ทั้งสิ้น
// ============================================================
static void read_kline_data(){
    kline_get_snapshot(&g_kline_snapshot);
    const bool ecu_connected = g_kline_snapshot.connected;
    const bool t17_valid = ecu_connected && g_kline_snapshot.t17_valid &&
                           kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_RPM);

    if(ecu_connected){
        g_t17_valid = t17_valid;
        g_rpm   = g_t17_valid ? g_kline_snapshot.rpm      : 0.0f;
        g_ect   = g_t17_valid ? g_kline_snapshot.ect_c    : 0.0f;
        g_tps   = g_t17_valid ? g_kline_snapshot.tps      : 0.0f;
        g_speed = 0.0f; // ECU speed is experimental; dashboard speed is GPS only.
        g_iat   = g_t17_valid ? g_kline_snapshot.iat_c   : 0.0f;
        g_batt  = g_t17_valid ? g_kline_snapshot.batt_v   : 0.0f;
        g_inj   = g_t17_valid ? g_kline_snapshot.inj_ms   : 0.0f;
        // หน้า 6 มี DTC scanner แบบ asynchronous:
        // ห้ามล้างโค้ดที่อ่านสำเร็จทุก tick เพราะระหว่างรอรอบสแกนถัดไป
        // kline_get_dtc_summary() อาจยังไม่มีผลลัพธ์ใหม่ ทำให้จอเดิมกระพริบ
        // สลับระหว่าง "รหัส DTC" กับ "NO ERROR"
        // เมื่ออยู่หน้า 6 ให้คงผล DTC ล่าสุดไว้จนกว่าจะได้ผลสแกนใหม่
        // หรือ ECU หลุดการเชื่อมต่อแล้วจึงค่อยล้างสถานะ
        if(g_page != 6){
            g_dtc_active = false;
            g_dtc[0] = '\0';
        }
    } else {
        // ── ไม่มี ECU ต่ออยู่: ค่าจริงคือ 0/ไม่มีข้อมูล ไม่ใช่ค่าจำลอง ──
        // เดิมมีโหมดจำลอง sine wave + DTC ปลอมโชว์วนทุก 30 วิ เพื่อกันจอ
        // ว่างเปล่าตอน demo แต่ทำให้แถบสถานะ DTC กระพริบพื้นหลังแดงเป็น
        // รอบ ๆ ทั้งที่ไม่มี fault จริง จึงตัดออก - ตอนไม่ต่อ ECU ให้เห็น
        // เป็น 0/ไม่มี error ตรงไปตรงมา พื้นหลังจะได้นิ่ง ไม่กระพริบ
        g_t17_valid = false;
        g_rpm = 0; g_ect = 0; g_tps = 0; g_speed = 0; g_inj = 0; g_iat = 0; g_batt = 0;
        g_dtc_active = false;
        g_dtc[0] = '\0';
    }

    // ── ใช้ความเร็วจาก GPS แทน kl_speed เสมอเมื่อมีสัญญาณ GPS ──
    // K-line decoder รุ่นนี้ยังไม่ถอดรหัสความเร็วจาก ECU (ค้างที่ 0)
    // GPS ให้ค่าความเร็วจริงและไม่ขึ้นกับสถานะเชื่อมต่อ ECU เลย
    // จึงใช้แทนได้ทั้งตอนต่อ ECU จริงและตอนจำลอง (ไม่มี ECU)
    // หน้า 2/กราฟใช้ความเร็วจาก GPS ATGM336H เท่านั้น
    // ถ้ายังไม่มี GPS fix ให้เป็น 0 แทนการใช้ kl_speed จาก ECU
    g_speed = gps_speed_valid() ? gps_speed_kmh() : 0.0f;

    if(g_rpm>g_rpm_peak) g_rpm_peak=g_rpm;   // ── อัปเดต peak-hold ──

    // ── ตรวจสอบความถูกต้อง TPS: byte ดิบ*0.5 ตามที่ยืนยันกับรถจริงแล้ว
    // (T17, "ทดสอบยืนยันกับรถจริงแล้ว") ปกติควรอยู่ 0-100% แต่กันไว้เผื่อ
    // สัญญาณรบกวน/ค่าผิดปกติชั่วขณะ ไม่ให้แถบ/ตัวเลขแสดงเกินสเกล ──
    if(g_tps<0.0f)   g_tps=0.0f;
    if(g_tps>100.0f) g_tps=100.0f;
}

// ── Update ────────────────────────────────────────────────────
float gauge_ui_get_afr_est(){
    return f_afr;
}

bool gauge_ui_is_afr_valid(){
    return g_afr_valid;
}

bool gauge_ui_is_day_mode(){
    return g_day_mode;
}

void gauge_ui_update(){
    static uint32_t last=0;
    if(millis()-last<50) return; last=millis();
    read_kline_data();

    // Cheap check every tick; only does real work once the background clear
    // request actually finishes (see gauge_ui_trigger_clear_dtc()).
    finalize_clear_dtc_if_ready();

    // ── RPM REAL-TIME ───────────────────────────────────────────────
    // T17 is polled frequently by the K-Line background task. The previous
    // median + EMA + slew-rate chain introduced multi-second lag at large RPM
    // changes, so the gauge now follows the newest validated ECU sample.
    if(!g_t17_valid){
        f_rpm = f_ect = f_tps = f_batt = f_inj = f_ign = 0.0f;
    } else {
        f_rpm = constrain((float)g_rpm, 0.0f, 12000.0f);
        f_ect  = ema(f_ect,  g_ect,  0.08f);
        f_tps  = ema(f_tps,  g_tps,  0.45f);
        f_batt = ema(f_batt, g_batt, 0.10f);
        f_inj  = ema(f_inj,  g_inj,  0.30f);
        f_ign  = ema(f_ign,  g_kline_snapshot.ignition_deg, 0.30f);
    }

    // ── LINK LED: กะพริบเฉพาะตอนหน้า Dashboard แสดงอยู่ ───────────
    static uint32_t hb=0; hb++;
    const bool connected = kline_is_connected();
    if(g_page == 2){
        const lv_color_t link_col = connected ? ACCENT_OK : ACCENT_ERR;
        set_bg_color_if_changed(g_link_dot, link_col);
        // ลดการสลับ opacity จากทุก 50ms เหลือประมาณทุก 150ms
        static uint32_t link_blink_ms = 0;
        static bool link_blink_on = true;
        const uint32_t blink_now = millis();
        if(blink_now - link_blink_ms >= 150){
            link_blink_ms = blink_now;
            link_blink_on = !link_blink_on;
            set_bg_opa_if_changed(g_link_dot, link_blink_on ? LV_OPA_COVER : LV_OPA_30);
        }
    }

    static uint32_t gdbg=0;
    if(millis()-gdbg>2000){ gdbg=millis();
        UI_TRACE("### values: rpm=%.0f speed=%.0f hb=%lu ###\n", g_rpm, g_speed, hb);
    }

    // ── RPM segment bar: ไล่สีเขียว -> เหลือง -> แดง กะพริบตอน redline ──
    // ใช้ค่าที่กรองแล้ว (f_rpm) + partial opacity ที่ช่องขอบเขต ให้บาร์
    // ไหลลื่นแทนการกระโดดทีละช่องแบบ digital step ──────────────────
    float active_f = f_rpm/10000.0f*NUM_TICKS;
    int   active   = (int)active_f;
    float frac     = active_f - active;   // เศษ 0.0-1.0 ของช่องขอบเขต
    bool redline = active >= (int)(NUM_TICKS*0.90f);
    static bool rl_on=true; static uint32_t rl_t=0;
    if(redline && millis()-rl_t>110){ rl_t=millis(); rl_on=!rl_on; }
    // P02 RPM bar belongs to g_scr_main/g_ticks. Update it independently of
    // the current page so it is already fresh when P02 is opened.
    for(int i=0;i<NUM_TICKS;i++){
        lv_color_t col; lv_opa_t opa=LV_OPA_COVER;
        if(i>active)                        { col=GRAY_LINE; }
        else if(i==active){
            // ── ช่องขอบเขต: fade-in ตามเศษทศนิยม แทนกระโดดเต็ม/ว่างทันที ──
            col = (i<(int)(NUM_TICKS*0.60f))?ACCENT_OK
                : (i<(int)(NUM_TICKS*0.85f))?ACCENT_WARN
                : ACCENT_ERR;
            opa = LV_OPA_30 + (lv_opa_t)(frac*(LV_OPA_COVER-LV_OPA_30));
        }
        else if(i<(int)(NUM_TICKS*0.60f))  col=ACCENT_OK;
        else if(i<(int)(NUM_TICKS*0.85f))  col=ACCENT_WARN;
        else if(redline && !rl_on)         col=lv_color_hex(0x401010);
        else                                col=ACCENT_ERR;
        set_bg_color_if_changed(g_ticks[i],col);
        set_bg_opa_if_changed(g_ticks[i],opa);
    }

    char buf[32];

    // หน้า 2: ความเร็วหลักใช้ค่าจาก GPS ATGM336H โดยตรง
    // g_speed ถูกผูกกับ gps_speed_kmh() ใน read_kline_data() เมื่อ GPS fix
    // ── Speed page update ───────────────────────────────────────
    // Speed is already filtered once, at the source (gps_speed_kmh() applies
    // an EMA + zero-speed lock inside gps.cpp). Re-applying a second EMA here
    // on top of that stacked two filters in series, which adds extra lag
    // (the number visibly took longer to "catch up" to real speed, especially
    // right after pulling away from a stop) and also meant this label and the
    // GPS-box label below could show two slightly different numbers for the
    // same instant. f_speed now just tracks the single already-filtered
    // value directly - no second filter stage.
    f_speed = g_speed;
    if(f_speed > g_speed_max) g_speed_max = f_speed;
    if(f_speed < 0) f_speed = 0;

    // Secondary pages and logs consume the refreshed f_rpm/f_speed values
    // in the same UI cycle, preventing stale RPM values on other pages.
    extended_pages_update();
    sqxzgauge_page_update();

    if(g_lbl_speed_page){
        snprintf(buf, sizeof(buf), "%.0f", f_speed);
        LABEL_SET_IF_CHANGED(g_lbl_speed_page, buf);
        set_text_color_if_changed(g_lbl_speed_page,
            (f_speed >= 120) ? ACCENT_ERR : (f_speed >= 90 ? ACCENT_WARN : theme_primary_text()));
    }
    if(g_lbl_speed_max){
        snprintf(buf, sizeof(buf), "%.0f", g_speed_max);
        LABEL_SET_IF_CHANGED(g_lbl_speed_max, buf);
    }
    if(g_lbl_rpm_max_log){
        snprintf(buf, sizeof(buf), "RPM MAX : %.0f RPM", g_rpm_peak);
        LABEL_SET_IF_CHANGED(g_lbl_rpm_max_log, buf);
    }
    if(g_lbl_speed_max_log){
        snprintf(buf, sizeof(buf), "SPEED MAX : %.0f KM/H", g_speed_max);
        LABEL_SET_IF_CHANGED(g_lbl_speed_max_log, buf);
    }
    if(g_lbl_ecm_id){
        char ecmIdBuf[48];
        char ecmLine[64];
        if(kline_ecm_id_valid() && kline_get_ecm_id(ecmIdBuf, sizeof(ecmIdBuf))){
            snprintf(ecmLine, sizeof(ecmLine), "ECM ID: %s", ecmIdBuf);
        } else {
            snprintf(ecmLine, sizeof(ecmLine), "ECM ID: --");
        }
        LABEL_SET_IF_CHANGED(g_lbl_ecm_id, ecmLine);
    }
    // ── GPS: ความเร็ว (กล่องที่ 3) + เวลา (แถบสถานะ) ────────────
    // Same f_speed value as the main gauge above now (both come from the
    // single gps_speed_kmh() filter stage), so the two labels can no longer
    // disagree with each other.
    if(g_lbl_gps_speed){
        if(gps_has_fix()){
            snprintf(buf, sizeof(buf), "%.0f", f_speed);
            LABEL_SET_IF_CHANGED(g_lbl_gps_speed, buf);
            set_text_color_if_changed(g_lbl_gps_speed, ACCENT_BLUE);
        } else {
            // ยังไม่มีตำแหน่ง GPS ที่เชื่อถือได้
            LABEL_SET_IF_CHANGED(g_lbl_gps_speed, "--");
            set_text_color_if_changed(g_lbl_gps_speed, theme_subtext());
        }
    }
    if(g_lbl_speed_time){
        char gps_clock[12];
        gps_time_str(gps_clock, sizeof(gps_clock));
        LABEL_SET_IF_CHANGED(g_lbl_speed_time, gps_clock);
        set_text_color_if_changed(g_lbl_speed_time, theme_primary_text());
    }
    if(g_lbl_speed_status){
        const GpsQuality q = gps_quality();
        const char *qs = "NO GPS";
        lv_color_t qc = ACCENT_ERR;
        switch(q){
            case GPS_QUALITY_EXCELLENT: qs = "GPS EXCELLENT"; qc = ACCENT_OK; break;
            case GPS_QUALITY_GOOD:      qs = "GPS GOOD";      qc = ACCENT_OK; break;
            case GPS_QUALITY_FAIR:      qs = "GPS FAIR";      qc = ACCENT_WARN; break;
            case GPS_QUALITY_WEAK:      qs = "GPS WEAK";      qc = ACCENT_WARN; break;
            case GPS_QUALITY_NO_FIX:    qs = "GPS SEARCH";    qc = ACCENT_WARN; break;
            default:                    qs = "GPS NO DATA";   qc = ACCENT_ERR; break;
        }
        char gps_status[32];
        snprintf(gps_status, sizeof(gps_status), "%s %dS", qs, gps_satellites());
        LABEL_SET_IF_CHANGED(g_lbl_speed_status, gps_status);
        set_text_color_if_changed(g_lbl_speed_status, qc);
        if(g_speed_link_dot){
            static uint32_t sb_hb = 0; sb_hb++;
            const bool gps_ok = gps_data_is_fresh() && gps_has_fix();
            lv_obj_set_style_bg_color(g_speed_link_dot, gps_ok ? ACCENT_OK : ACCENT_WARN, 0);
            lv_obj_set_style_bg_opa(g_speed_link_dot, (sb_hb & 1) ? LV_OPA_COVER : LV_OPA_30, 0);
        }
    }

    // ── TRIP DISTANCE: สะสมระยะทางจาก GPS Latitude/Longitude ─────────
    // กัน GPS drift ตอนรถหยุด และตัด jump ที่ผิดปกติออก
    if(gps_has_fix()) {
        const double lat = gps_latitude();
        const double lon = gps_longitude();
        const double DEG2RAD = 0.017453292519943295;
        if(!g_trip_have_last) {
            g_trip_last_lat = lat;
            g_trip_last_lon = lon;
            g_trip_have_last = true;
        } else {
            const double dlat = (lat - g_trip_last_lat) * DEG2RAD;
            const double dlon = (lon - g_trip_last_lon) * DEG2RAD;
            const double a = sin(dlat*0.5)*sin(dlat*0.5) +
                             cos(g_trip_last_lat*DEG2RAD) * cos(lat*DEG2RAD) *
                             sin(dlon*0.5)*sin(dlon*0.5);
            const double c = 2.0 * atan2(sqrt(a), sqrt(fmax(0.0, 1.0-a)));
            const double step_km = 6371.0 * c;
            const float gps_spd = gps_speed_kmh();
            // รับระยะเฉพาะเมื่อเคลื่อนที่พอสมควร หรือเป็นช่วง GPS ที่มี step เล็ก
            // แต่ไม่เกิน 100 m/refresh เพื่อกัน jump จาก multipath
            if(step_km >= 0.002 && step_km <= 0.100 && gps_spd >= 1.0f) {
                g_trip_km += (float)step_km;
            }
            g_trip_last_lat = lat;
            g_trip_last_lon = lon;
        }
    } else {
        g_trip_have_last = false;
    }
    if(g_lbl_trip){
        snprintf(buf, sizeof(buf), "%.2f", g_trip_km);
        LABEL_SET_IF_CHANGED(g_lbl_trip, buf);
    }

    // ── PAGE 16: DISTANCE TEST — สะสมระยะทางจริงจาก GPS ระหว่างวิ่ง
    // (วิธีเดียวกับ TRIP DISTANCE ด้านบน) แล้วจับเวลาจนครบระยะเป้าหมาย
    // ทำงานเป็น background state machine ไม่ขึ้นกับว่ากำลังเปิดหน้า 15 อยู่หรือไม่
    // เพื่อให้ผู้ใช้เริ่มจับเวลาที่หน้า 15 แล้วสลับไปดูความเร็วที่หน้าอื่นระหว่างวิ่งได้
    // AUTO START: ยังไม่ต้องกด START — เมื่อ GPS ตรวจพบว่ารถกำลังเคลื่อนที่
    // เกิน MOVE_START_KMH ให้เริ่มจับเวลาและเริ่มสะสมระยะจากตำแหน่งปัจจุบันทันที
    if(g_dist_state == DIST_READY){
        if(gps_speed_valid() && gps_speed_kmh() > MOVE_START_KMH){
            disttest_start();
        }
    }

    if(g_dist_state == DIST_RUNNING){
        if(gps_has_fix()){
            const double lat = gps_latitude();
            const double lon = gps_longitude();
            const double DEG2RAD = 0.017453292519943295;
            if(!g_dist_have_last){
                g_dist_last_lat = lat;
                g_dist_last_lon = lon;
                g_dist_have_last = true;
            } else {
                const double dlat = (lat - g_dist_last_lat) * DEG2RAD;
                const double dlon = (lon - g_dist_last_lon) * DEG2RAD;
                const double a = sin(dlat*0.5)*sin(dlat*0.5) +
                                 cos(g_dist_last_lat*DEG2RAD) * cos(lat*DEG2RAD) *
                                 sin(dlon*0.5)*sin(dlon*0.5);
                const double c = 2.0 * atan2(sqrt(a), sqrt(fmax(0.0, 1.0-a)));
                const double step_km = 6371.0 * c;
                const float gps_spd = gps_speed_kmh();
                if(step_km >= 0.002 && step_km <= 0.100 && gps_spd >= 1.0f){
                    g_dist_covered_m += (float)(step_km * 1000.0);
                }
                g_dist_last_lat = lat;
                g_dist_last_lon = lon;
            }
        }

        const float target_m = DIST_TEST_PRESETS_M[g_dist_preset_idx];
        const float run_speed = gps_speed_valid() ? gps_speed_kmh() : 0.0f;
        if(run_speed > g_dist_run_max_speed_kmh) g_dist_run_max_speed_kmh = run_speed;
        if(f_rpm > g_dist_run_max_rpm) g_dist_run_max_rpm = f_rpm;
        const uint32_t elapsed_ms = millis() - g_dist_start_ms;
        if(g_dist_covered_m >= target_m){
            // ครบระยะแล้ว: หยุดจับเวลาและคำนวณความเร็วเฉลี่ยของรอบนี้
            g_dist_result_ms = elapsed_ms;
            g_dist_result_speed_kmh = (elapsed_ms > 0)
                ? (target_m / 1000.0f) / ((float)elapsed_ms / 3600000.0f)
                : 0.0f;
            dist_run_history_push(target_m, g_dist_result_ms, g_dist_result_speed_kmh,
                                  g_dist_run_max_speed_kmh, g_dist_run_max_rpm);
            g_dist_state = DIST_DONE;
        }

        if(g_lbl_dist_status){
            char st[24]; snprintf(st, sizeof(st), "RUNNING %.0f/%.0f m", g_dist_covered_m, target_m);
            LABEL_SET_IF_CHANGED(g_lbl_dist_status, st);
            lv_obj_set_style_text_color(g_lbl_dist_status, ACCENT_WARN, 0);
        }
        if(g_lbl_dist_time){
            snprintf(buf, sizeof(buf), "%.2f", elapsed_ms / 1000.0f);
            LABEL_SET_IF_CHANGED(g_lbl_dist_time, buf);
        }
        if(g_lbl_dist_speed){
            LABEL_SET_IF_CHANGED(g_lbl_dist_speed, "--");
        }
    } else if(g_dist_state == DIST_DONE){
        if(g_lbl_dist_status){
            LABEL_SET_IF_CHANGED(g_lbl_dist_status, "DONE - TAP RESET");
            lv_obj_set_style_text_color(g_lbl_dist_status, ACCENT_OK, 0);
        }
        if(g_lbl_dist_time){
            snprintf(buf, sizeof(buf), "%.2f", g_dist_result_ms / 1000.0f);
            LABEL_SET_IF_CHANGED(g_lbl_dist_time, buf);
        }
        if(g_lbl_dist_speed){
            snprintf(buf, sizeof(buf), "%.1f", g_dist_result_speed_kmh);
            LABEL_SET_IF_CHANGED(g_lbl_dist_speed, buf);
        }
    } else { // DIST_READY
        if(g_lbl_dist_status){
            LABEL_SET_IF_CHANGED(g_lbl_dist_status, "READY - DRIVE TO START");
            lv_obj_set_style_text_color(g_lbl_dist_status, ACCENT_BLUE, 0);
        }
        if(g_lbl_dist_time)  LABEL_SET_IF_CHANGED(g_lbl_dist_time, "--");
        if(g_lbl_dist_speed) LABEL_SET_IF_CHANGED(g_lbl_dist_speed, "--");
    }

    // ── แถบ RPM หน้า 3: สเกล 0–10,000 RPM ─────────────────────────
    // g_rpm_bar2 อยู่บน g_scr_black (P03) ไม่ใช่ P02. อัปเดตทุก UI tick
    // เพื่อให้หน้า P03 เปิดขึ้นมาแล้วบาร์ใช้ RPM ล่าสุดทันที.
    {
        const float rpm_pos = constrain(f_rpm / 10000.0f * 16.0f, 0.0f, 16.0f);
        const int RPM_BAR_OK = 9;
        const int RPM_BAR_WARN = 13;
        for(int i=0; i<16; ++i){
            const float fill = constrain(rpm_pos - (float)i, 0.0f, 1.0f);
            lv_color_t c = BORDER_SUBTLE;
            if(i < RPM_BAR_OK) c = ACCENT_OK;
            else if(i < RPM_BAR_WARN) c = ACCENT_WARN;
            else c = (redline && !rl_on) ? lv_color_hex(0x401010) : ACCENT_ERR;

            set_bg_color_if_changed(g_rpm_bar2[i], c);
            set_bg_opa_if_changed(g_rpm_bar2[i],
                (fill <= 0.0f) ? LV_OPA_20 :
                (fill >= 1.0f) ? LV_OPA_COVER :
                (lv_opa_t)(LV_OPA_30 + fill * (LV_OPA_COVER - LV_OPA_30)));
        }
    }

    // ── แถบความเร็วหน้า 3: สเกล 0–160 km/h ───────────────────────
    if(g_page == 3) {
        const float spd_pos = constrain(f_speed / 160.0f * 16.0f, 0.0f, 16.0f);
        const int SPD_BAR_OK = 9;
        const int SPD_BAR_WARN = 13;
        for(int i=0; i<16; ++i){
            const float fill = constrain(spd_pos - (float)i, 0.0f, 1.0f);
            lv_color_t c = (i < SPD_BAR_OK) ? ACCENT_OK :
                           (i < SPD_BAR_WARN) ? ACCENT_WARN : ACCENT_ERR;
            set_bg_color_if_changed(g_spd_bar2[i], c);
            set_bg_opa_if_changed(g_spd_bar2[i],
                (fill <= 0.0f) ? LV_OPA_20 :
                (fill >= 1.0f) ? LV_OPA_COVER :
                (lv_opa_t)(LV_OPA_30 + fill * (LV_OPA_COVER - LV_OPA_30)));
        }
    }


    // ── หน้า 3: อัปเดตกราฟ RPM / SPEED ─────────────────────────
    static uint32_t graph_t = 0;
    if(g_page == 4 && millis() - graph_t >= 100){
        graph_t = millis();
        if(g_series_rpm && g_graph_rpm){
            lv_chart_set_next_value(g_graph_rpm, g_series_rpm, (lv_coord_t)constrain((int)f_rpm, 0, 10000));
            lv_chart_refresh(g_graph_rpm);
        }
        if(g_series_speed && g_graph_speed){
            lv_chart_set_next_value(g_graph_speed, g_series_speed, (lv_coord_t)constrain((int)f_speed, 0, 160));
            lv_chart_refresh(g_graph_speed);
        }
        if(g_lbl_graph_rpm){
            snprintf(buf, sizeof(buf), "%.0f RPM", f_rpm);
            LABEL_SET_IF_CHANGED(g_lbl_graph_rpm, buf);
        }
        if(g_lbl_graph_speed){
            snprintf(buf, sizeof(buf), "%.0f KM/H", f_speed);
            LABEL_SET_IF_CHANGED(g_lbl_graph_speed, buf);
        }
        if(g_lbl_graph_status){
            // หัวเรื่องคงเป็น REAL TIME CHART; ไม่ใช้สถานะ ECU แล้ว
            lv_obj_set_style_text_color(g_lbl_graph_status, theme_primary_text(), 0);
            LABEL_SET_IF_CHANGED(g_lbl_graph_status, "LIVE");
        }
        if(g_lbl_graph_gps_status){
            // ── จับเวลา "รถเคลื่อนที่จนหยุด" + รอบเครื่องสูงสุด (peak) ระหว่างวิ่ง ──
            if(g_move_state == MOVE_STOPPED){
                // รถเริ่มขยับจากหยุดนิ่ง -> เริ่มจับเวลารอบใหม่ รีเซ็ต peak RPM
                if(gps_speed_valid() && f_speed > MOVE_START_KMH){
                    g_move_state    = MOVE_RUNNING;
                    g_move_start_ms = millis();
                    g_move_peak_rpm = f_rpm;
                }
            } else { // MOVE_RUNNING
                // อัปเดต peak RPM ทุกครั้งที่รอบเครื่องปัจจุบันสูงกว่าค่าที่บันทึกไว้
                if(f_rpm > g_move_peak_rpm) g_move_peak_rpm = f_rpm;
                // รถกลับมาหยุดนิ่ง -> ปิดรอบ บันทึกเวลา + รอบสูงสุดที่ทำได้
                if(f_speed <= MOVE_START_KMH){
                    g_move_result_ms  = millis() - g_move_start_ms;
                    g_move_result_rpm = g_move_peak_rpm;
                    g_move_has_result = true;
                    g_move_state      = MOVE_STOPPED;
                }
            }

            if(g_move_state == MOVE_RUNNING){
                snprintf(buf, sizeof(buf), "MOVING: %.2f s",
                         (millis() - g_move_start_ms) / 1000.0f);
                LABEL_SET_IF_CHANGED(g_lbl_graph_gps_status, buf);
                lv_obj_set_style_text_color(g_lbl_graph_gps_status, ACCENT_WARN, 0);
            } else if(g_move_has_result){
                snprintf(buf, sizeof(buf), "%.2f s | PEAK %.0f RPM",
                         g_move_result_ms / 1000.0f, g_move_result_rpm);
                LABEL_SET_IF_CHANGED(g_lbl_graph_gps_status, buf);
                lv_obj_set_style_text_color(g_lbl_graph_gps_status, ACCENT_OK, 0);
            } else {
                LABEL_SET_IF_CHANGED(g_lbl_graph_gps_status, "READY");
                lv_obj_set_style_text_color(g_lbl_graph_gps_status, theme_subtext(), 0);
            }
        }
    }

    // ── P02: RPM value + peak + TPS ─────────────────────────────────
    // g_lbl_rpm/g_lbl_rpm_peak/g_lbl_tps อยู่บน g_scr_main (P02), so they
    // must be updated when P02 is visible; the previous code checked P03
    // here, which left the P02 RPM value frozen at its startup text.
    if(g_page == 2){
        snprintf(buf,sizeof(buf),"%.0f",f_rpm);      LABEL_SET_IF_CHANGED(g_lbl_rpm,buf);
        snprintf(buf,sizeof(buf),"%.0f",g_rpm_peak); LABEL_SET_IF_CHANGED(g_lbl_rpm_peak,buf);
        snprintf(buf,sizeof(buf),"%.0f",f_tps);      LABEL_SET_IF_CHANGED(g_lbl_tps,buf);
    }

    // ── แถวสถิติ BATT / INJ / ECT / INC (หน่วยต่อท้ายในตัวเลขเดียวกัน) ──
    snprintf(buf,sizeof(buf),"%.1fV",f_batt);    LABEL_SET_IF_CHANGED(g_lbl_batt,buf);
    lv_obj_set_style_text_color(g_lbl_batt,(f_batt<12.0f)?ACCENT_ERR:theme_primary_text(),0);

    snprintf(buf,sizeof(buf),"%.1f",f_inj);      LABEL_SET_IF_CHANGED(g_lbl_inj,buf);

    snprintf(buf,sizeof(buf),"%.0f",f_ect);      LABEL_SET_IF_CHANGED(g_lbl_ect,buf);
    // (แก้ไข) ค่าปกติเดิม hardcode เป็น WHITE ทำให้ตัวเลข ECT มองไม่เห็นในโหมด
    // กลางวัน (พื้นกล่องเป็นสีขาว/สว่าง) จึงเปลี่ยนมาใช้ theme_primary_text()
    // ให้สลับสีตาม DAY/NIGHT เหมือนช่อง BATT ข้างๆ
    lv_color_t ect_col=(f_ect>110)?ACCENT_ERR:(f_ect<50)?ACCENT_TEMP:theme_primary_text();
    lv_obj_set_style_text_color(g_lbl_ect,ect_col,0);

    snprintf(buf,sizeof(buf),"%.0f",f_ign);      LABEL_SET_IF_CHANGED(g_lbl_inc,buf);

    snprintf(buf,sizeof(buf),"%.0f%%",f_tps);    LABEL_SET_IF_CHANGED(g_lbl_tps_stat,buf);

    // ── AFR: ค่าประมาณจาก narrowband O2 ───────────────────────────────
    // สำคัญ: narrowband O2 ให้ข้อมูลที่เชื่อถือได้หลัก ๆ รอบ stoichiometric
    // ไม่ได้มีความสัมพันธ์แรงดัน→AFR แบบหนึ่งต่อหนึ่งตลอดช่วงเหมือน wideband
    // ดังนั้นตัวเลขนี้ต้องถือเป็น EST. และต้องคาลิเบรตด้วย wideband หากต้องการ
    // ความตรงกับ AFR จริงแบบมิเตอร์วัดส่วนผสมเชื้อเพลิง
    //
    // ใช้ median-of-3 เพื่อลด spike แต่ไม่ใช้ low-pass ที่หนักเกินไป เพราะทำให้
    // AFR lag ตามคันเร่ง/โหลดจริง
    // IMPORTANT: keep the decoder's native 0..2.5 V range intact here.
    // The ECU decoder may legitimately report values above 1.0 V; clamping to
    // 1.0 V here would flatten the lean side and make the estimated AFR wrong.
    g_afr_valid = kline_snapshot_sensor_valid(&g_kline_snapshot, KLINE_SENSOR_O2);
    float o2v_raw = g_kline_snapshot.o2_v;
    if(!isfinite(o2v_raw)) o2v_raw = 0.45f;
    if(o2v_raw < 0.0f) o2v_raw = 0.0f;
    if(o2v_raw > 2.5f) o2v_raw = 2.5f;

    // Two-stage O2 filtering:
    // 1) median-of-5 rejects short K-line/ADC spikes;
    // 2) EMA smooths the remaining signal without making it feel dead.
    static float o2_hist[5] = {0.45f, 0.45f, 0.45f, 0.45f, 0.45f};
    static uint8_t o2_hi = 0;
    o2_hist[o2_hi] = o2v_raw;
    o2_hi = (uint8_t)((o2_hi + 1) % 5);
    float o2s[5] = {o2_hist[0], o2_hist[1], o2_hist[2], o2_hist[3], o2_hist[4]};
    for(int i=1;i<5;i++){
        float key=o2s[i];
        int j=i-1;
        while(j>=0 && o2s[j]>key){ o2s[j+1]=o2s[j]; --j; }
        o2s[j+1]=key;
    }
    const float o2v_median = o2s[2];
    static float o2v_filtered = 0.45f;
    o2v_filtered = ema(o2v_filtered, o2v_median, 0.30f);
    const float o2v = o2v_filtered;

    // IMPORTANT: this motorcycle uses a conventional narrowband O2 sensor.
    // A narrowband sensor is fundamentally a rich/lean switching sensor around
    // stoichiometric. It cannot measure true AFR accurately across the whole
    // 12:1..18:1 range. The display therefore remains explicitly EST. and uses
    // a conservative, monotonic calibration curve instead of a false inverse
    // equation. Only the neighbourhood of stoich should be treated as close
    // to a real AFR measurement.
    constexpr float O2_STOICH_V = 0.45f;
    constexpr float AFR_STOICH  = 14.7f;

    // Conservative narrowband estimate curve. These anchors are intentionally
    // compressed at the rich/lean ends because the sensor has little resolving
    // power there. The 14.7 point is the only hard reference point used here.
    // For true AFR accuracy across the range, replace this with a calibrated
    // wideband transfer curve from an external reference sensor.
    static const float O2_BP[]  = {
        0.05f, 0.15f, 0.25f, 0.35f, 0.40f, 0.45f,
        0.50f, 0.55f, 0.65f, 0.75f, 0.85f, 0.95f, 1.05f
    };
    static const float AFR_BP[] = {
        17.2f, 16.6f, 16.0f, 15.3f, 14.95f, 14.70f,
        14.45f, 14.15f, 13.70f, 13.30f, 12.90f, 12.55f, 12.30f
    };
    constexpr int AFR_BP_N = sizeof(O2_BP) / sizeof(O2_BP[0]);

    float afr_est = AFR_BP[0];
    if(o2v <= O2_BP[0]) {
        afr_est = AFR_BP[0];
    } else if(o2v >= O2_BP[AFR_BP_N-1]) {
        afr_est = AFR_BP[AFR_BP_N-1];
    } else {
        for(int i=1; i<AFR_BP_N; ++i){
            if(o2v <= O2_BP[i]){
                const float x0 = O2_BP[i-1], x1 = O2_BP[i];
                const float y0 = AFR_BP[i-1], y1 = AFR_BP[i];
                const float t = (x1 > x0) ? ((o2v-x0)/(x1-x0)) : 0.0f;
                afr_est = y0 + (y1-y0)*t;
                break;
            }
        }
    }

    // Keep the published range explicit and finite.
    constexpr float AFR_MIN_EST = 12.3f;
    constexpr float AFR_MAX_EST = 17.2f;
    if(afr_est < AFR_MIN_EST) afr_est = AFR_MIN_EST;
    if(afr_est > AFR_MAX_EST) afr_est = AFR_MAX_EST;

    // Final AFR display filter: smooth normal noise, then apply a slew-rate
    // limit so one bad sample cannot make the number jump several AFR points
    // in a single UI refresh. The limit is deliberately moderate so genuine
    // rich/lean transitions remain visible in real time.
    static float afr_smooth = 14.7f;
    afr_smooth = ema(afr_smooth, afr_est, 0.30f);
    const float MAX_AFR_STEP = 0.25f;
    float afr_delta = afr_smooth - f_afr;
    if(afr_delta > MAX_AFR_STEP) afr_delta = MAX_AFR_STEP;
    if(afr_delta < -MAX_AFR_STEP) afr_delta = -MAX_AFR_STEP;
    f_afr += afr_delta;
    if(f_afr < AFR_MIN_EST) f_afr = AFR_MIN_EST;
    if(f_afr > AFR_MAX_EST) f_afr = AFR_MAX_EST;

    // ── CO2% (EST.): ประมาณจาก AFR/lambda ด้วยเส้นโค้งพีคที่จุด stoich
    // (Gaussian รอบ λ=1) - CO2 ไอเสียเบนซินสูงสุด ~15% ที่ AFR=14.7 แล้ว
    // ลดลงทั้งสองด้าน (รวยเกิน→CO/HC สูงขึ้นแทน, บางเกิน→เจือจางด้วยอากาศ
    // ส่วนเกิน) เป็นค่าประมาณดูแนวโน้มเท่านั้น ไม่ใช่การวัด CO2 จากไอเสียจริง ──
    const float CO2_PEAK = 15.5f;
    float lambda = f_afr/14.7f;
    float dl = lambda - 1.0f;
    float co2_est = CO2_PEAK * expf(-8.0f*dl*dl);
    if(co2_est<0.0f) co2_est=0.0f; if(co2_est>CO2_PEAK) co2_est=CO2_PEAK;
    f_co2 = ema(f_co2, co2_est, 0.3f);

    if(g_afr_valid) snprintf(buf,sizeof(buf),"%.1f | CO2 %.0f%%",f_afr,f_co2);
    // (แก้ไข) ตอนยังไม่มีข้อมูล AFR ให้แสดง 0.0 แทน --.-
    else snprintf(buf,sizeof(buf),"0.0 | CO2 --%%");
    LABEL_SET_IF_CHANGED(g_lbl_afr_val,buf);
    // สีของตัวเลข AFR อิงโซนเดิม (ไม่มีแถบบาร์แล้ว เหลือแค่สีตัวเลข)
    // (RICH < 13.5, OK 13.5-15.8, LEAN > 15.8)
    lv_color_t afr_col = !g_afr_valid ? theme_subtext() : ((f_afr<13.5f)?ACCENT_RICH:(f_afr>15.8f)?ACCENT_LEAN:ACCENT_OK);
    lv_obj_set_style_text_color(g_lbl_afr_val, afr_col, 0);

    // ── PAGE 14 cluster: AFR readout (แทนที่แถบ FUEL เดิม) ──────────────
    // ใช้ค่า f_afr/g_afr_valid ตัวเดียวกับหน้าหลักด้านบน ไม่คำนวณซ้ำ
    if(g_page == 14 && g_cluster_afr_val){
        if(g_afr_valid) snprintf(buf,sizeof(buf),"%.1f",f_afr);
        else snprintf(buf,sizeof(buf),"--.-");
        LABEL_SET_IF_CHANGED(g_cluster_afr_val,buf);
        set_text_color_if_changed(g_cluster_afr_val, afr_col);
    }

    // ── หน้า 5: อัปเดตกราฟ 2 เส้นในแนวนอน ─────────────────────
    // ซ้าย = RPM, ขวา = AFR, X = ตัวอย่างล่าสุดที่ไหลจากซ้ายไปขวา
    if(g_page == 5 && g_afr_rpm_line && g_afr_rpm_afr_line){
        const uint32_t now_ms = millis();
        if(now_ms - g_afr_rpm_sample_ms >= 100UL){
            g_afr_rpm_sample_ms = now_ms;

            // ใช้ขนาดเดียวกับที่คำนวณใน init (AF_W=314, AF_H=151, margin 34/19)
            const int PLOT_W = 314 - 68;   // 246
            const int PLOT_H = 151 - 19;   // 132
            const float RPM_MAX = 10000.0f;
            const float AFR_MIN = 12.0f;
            const float AFR_MAX = 18.0f;
            const int MAX_POINTS = 48;

            float rpm_v = g_t17_valid ? f_rpm : 0.0f;
            float afr_v = g_afr_valid ? f_afr : 14.7f;
            if(rpm_v < 0.0f) rpm_v = 0.0f;
            if(rpm_v > RPM_MAX) rpm_v = RPM_MAX;
            if(afr_v < AFR_MIN) afr_v = AFR_MIN;
            if(afr_v > AFR_MAX) afr_v = AFR_MAX;

            if(g_afr_rpm_count < MAX_POINTS) {
                int i = g_afr_rpm_count;
                g_afr_rpm_points[i].x = (lv_coord_t)roundf(((float)i / (MAX_POINTS-1)) * (PLOT_W-1));
                g_afr_rpm_afr_points[i].x = g_afr_rpm_points[i].x;
                g_afr_rpm_count++;
            } else {
                for(int i=1;i<MAX_POINTS;i++){
                    g_afr_rpm_points[i-1] = g_afr_rpm_points[i];
                    g_afr_rpm_afr_points[i-1] = g_afr_rpm_afr_points[i];
                }
                for(int i=0;i<MAX_POINTS;i++){
                    g_afr_rpm_points[i].x = (lv_coord_t)roundf(((float)i / (MAX_POINTS-1)) * (PLOT_W-1));
                    g_afr_rpm_afr_points[i].x = g_afr_rpm_points[i].x;
                }
                g_afr_rpm_count = MAX_POINTS;
            }

            const int idx = g_afr_rpm_count-1;
            g_afr_rpm_points[idx].y = (lv_coord_t)roundf(((RPM_MAX-rpm_v)/RPM_MAX)*(PLOT_H-1));
            g_afr_rpm_afr_points[idx].y = (lv_coord_t)roundf(((AFR_MAX-afr_v)/(AFR_MAX-AFR_MIN))*(PLOT_H-1));

            lv_line_set_points(g_afr_rpm_line, g_afr_rpm_points, g_afr_rpm_count);
            lv_line_set_points(g_afr_rpm_afr_line, g_afr_rpm_afr_points, g_afr_rpm_count);

            {
                char rpm_s[20], afr_s[20];
                if(g_t17_valid) snprintf(rpm_s, sizeof(rpm_s), "%.0f", f_rpm);
                else snprintf(rpm_s, sizeof(rpm_s), "--");
                if(g_afr_valid) snprintf(afr_s, sizeof(afr_s), "%.1f", f_afr);
                // (แก้ไข) ตอนยังไม่มีข้อมูล AFR ให้แสดง 0.0 แทน --.-
                else snprintf(afr_s, sizeof(afr_s), "0.0");
                if(g_lbl_rpm_value) {
                    LABEL_SET_IF_CHANGED(g_lbl_rpm_value, rpm_s);
                    lv_obj_set_style_text_color(g_lbl_rpm_value, ACCENT_BLUE, 0);
                }
                if(g_lbl_afr_value) {
                    LABEL_SET_IF_CHANGED(g_lbl_afr_value, afr_s);
                    lv_obj_set_style_text_color(g_lbl_afr_value,
                        !g_afr_valid ? theme_subtext() : ((f_afr < 13.5f) ? ACCENT_ERR : (f_afr > 15.8f) ? ACCENT_WARN : ACCENT_OK), 0);
                }
            }
        }
    }

    // ── หน้า 6: อัปเดต DTC / ERROR CODE ───────────────────────
    if(g_lbl_dtc_conn && g_lbl_dtc_code && g_lbl_dtc_desc){
        bool connected_now = kline_is_connected();
        LABEL_SET_IF_CHANGED(g_lbl_dtc_conn, connected_now ? "ECU CONNECTED" : "ECU NOT CONNECTED");
        lv_obj_set_style_text_color(g_lbl_dtc_conn, connected_now ? ACCENT_OK : ACCENT_WARN, 0);

        // เมื่ออยู่หน้า 6 ให้อ่าน DTC Memory เป็นระยะ เพื่อให้แสดงหลายโค้ดจริง
        // เช่น "DTCs: 007-2, 008-1, 012-1" แทนการเก็บเฉพาะโค้ดแรก
        if(g_page == 6 && connected_now && !g_dtc_page4_confirm_open && !g_dtc_page4_verify_open &&
           millis() - g_dtc_scan_ms >= 2500UL){
            g_dtc_scan_ms = millis();
            kline_request_dtc_scan();
        }

        char summary[160] = {0};
        if(g_page == 6 && connected_now && kline_get_dtc_summary(summary, sizeof(summary))){
            if(strncmp(summary, "DTCs:", 5) == 0 && strlen(summary) > 5){
                const char *list = summary + 5;
                while(*list == ' ') ++list;
                g_dtc_active = (*list != '\0');
                strncpy(g_dtc, list, sizeof(g_dtc)-1);
                g_dtc[sizeof(g_dtc)-1] = '\0';
            } else {
                g_dtc_active = false;
                g_dtc[0] = '\0';
            }
        }

        if(g_dtc_active && g_dtc[0]){
            char page_code[140];
            snprintf(page_code, sizeof(page_code), "%s", g_dtc);
            LABEL_SET_IF_CHANGED(g_lbl_dtc_code, page_code);
            lv_obj_set_style_text_font(g_lbl_dtc_code, &lv_font_montserrat_14, 0);
            lv_label_set_long_mode(g_lbl_dtc_code, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(g_lbl_dtc_code, 268);
            lv_obj_set_height(g_lbl_dtc_code, 38);
            lv_obj_set_style_text_color(g_lbl_dtc_code, ACCENT_ERR, 0);
            lv_obj_align(g_lbl_dtc_code, LV_ALIGN_TOP_MID, 0, 3);
            LABEL_SET_IF_CHANGED(g_lbl_dtc_desc, "MULTIPLE DTC CODES DETECTED");
        } else if(!connected_now){
            LABEL_SET_IF_CHANGED(g_lbl_dtc_code, "---");
            lv_obj_set_style_text_font(g_lbl_dtc_code, &A4SPEED_16, 0);
            lv_obj_set_style_text_color(g_lbl_dtc_code, ACCENT_WARN, 0);
            LABEL_SET_IF_CHANGED(g_lbl_dtc_desc, "NO ECU CONNECTION");
        } else {
            LABEL_SET_IF_CHANGED(g_lbl_dtc_code, "NO ERROR");
            lv_obj_set_style_text_font(g_lbl_dtc_code, &A4SPEED_16, 0);
            lv_obj_set_style_text_color(g_lbl_dtc_code, ACCENT_OK, 0);
            lv_obj_set_style_text_align(g_lbl_dtc_code, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_width(g_lbl_dtc_code, 268);
            lv_obj_set_height(g_lbl_dtc_code, 20);
            lv_obj_align(g_lbl_dtc_code, LV_ALIGN_TOP_MID, 0, 12);
            LABEL_SET_IF_CHANGED(g_lbl_dtc_desc, "SYSTEM OK - NO ACTIVE DTC");
        }
    }

    if(g_dtc_active){
        static bool blink=true; static uint32_t bt=0;
        if(millis()-bt>400){bt=millis(); blink=!blink;}
        char d[48]; snprintf(d,sizeof(d),"DTC %s: %s",g_dtc,honda_dtc_desc(g_dtc));
        LABEL_SET_IF_CHANGED(g_dtc_label,d);
        lv_obj_set_style_text_font(g_dtc_label,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(g_dtc_label,ACCENT_ERR,0);
        lv_obj_clear_flag(g_dtc_label, LV_OBJ_FLAG_HIDDEN);
    } else if(!kline_is_connected()){
        // ── ยังไม่ได้ต่อ ECU จริง / handshake ไม่ผ่าน กำลังใช้ค่าจำลอง ──
        static bool blink=true; static uint32_t bt=0;
        if(millis()-bt>500){bt=millis(); blink=!blink;}
        // ใช้ฟอนต์ A4SPEED เฉพาะข้อความสถานะ ECU บนหน้าแรก
        lv_obj_set_style_text_font(g_dtc_label,&A4SPEED_14,0);
        LABEL_SET_IF_CHANGED(g_dtc_label,"ECU NOT CONNECTED");
        lv_obj_set_style_text_color(g_dtc_label,ACCENT_WARN,0);
        // ตัวหนังสือกระพริบจริง (ซ่อน/แสดงสลับกัน) ไม่ใช่แค่พื้นหลัง
        if(blink) lv_obj_clear_flag(g_dtc_label, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_add_flag(g_dtc_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(g_dtc_label, LV_OBJ_FLAG_HIDDEN);
        LABEL_SET_IF_CHANGED(g_dtc_label,"SYSTEM OK - NO DTC (LIVE)");
        lv_obj_set_style_text_font(g_dtc_label,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(g_dtc_label,theme_subtext(),0);
    }
}
// ── Confirmation สำหรับ RESET TRIP ─────────────────────────────
static void page2_open_trip_confirm(){
    if(!g_trip_confirm_overlay) return;
    if(g_trip_confirm_open) return;
    g_trip_confirm_open = true;
    lv_obj_clear_flag(g_trip_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(g_trip_confirm_overlay);
    lv_refr_now(NULL);
}

static void page2_trip_confirm_no_cb(lv_event_t *e){
    if(e && lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    UI_TRACE_LINE("[UI] RESET TRIP CONFIRM -> CANCEL");
    if(g_trip_confirm_overlay){
        lv_obj_add_flag(g_trip_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    g_trip_confirm_open = false;
    g_reset_trip_hit = false;
    lv_refr_now(NULL);
}

static void page2_trip_confirm_yes_cb(lv_event_t *e){
    if(e && lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    UI_TRACE_LINE("[UI] RESET TRIP CONFIRM -> YES");
    if(g_trip_confirm_overlay){
        lv_obj_add_flag(g_trip_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    g_trip_confirm_open = false;

    g_trip_km = 0.0f;
    g_trip_have_last = false;
    g_trip_last_lat = 0.0;
    g_trip_last_lon = 0.0;
    g_reset_trip_hit = true;
    UI_TRACE_LINE("[GPS] TRIP RESET CONFIRMED");
    if(g_lbl_trip) lv_label_set_text(g_lbl_trip, "0.00");
    lv_refr_now(NULL);
}

// ── ปุ่ม CLEAR DTC จริงของ LVGL ───────────────────────────────
static void clear_dtc_btn_cb(lv_event_t *e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED) return;
    // (แก้ไข) ป้องกันไม่ให้ LVGL click event เปิด CONFIRM ค้างไว้ ขณะที่จอถูกสลับ
    // ออกจากหน้า 6 ไปแล้ว ซึ่งเป็นสาเหตุที่ทำให้กลับเข้าหน้า DTC อีกครั้งแล้วเจอ
    // "CONFIRM CLEAR DTC" เด้งขึ้นมาเองทั้งที่ยังไม่ได้กดปุ่ม
    if(g_page != 6) return;
    if(g_dtc_page4_verify_open) return;
    if(g_dtc_page4_confirm_open) return; // เปิดอยู่แล้วจาก raw touch handler
    UI_TRACE_LINE("[UI] LVGL CLEAR DTC BUTTON CLICKED -> OPEN CONFIRM ON PAGE 6");
    page4_open_confirm();
}

static void page4_open_confirm(){
    if(!g_dtc_page4_confirm_overlay) return;
    // เปิด CONFIRM ได้เฉพาะตอนที่อยู่หน้า 6 จริง ๆ เท่านั้น
    if(g_page != 6) return;
    if(g_dtc_page4_verify_open) return;
    g_dtc_page4_confirm_open = true;
    lv_obj_clear_flag(g_dtc_page4_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(g_dtc_page4_confirm_overlay);
    lv_refr_now(NULL);
}

// ปิด overlay ทั้งหมดของหน้า 6 และรีเซ็ตสถานะ — ใช้ตอนออกจากหน้า/เข้าหน้าใหม่
// เพื่อไม่ให้มี dialog ค้างข้ามรอบการเข้าหน้า
static void dtc_close_confirm_overlay(){
    if(g_dtc_page4_confirm_overlay){
        lv_obj_add_flag(g_dtc_page4_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    g_dtc_page4_confirm_open = false;
}

static void page4_confirm_no_cb(lv_event_t *e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED) return;
    UI_TRACE_LINE("[UI] PAGE4 CLEAR CONFIRM -> CANCEL");
    if(g_dtc_page4_confirm_overlay){
        lv_obj_add_flag(g_dtc_page4_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    g_dtc_page4_confirm_open = false;
    lv_refr_now(NULL);
}

static void page4_confirm_yes_cb(lv_event_t *e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED) return;
    UI_TRACE_LINE("[UI] PAGE4 CLEAR CONFIRM -> YES");
    if(g_dtc_page4_confirm_overlay){
        lv_obj_add_flag(g_dtc_page4_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    g_dtc_page4_confirm_open = false;

    // Execute CLEAR only after explicit confirmation, and keep the result on page 5.
    gauge_ui_trigger_clear_dtc();
}

// ── ทำ CLEAR DTC จริง (เรียกหลังยืนยันจากหน้า 5 เท่านั้น) ──
// Async: only kicks off the request here. The kline background task (core 0)
// does the actual read->clear->reconnect->rescan sequence and this UI thread
// never blocks waiting for it - see finalize_clear_dtc_if_ready(), polled
// from gauge_ui_update(), for where the result gets shown once ready.
static bool g_clear_dtc_awaiting_result = false;

void gauge_ui_trigger_clear_dtc(){
    UI_TRACE_LINE("[UI] CLEAR DTC triggered -> requesting async READ BEFORE / CLEAR / RESCAN");
    kline_request_clear_dtc();
    g_clear_dtc_awaiting_result = true;

    if(g_lbl_dtc_result){
        lv_label_set_text(g_lbl_dtc_result, "CLEARING...");
        lv_obj_set_style_text_color(g_lbl_dtc_result, ACCENT_WARN, 0);
    }
    if(g_dtc_page4_verify_label && g_dtc_page4_verify_overlay){
        lv_label_set_text(g_dtc_page4_verify_label,
            "CLEARING DTC...\n\nReading current codes, clearing ECU memory,\nthen reconnecting to verify.\n\nPlease wait, this can take a few seconds.");
        g_dtc_page4_verify_open = true;
        lv_obj_clear_flag(g_dtc_page4_verify_overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(g_dtc_page4_verify_overlay);
    }
}

// Polled once per gauge_ui_update() tick. Cheap no-op (single bool check)
// until kline_take_clear_dtc_result() actually has something for us.
static void finalize_clear_dtc_if_ready(){
    if(!g_clear_dtc_awaiting_result) return;

    KlineClearResult r = KLINE_CLEAR_NOT_CONNECTED;
    bool verified = false;
    char before[180];
    char after[180];
    if(!kline_take_clear_dtc_result(&r, &verified, before, sizeof(before), after, sizeof(after))){
        return; // still busy - keep showing the "please wait" state
    }
    g_clear_dtc_awaiting_result = false;

    const char *resultText = "ECU NOT CONNECTED";
    lv_color_t col = ACCENT_WARN;

    if(r == KLINE_CLEAR_OK){
        if(verified){
            resultText = "DTC CLEAR VERIFIED";
            col = ACCENT_OK;
            g_dtc_active = false;
            g_dtc[0] = '\0';
            log_clear_event("SUCCESS / VERIFIED", before);
        } else {
            resultText = "CLEAR OK / VERIFY FAILED";
            col = ACCENT_WARN;
            log_clear_event("SUCCESS / VERIFY FAIL", before);
        }
    } else if(r == KLINE_CLEAR_FAILED){
        resultText = "CLEAR FAILED - ECU NO ACK";
        col = ACCENT_ERR;
        log_clear_event("FAILED", before);
    } else {
        resultText = "ECU NOT CONNECTED";
        col = ACCENT_WARN;
        log_clear_event("NO ECU", before);
    }

    if(g_lbl_dtc_result){
        lv_label_set_text(g_lbl_dtc_result, resultText);
        lv_obj_set_style_text_color(g_lbl_dtc_result, col, 0);
    }

    if(g_dtc_page4_verify_label && g_dtc_page4_verify_overlay){
        char report[420];
        if(r == KLINE_CLEAR_OK && verified){
            snprintf(report, sizeof(report),
                     "BEFORE CLEAR\n%s\n\nCLEAR: SUCCESS\n\nAFTER RESCAN\n%s\n\nDTC CLEAR VERIFIED",
                     before, after);
        } else if(r == KLINE_CLEAR_OK){
            snprintf(report, sizeof(report),
                     "BEFORE CLEAR\n%s\n\nCLEAR: SUCCESS\n\nAFTER RESCAN\n%s\n\nVERIFY FAILED - DTC MAY STILL EXIST",
                     before, after);
        } else {
            snprintf(report, sizeof(report),
                     "BEFORE CLEAR\n%s\n\nCLEAR: %s\n\nAFTER RESCAN\n%s",
                     before, resultText, after);
        }
        lv_label_set_text(g_dtc_page4_verify_label, report);
        g_dtc_page4_verify_open = true;
        lv_obj_clear_flag(g_dtc_page4_verify_overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(g_dtc_page4_verify_overlay);
    }
}

static void page4_verify_ok_cb(lv_event_t *e){
    if(lv_event_get_code(e)!=LV_EVENT_CLICKED) return;
    UI_TRACE_LINE("[UI] PAGE4 VERIFY RESULT -> OK");
    if(g_dtc_page4_verify_overlay){
        lv_obj_add_flag(g_dtc_page4_verify_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    g_dtc_page4_verify_open = false;
    lv_refr_now(NULL);
}



static void settings_set_day_mode(bool day){
    g_day_mode = day;
    settings_apply_theme();
    lv_refr_now(NULL);
    UI_TRACE("[UI] DISPLAY MODE -> %s\n", g_day_mode ? "DAY" : "NIGHT");
}

static void settings_set_brightness(int delta){
    int v = (int)g_brightness + delta;
    if(v < 32) v = 32;
    if(v > 255) v = 255;
    g_brightness = (uint8_t)v;
    ledcWrite(BACKLIGHT_CHANNEL, g_brightness);
    settings_apply_theme();
    UI_TRACE("[UI] BRIGHTNESS -> %u / 255\n", (unsigned)g_brightness);
    lv_refr_now(NULL);
}

static void go_to_menu(){
    // กลับไปหน้าเมนูหลัก (หน้า 0) — ใช้เป็นปลายทางร่วมของการแตะทั่วไป/
    // ปุ่ม "กลับเมนู" บนทุกหน้า แทนที่การไล่หน้าถัดไปแบบเดิม (1->2->...->14->1)
    //
    // (แก้ไข) ก่อนออกจากหน้า ต้องปิด CONFIRM CLEAR DTC ที่อาจค้างอยู่เสมอ
    // มิฉะนั้นเมื่อกลับเข้าหน้า DTC อีกครั้งจะเห็น dialog เด้งขึ้นมาเอง
    dtc_close_confirm_overlay();
    // overlay แสดงผลลัพธ์ (VERIFY) ให้คงไว้เฉพาะตอนที่ยังรอผล CLEAR อยู่จริง
    // ผู้ใช้จะได้เห็นผลลัพธ์เมื่อกลับเข้าหน้า DTC; นอกนั้นให้ปิดทิ้ง
    if(!g_clear_dtc_awaiting_result && g_dtc_page4_verify_overlay){
        lv_obj_add_flag(g_dtc_page4_verify_overlay, LV_OBJ_FLAG_HIDDEN);
        g_dtc_page4_verify_open = false;
    }
    g_page = 0;
    g_show_black = false;
    if(g_scr_menu){ lv_scr_load(g_scr_menu); }
}

static void settings_back_to_main(){
    // ปุ่ม "BACK TO MENU" บนหน้า SETTINGS (หน้า 1) — เดิมไปหน้า 2 ตามรอบ,
    // ตอนนี้กลับไปหน้าเมนูแทน เพราะทุกหน้าเข้าถึงได้จากเมนูโดยตรงอยู่แล้ว
    go_to_menu();
}

void gauge_ui_handle_touch_release_raw(int32_t raw_x, int32_t raw_y, int32_t mapped_x, int32_t mapped_y){
    // PAGE 3 (SPEED DASHBOARD, g_scr_black) RESET TRIP: hit-test against the
    // button's ACTUAL on-screen box only (plus a small tap tolerance).
    // mapped_x/mapped_y are already calibrated to screen space by
    // touch_map_x()/touch_map_y() in main.cpp, so a single accurate check
    // here is enough.
    // (แก้ไข: เดิมมีการเดา 8 ทิศทางแกนจากพิกัด raw ครอบคลุมพื้นที่กว้างเกือบ
    //  ทุกมุมจอ ทำให้แตะมุมอื่นๆ ก็สั่งรีเซ็ตทริปได้ทั้งที่ไม่ได้แตะปุ่มจริง
    //  ตอนนี้เช็คเฉพาะกรอบจริงของปุ่ม RESET TRIP เท่านั้น)
    (void)raw_x;
    (void)raw_y;
    if(g_page == 3 && !g_trip_confirm_open && g_btn_reset_trip){
        lv_area_t btn_area;
        lv_obj_get_coords(g_btn_reset_trip, &btn_area);
        const int32_t TOL = 6; // px ของระยะผ่อนปรนรอบปุ่ม เผื่อแตะไม่แม่นเป๊ะ

        const bool hit = (mapped_x >= btn_area.x1 - TOL && mapped_x <= btn_area.x2 + TOL &&
                          mapped_y >= btn_area.y1 - TOL && mapped_y <= btn_area.y2 + TOL);

        if(hit){
            UI_TRACE("[UI] PAGE2 RESET TRIP HIT mapped=(%ld,%ld) btn=(%d,%d,%d,%d)\n",
                          (long)mapped_x, (long)mapped_y,
                          (int)btn_area.x1, (int)btn_area.y1, (int)btn_area.x2, (int)btn_area.y2);
            page2_open_trip_confirm();
            return;
        }
    }
    gauge_ui_handle_touch_release(mapped_x, mapped_y);
}

void gauge_ui_handle_touch_release(int32_t x, int32_t y){
    // หน้า 0: MENU — แตะแผ่นไหนก็ไปหน้านั้น ใช้ hit-test จากกรอบจริงของปุ่ม
    // เมนูหลักมี 12 แผ่น; PAGE 8 เป็นจุดเข้า sequence 8 -> 9 -> 11 -> 12 -> 13 -> MENU
    if(g_page == 0){
        for(int i = 0; i < 12; i++){
            if(!g_menu_tiles[i]) continue;
            lv_area_t a;
            lv_obj_get_coords(g_menu_tiles[i], &a);
            if(x >= a.x1 && x <= a.x2 && y >= a.y1 && y <= a.y2){
                const uint8_t target = MENU_PAGE_TARGET[i];
                UI_TRACE("[UI] MENU TILE %d TOUCH -> PAGE%u\n", i, (unsigned)target);
                g_page = target;
                g_show_black = (g_page == 3);
                switch(g_page){
                    case 1:  if(g_scr_settings)     lv_scr_load(g_scr_settings);     break;
                    case 2:  if(g_scr_main)         lv_scr_load(g_scr_main);         break;
                    case 3:  if(g_scr_black)        lv_scr_load(g_scr_black);        break;
                    case 4:  if(g_scr_graph)        lv_scr_load(g_scr_graph);        break;
                    case 5:  if(g_scr_afr_rpm)      lv_scr_load(g_scr_afr_rpm);      break;
                    case 6:
                        // (แก้ไข) เข้าหน้า DTC ทีไรต้องเริ่มจากสถานะสะอาดเสมอ
                        // ไม่มี CONFIRM CLEAR DTC ค้างจากการเข้าครั้งก่อน
                        if(!g_dtc_page4_verify_open) dtc_close_confirm_overlay();
                        if(g_scr_dtc)               lv_scr_load(g_scr_dtc);
                        break;
                    case 7:  if(g_scr_log)          lv_scr_load(g_scr_log);          break;
                    case 8:  if(g_scr_health)       lv_scr_load(g_scr_health);       break;
                    case 9:  if(g_scr_watchdog)     lv_scr_load(g_scr_watchdog);     break;
                    case 10: if(g_scr_sensors)      lv_scr_load(g_scr_sensors);      break;
                    case 11: if(g_scr_data_logger)  lv_scr_load(g_scr_data_logger);  break;
                    case 12: if(g_scr_alarm){ g_page12_view = 0; lv_scr_load(g_scr_alarm); } break;
                    case 13: if(g_scr_fuel_table)   lv_scr_load(g_scr_fuel_table);   break;
                    case 14: if(g_scr_fueltrim)     lv_scr_load(g_scr_fueltrim);     break;
                    case 15: { void *ntp = nullptr; sqxzgauge_page_get_screen(&ntp); if(ntp) lv_scr_load((lv_obj_t*)ntp); } break;
                    case 16: if(g_scr_disttest)     lv_scr_load(g_scr_disttest);     break;
                    default: break;
                }
                return;
            }
        }
        // แตะพื้นที่ว่างในหน้าเมนู (ไม่โดนแผ่นไหน) — ไม่ทำอะไร
        return;
    }

    // หน้า 3 (SPEED DASHBOARD, g_scr_black) รับสัมผัสโดยตรงจาก XPT2046
    // แยกจาก LVGL click event เพื่อไม่ให้ generic page-toggle กลืนการแตะ RESET TRIP
    // (แก้ไข: เดิมเช็ค g_page==2 ซึ่งเป็นหน้าจอเปล่าที่ไม่ได้ใช้งาน (g_scr_main)
    //  ทำให้ปุ่ม RESET TRIP กดไม่ติดเพราะเงื่อนไขนี้ไม่เคยตรงตอนแสดงหน้า dashboard จริง)
    if(g_page == 3){
        UI_TRACE("[UI] PAGE3 TOUCH RELEASE x=%ld y=%ld\n", (long)x, (long)y);

        // ถ้า dialog เปิดอยู่ ให้ raw touch จัดการปุ่ม CANCEL / YES โดยตรง
        // พิกัดอิงตำแหน่ง dialog 20,62 ขนาด 280x116 และปุ่มสูง 40px
        if(g_trip_confirm_open){
            if(x >= 20 && x <= 145 && y >= 117 && y <= 157){
                UI_TRACE_LINE("[UI] PAGE3 RESET CONFIRM -> CANCEL");
                page2_trip_confirm_no_cb(NULL);
                return;
            }
            if(x >= 160 && x <= 285 && y >= 117 && y <= 157){
                UI_TRACE_LINE("[UI] PAGE3 RESET CONFIRM -> YES");
                page2_trip_confirm_yes_cb(NULL);
                return;
            }
            UI_TRACE_LINE("[UI] PAGE3 RESET CONFIRM -> TOUCH OUTSIDE IGNORED");
            return;
        }


        // แตะส่วนอื่นของหน้า 3 (dashboard) = กลับไปหน้าเมนู
        go_to_menu();
        UI_TRACE_LINE("[UI] PAGE3 GENERAL TOUCH -> MENU");
        return;
    }

    // หน้า 1: SETTINGS รับสัมผัสโดยตรงจาก XPT2046 เพื่อไม่ให้แตะปุ่มแล้วเปลี่ยนหน้า
    if(g_page == 7){
        // PAGE 7 = LOG. แตะที่ไหนก็กลับไปหน้าเมนู (เข้าหน้าอื่นได้จากเมนูโดยตรงอยู่แล้ว)
        UI_TRACE("[UI] PAGE7 LOG TOUCH RELEASE x=%ld y=%ld -> MENU\n", (long)x, (long)y);
        go_to_menu();
        return;
    }

    // PAGE 12: ALARMS เป็นหนึ่งหน้าของชุดที่เริ่มจาก PAGE 8
    // แตะ 1 ครั้ง = ไป PAGE 13 (K-LINE TX/RX LOG)
    // แตะอีก 1 ครั้งบน PAGE 13 (ผ่าน generic handler) = กลับเมนูหลัก
    if(g_page == 12){
        UI_TRACE("[UI] PAGE12 ALARMS TOUCH RELEASE x=%ld y=%ld -> PAGE13\n",
                      (long)x, (long)y);
        g_page12_view = 0;
        if(g_scr_fuel_table){
            // (แก้ไข) ต้องอัปเดต g_page เป็น 13 ด้วย ไม่งั้นระบบยังคิดว่าอยู่หน้า 12
            // แตะกี่ครั้งก็วนโหลดหน้า 13 ซ้ำ ไม่กลับเมนูหลักสักที
            g_page = 13;
            g_show_black = false;
            lv_scr_load(g_scr_fuel_table);
            return;
        }
        go_to_menu();
        return;
    }

    // หน้า 16: DISTANCE TEST รับสัมผัสโดยตรงจาก XPT2046 เพื่อแยกปุ่ม -/+/START-STOP/RESET
    // ออกจากการแตะทั่วไป (เหมือนวิธีของหน้า 1/6) ไม่พึ่ง LVGL click event
    if(g_page == 16){
        UI_TRACE("[UI] PAGE16 DIST TEST TOUCH RELEASE x=%ld y=%ld\n", (long)x, (long)y);

        // แผงเลือกระยะทาง: กล่องอยู่ที่ (8,34) ขนาด 304x58
        // ปุ่ม [-] ที่ x 6..56 ในกล่อง -> จอจริง x 14..64, y 44..82
        if(x >= 14 && x <= 64 && y >= 44 && y <= 82){
            disttest_preset_adjust(-1);
            return;
        }
        // ปุ่ม [+] ที่ x 248..298 ในกล่อง -> จอจริง x 256..306, y 44..82
        if(x >= 256 && x <= 306 && y >= 44 && y <= 82){
            disttest_preset_adjust(+1);
            return;
        }
        // ปุ่ม AUTO START: แสดงโหมดอัตโนมัติเท่านั้น ไม่ใช้เป็น START/STOP
        if(x >= 8 && x <= 156 && y >= 190 && y <= 230){
            UI_TRACE_LINE("[UI] PAGE16 AUTO START button - no manual action");
            return;
        }
        // ปุ่ม RESET: (164,190) ขนาด 148x40
        if(x >= 164 && x <= 312 && y >= 190 && y <= 230){
            UI_TRACE_LINE("[UI] PAGE16 RESET");
            disttest_reset();
            return;
        }
        // แตะพื้นที่อื่นในหน้า 16 = กลับไปหน้าเมนู (ไม่กระทบการจับเวลาที่กำลังวิ่งอยู่เบื้องหลัง)
        go_to_menu();
        return;
    }

    // หน้า 1: SETTINGS รับสัมผัสโดยตรงจาก XPT2046 เพื่อไม่ให้แตะปุ่มแล้วเปลี่ยนหน้า
    if(g_page == 1){
        UI_TRACE("[UI] PAGE1 TOUCH RELEASE x=%ld y=%ld\n", (long)x, (long)y);
        if(x >= 145 && x <= 218 && y >= 60 && y <= 118){
            settings_set_day_mode(false);
            return;
        }
        if(x >= 218 && x <= 300 && y >= 60 && y <= 118){
            settings_set_day_mode(true);
            return;
        }
        if(x >= 5 && x <= 90 && y >= 145 && y <= 205){
            settings_set_brightness(-16);
            return;
        }
        if(x >= 220 && x <= 315 && y >= 145 && y <= 205){
            settings_set_brightness(+16);
            return;
        }
        // แตะปุ่ม GO TO PAGE 2 ด้านล่าง = ไปหน้า 2
        // Hitbox ให้ตรงกับพื้นที่ปุ่มที่แสดงจริง (back_area: x=8..312, y=198..228)
        if(x >= 8 && x <= 312 && y >= 198 && y <= 228){
            settings_back_to_main();
            return;
        }

        // หน้า 1 ต้องวนต่อได้ไม่จำกัด: หากแตะพื้นที่อื่นที่ไม่ใช่ปุ่มตั้งค่า
        // ให้ไปหน้า 2 ทันที เพื่อให้รอบ 1 -> 2 -> ... -> 7 -> 1 -> 2 ...
        // ทำงานต่อได้เรื่อย ๆ โดยไม่ต้องแตะตำแหน่งเฉพาะเดิม
        UI_TRACE("[UI] PAGE1 GENERAL TOUCH x=%ld y=%ld -> PAGE2\n", (long)x, (long)y);
        settings_back_to_main();
        return;
    }

    // หน้า 6: รับสัมผัสโดยตรงจาก XPT2046 ไม่พึ่ง LVGL click event
    if(g_page != 6) return;

    UI_TRACE("[UI] PAGE6 TOUCH RELEASE x=%ld y=%ld\n", (long)x, (long)y);

    // While the page-4 confirmation dialog is open, do NOT perform any page
    // navigation or fallback action here. LVGL must receive the release so
    // the dialog's YES/CANCEL buttons can generate their normal CLICKED event.
    if(g_dtc_page4_confirm_open || g_dtc_page4_verify_open){
        UI_TRACE_LINE("[UI] PAGE6 MODAL OPEN -> RELEASE HANDLED BY LVGL");
        return;
    }

    // CLEAR DTC has priority over the page-6 fallback.
    // The raw touch handler must open the confirmation dialog itself because
    // this page also uses a whole-screen fallback navigation rule.
    //
    // (แก้ไข) เดิมใช้กรอบตายตัว y=148..205 ซึ่ง "ไม่ตรง" กับปุ่ม CLEAR DTC จริง
    // (อยู่ที่ 50,130 ขนาด 220x40 คือ y=130..170) ทำให้เวลาแตะครึ่งบนของปุ่ม
    // เงื่อนไขนี้ไม่ตรง แล้วหลุดไป fallback -> go_to_menu() เด้งกลับหน้าแรกทันที
    // ขณะที่ LVGL ยังยิง CLICKED ของปุ่มตามมา ทำให้ CONFIRM ถูกเปิดค้างไว้
    // และไปโผล่ตอนกลับเข้าหน้า DTC ครั้งถัดไป
    // ตอนนี้ใช้พิกัดจริงของ object เหมือนวิธีของปุ่ม RESET TRIP หน้า 3
    const int32_t DTC_TOL = 6; // px ผ่อนปรนรอบปุ่ม เผื่อแตะไม่แม่นเป๊ะ

    if(g_btn_clear_dtc){
        lv_area_t a;
        lv_obj_get_coords(g_btn_clear_dtc, &a);
        if(x >= a.x1 - DTC_TOL && x <= a.x2 + DTC_TOL &&
           y >= a.y1 - DTC_TOL && y <= a.y2 + DTC_TOL){
            UI_TRACE("[UI] PAGE6 CLEAR DTC HIT x=%ld y=%ld btn=(%d,%d,%d,%d) -> OPEN CONFIRM\n",
                          (long)x, (long)y,
                          (int)a.x1, (int)a.y1, (int)a.x2, (int)a.y2);
            page4_open_confirm();
            return;
        }
    }

    // BACK TO MENU button (เดิมชื่อ NEXT PAGE): priority over the page-6 fallback.
    if(g_btn_dtc_back){
        lv_area_t a;
        lv_obj_get_coords(g_btn_dtc_back, &a);
        if(x >= a.x1 - DTC_TOL && x <= a.x2 + DTC_TOL &&
           y >= a.y1 - DTC_TOL && y <= a.y2 + DTC_TOL){
            UI_TRACE("[UI] PAGE6 BACK TO MENU x=%ld y=%ld\n", (long)x, (long)y);
            go_to_menu();
            return;
        }
    }

    // Any other touch anywhere on page 6 goes back to the menu.
    // This is intentionally the fallback action for the whole screen.
    UI_TRACE("[UI] PAGE6 TOUCH x=%ld y=%ld -> MENU\n", (long)x, (long)y);
    go_to_menu();
    return;
}

bool gauge_ui_touch_toggle_at(int32_t x, int32_t y){
    (void)x; (void)y;
    if(!g_scr_menu || !g_scr_main || !g_scr_black || !g_scr_graph || !g_scr_afr_rpm || !g_scr_dtc || !g_scr_log || !g_scr_settings ||
       !g_scr_health || !g_scr_watchdog || !g_scr_sensors || !g_scr_data_logger || !g_scr_alarm || !g_scr_performance || !g_scr_fueltrim ||
       !g_scr_fuel_table || !g_scr_disttest || !sqxzgauge_page_ready()) return false;

    // หน้าพิเศษใช้ handle_touch_release() จัดการหลังปล่อยนิ้ว
    // เพื่อแยก CLEAR DTC / ปุ่มเมนู ออกจากการแตะทั่วไปสำหรับเปลี่ยนหน้า
    // หน้า 0(เมนู)/1/3/6/7/12/16 ใช้ dedicated release handler เพื่อไม่ให้ touch ถูกกลืน
    // โดย generic page-toggle หรือ LVGL click event; PAGE 12 ใช้ handler เฉพาะเพื่อ
    // ส่งต่อไป PAGE 13 ให้ครบ sequence 8 -> 9 -> 11 -> 12 -> 13 -> MENU
    if(g_page == 0 || g_page == 1 || g_page == 3 || g_page == 6 || g_page == 7 || g_page == 12 || g_page == 16){
        return false;
    }

    // ชุด PAGE 8 จะไล่ทีละหน้า: 8 -> 9 -> 11 -> 12 -> 13 -> MENU
    // ทุกหน้าอื่นที่ไม่ได้อยู่ในชุดนี้ แตะ 1 ครั้ง = กลับเมนูตามเดิม
    uint8_t next_page = 0;
    switch(g_page){
        case 8:  next_page = 9;  break;
        case 9:  next_page = 11; break;
        case 11: next_page = 12; break;
        case 13: next_page = 0;  break;
        default: next_page = 0;  break;
    }

    if(next_page == 0){
        g_page = 0;
        g_show_black = false;
        lv_scr_load(g_scr_menu);
        return true;
    }

    g_page = next_page;
    g_show_black = (g_page == 3);
    switch(g_page){
        case 9:  if(g_scr_watchdog)    lv_scr_load(g_scr_watchdog);    break;
        case 11: if(g_scr_data_logger) lv_scr_load(g_scr_data_logger); break;
        case 12: if(g_scr_alarm){ g_page12_view = 0; lv_scr_load(g_scr_alarm); } break;
        default: return false;
    }
    return true;
}

bool gauge_ui_is_dtc_page(){
    return g_page == 6;
}

uint8_t gauge_ui_get_page(){ return g_page; }
