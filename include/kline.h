#pragma once
#include <stdint.h>

// ============================================================
//  Honda K-Line engine - Wave 110i/125i (ยืนยันแล้วว่าใช้งานได้จริง)
//  ปรับจากโค้ดที่ผู้ใช้ทดสอบกับรถจริงแล้ว เชื่อมเข้ากับ gauge_ui.cpp เดิม
// ============================================================

void kline_init();      // เรียกครั้งเดียวใน setup()
void kline_start_background(); // เริ่มงาน K-Line แยกจาก LVGL loop
void kline_update();    // worker step; ใช้ภายใน background task

bool kline_is_connected();

// ── Diagnostic TX/RX log (RAM only) ────────────────────────────────────
// newest_index=0 returns the newest transaction.
struct KlineTxRxLogEntry {
  uint32_t ms;
  uint8_t tx_len;
  uint8_t rx_len;
  bool ok;
  uint8_t tx[24];
  uint8_t rx[40];
};
uint8_t kline_txrx_log_count();
bool kline_get_txrx_log(uint8_t newest_index, KlineTxRxLogEntry *out);

// Free stack (in bytes) still remaining on the kline background task, as of
// its last measurement. Returns 0 if the task hasn't been started yet.
// Use this to confirm the 4096-byte stack given to xTaskCreatePinnedToCore()
// actually has margin instead of assuming it - if this trends toward 0 the
// stack size in kline.cpp must be increased before it silently corrupts
// memory.
uint32_t kline_task_stack_high_water_mark();

enum KlineClearResult {
    KLINE_CLEAR_OK,
    KLINE_CLEAR_FAILED,
    KLINE_CLEAR_NOT_CONNECTED,
};

// Read stored DTCs from ECU memory (queries 73/01..03).
// out receives a compact human-readable summary.
bool kline_read_dtc_memory(char *out, size_t out_len);
void kline_request_dtc_scan();
bool kline_get_dtc_summary(char *out, size_t out_len);

// ── Async clear+verify API ──────────────────────────────────────────────
// Async clear + verify. The background task performs the full read -> clear ->
// reconnect -> rescan sequence while the UI remains responsive.
void kline_request_clear_dtc();
bool kline_clear_dtc_is_busy();
// Returns true and fills the outputs exactly once when a requested clear
// finishes; returns false (outputs untouched) while still busy or if there
// is no new result to collect. Call this from the UI thread on a timer.
bool kline_take_clear_dtc_result(KlineClearResult *result, bool *verified,
                                  char *before_out, size_t before_len,
                                  char *after_out, size_t after_len);

// ── Coherent ECU data snapshot ─────────────────────────────────────────
// The K-Line task publishes one complete snapshot atomically. UI/other tasks
// copy that snapshot once per update instead of reading individual globals
// while the background task is modifying them.
struct KlineSnapshot {
  float   rpm;
  float   speed_kmh;
  float   tps;
  float   tps_volt;
  float   ect_c;
  float   iat_c;
  float   map_v;
  float   batt_v;
  float   inj_ms;
  float   ignition_deg;
  float   o2_v;
  float   stft_pct;
  float   o2_heater_v;
  uint8_t fast_idle;
  uint8_t mode;
  bool    engine_running;
  bool    connected;
  bool    speed_valid;
  bool    t17_valid;
  bool    t20_valid;
  uint32_t published_ms;
  uint32_t t17_last_good_ms;
  uint32_t t20_last_good_ms;
};

// Returns a coherent copy. Safe to call from the LVGL/UI task while the
// K-Line task is running on another core.
bool kline_get_snapshot(KlineSnapshot *out);

// Stability/health helpers
uint32_t kline_last_good_data_ms();
uint32_t kline_reconnect_count();
uint32_t kline_task_runtime_ms();

// Phase 2: per-frame sensor validity. A sensor is valid only while its
// source table has produced a fresh, structurally valid frame.
enum KlineSensor : uint8_t {
  KLINE_SENSOR_RPM = 0,
  KLINE_SENSOR_TPS,
  KLINE_SENSOR_ECT,
  KLINE_SENSOR_IAT,
  KLINE_SENSOR_MAP,
  KLINE_SENSOR_BATT,
  KLINE_SENSOR_INJ,
  KLINE_SENSOR_IGN,
  KLINE_SENSOR_O2,
  KLINE_SENSOR_STFT,
  KLINE_SENSOR_O2_HEATER
};
bool kline_sensor_valid(KlineSensor sensor);
bool kline_snapshot_sensor_valid(const KlineSnapshot *snapshot, KlineSensor sensor);
uint32_t kline_t17_last_good_ms();
uint32_t kline_t20_last_good_ms();

// ── ECM ID (Query 71, table 00) ─────────────────────────────────────────
// Read once automatically right after each successful ECU connect.
// out receives the ID payload as a space-separated hex string, e.g.
// "01 02 32 0F 01 00 00 00 00".
bool kline_get_ecm_id(char *out, size_t out_len);
bool kline_ecm_id_valid();
