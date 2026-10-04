#pragma once
void gauge_ui_init();
void gauge_ui_update();
void gauge_ui_handle_touch_release(int32_t x, int32_t y);
void gauge_ui_handle_touch_release_raw(int32_t raw_x, int32_t raw_y, int32_t mapped_x, int32_t mapped_y);

bool gauge_ui_touch_toggle_at(int32_t x, int32_t y);
uint8_t gauge_ui_get_page();
uint32_t app_watchdog_feed_count();
uint32_t app_watchdog_last_feed_ms();
bool app_watchdog_ready();
void gauge_ui_trigger_clear_dtc();

// SQXZGAUGE page accessors: use the same AFR estimate/validity already computed by gauge_ui.
float gauge_ui_get_afr_est();
bool gauge_ui_is_afr_valid();
bool gauge_ui_is_day_mode();
