# Changes in this revision

- gps.cpp: fixed speed filter (EMA no longer zeroed right after unlock); GPS RX buffer 256 -> 1024 B
- kline.cpp: DTC scan/clear state is volatile + guarded by a short critical section; UI no longer
  silently drops DTC requests because the K-Line task holds the mutex; task stack 4096 -> 8192
- gauge_ui.cpp.cpp: LittleFS.begin(true); run files wrap at 20 slots; events.csv / run_summary.csv
  rotate to .old at 32 KB; free-space check before writing a run file
- app_config.h + gauge.cpp + gauge_ui.cpp.cpp: ECT / battery / RPM-scale thresholds now come from
  app_config.h (ALARM_ECT_HIGH_C, ALARM_BATT_LOW_V, RPM_GAUGE_MAX, ECT_COLD_C)
  NOTE: on-screen colour thresholds moved from ECT 110 -> 105 C and battery 12.0 -> 11.5 V
  to match the alarms. Edit app_config.h if you prefer the old values.

# Cleanup (unused code removed)
- Removed fonts A4SPEED_50, dseg7_48 (+ extern / LV_FONT_CUSTOM_DECLARE entries) and LV_FONT_MONTSERRAT_22
- Removed unused: median3, theme_panel/btn/line, disttest_startstop_pressed, g_lbl_speed_iat, GAP,
  O2_STOICH_V, AFR_STOICH, NTP_AFR_INVALID, ALARM_HYST_MS
- Removed uncalled API: kline_read_dtc_memory, kline_clear_dtc_is_busy, kline_task_runtime_ms,
  kline_sensor_valid, kline_t17/t20_last_good_ms, gauge_ui_is_dtc_page, app_touch_ready
- Removed .vscode/; renamed gauge_ui.cpp.cpp -> gauge_ui.cpp

# DAY mode graph readability (P04 / P05)
- Grid lines of the P04 chart stay night-grey (GRAPH_GRID) in DAY mode (P05 already did)
- Axis scale numbers beside the P04/P05 graphs are black in DAY mode; NIGHT keeps blue/green/amber

# Compiler warnings (yellow squiggles) fixed
- gauge_ui.cpp: misleading-indentation (2x), enum/non-enum conditional (2x, now cast to lv_opa_t)
- kline.cpp: memset on non-trivial struct (2x, cast to void*)

# C++20 deprecation warnings (IntelliSense)
- volatile ++ (sgTxRxLogCount, sgReconnectCount, g_watchdog_feed_count) -> x = x + 1
- enum/float arithmetic for LV_OPA_* in gauge_ui.cpp -> explicit int casts
- lib/lvgl/src/core/lv_obj_style.h: lv_obj_remove_style_all uses explicit uint32_t cast for PART|STATE

# DAY mode graph background (P04 / P05)
- Chart plot area (background, border, grid) stays night dark-grey in DAY mode on both pages

# P06 (DTC) layout + CLEAR DTC checks
- Even 8 px vertical spacing; card, CLEAR and BACK buttons share x=18, w=284; DTC text vertically centred in card
- CONFIRM dialog: pad 0, equal margins, buttons 12 / 144 (symmetric)
- VERIFY dialog: enlarged to 308x228, label can no longer run over OK (DOT mode), OK centred, compact report text
- CLEAR DTC guards: no CONFIRM while ECU disconnected (shows "ECU NOT CONNECTED" instead of hanging on CLEARING...),
  no second CONFIRM while a clear is still in progress
- "---" (no ECU) state re-aligns the code label (was left at the multi-DTC position)
