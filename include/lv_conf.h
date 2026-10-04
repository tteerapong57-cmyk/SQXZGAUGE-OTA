#if 1
#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0

// เปลี่ยนจาก static pool 64KB มาใช้ malloc/free ของระบบแทน
// เดิม LV_MEM_SIZE 64KB อาจไม่พอหลังเพิ่มฟอนต์ dseg7_24 เข้ามา
// ทำให้ lv_mem_alloc ล้มเหลว -> เข้า LV_USE_ASSERT_MALLOC -> ค้าง/รีเซ็ตวน
// ESP32 มี heap เหลือเฟือ (มักเกิน 200KB) ไม่ควรถูกจำกัดด้วย pool ตายตัว
#define LV_MEM_CUSTOM 1
#if LV_MEM_CUSTOM
    #define LV_MEM_CUSTOM_INCLUDE <stdlib.h>
    #define LV_MEM_CUSTOM_ALLOC   malloc
    #define LV_MEM_CUSTOM_FREE    free
    #define LV_MEM_CUSTOM_REALLOC realloc
#endif

#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

#define LV_DPI_DEF 130

// Faster touch polling and display servicing reduce perceived touch lag on
// the ESP32-CYD without forcing the main loop to busy-spin.
#define LV_INDEV_DEF_READ_PERIOD 8
#define LV_DISP_DEF_REFR_PERIOD 16

// เปิดใช้ transform (zoom/rotate) ของ widget อย่างชัดเจน กันปัญหา
// label ที่ถูก transform_zoom แล้วไม่ถูกวาด (blank) เนื่องจาก
// ค่านี้ไม่ถูกประกาศไว้เดิม ทำให้พึ่งพา default ที่ไม่แน่นอน
#define LV_DRAW_COMPLEX 1

#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR  0
#define LV_USE_LOG          0
#define LV_USE_ASSERT_NULL    1
#define LV_USE_ASSERT_MALLOC  1

#define LV_USE_ARC    1
#define LV_USE_BAR    1
#define LV_USE_BTN    1
#define LV_USE_LABEL  1
#define LV_USE_LINE   1
#define LV_USE_METER  1
#define LV_USE_IMG    1

#define LV_FONT_MONTSERRAT_10 1
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

#define LV_FONT_CUSTOM_DECLARE \
    extern const lv_font_t dseg7_24; \
    extern const lv_font_t dseg7_32; \
    extern const lv_font_t dseg7_60;

#endif
#endif
