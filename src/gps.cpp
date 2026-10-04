#include "gps.h"
#include "app_config.h"
#include <TinyGPSPlus.h>
#include <esp_system.h>

// ── Pin/UART ──────────────────────────────────────────────────
// GPS TXD -> ESP32 IO35 (input-only, ใช้รับได้ปกติ)
// ไม่ต่อ RXD ของ GPS เข้า ESP32 เลย จึงส่งค่า txPin เป็น -1
// ใช้ HardwareSerial(1) เพราะ K-line ใน kline.cpp ใช้ HardwareSerial(2) ไปแล้ว
constexpr int GPS_RX_PIN = 35;
constexpr int GPS_BAUD = 9600;

static TinyGPSPlus    gps;
static HardwareSerial GPS_SERIAL(1);

static uint32_t s_last_data_ms = 0;
static uint32_t s_uart_recovery_count = 0;
static uint32_t s_last_recovery_ms = 0;

// ── กรองความเร็ว GPS กันค่ากระตุก/ไม่นิ่ง ────────────────────────
// GPS ราคาประหยัดอย่าง ATGM336H (เดิมใช้ NEO-7M) จะมี noise สูงในค่า
// ความเร็วดิบจาก NMEA (RMC/VTG) โดยเฉพาะตอนจอดนิ่งหรือวิ่งช้า ๆ อาจเด้ง 2-8 km/h ทั้งที่
// รถไม่ได้เคลื่อนที่ (multipath/สัญญาณสะท้อนตึก) ทำให้ตัวเลขบนหน้าจอ
// "ไม่นิ่ง" สั่นกระตุกตลอดเวลา จึงกรองด้วย EMA + "zero-speed lock" แบบ
// hysteresis: ตอนจอดอยู่ ต้องเห็นค่าความเร็วเกิน SPEED_LOCK_EXIT_KMH
// ติดต่อกันหลายครั้ง (SPEED_CONFIRM_SAMPLES) ก่อนถึงจะยอมปลดล็อกว่า
// "รถวิ่งจริง" กัน noise เด้งครั้งเดียวหลุดออกจาก 0 ได้เลย
static float   s_speed_ema        = 0.0f;
static bool    s_speed_zero_lock  = true;   // เริ่มต้นล็อกไว้ที่ 0 ก่อน จนกว่าจะพิสูจน์ว่าเคลื่อนที่จริง
static uint8_t s_move_confirm     = 0;      // นับจำนวนครั้งติดต่อกันที่ความเร็ว "ดูเหมือน" วิ่งจริง

constexpr float   SPEED_EMA_ALPHA       = 0.30f; // ยิ่งน้อยยิ่งนิ่ง แต่ตอบสนองช้าลง
constexpr float   SPEED_DEADBAND_KMH    = 2.5f;  // ต่ำกว่านี้ถือว่าเป็น noise ปัดเป็น 0 (ตอนกำลังวิ่งอยู่)
constexpr float   SPEED_LOCK_EXIT_KMH   = 5.0f;  // ต้องเกินค่านี้ถึงจะเริ่มนับว่าอาจวิ่งจริง (สูงกว่า deadband พอควร กัน noise หลอก)
constexpr uint8_t SPEED_CONFIRM_SAMPLES = 3;     // ต้องเกิน LOCK_EXIT ติดกันกี่ครั้ง (~3 วินาที ที่ 1Hz) ถึงจะปลดล็อก
constexpr int     MIN_SATS_FOR_SPEED    = 4;     // ดาวเทียมน้อยกว่านี้ = fix อ่อน ไม่เชื่อค่าความเร็วที่ได้ (บังคับเป็น noise)

void gps_init(){
    GPS_SERIAL.end();
    delay(10);
    GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
    s_last_data_ms = millis();
    s_last_recovery_ms = millis();
}

void gps_update(){
    bool parsed_any = false;
    while(GPS_SERIAL.available()){
        if(gps.encode(GPS_SERIAL.read())){
            parsed_any = true;
            // มี NMEA sentence ใหม่ถูก parse สำเร็จ -> อัปเดตตัวกรองความเร็ว
            // ทำตรงนี้ (ไม่ใช่ใน gps_speed_kmh()) เพื่อให้กรองเฉพาะตอนมี
            // ข้อมูลใหม่จริง ๆ ไม่ใช่ทุกครั้งที่ UI polling เข้ามาอ่านค่า
            if(gps.speed.isValid() && gps.speed.age() < 5000){
                float raw = (float)gps.speed.kmph();

                // fix อ่อน (ดาวเทียมน้อย) -> ความเร็วดิบไม่น่าเชื่อถือ ตัดทิ้งเลย
                int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
                if(sats > 0 && sats < MIN_SATS_FOR_SPEED) raw = 0.0f;

                if(s_speed_zero_lock){
                    // ── ล็อกอยู่ที่ 0: ต้องเห็นค่าสูงเกิน LOCK_EXIT ต่อเนื่อง
                    // หลายครั้งก่อนเชื่อว่ารถเริ่มวิ่งจริง ไม่ใช่ noise เด้งครั้งเดียว ──
                    if(raw >= SPEED_LOCK_EXIT_KMH){
                        if(s_move_confirm < 255) s_move_confirm++;
                        if(s_move_confirm >= SPEED_CONFIRM_SAMPLES){
                            s_speed_zero_lock = false;
                            s_speed_ema = raw;   // เริ่มค่า EMA จากจุดที่เพิ่งยืนยันว่าวิ่งจริง กันกระตุกตอนปลดล็อก
                        }
                    } else {
                        s_move_confirm = 0;      // มีจังหวะไหนต่ำกว่าเกณฑ์ นับใหม่ตั้งแต่ต้น
                    }
                    s_speed_ema = 0.0f;
                } else {
                    // ── กำลังวิ่งอยู่: กรองด้วย EMA ตามปกติ ──
                    s_speed_ema += SPEED_EMA_ALPHA * (raw - s_speed_ema);
                    if(s_speed_ema < SPEED_DEADBAND_KMH){
                        // ความเร็วตกลงมาต่ำมาก (รถหยุด/จอด) -> กลับเข้าโหมดล็อก 0 ทันที
                        s_speed_ema = 0.0f;
                        s_speed_zero_lock = true;
                        s_move_confirm = 0;
                    }
                }
            }
        }
    }

    if(parsed_any){
        s_last_data_ms = millis();
    }

    // Recover a dead/stuck UART only when no NMEA sentence has been parsed
    // for a long time. This avoids touching the working GPS during normal use.
    const uint32_t now = millis();
    if((now - s_last_data_ms) >= GPS_UART_RECOVER_MS &&
       (now - s_last_recovery_ms) >= GPS_UART_RECOVER_MS){
        GPS_SERIAL.end();
        delay(5);
        GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
        s_last_recovery_ms = now;
        s_last_data_ms = now;
        ++s_uart_recovery_count;
    }
}

bool gps_has_fix(){
    // TinyGPS++ จะมี location.valid เมื่อได้รับ GGA/RMC ที่มีพิกัดใช้ได้
    // อย่าผูกกับอายุข้อมูลที่สั้นเกินไป เพราะ ATGM336H (เช่นเดียวกับ NEO-7M) บางตัวส่ง 1Hz และ
    // loop/display อาจทำให้ช่วงห่างแตะเกิน 3 วินาทีได้ชั่วคราว
    return gps.location.isValid() && gps.location.age() < GPS_STALE_MS;
}

float gps_speed_kmh(){
    // ความเร็วจาก RMC มีได้แม้ช่วงนั้น location จะยังไม่ผ่านเงื่อนไข fix
    // แต่ต้องมีข้อมูลล่าสุดเพื่อไม่ให้ค่าค้างนานเกินไป
    if(!gps_speed_valid()){
        // ข้อมูลเก่าเกิน/ไม่ valid -> รีเซ็ตตัวกรองทั้งหมด กันค่าค้างจากรอบก่อน
        // และกลับไปล็อกที่ 0 ไว้ก่อน (ต้องพิสูจน์ว่าวิ่งจริงใหม่อีกครั้ง)
        s_speed_ema = 0.0f;
        s_speed_zero_lock = true;
        s_move_confirm = 0;
        return 0.0f;
    }
    return s_speed_ema;      // ค่าที่กรองแล้ว (นิ่งกว่าค่าดิบจาก gps.speed.kmph())
}

double gps_latitude(){
    return (gps.location.isValid() && gps.location.age() < GPS_STALE_MS) ? gps.location.lat() : 0.0;
}

double gps_longitude(){
    return (gps.location.isValid() && gps.location.age() < GPS_STALE_MS) ? gps.location.lng() : 0.0;
}

void gps_time_str(char *buf, size_t len){
    // ── ต้อง fix ตำแหน่งจริงก่อน ถึงจะเชื่อค่าเวลาได้ ──
    // เหตุผล: บางโมดูล (รวมถึง ATGM336H และ NEO-7M) จะส่งฟิลด์เวลาออกมาเป็น "00:00:00"
    // (placeholder) ตั้งแต่ก่อนล็อกดาวเทียมได้จริง ซึ่ง gps.time.isValid()
    // จะเป็น true ทันทีที่ parse ฟิลด์นี้สำเร็จ แม้ค่าจะยังเป็น 0 อยู่ก็ตาม
    // เดิมเช็คแค่ isValid() จึงเห็นค่า 00:00:00 UTC -> บวก +7 กลายเป็น
    // "07:00:00" ค้างอยู่ตลอดตอนยังไม่ fix — แก้โดยผูกกับ gps_has_fix() ด้วย
    if(gps.time.isValid() && gps.date.isValid() &&
       gps.time.age() < GPS_STALE_MS && gps.date.age() < GPS_STALE_MS){
        // GPS ส่งเวลาแบบ UTC -> แปลงเป็นเวลาไทย (UTC+7)
        int hh = gps.time.hour() + 7;
        if(hh >= 24) hh -= 24;
        snprintf(buf, len, "%02d:%02d:%02d", hh, gps.time.minute(), gps.time.second());
    } else {
        snprintf(buf, len, "--:--:--");
    }
}

int gps_satellites(){
    return gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
}

float gps_hdop(){
    if(!gps.hdop.isValid()) return 99.9f;
    const double v = gps.hdop.hdop();
    if(!isfinite(v) || v <= 0.0) return 99.9f;
    return (float)v;
}

GpsQuality gps_quality(){
    if(!gps_data_is_fresh()) return GPS_QUALITY_NO_DATA;
    if(!gps_has_fix()) return GPS_QUALITY_NO_FIX;

    const int sats = gps_satellites();
    const float hd = gps_hdop();
    if(sats <= 0) return GPS_QUALITY_NO_FIX;

    // Use both satellite count and HDOP. If HDOP is unavailable, satellite
    // count still provides a useful fallback rather than claiming EXCELLENT.
    if(sats >= 8 && hd <= 1.5f) return GPS_QUALITY_EXCELLENT;
    if(sats >= 6 && hd <= 2.5f) return GPS_QUALITY_GOOD;
    if(sats >= 4 && hd <= 4.0f) return GPS_QUALITY_FAIR;
    return GPS_QUALITY_WEAK;
}

bool gps_speed_valid(){
    const GpsQuality q = gps_quality();
    return q >= GPS_QUALITY_FAIR && gps.speed.isValid() && gps.speed.age() < GPS_STALE_MS;
}

uint32_t gps_chars_processed(){
    return gps.charsProcessed();
}

bool gps_data_is_fresh(){
    return (millis() - s_last_data_ms) < GPS_STALE_MS;
}

uint32_t gps_last_data_ms(){
    return s_last_data_ms;
}

uint32_t gps_uart_recovery_count(){
    return s_uart_recovery_count;
}
