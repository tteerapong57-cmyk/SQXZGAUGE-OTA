#pragma once
#include <Arduino.h>

// ── ATGM336H (ชิป Zhongkewei/UC6226, รองรับ GPS+BDS ร่วมกัน) ──
// ต่อเฉพาะ 3 เส้น: VCC->3.3V(หรือ 5V ถ้าบอร์ดมีเรกูเลเตอร์ในตัว),
// GND->GND, GPS TXD -> ESP32 IO35
// (ไม่ต่อ RXD ของ GPS เข้ากับ ESP32 เลย จึงอ่านได้อย่างเดียว
//  ส่งคำสั่งไปตั้งค่าโมดูลไม่ได้ - ไม่จำเป็นสำหรับความเร็ว/เวลา)
//
// หมายเหตุ: ATGM336H ค่าเริ่มต้นจากโรงงานคือ 9600 baud, ส่ง NMEA ที่ 1Hz
// เหมือน NEO-7M เดิม จึงไม่ต้องแก้ GPS_BAUD ใน gps.cpp แต่ต่างจาก
// NEO-7M ตรงที่เป็นโมดูลรวมดาวเทียม GPS+BDS (BeiDou) จึงส่ง NMEA แบบ
// talker ID "GN" (เช่น $GNRMC, $GNGGA) แทน "GP" (เช่น $GPRMC) ล้วน ๆ
// ไลบรารี TinyGPSPlus รองรับ GNRMC/GNGGA อยู่แล้วตั้งแต่เวอร์ชันเก่า
// จึงไม่ต้องแก้โค้ด parsing ใด ๆ ทำงานได้เหมือนเดิมทันที

void  gps_init();
void  gps_update();               // เรียกทุกรอบ loop() เพื่อดึงค่าจาก Serial
bool  gps_has_fix();              // มีสัญญาณดาวเทียมล็อกตำแหน่งอยู่ไหม
float gps_speed_kmh();            // ความเร็วจาก GPS (km/h) - 0 ถ้ายังไม่ fix
double gps_latitude();             // Latitude ล่าสุด (0 ถ้ายังไม่มี fix)
double gps_longitude();            // Longitude ล่าสุด (0 ถ้ายังไม่มี fix)
void  gps_time_str(char *buf, size_t len);   // "HH:MM:SS" เวลาไทย (UTC+7)

// ── ฟังก์ชันช่วยดีบัก: เช็คว่าสาย/baud ถูกต้องไหม โดยไม่ต้องรอ fix ──
int      gps_satellites();        // จำนวนดาวเทียมที่รับสัญญาณได้ (0 ถ้ายังไม่มี/สายผิด)
uint32_t gps_chars_processed();   // จำนวนไบต์ NMEA ที่รับเข้ามาแล้วทั้งหมด (0 = ไม่มีข้อมูลเข้าเลย แปลว่าสาย/baud ผิด)

// Stability/health helpers
bool     gps_data_is_fresh();
uint32_t gps_last_data_ms();
uint32_t gps_uart_recovery_count();

// Phase 2: GPS quality / confidence helpers.
enum GpsQuality : uint8_t {
    GPS_QUALITY_NO_DATA = 0,
    GPS_QUALITY_NO_FIX,
    GPS_QUALITY_WEAK,
    GPS_QUALITY_FAIR,
    GPS_QUALITY_GOOD,
    GPS_QUALITY_EXCELLENT
};
GpsQuality gps_quality();
float      gps_hdop();
bool       gps_speed_valid();
