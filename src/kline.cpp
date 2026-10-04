// ============================================================================
// Honda K-Line engine - Wave 110i / 125i (ทดสอบยืนยันกับรถจริงแล้ว)
// ปรับจากโค้ด standalone ของผู้ใช้ ให้เข้ากับ interface เดิมของ
// gauge_ui.cpp (kline_init/kline_update/kline_is_connected/KlineSnapshot API)
// ============================================================================

#include <Arduino.h>
#include <string.h>
#include <esp_task_wdt.h>
#include "kline.h"
#include "app_config.h"

constexpr int KLINE_RX_PIN = 22;
constexpr int KLINE_TX_PIN = 27;
constexpr int KLINE_BAUD = 10400;

#define SG_RESPONSE_GAP_MS             15
#define SG_KLINE_GAP_MS                20
#define SG_T17_TIMEOUT_MS             120
#define SG_T20_TIMEOUT_MS             120
#define SG_DTC_TIMEOUT_MS             300
#define SG_CONNECT_RETRY_MS           2500
#define SG_RECONNECT_PAUSE_MS         3000

#define SG_MAIN_PAYLOAD_MAX             32
#define SG_AUX_PAYLOAD_MAX              16
#define SG_T17_FRAME_MAX                32
#define SG_T20_FRAME_MAX                24
#define SG_DTC_FRAME_MAX                32

struct GaugeData {
  uint16_t rpm = 0;
  float tps = 0.0f;
  float tpsVolt = 0.0f;
  int16_t tempC = 0;
  int16_t iatC = 0;
  float mapVolt = 0.0f;
  uint8_t mapRaw = 0;
  float batteryV = 0.0f;
  float injectorMs = 0.0f;
  float ignitionDeg = 0.0f;
  float o2Volt = 0.0f;
  float stftPct = 0.0f;
  float o2HeaterVolt = 0.0f;
  uint8_t fastIdle = 0;
  uint8_t mode = 0;
  float speedKmh = 0.0f;   // EXPERIMENTAL: จาก table 0x10 offset 17 (CBR600RR doc)
  bool speedOk = false;    // true ถ้าเคยอ่านสำเร็จอย่างน้อย 1 ครั้ง
  bool ecuConnected = false;
  bool t17Ok = false;
  bool t20Ok = false;
};

static HardwareSerial KLINE(2);

// ── K-Line TX/RX diagnostic ring buffer ────────────────────────────────
// เก็บเฉพาะเฟรมล่าสุดใน RAM เพื่อแสดงบน PAGE 14; ยังไม่เขียนลง Flash
#define KLINE_LOG_CAP 8
#define KLINE_LOG_TX_MAX 24
#define KLINE_LOG_RX_MAX 40
static KlineTxRxLogEntry sgTxRxLog[KLINE_LOG_CAP];
static volatile uint8_t sgTxRxLogHead = 0;
static volatile uint8_t sgTxRxLogCount = 0;
static portMUX_TYPE sgTxRxLogMux = portMUX_INITIALIZER_UNLOCKED;

static void sgLogKLineTransaction(const uint8_t *tx, uint8_t txLen,
                                  const uint8_t *rx, uint8_t rxLen) {
  if (!tx || txLen == 0) return;
  uint8_t nTx = txLen > KLINE_LOG_TX_MAX ? KLINE_LOG_TX_MAX : txLen;
  uint8_t nRx = rxLen > KLINE_LOG_RX_MAX ? KLINE_LOG_RX_MAX : rxLen;
  portENTER_CRITICAL(&sgTxRxLogMux);
  KlineTxRxLogEntry &e = sgTxRxLog[sgTxRxLogHead];
  e.ms = millis();
  e.tx_len = nTx;
  e.rx_len = nRx;
  e.ok = (nRx > 0);
  memcpy(e.tx, tx, nTx);
  if (nRx && rx) memcpy(e.rx, rx, nRx);
  if (nRx < KLINE_LOG_RX_MAX) memset(e.rx + nRx, 0, KLINE_LOG_RX_MAX - nRx);
  if (nTx < KLINE_LOG_TX_MAX) memset(e.tx + nTx, 0, KLINE_LOG_TX_MAX - nTx);
  sgTxRxLogHead = (uint8_t)((sgTxRxLogHead + 1) % KLINE_LOG_CAP);
  if (sgTxRxLogCount < KLINE_LOG_CAP) sgTxRxLogCount = sgTxRxLogCount + 1;
  portEXIT_CRITICAL(&sgTxRxLogMux);
}

static void sgPrintKLineTransaction(const uint8_t *tx, uint8_t txLen,
                                    const uint8_t *rx, uint8_t rxLen) {
  Serial.print("[KLINE LOG] TX:");
  for (uint8_t i=0;i<txLen;i++) Serial.printf(" %02X", tx[i]);
  Serial.print(" | RX:");
  for (uint8_t i=0;i<rxLen;i++) Serial.printf(" %02X", rx[i]);
  Serial.println();
}

struct DtcData {
  bool busy = false;
};

static GaugeData sgGauge;
static DtcData sgDtc;
static uint8_t sgMainPayload[SG_MAIN_PAYLOAD_MAX];
static uint8_t sgAuxPayload[SG_AUX_PAYLOAD_MAX];

static uint8_t sgMainPayloadLen = 0;
static uint8_t sgAuxPayloadLen = 0;

static uint32_t sgConsecutiveFail = 0;
static uint32_t sgLastConnectTryMs = 0;
static uint32_t sgLastKLineCmdDoneMs = 0;
static uint32_t sgReconnectNotBeforeMs = 0;

static uint8_t sgPollStep = 0;
static bool sgReconnectPending = false;
static SemaphoreHandle_t sgKlineMutex = nullptr;
static TaskHandle_t sgKlineTaskHandle = nullptr;
// สถานะเหล่านี้ถูกอ่าน/เขียนข้าม core (UI core1 <-> K-Line task core0)
// จึงเป็น volatile; ส่วนสตริงใช้ critical section สั้น ๆ ด้านล่างแทน mutex ยาว
static volatile bool sgDtcScanRequested = false;
static volatile bool sgDtcScanBusy = false;
static volatile bool sgDtcSummaryValid = false;
static char sgDtcSummary[160] = "";
static portMUX_TYPE sgDtcMux = portMUX_INITIALIZER_UNLOCKED;

// ── Async clear+verify state (see kline_request_clear_dtc in kline.h) ──
static volatile bool sgClearRequested = false;
static volatile bool sgClearBusy = false;
static volatile bool sgClearDone = false;   // result ready, not yet collected by UI
static char sgClearBefore[180] = "";
static char sgClearAfter[180]  = "";
static KlineClearResult sgClearResult = KLINE_CLEAR_NOT_CONNECTED;
static bool sgClearVerified = false;

// Stability supervision
static volatile uint32_t sgLastGoodDataMs = 0;
static volatile uint32_t sgT17LastGoodMs = 0;
static volatile uint32_t sgT20LastGoodMs = 0;
static volatile uint32_t sgReconnectCount = 0;
static volatile uint32_t sgTaskLastRunMs = 0;

// ── AFR (T20) real-time tuning: เดิม T17/T20/T10 แย่งคิวกันแบบ round-robin
//    3 ทาง ทำให้ AFR (T20) อัปเดตแค่ 1 ใน 3 รอบ (ช้า/หน่วง) ── ตอนนี้ให้
//    T20 (O2/AFR) ให้ความสำคัญสูงกว่า T17 เพื่อให้ค่า AFR สดที่สุด
//    โดยยังคงอ่าน T17 เป็นระยะ และย้าย T10 (speed, experimental)
//    ไปอ่านตามเวลาจริงแทนแบบไม่แย่งคิวหลัก ─────────────────────────
#define SG_SPEED_POLL_INTERVAL_MS   250
static uint32_t sgLastT10Ms = 0;

// ── ECM ID (Query 71, table 00): read once per successful connection ──
static bool sgEcmIdValid = false;
static bool sgEcmIdRequested = false;   // set true on each fresh connect
static char sgEcmIdHex[48] = "";

static KlineSnapshot sgPublishedSnapshot = {};
static portMUX_TYPE sgSnapshotMux = portMUX_INITIALIZER_UNLOCKED;

static const uint8_t SG_WAKEUP_FE[] = {
  0xFE, 0x04, 0x72, 0x8C
};

static const uint8_t SG_CONNECT_TN[] = {
  0x2C, 0x54, 0x4E, 0x23
};

// Request ECM ID = 72 05 71 00 18  (Query 71, table 00)
static const uint8_t SG_ECMID_CMD[] = {
  0x72, 0x05, 0x71, 0x00, 0x18
};
#define SG_ECMID_FRAME_MAX  24
#define SG_ECMID_TIMEOUT_MS 120

static const uint8_t SG_T17_CMD[] = {
  0x72, 0x05, 0x71, 0x17, 0x01
};

static const uint8_t SG_T20_CMD[] = {
  0x72, 0x05, 0x71, 0x20, 0xF8
};

// EXPERIMENTAL: table 0x10 จากเอกสาร CBR600RR (offset 17 = Speed km/h)
// checksum 0x08 คำนวณแล้ว: 0x100-(0x72+0x05+0x71+0x10)=0x08 - ตรงกับเอกสารพอดี
static const uint8_t SG_T10_CMD[] = {
  0x72, 0x05, 0x71, 0x10, 0x08
};
#define SG_T10_FRAME_MAX  32
#define SG_T10_TIMEOUT_MS 120
#define SG_SPEED_OFFSET   17   // ตำแหน่งไบต์ความเร็วใน reply (นับจาก dest addr=0)

static uint8_t sgChecksum(const uint8_t *data, uint8_t count) {
  if (data == nullptr || count == 0) {
    return 0;
  }
  uint16_t sum = 0;
  for (uint8_t i = 0; i < count; ++i) {
    sum += data[i];
  }
  return (uint8_t)(0x100U - (sum & 0xFFU));
}

static void sgKLineFlushRx() {
  while (KLINE.available()) {
    (void)KLINE.read();
  }
}

static void sgKLineBegin() {
  KLINE.end();

  pinMode(KLINE_TX_PIN, OUTPUT);
  digitalWrite(KLINE_TX_PIN, HIGH);

  pinMode(KLINE_RX_PIN, INPUT);

  KLINE.begin(
    KLINE_BAUD,
    SERIAL_8N1,
    KLINE_RX_PIN,
    KLINE_TX_PIN
  );

  delay(10);
}

static uint8_t sgKLineReadUntilQuiet(
  uint8_t *out,
  uint8_t maxLen,
  uint16_t timeoutMs
) {
  if (out == nullptr || maxLen == 0) {
    return 0;
  }

  uint8_t n = 0;

  const uint32_t beginMs = millis();
  uint32_t lastByteMs = beginMs;

  while ((millis() - beginMs) < timeoutMs && n < maxLen) {

    while (KLINE.available() && n < maxLen) {
      out[n++] = (uint8_t)KLINE.read();
      lastByteMs = millis();
    }

    esp_task_wdt_reset();

    if (n > 0 &&
        (millis() - lastByteMs) >= SG_RESPONSE_GAP_MS) {
      break;
    }

    delay(1);
  }

  return n;
}

static bool sgValidateReply(
  const uint8_t *full,
  uint8_t fullLen,
  uint8_t echoLen
) {
  if (full == nullptr) {
    return false;
  }

  if (fullLen <= echoLen + 1) {
    return false;
  }

  const uint8_t replyLen = fullLen - echoLen;

  if (replyLen < 2) {
    return false;
  }

  const uint8_t *reply = full + echoLen;

  return sgChecksum(
    reply,
    replyLen - 1
  ) == reply[replyLen - 1];
}

static uint8_t sgKLineSendCapture(
  const uint8_t *tx,
  uint8_t txLen,
  uint8_t *rx,
  uint8_t rxMax,
  uint16_t timeoutMs
) {
  if (tx == nullptr || txLen == 0 ||
      rx == nullptr || rxMax == 0) {
    return 0;
  }

  sgKLineFlushRx();

  KLINE.write(tx, txLen);
  KLINE.flush();

  return sgKLineReadUntilQuiet(
    rx,
    rxMax,
    timeoutMs
  );
}

static bool sgKLineWakeup() {
  KLINE.end();

  pinMode(KLINE_TX_PIN, OUTPUT);

  digitalWrite(KLINE_TX_PIN, LOW);
  delay(70);

  digitalWrite(KLINE_TX_PIN, HIGH);
  delay(130);

  sgKLineBegin();

  uint8_t rx[32];

  uint8_t n = sgKLineSendCapture(
    SG_WAKEUP_FE,
    sizeof(SG_WAKEUP_FE),
    rx,
    sizeof(rx),
    200
  );

  if (n == 0) {
    n = sgKLineSendCapture(
      SG_CONNECT_TN,
      sizeof(SG_CONNECT_TN),
      rx,
      sizeof(rx),
      200
    );
  }

  sgGauge.ecuConnected = (n > 0);

  return sgGauge.ecuConnected;
}

static void sgScheduleReconnect(const char *reason) {
  sgReconnectCount = sgReconnectCount + 1;
  if (ENABLE_RUNTIME_DEBUG) {
    Serial.printf(
      "[K-LINE] Reconnecting... (%s)\n",
      reason ? reason : "UNKNOWN"
    );
  }

  sgGauge.ecuConnected = false;
  sgGauge.t17Ok = false;
  sgGauge.t20Ok = false;
  sgT17LastGoodMs = 0;
  sgT20LastGoodMs = 0;

  sgConsecutiveFail = 0;
  sgPollStep = 0;

  // Re-arm the one-shot ECM ID read for the next connection.
  sgEcmIdValid = false;
  sgEcmIdRequested = false;

  sgReconnectPending = true;

  sgReconnectNotBeforeMs =
    millis() + SG_RECONNECT_PAUSE_MS;

  sgLastConnectTryMs = millis();
}

static void sgEnsureConnected() {
  const uint32_t now = millis();

  if (sgGauge.ecuConnected) {
    return;
  }

  if (sgReconnectPending) {
    if ((int32_t)(now - sgReconnectNotBeforeMs) < 0) {
      return;
    }

    sgReconnectPending = false;
  }

  if (now - sgLastConnectTryMs < SG_CONNECT_RETRY_MS) {
    return;
  }

  sgLastConnectTryMs = now;

  if (sgKLineWakeup()) {
    Serial.println(
      "[ECU CONNECTED] Wave 110i/125i ECU Link Established"
    );
    // New session - the ID has to be re-read once the link is up.
    sgEcmIdValid = false;
    sgEcmIdRequested = false;
  }
}

static bool sgDecodeWaveMain() {
  if (sgMainPayloadLen < 0x12) {
    if (ENABLE_RUNTIME_DEBUG) {
      Serial.printf(
        "[DECODE] T17 payload too short: %u\n",
        sgMainPayloadLen
      );
    }

    return false;
  }

  sgGauge.rpm =
    ((uint16_t)sgMainPayload[0x04] << 8) |
    sgMainPayload[0x05];

  sgGauge.tpsVolt =
    (float)sgMainPayload[0x06] *
    (5.0f / 256.0f);

  sgGauge.tps =
    (float)sgMainPayload[0x07] * 0.5f;

  sgGauge.tempC =
    (int16_t)sgMainPayload[0x09] - 40;

  sgGauge.batteryV =
    (float)sgMainPayload[0x0E] / 10.0f;

  // Extended T17 fields used by Honda Keihin variants:
  // D15/D16 = IAT sensor voltage/temperature,
  // D17 = MAP sensor voltage, D18 = companion MAP/link byte.
  // We expose the directly documented voltage and raw companion byte;
  // no unsupported kPa conversion is invented here.
  if (sgMainPayloadLen > 0x0B) {
    sgGauge.iatC = (int16_t)sgMainPayload[0x0B] - 40;
  }
  if (sgMainPayloadLen > 0x0C) {
    sgGauge.mapVolt = (float)sgMainPayload[0x0C] * (5.0f / 256.0f);
  }
  if (sgMainPayloadLen > 0x0D) {
    sgGauge.mapRaw = sgMainPayload[0x0D];
  }
  if (sgMainPayloadLen > 0x12) {
    sgGauge.fastIdle = sgMainPayload[0x12];
  }
  if (sgMainPayloadLen > 0x13) {
    sgGauge.mode = sgMainPayload[0x13];
  }

  const uint16_t rawInj =
    ((uint16_t)sgMainPayload[0x0F] << 8) |
    sgMainPayload[0x10];

  sgGauge.injectorMs =
    (float)rawInj / 250.0f;

  sgGauge.ignitionDeg =
    ((float)sgMainPayload[0x11] / 2.0f) -
    64.0f;

  return true;
}

static bool sgDecodeWaveAux() {
  if (sgAuxPayloadLen <= 0x04) {
    if (ENABLE_RUNTIME_DEBUG) {
      Serial.printf(
        "[DECODE] T20 payload too short: %u\n",
        sgAuxPayloadLen
      );
    }

    return false;
  }

  const uint8_t o2Raw = sgAuxPayload[0x04];
  sgGauge.o2Volt = (float)o2Raw * (5.0f / 256.0f);
  if (sgGauge.o2Volt < 0.0f) sgGauge.o2Volt = 0.0f;
  if (sgGauge.o2Volt > 5.0f) sgGauge.o2Volt = 5.0f;

  // T20 documented fields: D9=O2 voltage, D10=STFT raw, D11=O2 heater voltage.
  // Treat 0x80 as the neutral center for STFT, yielding a signed percentage.
  if (sgAuxPayloadLen > 0x05) {
    const int stftRaw = (int)sgAuxPayload[0x05];
    sgGauge.stftPct = ((float)stftRaw - 128.0f) * (100.0f / 128.0f);
    if (sgGauge.stftPct < -100.0f) sgGauge.stftPct = -100.0f;
    if (sgGauge.stftPct > 100.0f) sgGauge.stftPct = 100.0f;
  }
  if (sgAuxPayloadLen > 0x06) {
    sgGauge.o2HeaterVolt = (float)sgAuxPayload[0x06] * (5.0f / 256.0f);
    if (sgGauge.o2HeaterVolt < 0.0f) sgGauge.o2HeaterVolt = 0.0f;
    if (sgGauge.o2HeaterVolt > 5.0f) sgGauge.o2HeaterVolt = 5.0f;
  }

  return true;
}

static bool sgReadTable17() {
  uint8_t frame[SG_T17_FRAME_MAX];

  sgKLineFlushRx();

  KLINE.write(
    SG_T17_CMD,
    sizeof(SG_T17_CMD)
  );

  KLINE.flush();

  const uint8_t n =
    sgKLineReadUntilQuiet(
      frame,
      sizeof(frame),
      SG_T17_TIMEOUT_MS
    );

  sgLogKLineTransaction(SG_T17_CMD, sizeof(SG_T17_CMD), frame, n);
  if (ENABLE_RUNTIME_DEBUG) {
    sgPrintKLineTransaction(SG_T17_CMD, sizeof(SG_T17_CMD), frame, n);
  }

  const uint8_t echoLen =
    sizeof(SG_T17_CMD);

  if (n < echoLen + 2 ||
      !sgValidateReply(
        frame,
        n,
        echoLen
      )) {

    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 FAIL");
    }

    return false;
  }

  const uint8_t lengthIndex =
    echoLen + 1;

  if (lengthIndex >= n) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 LENGTH INDEX FAIL");
    }

    return false;
  }

  const uint8_t declaredLen =
    frame[lengthIndex];

  if (declaredLen < 2) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 INVALID LENGTH");
    }

    return false;
  }

  const uint8_t replyLen =
    declaredLen - 1;

  if (replyLen > SG_MAIN_PAYLOAD_MAX) {
    Serial.printf(
      "[T17] Payload too large: %u\n",
      replyLen
    );

    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 OVERSIZE");
    }

    return false;
  }

  const uint16_t payloadStart = echoLen;

  if (payloadStart + replyLen > n) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 FRAME LENGTH FAIL");
    }

    return false;
  }

  memcpy(
    sgMainPayload,
    frame + payloadStart,
    replyLen
  );

  sgMainPayloadLen = replyLen;

  if (!sgDecodeWaveMain()) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 DECODE FAIL");
    }

    return false;
  }

  sgGauge.t17Ok = true;
  sgT17LastGoodMs = millis();
  sgLastGoodDataMs = sgT17LastGoodMs;
  sgConsecutiveFail = 0;

  return true;
}

static bool sgReadTable20() {
  uint8_t frame[SG_T20_FRAME_MAX];

  sgKLineFlushRx();

  KLINE.write(
    SG_T20_CMD,
    sizeof(SG_T20_CMD)
  );

  KLINE.flush();

  const uint8_t n =
    sgKLineReadUntilQuiet(
      frame,
      sizeof(frame),
      SG_T20_TIMEOUT_MS
    );

  sgLogKLineTransaction(SG_T20_CMD, sizeof(SG_T20_CMD), frame, n);
  if (ENABLE_RUNTIME_DEBUG) {
    sgPrintKLineTransaction(SG_T20_CMD, sizeof(SG_T20_CMD), frame, n);
  }

  const uint8_t echoLen =
    sizeof(SG_T20_CMD);

  if (n < echoLen + 2 ||
      !sgValidateReply(
        frame,
        n,
        echoLen
      )) {

    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 FAIL");
    }

    return false;
  }

  const uint8_t lengthIndex =
    echoLen + 1;

  if (lengthIndex >= n) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 LENGTH INDEX FAIL");
    }

    return false;
  }

  const uint8_t declaredLen =
    frame[lengthIndex];

  if (declaredLen < 2) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 INVALID LENGTH");
    }

    return false;
  }

  const uint8_t replyLen =
    declaredLen - 1;

  if (replyLen > SG_AUX_PAYLOAD_MAX) {
    Serial.printf(
      "[T20] Payload too large: %u\n",
      replyLen
    );

    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 OVERSIZE");
    }

    return false;
  }

  const uint16_t payloadStart = echoLen;

  if (payloadStart + replyLen > n) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 FRAME LENGTH FAIL");
    }

    return false;
  }

  memcpy(
    sgAuxPayload,
    frame + payloadStart,
    replyLen
  );

  sgAuxPayloadLen = replyLen;

  if (!sgDecodeWaveAux()) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 DECODE FAIL");
    }

    return false;
  }

  sgGauge.t20Ok = true;
  sgT20LastGoodMs = millis();
  sgLastGoodDataMs = sgT20LastGoodMs;
  sgConsecutiveFail = 0;

  return true;
}

// ============================================================================
// EXPERIMENTAL: table 0x10 - อ่านเฉพาะ offset 17 (Speed km/h ตามเอกสาร CBR)
// แยกอิสระจาก T17/T20 โดยสิ้นเชิง - ถ้า table นี้ใช้ไม่ได้กับ ECU รุ่นนี้
// จะแค่ไม่มีค่าความเร็ว ไม่กระทบ/ไม่นับรวมกับ sgConsecutiveFail ของตัวหลัก
// ============================================================================
static void sgReadTable10SpeedExperimental() {
  uint8_t frame[SG_T10_FRAME_MAX];

  sgKLineFlushRx();

  KLINE.write(SG_T10_CMD, sizeof(SG_T10_CMD));
  KLINE.flush();

  const uint8_t n = sgKLineReadUntilQuiet(frame, sizeof(frame), SG_T10_TIMEOUT_MS);
  sgLogKLineTransaction(SG_T10_CMD, sizeof(SG_T10_CMD), frame, n);
  if (ENABLE_RUNTIME_DEBUG) {
    sgPrintKLineTransaction(SG_T10_CMD, sizeof(SG_T10_CMD), frame, n);
  }
  const uint8_t echoLen = sizeof(SG_T10_CMD);

  if (n < echoLen + 2 || !sgValidateReply(frame, n, echoLen)) {
    // เงียบๆ ไม่ log ทุกครั้ง (กันรก serial) - log แค่ทุก ๆ ครั้งที่ 20 ครั้งที่ fail
    static uint16_t failCount = 0;
    failCount++;
    if (ENABLE_RUNTIME_DEBUG && failCount % 20 == 1) {
      Serial.printf("[T10-SPEED experimental] ยังไม่ได้ response ที่ใช้ได้ (fail #%u)\n", failCount);
    }
    return;
  }

  const uint8_t replyLen = n - echoLen;
  const uint8_t *reply = frame + echoLen;

  if (SG_SPEED_OFFSET >= replyLen) {
    static bool warnedOnce = false;
    if (ENABLE_RUNTIME_DEBUG && !warnedOnce) {
      warnedOnce = true;
      Serial.printf("[T10-SPEED experimental] reply สั้นเกินไป (%u ไบต์) เข้าไม่ถึง offset %d\n",
                     replyLen, SG_SPEED_OFFSET);
    }
    return;
  }

  sgGauge.speedKmh = (float)reply[SG_SPEED_OFFSET];
  sgGauge.speedOk = true;

  static uint32_t dbg=0;
  if (ENABLE_RUNTIME_DEBUG && millis()-dbg>2000) {
    dbg=millis();
    Serial.printf("[T10-SPEED experimental] อ่านได้! speed=%.0f km/h (raw byte @17=0x%02X)\n",
                  sgGauge.speedKmh, reply[SG_SPEED_OFFSET]);
  }
}


// ============================================================================
// READ ECM ID (Query 71, table 00)
// Request: 72 05 71 00 18
// Reply:   02 LEN 71 00 <ID bytes...> CS
// Read once right after each successful connect - the ID does not change
// while the ECU stays connected, so there is no need to re-poll it like
// T17/T20.
// ============================================================================
static bool sgReadEcmId() {
  uint8_t frame[SG_ECMID_FRAME_MAX];

  sgKLineFlushRx();
  KLINE.write(SG_ECMID_CMD, sizeof(SG_ECMID_CMD));
  KLINE.flush();

  const uint8_t n = sgKLineReadUntilQuiet(frame, sizeof(frame), SG_ECMID_TIMEOUT_MS);
  sgLogKLineTransaction(SG_ECMID_CMD, sizeof(SG_ECMID_CMD), frame, n);
  if (ENABLE_RUNTIME_DEBUG) {
    sgPrintKLineTransaction(SG_ECMID_CMD, sizeof(SG_ECMID_CMD), frame, n);
  }

  const uint8_t echoLen = sizeof(SG_ECMID_CMD);
  if (n < echoLen + 3 || !sgValidateReply(frame, n, echoLen)) {
    return false;
  }

  const uint8_t *reply = frame + echoLen;
  const uint8_t replyLen = n - echoLen;

  // 02 LEN 71 00 <ID bytes...> CS
  if (replyLen < 5 || reply[0] != 0x02 || reply[2] != 0x71 || reply[3] != 0x00) {
    return false;
  }

  const uint8_t declaredLen = reply[1];
  if (declaredLen < 3 || declaredLen > replyLen) {
    return false;
  }

  // header (71 00) = 2 bytes, checksum = 1 byte
  const uint8_t idLen = (uint8_t)(declaredLen - 3);
  const uint8_t *idBytes = reply + 4;

  char hex[sizeof(sgEcmIdHex)];
  size_t used = 0;
  for (uint8_t i = 0; i < idLen && used + 3 < sizeof(hex); ++i) {
    used += snprintf(hex + used, sizeof(hex) - used, "%s%02X",
                      (i == 0) ? "" : " ", idBytes[i]);
  }
  hex[sizeof(hex) - 1] = '\0';

  strncpy(sgEcmIdHex, hex, sizeof(sgEcmIdHex) - 1);
  sgEcmIdHex[sizeof(sgEcmIdHex) - 1] = '\0';
  sgEcmIdValid = true;

  if (ENABLE_RUNTIME_DEBUG) {
    Serial.printf("[ECM ID] %s\n", sgEcmIdHex);
  }

  return true;
}

bool kline_get_ecm_id(char *out, size_t out_len) {
  if (out == nullptr || out_len == 0) return false;
  if (!sgEcmIdValid) {
    out[0] = '\0';
    return false;
  }
  strncpy(out, sgEcmIdHex, out_len - 1);
  out[out_len - 1] = '\0';
  return true;
}

bool kline_ecm_id_valid() {
  return sgEcmIdValid;
}

// ============================================================================
// READ DTC MEMORY (Honda Keihin Query 73)
// Request: 72 05 73 xx CS, xx = memory table 01..03
// The ECU response is read together with the 5-byte request echo.
// Each response carries up to 7 DTC data bytes, where all-zero data means
// there is no stored DTC in that memory block.
// ============================================================================
static bool sgReadDtcTable73(uint8_t table, uint8_t *data, uint8_t *dataLen) {
  if (data == nullptr || dataLen == nullptr || table < 1 || table > 3) {
    return false;
  }

  uint8_t cmd[5] = { 0x72, 0x05, 0x73, table, 0x00 };
  cmd[4] = sgChecksum(cmd, 4);

  uint8_t frame[SG_DTC_FRAME_MAX];
  sgKLineFlushRx();
  KLINE.write(cmd, sizeof(cmd));
  KLINE.flush();

  const uint8_t n = sgKLineReadUntilQuiet(frame, sizeof(frame), SG_DTC_TIMEOUT_MS);
  const uint8_t echoLen = sizeof(cmd);
  if (n < echoLen + 6 || !sgValidateReply(frame, n, echoLen)) {
    return false;
  }

  const uint8_t *reply = frame + echoLen;
  const uint8_t replyLen = n - echoLen;

  // 02 LEN 73 TABLE + DATA... + CS
  if (replyLen < 5 || reply[0] != 0x02 || reply[2] != 0x73 || reply[3] != table) {
    return false;
  }

  const uint8_t declaredLen = reply[1];
  if (declaredLen < 5 || declaredLen > replyLen) {
    return false;
  }

  const uint8_t ndata = (uint8_t)(declaredLen - 5); // header(4) + checksum(1)
  if (ndata > 16) return false;

  memcpy(data, reply + 4, ndata);
  *dataLen = ndata;
  return true;
}

static bool sgReadDtcMemorySummary(char *out, size_t outLen, bool *hasDtcOut) {
  if (out == nullptr || outLen == 0) return false;
  if (hasDtcOut) *hasDtcOut = false;
  out[0] = '\0';

  if (!sgGauge.ecuConnected) {
    snprintf(out, outLen, "ECU NOT CONNECTED");
    return false;
  }

  bool allOk = true;
  bool anyDtc = false;
  bool firstShown = true;
  size_t used = 0;
  used += snprintf(out + used, outLen > used ? outLen - used : 0,
                   "DTCs:");

  for (uint8_t table = 1; table <= 3; ++table) {
    uint8_t data[16] = {0};
    uint8_t dataLen = 0;
    if (!sgReadDtcTable73(table, data, &dataLen)) {
      allOk = false;
      Serial.printf("[DTC RAW] T%u READ FAIL\n", table);
      continue;
    }

    // Honda Query 73 response payload is D9..D15 (7 bytes):
    // D9 is a count/status byte, then D10/D11, D12/D13, D14/D15 are
    // DTC pairs. Example: 01 07 02 08 01 0C 01 => 007-2, 008-1, 012-1.
    if (ENABLE_RUNTIME_DEBUG) {
      Serial.printf("[DTC RAW] T%u:", table);
      for (uint8_t i = 0; i < dataLen; ++i) {
        Serial.printf(" %02X", data[i]);
      }
      Serial.println();
    }

    bool tableHasDtc = false;
    for (uint8_t i = 1; i + 1 < dataLen; i += 2) {
      const uint8_t code = data[i];
      const uint8_t sub  = data[i + 1];
      if (code == 0x00 && sub == 0x00) continue;
      tableHasDtc = true;
      anyDtc = true;

      char dtcCode[8];
      snprintf(dtcCode, sizeof(dtcCode), "%03u-%u", (unsigned)code, (unsigned)sub);
      used += snprintf(out + used, outLen > used ? outLen - used : 0,
                       "%s%s", firstShown ? " " : ", ", dtcCode);
      firstShown = false;
    }

    // A non-zero count with no complete pair is still suspicious, but do not
    // fabricate a DTC code from the count byte.
    if (dataLen > 0 && data[0] != 0 && !tableHasDtc && dataLen < 3) {
      allOk = false;
    }

    if (used >= outLen) used = outLen - 1;
    out[used] = '\0';
    delay(SG_KLINE_GAP_MS);
  }

  if (!anyDtc && allOk) {
    snprintf(out, outLen, "NO STORED DTC");
  }
  if (hasDtcOut) *hasDtcOut = anyDtc;
  return allOk;
}

void kline_request_dtc_scan(){
  // (แก้) เดิมพยายามจับ sgKlineMutex แบบ timeout 0 แต่ K-Line task ถือ mutex
  // ไว้เกือบตลอดเวลา คำขอจึงถูกทิ้งเงียบ ๆ บ่อย ตอนนี้แค่ตั้ง flag ให้ task ไปทำเอง
  if (sgGauge.ecuConnected && !sgDtcScanBusy) {
    sgDtcScanRequested = true;
    portENTER_CRITICAL(&sgDtcMux);
    sgDtcSummaryValid = false;
    portEXIT_CRITICAL(&sgDtcMux);
  }
}

bool kline_get_dtc_summary(char *out, size_t outLen){
  if (out == nullptr || outLen == 0) return false;

  portENTER_CRITICAL(&sgDtcMux);
  const bool valid = sgDtcSummaryValid;
  if (valid) {
    strncpy(out, sgDtcSummary, outLen - 1);
    out[outLen - 1] = '\0';
  } else {
    out[0] = '\0';
  }
  portEXIT_CRITICAL(&sgDtcMux);
  return valid;
}

static bool sgDtcClearInternal() {
  if (sgGauge.rpm > 0) {
    Serial.println("[DTC] Cannot clear while engine is running!");
    return false;
  }

  Serial.println("[DTC] Clearing DTC memories 01..03...");
  sgGauge.ecuConnected = false;
  sgKLineFlushRx();

  if (!sgKLineWakeup()) {
    Serial.println("[DTC] CLEAR WAKEUP FAIL");
    sgScheduleReconnect("CLEAR WAKEUP FAIL");
    return false;
  }

  bool allAck = true;

  for (uint8_t table = 1; table <= 3; ++table) {
    uint8_t cmd[5] = {0x72, 0x05, 0x60, table, 0x00};
    cmd[4] = sgChecksum(cmd, 4);

    uint8_t rx[16] = {0};
    const uint8_t n = sgKLineSendCapture(cmd, sizeof(cmd), rx, sizeof(rx), SG_DTC_TIMEOUT_MS);

    if (ENABLE_RUNTIME_DEBUG) {
      Serial.printf("[DTC RAW] CLEAR T%u TX:", table);
      for (uint8_t i = 0; i < sizeof(cmd); ++i) Serial.printf(" %02X", cmd[i]);
      Serial.println();
      Serial.printf("[DTC RAW] CLEAR T%u RX:", table);
      for (uint8_t i = 0; i < n; ++i) Serial.printf(" %02X", rx[i]);
      Serial.println();
    }

    bool ack = false;
    if (n >= 10 && sgValidateReply(rx, n, 5)) {
      const uint8_t *reply = rx + 5;
      const uint8_t replyLen = n - 5;
      // Expected positive response: 02 05 60 TABLE CS
      ack = (replyLen >= 5 && reply[0] == 0x02 && reply[1] == 0x05 &&
             reply[2] == 0x60 && reply[3] == table);
    }

    Serial.printf("[DTC] CLEAR T%u: %s\n", table, ack ? "ACK" : "NO ACK");
    if (!ack) allAck = false;
    delay(SG_KLINE_GAP_MS);
  }

  Serial.println(allAck ? "[DTC] Clear SUCCESS!" : "[DTC] Clear FAILED.");

  sgScheduleReconnect("CLEAR DONE");
  return allAck;
}

static void sgEngineTask() {
  const uint32_t now = millis();

  sgTaskLastRunMs = millis();
  sgEnsureConnected();

  if (!sgGauge.ecuConnected) {
    return;
  }

  // A successful T17/T20 frame refreshes sgLastGoodDataMs. If both stop
  // arriving, force a clean ECU session instead of keeping stale values alive.
  const uint32_t nowAfterConnect = millis();
  if (sgLastGoodDataMs != 0 &&
      (nowAfterConnect - sgLastGoodDataMs) >= KLINE_LINK_STALE_MS &&
      !sgDtcScanBusy && !sgClearBusy) {
    sgScheduleReconnect("LINK STALE");
    return;
  }

  if (now - sgLastKLineCmdDoneMs <
      SG_KLINE_GAP_MS) {
    return;
  }

  // ── ECM ID: one-shot read right after connecting, before the regular
  //    T17/T20 poll loop starts. Failure just leaves it unread for
  //    this session; it isn't critical enough to trigger a reconnect.
  if (!sgEcmIdRequested) {
    sgEcmIdRequested = true;
    sgReadEcmId();
    sgLastKLineCmdDoneMs = millis();
    return;
  }

  // ── T10 speed is experimental and disabled in the normal stable build.
  // GPS is the dashboard speed source, so T10 should not consume K-Line bus
  // time unless explicitly enabled for diagnostic testing.
  if (ENABLE_KLINE_T10_SPEED &&
      (now - sgLastT10Ms >= SG_SPEED_POLL_INTERVAL_MS)) {
    sgReadTable10SpeedExperimental();
    sgLastT10Ms = now;
  }

  // RPM (T17) is the primary real-time gauge value. Poll T17 on 2 of every
  // 3 cycles and keep T20 (AFR/O2) on the remaining cycle, so RPM latency
  // stays low instead of waiting through a 3-way round-robin schedule.
  if (sgPollStep < 2) {
    sgReadTable17();
  } else {
    sgReadTable20();
  }
  sgPollStep = (sgPollStep + 1) % 3;

  sgLastKLineCmdDoneMs =
    millis();
}

static void sgProcessDtcScanRequest() {
  if (!sgDtcScanRequested || sgDtcScanBusy || !sgGauge.ecuConnected) {
    return;
  }

  sgDtcScanRequested = false;
  sgDtcScanBusy = true;

  char summary[sizeof(sgDtcSummary)] = {0};
  const bool ok = sgReadDtcMemorySummary(summary, sizeof(summary), nullptr);
  if (ok) {
    portENTER_CRITICAL(&sgDtcMux);
    strncpy(sgDtcSummary, summary, sizeof(sgDtcSummary) - 1);
    sgDtcSummary[sizeof(sgDtcSummary) - 1] = '\0';
    sgDtcSummaryValid = true;
    portEXIT_CRITICAL(&sgDtcMux);
  }

  sgDtcScanBusy = false;
}

// Runs entirely inside the K-Line background task (called from kline_update(),
// which already owns sgKlineMutex). The UI only requests the operation and
// polls for the result, so slow ECU responses never block LVGL.
static void sgProcessClearDtcRequest() {
  if (!sgClearRequested || sgClearBusy) return;

  sgClearRequested = false;
  sgClearBusy = true;
  sgClearDone = false;

  char before[sizeof(sgClearBefore)] = "BEFORE: SCAN FAILED";
  char after[sizeof(sgClearAfter)]   = "AFTER: RESCAN NOT RUN";
  KlineClearResult r = KLINE_CLEAR_NOT_CONNECTED;
  bool verified = false;

  bool hadBefore = false;
  const bool beforeOk = sgReadDtcMemorySummary(before, sizeof(before), &hadBefore);
  if (ENABLE_RUNTIME_DEBUG) {
    Serial.printf("[DTC] BEFORE SCAN: ok=%d hasDTC=%d %s\n",
                  beforeOk ? 1 : 0, hadBefore ? 1 : 0, before);
  }

  if (sgGauge.ecuConnected) {
    r = sgDtcClearInternal() ? KLINE_CLEAR_OK : KLINE_CLEAR_FAILED;
  }

  if (r == KLINE_CLEAR_OK) {
    // Clear intentionally puts the ECU back into the disconnected state.
    // Re-open the session immediately for the verification scan rather than
    // waiting for the normal background reconnect timer.
    sgReconnectPending = false;
    sgReconnectNotBeforeMs = 0;
    delay(120);

    if (!sgKLineWakeup()) {
      snprintf(after, sizeof(after), "RESCAN FAILED: ECU NO RESPONSE");
      if (ENABLE_RUNTIME_DEBUG) {
        Serial.println("[DTC] AFTER SCAN: ECU did not reconnect");
      }
    } else {
      bool hadAfter = false;
      const bool afterOk = sgReadDtcMemorySummary(after, sizeof(after), &hadAfter);
      if (ENABLE_RUNTIME_DEBUG) {
        Serial.printf("[DTC] AFTER SCAN: ok=%d hasDTC=%d %s\n",
                      afterOk ? 1 : 0, hadAfter ? 1 : 0, after);
      }
      if (afterOk) {
        portENTER_CRITICAL(&sgDtcMux);
        strncpy(sgDtcSummary, after, sizeof(sgDtcSummary) - 1);
        sgDtcSummary[sizeof(sgDtcSummary) - 1] = '\0';
        sgDtcSummaryValid = true;
        portEXIT_CRITICAL(&sgDtcMux);
      }
      verified = afterOk && !hadAfter;
      if (ENABLE_RUNTIME_DEBUG) {
        Serial.println(verified ? "[DTC] VERIFY: NO STORED DTC - CLEAR CONFIRMED"
                                : "[DTC] VERIFY: DTC STILL PRESENT / SCAN FAILED");
      }
    }
  } else {
    snprintf(after, sizeof(after), "RESCAN SKIPPED: CLEAR FAILED");
  }

  portENTER_CRITICAL(&sgDtcMux);
  strncpy(sgClearBefore, before, sizeof(sgClearBefore) - 1);
  sgClearBefore[sizeof(sgClearBefore) - 1] = '\0';
  strncpy(sgClearAfter, after, sizeof(sgClearAfter) - 1);
  sgClearAfter[sizeof(sgClearAfter) - 1] = '\0';
  sgClearResult   = r;
  sgClearVerified = verified;
  sgClearBusy     = false;
  sgClearDone     = true;
  portEXIT_CRITICAL(&sgDtcMux);
}

void kline_request_clear_dtc(){
  if (sgClearBusy || sgClearRequested) return; // already in progress
  sgClearDone = false;
  sgClearRequested = true;
}

bool kline_take_clear_dtc_result(KlineClearResult *result, bool *verified,
                                  char *before_out, size_t before_len,
                                  char *after_out, size_t after_len){
  if (!sgClearDone) return false;

  portENTER_CRITICAL(&sgDtcMux);
  if (result) *result = sgClearResult;
  if (verified) *verified = sgClearVerified;
  if (before_out && before_len) {
    strncpy(before_out, sgClearBefore, before_len - 1);
    before_out[before_len - 1] = '\0';
  }
  if (after_out && after_len) {
    strncpy(after_out, sgClearAfter, after_len - 1);
    after_out[after_len - 1] = '\0';
  }
  sgClearDone = false; // consumed
  portEXIT_CRITICAL(&sgDtcMux);
  return true;
}

static void sgKlineTask(void *param) {
  (void)param;

  // Watch the ECU task too. Every blocking ECU read has a bounded timeout,
  // so a healthy loop can safely feed the task watchdog between transactions.
  // If this task ever truly hangs, the ESP32 can recover instead of freezing
  // the dashboard forever.
  const esp_err_t wdtAddErr = esp_task_wdt_add(NULL);
  if(wdtAddErr == ESP_OK || wdtAddErr == ESP_ERR_INVALID_STATE){
    Serial.println("[K-LINE] task watchdog attached");
  }

  for (;;) {
    kline_update();
    esp_task_wdt_reset();
    // Keep the task responsive without busy-spinning.
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void publishSnapshot() {
  KlineSnapshot next{};
  next.rpm            = sgGauge.rpm;
  next.speed_kmh      = sgGauge.speedKmh;
  next.tps            = sgGauge.tps;
  next.tps_volt       = sgGauge.tpsVolt;
  next.ect_c          = sgGauge.tempC;
  next.iat_c          = sgGauge.iatC;
  next.map_v          = sgGauge.mapVolt;
  next.batt_v         = sgGauge.batteryV;
  next.inj_ms         = sgGauge.injectorMs;
  next.ignition_deg   = sgGauge.ignitionDeg;
  next.o2_v           = sgGauge.o2Volt;
  next.stft_pct       = sgGauge.stftPct;
  next.o2_heater_v    = sgGauge.o2HeaterVolt;
  next.fast_idle      = sgGauge.fastIdle;
  next.mode            = sgGauge.mode;
  next.engine_running = (sgGauge.rpm > 200);
  next.connected      = sgGauge.ecuConnected;
  next.speed_valid    = sgGauge.speedOk;
  next.t17_valid      = sgGauge.t17Ok;
  next.t20_valid      = sgGauge.t20Ok;
  next.published_ms   = millis();
  next.t17_last_good_ms = sgT17LastGoodMs;
  next.t20_last_good_ms = sgT20LastGoodMs;

  portENTER_CRITICAL(&sgSnapshotMux);
  sgPublishedSnapshot = next;
  portEXIT_CRITICAL(&sgSnapshotMux);
}

bool kline_get_snapshot(KlineSnapshot *out) {
  if (!out) return false;
  portENTER_CRITICAL(&sgSnapshotMux);
  *out = sgPublishedSnapshot;
  portEXIT_CRITICAL(&sgSnapshotMux);
  return true;
}

void kline_init(){
  memset((void*)&sgGauge, 0, sizeof(sgGauge));
  portENTER_CRITICAL(&sgSnapshotMux);
  memset(&sgPublishedSnapshot, 0, sizeof(sgPublishedSnapshot));
  portEXIT_CRITICAL(&sgSnapshotMux);
  sgLastGoodDataMs = millis();
  sgT17LastGoodMs = 0;
  sgT20LastGoodMs = 0;
  sgReconnectCount = 0;
  sgTaskLastRunMs = millis();
  memset((void*)&sgDtc, 0, sizeof(sgDtc));
  memset(sgMainPayload, 0, sizeof(sgMainPayload));
  memset(sgAuxPayload, 0, sizeof(sgAuxPayload));
  sgDtcScanRequested = false;
  sgDtcScanBusy = false;
  portENTER_CRITICAL(&sgDtcMux);
  sgDtcSummaryValid = false;
  sgDtcSummary[0] = '\0';
  portEXIT_CRITICAL(&sgDtcMux);
  sgEcmIdValid = false;
  sgEcmIdRequested = false;
  sgEcmIdHex[0] = '\0';

  Serial.println("=================================================");
  Serial.println("   WAVE 110i / 125i K-LINE ENGINE INITIALIZED   ");
  Serial.println("=================================================");

  sgKLineBegin();
  // Allow the first background update to connect immediately instead of
  // waiting for the normal retry interval after power-up.
  sgLastConnectTryMs = millis() - SG_CONNECT_RETRY_MS;

  if (sgKlineMutex == nullptr) {
    sgKlineMutex = xSemaphoreCreateMutex();
  }
}

void kline_update(){
  if (sgKlineMutex != nullptr) {
    if (xSemaphoreTake(sgKlineMutex, pdMS_TO_TICKS(250)) != pdTRUE) {
      return;
    }
  }

  sgEngineTask();
  sgProcessDtcScanRequest();
  sgProcessClearDtcRequest();
  publishSnapshot();

  if (sgKlineMutex != nullptr) {
    xSemaphoreGive(sgKlineMutex);
  }
}

uint32_t kline_task_stack_high_water_mark(){
  if (sgKlineTaskHandle == nullptr) return 0;
  // uxTaskGetStackHighWaterMark returns the minimum free stack (in words)
  // ever recorded for this task since it started; multiply by 4 for bytes
  // on ESP32 (32-bit words).
  return (uint32_t)uxTaskGetStackHighWaterMark(sgKlineTaskHandle) * sizeof(StackType_t);
}

void kline_start_background(){
  if (sgKlineTaskHandle != nullptr) return;

  if (sgKlineMutex == nullptr) {
    sgKlineMutex = xSemaphoreCreateMutex();
    if (sgKlineMutex == nullptr) {
      Serial.println("[K-LINE] ERROR: mutex allocation failed");
      return;
    }
  }

  BaseType_t ok = xTaskCreatePinnedToCore(
      sgKlineTask,
      "kline",
      8192,   // เดิม 4096: path ลบ DTC มี buffer ~520 B + Serial.printf กิน stack มาก
      nullptr,
      2,
      &sgKlineTaskHandle,
      0);

  if (ok != pdPASS) {
    sgKlineTaskHandle = nullptr;
    Serial.println("[K-LINE] ERROR: background task creation failed");
  } else {
    Serial.println("[K-LINE] background task started on core 0");
  }
}

bool kline_is_connected(){
  KlineSnapshot s{};
  return kline_get_snapshot(&s) && s.connected;
}

uint8_t kline_txrx_log_count(){
  portENTER_CRITICAL(&sgTxRxLogMux);
  uint8_t n = sgTxRxLogCount;
  portEXIT_CRITICAL(&sgTxRxLogMux);
  return n;
}

bool kline_get_txrx_log(uint8_t newest_index, KlineTxRxLogEntry *out){
  if(!out) return false;
  portENTER_CRITICAL(&sgTxRxLogMux);
  if(newest_index >= sgTxRxLogCount){
    portEXIT_CRITICAL(&sgTxRxLogMux);
    return false;
  }
  int idx = (int)sgTxRxLogHead - 1 - (int)newest_index;
  while(idx < 0) idx += KLINE_LOG_CAP;
  *out = sgTxRxLog[idx];
  portEXIT_CRITICAL(&sgTxRxLogMux);
  return true;
}

uint32_t kline_last_good_data_ms(){
  return sgLastGoodDataMs;
}

uint32_t kline_reconnect_count(){
  return sgReconnectCount;
}

static bool snapshotT17Fresh(const KlineSnapshot &s, uint32_t now){
  return s.connected && s.t17_valid && s.t17_last_good_ms != 0 &&
         (now - s.t17_last_good_ms) < KLINE_LINK_STALE_MS;
}

static bool snapshotT20Fresh(const KlineSnapshot &s, uint32_t now){
  return s.connected && s.t20_valid && s.t20_last_good_ms != 0 &&
         (now - s.t20_last_good_ms) < KLINE_LINK_STALE_MS;
}

bool kline_snapshot_sensor_valid(const KlineSnapshot *snapshot, KlineSensor sensor){
  if (!snapshot) return false;
  const KlineSnapshot &s = *snapshot;
  const uint32_t now = millis();
  const bool t17 = snapshotT17Fresh(s, now);
  const bool t20 = snapshotT20Fresh(s, now);

  switch(sensor){
    case KLINE_SENSOR_RPM:
      return t17 && s.rpm >= 0.0f && s.rpm <= 12000.0f;
    case KLINE_SENSOR_TPS:
      return t17 && isfinite(s.tps) && s.tps >= 0.0f && s.tps <= 100.0f;
    case KLINE_SENSOR_ECT:
      return t17 && s.ect_c >= -40.0f && s.ect_c <= 160.0f;
    case KLINE_SENSOR_IAT:
      return t17 && s.iat_c >= -40.0f && s.iat_c <= 160.0f;
    case KLINE_SENSOR_MAP:
      return t17 && isfinite(s.map_v) && s.map_v >= 0.0f && s.map_v <= 5.1f;
    case KLINE_SENSOR_BATT:
      return t17 && isfinite(s.batt_v) && s.batt_v >= 0.0f && s.batt_v <= 20.0f;
    case KLINE_SENSOR_INJ:
      return t17 && isfinite(s.inj_ms) && s.inj_ms >= 0.0f && s.inj_ms <= 30.0f;
    case KLINE_SENSOR_IGN:
      return t17 && isfinite(s.ignition_deg) && s.ignition_deg >= -40.0f && s.ignition_deg <= 100.0f;
    case KLINE_SENSOR_O2:
      return t20 && isfinite(s.o2_v) && s.o2_v >= 0.0f && s.o2_v <= 5.1f;
    case KLINE_SENSOR_STFT:
      return t20 && isfinite(s.stft_pct) && s.stft_pct >= -100.0f && s.stft_pct <= 100.0f;
    case KLINE_SENSOR_O2_HEATER:
      return t20 && isfinite(s.o2_heater_v) && s.o2_heater_v >= 0.0f && s.o2_heater_v <= 5.1f;
    default:
      return false;
  }
}

