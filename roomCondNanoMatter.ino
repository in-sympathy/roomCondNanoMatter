/*
  Nano Matter AQI - ENS160 + BME680 + DS18B20 (+ optional Matter Air Quality)

  Stability-focused version:
  - Matter/HomeKit updates are rate-limited to 30 seconds.
  - OLED/sensor update interval is 5 seconds.
  - Serial debug spam is disabled by default.
  - DS18B20 conversion is non-blocking.
  - I2C bus recovery is attempted at boot.
  - Sensors are softly reinitialized after repeated I2C/read failures.
  - Main loop yields with delay(5) to avoid starving Matter/OpenThread tasks.

  LED behavior:
    - Solid BLUE        = eCO2 above SAFE_CO2_MAX
    - Solid PURPLE R+B  = tVOC above SAFE_TVOC_MAX
    - Solid RED         = BOTH eCO2 and tVOC above limits
    - OFF               = both within safe bounds

  Humidity/Pressure/Temperature do NOT drive the LED.
  BME680 gas is not used.
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// Sensors
#include "ScioSense_ENS160.h"
#include <Adafruit_BME680.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Minimal QR
#include <qrcode.h>

// Matter
#include <Matter.h>

// ---------- Optional Matter wrappers ----------
#if !defined(__has_include)
  #define __has_include(x) 0
#endif

#if __has_include(<MatterTemperature.h>)
  #include <MatterTemperature.h>
  #define HAS_MATTER_TEMP 1
#endif

#if __has_include(<MatterHumidity.h>)
  #include <MatterHumidity.h>
  #define HAS_MATTER_HUMI 1
#endif

#if __has_include(<MatterPressure.h>)
  #include <MatterPressure.h>
  #define HAS_MATTER_PRES 1
#endif

#if __has_include(<MatterAirQuality.h>)
  #include <MatterAirQuality.h>
  #define HAS_MATTER_AIRQ 1
#endif

#ifndef BTN_BUILTIN
  #define BTN_BUILTIN 7
#endif

// ---------- User-tunable settings ----------

// 0 = regular sensor serial output disabled.
// 1 = print full sensor readings every sensor update.
// Keep this OFF for long-term stable operation unless debugging over USB.
#define SERIAL_DEBUG 0

// Keep startup messages on. They are printed only once.
#define STARTUP_SERIAL_LOG 1

// Main intervals
const uint32_t SENSOR_READ_INTERVAL_MS          = 5000UL;   // OLED + sensor cache update
const uint32_t MATTER_PUBLISH_INTERVAL_MS       = 30000UL;  // Matter/HomeKit update
const uint32_t MATTER_STATUS_CHECK_INTERVAL_MS  = 1000UL;   // pairing screen status check
const uint32_t LED_UPDATE_INTERVAL_MS           = 250UL;    // solid LED refresh
const uint32_t I2C_RECOVERY_COOLDOWN_MS         = 60000UL;  // do not reinit sensors too often

// DS18B20 12-bit conversion time is up to 750 ms.
// We run it non-blocking so Matter/OpenThread is not starved.
const uint32_t DS18B20_CONVERSION_MS = 800UL;

// ---------- Pins / I2C ----------
const uint8_t DS18B20_PIN   = 5;     // D5 on Nano Matter
const uint8_t OLED_I2C_ADDR = 0x3C;
const uint32_t I2C_CLOCK_HZ = 400000UL;

// ---------- RGB LED detection ----------
#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
  #define HAS_RGB_LED 1
  #define PIN_LED_R LED_RED
  #define PIN_LED_G LED_GREEN
  #define PIN_LED_B LED_BLUE
#elif defined(LEDR) && defined(LEDG) && defined(LEDB)
  #define HAS_RGB_LED 1
  #define PIN_LED_R LEDR
  #define PIN_LED_G LEDG
  #define PIN_LED_B LEDB
#else
  #define HAS_RGB_LED 0
  #if defined(LED_RED)
    #define ALERT_LED_PIN LED_RED
  #else
    #define ALERT_LED_PIN LED_BUILTIN
  #endif
#endif

// Nano Matter RGB LED is usually active-LOW.
#ifndef LED_RGB_ACTIVE_HIGH
  #define LED_RGB_ACTIVE_HIGH 0
#endif

#ifndef ALERT_LED_ACTIVE_HIGH
  #define ALERT_LED_ACTIVE_HIGH 1
#endif

// ---------- Display ----------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------- Sensors ----------
ScioSense_ENS160 ens160(ENS160_I2CADDR_1); // 0x53
Adafruit_BME680  bme;                      // 0x76 or 0x77
OneWire          oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// ---------- Matter endpoints ----------
#ifdef HAS_MATTER_TEMP
  MatterTemperature matter_temp;
#endif

#ifdef HAS_MATTER_HUMI
  MatterHumidity matter_humi;
#endif

#ifdef HAS_MATTER_PRES
  MatterPressure matter_pres;
#endif

#ifdef HAS_MATTER_AIRQ
  MatterAirQuality matter_airq;
#endif

// ---------- Safe ranges ----------
const float SAFE_CO2_MAX   = 1000.0f; // ppm
const float SAFE_TVOC_MAX  = 500.0f;  // ppb
const float SAFE_H_MIN     = 30.0f;   // %
const float SAFE_H_MAX     = 70.0f;   // %
const float SAFE_T_MIN     = 18.0f;   // C
const float SAFE_T_MAX     = 25.0f;   // C
const float SAFE_P_MIN     = 950.0f;  // hPa
const float SAFE_P_MAX     = 1050.0f; // hPa

// ---------- UI state ----------
enum ScreenMode {
  SCREEN_WELCOME,
  SCREEN_SENSORS,
  SCREEN_PAIRING
};

ScreenMode screen = SCREEN_WELCOME;

uint32_t welcome_until_ms = 0;
uint32_t pairing_until_ms = 0;

bool pairedCached     = false;
bool threadConnCached = false;

// ---------- Timing state ----------
uint32_t lastSensorMs             = 0;
uint32_t lastMatterPublishMs      = 0;
uint32_t lastMatterStatusCheckMs  = 0;
uint32_t lastLedUpdateMs          = 0;
uint32_t lastI2cRecoveryMs        = 0;

// ---------- Sensor state ----------
float   eco2_ppm  = NAN;   // ENS160
float   tvoc_ppb  = NAN;   // ENS160
float   humi_pct  = NAN;   // BME680
float   pres_hpa  = NAN;   // BME680
float   tempC_ds  = NAN;   // DS18B20
uint8_t aqi_value = 0;     // 1..5 derived

float lastEco2 = NAN;
float lastTvoc = NAN;

bool bme_ok = false;
bool ens_ok = false;

uint8_t bmeFailCount = 0;
uint8_t ensFailCount = 0;

uint32_t dsRequestMs = 0;
bool dsConversionPending = false;

// ---------- Button ----------
bool     btnLast   = true;
uint32_t btnDownMs = 0;

// ---------- Pairing data ----------
String pairingPIN;
String pairingPayload;

// ---------- Display brightness ----------
static inline void setNormalBrightness() {
  oled.setContrast(180);
}

static inline void setPairingBrightness() {
  oled.setContrast(92);
}

// ---------- Forward declarations ----------
void recoverI2CBus();
void startWire();
bool initBME680();
bool initENS160();
void initSensors(bool firstBoot);
void softRecoverI2CAndSensors(const char* reason);

void drawLeftLine(int y, const char* text, const uint8_t* font);
void drawCenteredH(int y, const char* text, const uint8_t* font);
void drawCenteredInRightPanel(int xRight, int wRight, int y, const char* text, const uint8_t* font);

void drawWelcome();
String extractQrPayload();
void drawMaxQRIn64Frame_Left(const String& data, const int marginPx = 1);
void drawPairing();
void drawSensors();

uint8_t computeAirQualityIndex(float eco2_ppm_in, float tvoc_ppb_in);
const char* aqiLabel(uint8_t idx);

void startDs18b20Conversion();
void updateDs18b20NonBlocking();
void readSensors();
void printSensorReadings();

void matterPublish();
bool hasAnyValidSensorValue();

void handleButton();
void updateStatusLedSolid();

#if STARTUP_SERIAL_LOG
  #define STARTUP_PRINT(x)   Serial.print(x)
  #define STARTUP_PRINTLN(x) Serial.println(x)
#else
  #define STARTUP_PRINT(x)
  #define STARTUP_PRINTLN(x)
#endif

// ---------- LED helpers ----------
#if HAS_RGB_LED
static inline void setRgb(bool r, bool g, bool b) {
#if LED_RGB_ACTIVE_HIGH
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
#else
  // active-LOW: LOW = ON, HIGH = OFF
  digitalWrite(PIN_LED_R, r ? LOW : HIGH);
  digitalWrite(PIN_LED_G, g ? LOW : HIGH);
  digitalWrite(PIN_LED_B, b ? LOW : HIGH);
#endif
}
#else
static inline void setRgb(bool r, bool g, bool b) {
  bool on = (r || g || b);

#if ALERT_LED_ACTIVE_HIGH
  digitalWrite(ALERT_LED_PIN, on ? HIGH : LOW);
#else
  digitalWrite(ALERT_LED_PIN, on ? LOW : HIGH);
#endif
}
#endif

void updateStatusLedSolid() {
  const bool eco2_bad = (!isnan(eco2_ppm) && eco2_ppm > SAFE_CO2_MAX);
  const bool tvoc_bad = (!isnan(tvoc_ppb) && tvoc_ppb > SAFE_TVOC_MAX);

  if (eco2_bad && tvoc_bad) {
    // Both bad: solid RED
    setRgb(true, false, false);
  } else if (eco2_bad) {
    // eCO2 bad: solid BLUE
    setRgb(false, false, true);
  } else if (tvoc_bad) {
    // tVOC bad: solid PURPLE
    setRgb(true, false, true);
  } else {
    // OK: LED off
    setRgb(false, false, false);
  }
}

// ---------- I2C recovery ----------

void recoverI2CBus() {
#if defined(SDA) && defined(SCL)
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(10);

  // If a slave keeps SDA low, clock SCL up to 9 times and then generate STOP.
  if (digitalRead(SDA) == LOW || digitalRead(SCL) == LOW) {
    for (uint8_t i = 0; i < 9; i++) {
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(10);

      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(10);
    }

    // STOP condition: SDA low -> SCL high -> SDA high
    pinMode(SDA, OUTPUT);
    digitalWrite(SDA, LOW);
    delayMicroseconds(10);

    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(10);

    pinMode(SDA, INPUT_PULLUP);
    delayMicroseconds(10);
  }
#endif
}

void startWire() {
  recoverI2CBus();
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
}

bool initBME680() {
  bool ok = false;

  if (bme.begin(0x76)) {
    ok = true;
  } else if (bme.begin(0x77)) {
    ok = true;
  }

  if (!ok) {
    STARTUP_PRINTLN(F("ERROR: BME680 not found at 0x76/0x77."));
    return false;
  }

  bme.setTemperatureOversampling(BME680_OS_2X);
  bme.setHumidityOversampling(BME680_OS_4X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(0, 0); // gas off

  STARTUP_PRINTLN(F("BME680 OK"));
  return true;
}

bool initENS160() {
  if (!ens160.begin()) {
    STARTUP_PRINTLN(F("ERROR: ENS160 not found."));
    return false;
  }

  ens160.setMode(ENS160_OPMODE_STD);
  STARTUP_PRINTLN(F("ENS160 OK"));
  return true;
}

void initSensors(bool firstBoot) {
  bme_ok = initBME680();
  bmeFailCount = 0;

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);
  startDs18b20Conversion();
  STARTUP_PRINTLN(F("DS18B20 OK"));

  ens_ok = initENS160();
  ensFailCount = 0;

  // Do not block for ENS warm-up. It may output zeros briefly; the code ignores
  // transient zeros and keeps the last good value.
  if (firstBoot) {
    STARTUP_PRINTLN(F("Sensors initialized."));
  }
}

void softRecoverI2CAndSensors(const char* reason) {
  uint32_t ms = millis();
  if (ms - lastI2cRecoveryMs < I2C_RECOVERY_COOLDOWN_MS) {
    return;
  }

  lastI2cRecoveryMs = ms;

#if STARTUP_SERIAL_LOG
  Serial.print(F("[Recovery] I2C/sensor reinit: "));
  Serial.println(reason);
#endif

  setRgb(false, false, false);

  Wire.end();
  delay(20);

  recoverI2CBus();
  delay(20);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  initSensors(false);
}

// ---------- Text helpers ----------

void drawLeftLine(int y, const char* text, const uint8_t* font) {
  oled.setFont(font);
  oled.setCursor(0, y);
  oled.print(text);
}

void drawCenteredH(int y, const char* text, const uint8_t* font) {
  oled.setFont(font);
  int w = oled.getUTF8Width(text);
  int x = (128 - w) / 2;
  if (x < 0) x = 0;

  oled.setCursor(x, y);
  oled.print(text);
}

void drawCenteredInRightPanel(int xRight, int wRight, int y, const char* text, const uint8_t* font) {
  oled.setFont(font);
  int w = oled.getUTF8Width(text);
  int x = xRight + (wRight - w) / 2;
  if (x < xRight) x = xRight;

  oled.setCursor(x, y);
  oled.print(text);
}

// ---------- Welcome screen ----------

void drawWelcome() {
  oled.clearBuffer();
  setNormalBrightness();

  const uint8_t* f = u8g2_font_6x12_tf;
  oled.setFont(f);

  int asc = oled.getAscent();
  int dsc = -oled.getDescent();
  int lh  = asc + dsc;

  int total = 4 * lh;
  int top   = (64 - total) / 2 + asc;

  int y1 = top;
  int y2 = y1 + lh;
  int y4 = y2 + 2 * lh;

  drawCenteredH(y1, "Arduino", f);
  drawCenteredH(y2, "Nano Matter AQI", f);
  drawCenteredH(y4, "ENS160+BME680+DS18B20", f);

  oled.sendBuffer();
}

// ---------- Matter QR payload ----------

String extractQrPayload() {
  String url = Matter.getOnboardingQRCodeUrl();

  int di = url.indexOf("data=");
  if (di < 0) return String();

  String enc = url.substring(di + 5);

  int amp = enc.indexOf('&');
  if (amp >= 0) enc = enc.substring(0, amp);

  String out;
  out.reserve(enc.length());

  for (size_t i = 0; i < enc.length(); ++i) {
    char c = enc[i];

    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < enc.length()) {
      auto hx = [](char h) -> int {
        if (h >= '0' && h <= '9') return h - '0';
        if (h >= 'A' && h <= 'F') return 10 + (h - 'A');
        if (h >= 'a' && h <= 'f') return 10 + (h - 'a');
        return 0;
      };

      char v = (char)((hx(enc[i + 1]) << 4) | hx(enc[i + 2]));
      out += v;
      i += 2;
    } else {
      out += c;
    }
  }

  if (!out.startsWith("MT:")) return String();
  return out;
}

/*
  Pixel-accurate QR inside 64x64 frame @ (0,0), with exact 1-px pad.
*/
void drawMaxQRIn64Frame_Left(const String& data, const int marginPx) {
  const int frameX = 0;
  const int frameY = 0;
  const int box = 64;

  oled.drawFrame(frameX, frameY, box, box);

  if (data.length() == 0) return;

  uint8_t qrbuff[qrcode_getBufferSize(10)];
  QRCode qr;
  bool ok = false;

  for (uint8_t ver = 1; ver <= 10; ++ver) {
    if (qrcode_initText(&qr, qrbuff, ver, ECC_LOW, data.c_str()) == 0) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    qrcode_initText(&qr, qrbuff, 10, ECC_LOW, data.c_str());
  }

  const int size = qr.size;
  const int avail = box - 2 * marginPx;

  if (avail <= 0 || size <= 0) return;

  const int base  = avail / size;
  const int extra = avail % size;

  const int ox = frameX + marginPx;
  const int oy = frameY + marginPx;

  static uint16_t xEdge[182];
  static uint16_t yEdge[182];

  uint16_t sum = 0;
  xEdge[0] = 0;
  for (int i = 0; i < size; ++i) {
    sum += base + (i < extra ? 1 : 0);
    xEdge[i + 1] = sum;
  }

  sum = 0;
  yEdge[0] = 0;
  for (int i = 0; i < size; ++i) {
    sum += base + (i < extra ? 1 : 0);
    yEdge[i + 1] = sum;
  }

  oled.setDrawColor(1);

  for (int y = 0; y < size; y++) {
    const int py = oy + yEdge[y];
    const int h  = (int)yEdge[y + 1] - (int)yEdge[y];

    for (int x = 0; x < size; x++) {
      if (qrcode_getModule(&qr, x, y)) {
        const int px = ox + xEdge[x];
        const int w  = (int)xEdge[x + 1] - (int)xEdge[x];
        oled.drawBox(px, py, w, h);
      }
    }
  }
}

// ---------- Pairing screen ----------

void drawPairing() {
  oled.clearBuffer();
  setPairingBrightness();

  drawMaxQRIn64Frame_Left(pairingPayload, 1);

  const int xRight = 64 + 8;
  const int wRight = 128 - xRight;

  String d = pairingPIN;
  d.replace(" ", "");
  d.replace("-", "");

  String top6 = (d.length() >= 6)
                ? (d.substring(0, 3) + " " + d.substring(3, 6))
                : d;

  String bottom5 = (d.length() >= 11)
                   ? (d.substring(6, 9) + " " + d.substring(9, 11))
                   : (d.length() > 6 ? d.substring(6) : "");

  oled.setFont(u8g2_font_6x12_tf);
  const int ascBig   = oled.getAscent();
  const int descBig  = -oled.getDescent();
  const int lineHBig = ascBig + descBig;

  oled.setFont(u8g2_font_5x8_mf);
  const int ascSm   = oled.getAscent();
  const int descSm  = -oled.getDescent();
  const int lineHSm = ascSm + descSm;

  bool twoLineStatus = false;
  const char* s1 = nullptr;
  const char* s2 = nullptr;

  if (!Matter.isDeviceCommissioned()) {
    s1 = "Waiting for";
    s2 = "pairing...";
    twoLineStatus = true;
  } else if (!Matter.isDeviceThreadConnected()) {
    s1 = "Thread: connecting";
    twoLineStatus = false;
  } else {
    s1 = "Thread:";
    s2 = "connected";
    twoLineStatus = true;
  }

  const int blockH = 2 * lineHBig + lineHBig + (twoLineStatus ? 2 * lineHSm : lineHSm);
  const int topY = (64 - blockH) / 2;

  const int yCode1 = topY + ascBig;
  const int yCode2 = topY + lineHBig + ascBig;

  drawCenteredInRightPanel(xRight, wRight, yCode1, top6.c_str(), u8g2_font_6x12_tf);
  drawCenteredInRightPanel(xRight, wRight, yCode2, bottom5.c_str(), u8g2_font_6x12_tf);

  const int yStatusStart = topY + 2 * lineHBig + lineHBig;
  const int yS1 = yStatusStart + ascSm;

  drawCenteredInRightPanel(xRight, wRight, yS1, s1, u8g2_font_5x8_mf);

  if (twoLineStatus && s2) {
    const int yS2 = yS1 + lineHSm;
    drawCenteredInRightPanel(xRight, wRight, yS2, s2, u8g2_font_5x8_mf);
  }

  oled.sendBuffer();
}

// ---------- Sensor display ----------

void drawSensors() {
  oled.clearBuffer();
  setNormalBrightness();

  const uint8_t* f = u8g2_font_6x10_tf;
  oled.setFont(f);

  int asc = oled.getAscent();
  int dsc = -oled.getDescent();
  int lh  = asc + dsc;

  int y1 = asc;
  int y2 = y1 + lh;
  int y3 = y2 + lh;
  int y4 = y3 + lh;
  int y5 = y4 + lh;
  int y6 = y5 + lh;

  char line[64];

  if (aqi_value < 1 || aqi_value > 5) {
    snprintf(line, sizeof(line), "AQI: -");
  } else {
    snprintf(line, sizeof(line), "AQI: %s (%u)", aqiLabel(aqi_value), (unsigned)aqi_value);
  }
  drawLeftLine(y1, line, f);

  if (isnan(eco2_ppm)) snprintf(line, sizeof(line), "eCO2: ---- ppm");
  else                 snprintf(line, sizeof(line), "eCO2: %.0f ppm", roundf(eco2_ppm));
  drawLeftLine(y2, line, f);

  if (isnan(tvoc_ppb)) snprintf(line, sizeof(line), "tVOC: ---- ppb");
  else                 snprintf(line, sizeof(line), "tVOC: %.0f ppb", roundf(tvoc_ppb));
  drawLeftLine(y3, line, f);

  if (isnan(humi_pct)) snprintf(line, sizeof(line), "Humidity: -- %%");
  else                 snprintf(line, sizeof(line), "Humidity: %.0f %%", roundf(humi_pct));
  drawLeftLine(y4, line, f);

  if (isnan(pres_hpa)) snprintf(line, sizeof(line), "Pressure: ---- hPa");
  else                 snprintf(line, sizeof(line), "Pressure: %.0f hPa", roundf(pres_hpa));
  drawLeftLine(y5, line, f);

  if (isnan(tempC_ds)) snprintf(line, sizeof(line), "Temperature: --.- \xC2\xB0""C");
  else                 snprintf(line, sizeof(line), "Temperature: %.1f \xC2\xB0""C", tempC_ds);
  drawLeftLine(y6, line, f);

  oled.sendBuffer();
}

// ---------- AQI helpers ----------

uint8_t computeAirQualityIndex(float eco2_ppm_in, float tvoc_ppb_in) {
  auto clamp15 = [](int v) {
    if (v < 1) return 1;
    if (v > 5) return 5;
    return v;
  };

  int idx_voc;
  if (isnan(tvoc_ppb_in))      idx_voc = 3;
  else if (tvoc_ppb_in < 150) idx_voc = 1;
  else if (tvoc_ppb_in < 300) idx_voc = 2;
  else if (tvoc_ppb_in < 450) idx_voc = 3;
  else if (tvoc_ppb_in < 600) idx_voc = 4;
  else                        idx_voc = 5;

  int idx_co2;
  if (isnan(eco2_ppm_in))      idx_co2 = 3;
  else if (eco2_ppm_in < 600) idx_co2 = 1;
  else if (eco2_ppm_in < 1000) idx_co2 = 2;
  else if (eco2_ppm_in < 1500) idx_co2 = 3;
  else if (eco2_ppm_in < 2000) idx_co2 = 4;
  else                         idx_co2 = 5;

  int worse = (idx_voc >= idx_co2) ? idx_voc : idx_co2;
  return (uint8_t)clamp15(worse);
}

const char* aqiLabel(uint8_t idx) {
  switch (idx) {
    case 1: return "Excellent";
    case 2: return "Good";
    case 3: return "Fair";
    case 4: return "Poor";
    case 5: return "Very Poor";
    default: return "-";
  }
}

// ---------- DS18B20 non-blocking ----------

void startDs18b20Conversion() {
  ds18b20.requestTemperatures();
  dsRequestMs = millis();
  dsConversionPending = true;
}

void updateDs18b20NonBlocking() {
  uint32_t ms = millis();

  if (!dsConversionPending) {
    startDs18b20Conversion();
    return;
  }

  if (ms - dsRequestMs < DS18B20_CONVERSION_MS) {
    return;
  }

  float t = ds18b20.getTempCByIndex(0);
  if (t > -100.0f && t < 100.0f) {
    tempC_ds = t;
  }

  startDs18b20Conversion();
}

// ---------- Sensor reading ----------

void readSensors() {
  updateDs18b20NonBlocking();

  if (bme_ok) {
    if (bme.performReading()) {
      humi_pct = bme.humidity;
      pres_hpa = bme.pressure / 100.0f;
      bmeFailCount = 0;
    } else {
      if (bmeFailCount < 255) bmeFailCount++;
    }
  }

  if (ens_ok) {
    ens160.measure(0);

    float tv = ens160.getTVOC();
    float co = ens160.geteCO2();

    bool gotEnsData = false;

    if (tv > 0.0f) {
      tvoc_ppb = tv;
      lastTvoc = tv;
      gotEnsData = true;
    } else if (!isnan(lastTvoc)) {
      tvoc_ppb = lastTvoc;
    }

    if (co > 0.0f) {
      eco2_ppm = co;
      lastEco2 = co;
      gotEnsData = true;
    } else if (!isnan(lastEco2)) {
      eco2_ppm = lastEco2;
    }

    if (gotEnsData) {
      ensFailCount = 0;
    } else if (isnan(lastEco2) && isnan(lastTvoc)) {
      if (ensFailCount < 255) ensFailCount++;
    }
  }

  aqi_value = computeAirQualityIndex(eco2_ppm, tvoc_ppb);

#if SERIAL_DEBUG
  printSensorReadings();
#endif

  if (bmeFailCount >= 6) {
    softRecoverI2CAndSensors("BME680 repeated read failures");
    bmeFailCount = 0;
  }

  if (ensFailCount >= 6) {
    softRecoverI2CAndSensors("ENS160 repeated zero/no-data readings");
    ensFailCount = 0;
  }
}

void printSensorReadings() {
  Serial.println(F("--- Sensor readings ---"));

  Serial.print(F("AQI:                  "));
  Serial.print(aqi_value);
  Serial.print(F(" ("));
  Serial.print(aqiLabel(aqi_value));
  Serial.println(F(")"));

  Serial.print(F("eCO2 (ENS160):        "));
  if (isnan(eco2_ppm)) Serial.println(F("---- ppm"));
  else { Serial.print(eco2_ppm, 0); Serial.println(F(" ppm")); }

  Serial.print(F("tVOC (ENS160):        "));
  if (isnan(tvoc_ppb)) Serial.println(F("---- ppb"));
  else { Serial.print(tvoc_ppb, 0); Serial.println(F(" ppb")); }

  Serial.print(F("Humidity (BME680):    "));
  if (isnan(humi_pct)) Serial.println(F("-- %"));
  else { Serial.print(humi_pct, 0); Serial.println(F(" %")); }

  Serial.print(F("Pressure (BME680):    "));
  if (isnan(pres_hpa)) Serial.println(F("---- hPa"));
  else { Serial.print(pres_hpa, 0); Serial.println(F(" hPa")); }

  Serial.print(F("Temperature (DS18B20): "));
  if (isnan(tempC_ds)) Serial.println(F("--.- C"));
  else { Serial.print(tempC_ds, 1); Serial.println(F(" C")); }
}

// ---------- Matter publish ----------

bool hasAnyValidSensorValue() {
  return !isnan(humi_pct) || !isnan(pres_hpa) || !isnan(tempC_ds) || (aqi_value >= 1 && aqi_value <= 5);
}

void matterPublish() {
#ifdef HAS_MATTER_HUMI
  if (!isnan(humi_pct)) {
    matter_humi.set_measured_value(humi_pct);
  }
#endif

#ifdef HAS_MATTER_PRES
  if (!isnan(pres_hpa)) {
    matter_pres.set_measured_value(pres_hpa);
  }
#endif

#ifdef HAS_MATTER_TEMP
  if (!isnan(tempC_ds)) {
    matter_temp.set_measured_value_celsius(tempC_ds);
  }
#endif

#ifdef HAS_MATTER_AIRQ
  uint8_t val = aqi_value;
  if (val < 1) val = 1;
  if (val > 5) val = 5;
  matter_airq.set_air_quality(static_cast<MatterAirQuality::AirQuality_t>(val));
#endif
}

// ---------- Button handling ----------

void handleButton() {
  bool now = (digitalRead(BTN_BUILTIN) == LOW);
  uint32_t ms = millis();

  if (now && !btnLast) {
    btnDownMs = ms;
  } else if (!now && btnLast) {
    uint32_t held = ms - btnDownMs;

    if (held >= 6000UL) {
      Serial.println(F("Decommissioning..."));

      oled.clearBuffer();
      drawCenteredH(32, "Decommissioning...", u8g2_font_6x12_tf);
      oled.sendBuffer();

      delay(200);
      Matter.decommission();
    } else {
      pairing_until_ms = ms + 30000UL;
      screen = SCREEN_PAIRING;
      drawPairing();
    }
  }

  btnLast = now;
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);
  delay(20);

  STARTUP_PRINTLN();
  STARTUP_PRINTLN(F("Arduino Nano Matter AQI starting..."));

  pinMode(BTN_BUILTIN, INPUT_PULLUP);

#if HAS_RGB_LED
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  setRgb(false, false, false);
#else
  pinMode(ALERT_LED_PIN, OUTPUT);
 #if !ALERT_LED_ACTIVE_HIGH
  digitalWrite(ALERT_LED_PIN, HIGH);
 #else
  digitalWrite(ALERT_LED_PIN, LOW);
 #endif
#endif

  startWire();

  oled.begin();
  oled.setI2CAddress(OLED_I2C_ADDR << 1);
  oled.enableUTF8Print();

  setNormalBrightness();
  drawWelcome();

  welcome_until_ms = millis() + 3000UL;

  initSensors(true);

  Matter.begin();

#ifdef HAS_MATTER_TEMP
  matter_temp.begin();
#else
  STARTUP_PRINTLN(F("[Matter] Temperature cluster not found."));
#endif

#ifdef HAS_MATTER_HUMI
  matter_humi.begin();
#else
  STARTUP_PRINTLN(F("[Matter] Humidity cluster not found."));
#endif

#ifdef HAS_MATTER_PRES
  matter_pres.begin();
  STARTUP_PRINTLN(F("[Matter] Pressure cluster enabled."));
#else
  STARTUP_PRINTLN(F("[Matter] Pressure cluster NOT available."));
#endif

#ifdef HAS_MATTER_AIRQ
  matter_airq.begin();
  STARTUP_PRINTLN(F("[Matter] Air Quality cluster enabled."));
#else
  STARTUP_PRINTLN(F("[Matter] Air Quality cluster NOT available."));
#endif

  pairingPIN     = Matter.getManualPairingCode();
  pairingPayload = extractQrPayload();

  pairedCached     = Matter.isDeviceCommissioned();
  threadConnCached = Matter.isDeviceThreadConnected();

  if (!pairedCached) {
    STARTUP_PRINTLN(F("Matter device is NOT commissioned."));
    STARTUP_PRINT(F("Manual pairing code: "));
    STARTUP_PRINTLN(pairingPIN);
  }

  screen = SCREEN_WELCOME;
}

// ---------- Loop ----------

void loop() {
  uint32_t ms = millis();

  handleButton();

  // Welcome -> sensor screen
  if (screen == SCREEN_WELCOME && ms >= welcome_until_ms) {
    screen = SCREEN_SENSORS;

    readSensors();
    drawSensors();

    if (hasAnyValidSensorValue()) {
      matterPublish();
      lastMatterPublishMs = ms;
    }

    lastSensorMs = ms;
  }

  // Pairing screen timeout
  if (screen == SCREEN_PAIRING && ms >= pairing_until_ms) {
    screen = SCREEN_SENSORS;
    setNormalBrightness();
    drawSensors();
  }

  // Do not query Matter state thousands of times per second.
  if (ms - lastMatterStatusCheckMs >= MATTER_STATUS_CHECK_INTERVAL_MS) {
    lastMatterStatusCheckMs = ms;

    bool nowPaired = Matter.isDeviceCommissioned();
    bool nowThread = Matter.isDeviceThreadConnected();

    if ((nowPaired != pairedCached || nowThread != threadConnCached) && screen == SCREEN_PAIRING) {
      pairedCached = nowPaired;
      threadConnCached = nowThread;
      drawPairing();
    } else {
      pairedCached = nowPaired;
      threadConnCached = nowThread;
    }
  }

  // Sensor read + OLED update
  if (ms - lastSensorMs >= SENSOR_READ_INTERVAL_MS) {
    lastSensorMs = ms;

    readSensors();

    if (screen == SCREEN_SENSORS) {
      drawSensors();
    }
  }

  // Matter publish is intentionally slower than OLED updates.
  if (hasAnyValidSensorValue() && (ms - lastMatterPublishMs >= MATTER_PUBLISH_INTERVAL_MS)) {
    lastMatterPublishMs = ms;
    matterPublish();
  }

  // Solid LED update, rate-limited.
  if (ms - lastLedUpdateMs >= LED_UPDATE_INTERVAL_MS) {
    lastLedUpdateMs = ms;
    updateStatusLedSolid();
  }

  // Let Matter/OpenThread/RTOS background tasks breathe.
  delay(5);
}
