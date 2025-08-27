// // /*
  // Nano Matter AQI – ENS160 + BME680 + DS18B20
  //
  // Requested behavior:
  // - DISPLAY + SERIAL order: eCO2, tVOC, Humidity, Pressure, Temperature
  // - Matter/HomeKit export: eCO2 (ppm), tVOC (ppb), Humidity (%), Pressure (hPa), Temperature (°C)
  // - Pairing screen:
  //     • QR maximized inside 64×64 frame (EXACT 1-px pad), right column centered H+V
  //     • Brightness 36% while pairing; no flicker (single render)
  //     • Short press → show pairing 30s; Long press (>=6s) → Matter.decommission()
// */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// Sensors
#include "ScioSense_ENS160.h"
#include <Adafruit_BME680.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Minimal QR (Richard Moore)
#include <qrcode.h>

// Matter
#include <Matter.h>

// ---- Matter wrappers known on Nano Matter 3.0.0 ----
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

// Some Nano Matter builds DO NOT ship CO2/TVOC wrappers. We try multiple names.
// If none exist, we skip them (sketch compiles; Serial tells you they’re absent).
#if __has_include(<MatterCO2.h>)
  #include <MatterCO2.h>
  typedef MatterCO2 MatterCO2Class;
  #define HAS_MATTER_CO2 1
#elif __has_include(<MatterCarbonDioxide.h>)
  #include <MatterCarbonDioxide.h>
  typedef MatterCarbonDioxide MatterCO2Class;
  #define HAS_MATTER_CO2 1
#elif __has_include(<MatterCarbonDioxideConcentration.h>)
  #include <MatterCarbonDioxideConcentration.h>
  typedef MatterCarbonDioxideConcentration MatterCO2Class;
  #define HAS_MATTER_CO2 1
#endif

#if __has_include(<MatterVOC.h>)
  #include <MatterVOC.h>
  typedef MatterVOC MatterVOCClass;
  #define HAS_MATTER_VOC 1
#elif __has_include(<MatterTVOC.h>)
  #include <MatterTVOC.h>
  typedef MatterTVOC MatterVOCClass;
  #define HAS_MATTER_VOC 1
#elif __has_include(<MatterTotalVolatileOrganicCompounds.h>)
  #include <MatterTotalVolatileOrganicCompounds.h>
  typedef MatterTotalVolatileOrganicCompounds MatterVOCClass;
  #define HAS_MATTER_VOC 1
#elif __has_include(<MatterTotalVolatileOrganicCompoundsConcentration.h>)
  #include <MatterTotalVolatileOrganicCompoundsConcentration.h>
  typedef MatterTotalVolatileOrganicCompoundsConcentration MatterVOCClass;
  #define HAS_MATTER_VOC 1
#endif

#ifndef BTN_BUILTIN
  #define BTN_BUILTIN 7
#endif

// ---------- Pins / I2C ----------
const uint8_t DS18B20_PIN   = 5;     // D5 on Nano Matter
const uint8_t OLED_I2C_ADDR = 0x3C;

// ---------- Display ----------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------- Sensors ----------
ScioSense_ENS160 ens160(ENS160_I2CADDR_1); // 0x53
Adafruit_BME680  bme;                      // 0x76/0x77
OneWire          oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// ---------- Matter endpoints ----------
#ifdef HAS_MATTER_TEMP
  MatterTemperature matter_temp;
#endif
#ifdef HAS_MATTER_HUMI
  MatterHumidity    matter_humi;
#endif
#ifdef HAS_MATTER_PRES
  MatterPressure    matter_pres;
#endif
#ifdef HAS_MATTER_CO2
  MatterCO2Class    matter_co2;
#endif
#ifdef HAS_MATTER_VOC
  MatterVOCClass    matter_voc;
#endif

// ---------- UI state ----------
enum ScreenMode { SCREEN_WELCOME, SCREEN_SENSORS, SCREEN_PAIRING };
ScreenMode screen = SCREEN_WELCOME;

uint32_t welcome_until_ms = 0;
uint32_t pairing_until_ms = 0;

bool pairedCached = false;
bool threadConnCached = false;

uint32_t lastSensorMs = 0;

// Display brightness presets
static inline void setNormalBrightness()  { oled.setContrast(180); } // ~70%
static inline void setPairingBrightness() { oled.setContrast(92); }  // ~36%

// Sensor cache (OLED order = eCO2, tVOC, Hum, Pres, Temp)
float eco2_ppm = NAN;   // ENS160
float tvoc_ppb = NAN;   // ENS160
float humi_pct = NAN;   // BME680
float pres_hpa = NAN;   // BME680
float tempC_ds = NAN;   // DS18B20

// keep last known non-zero for ENS160
float lastEco2 = NAN, lastTvoc = NAN;

// Button
bool     btnLast      = true;
uint32_t btnDownMs    = 0;

// Pairing data
String pairingPIN;       // manual 11-digit code
String pairingPayload;   // MT:... QR payload

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
  int x = xRight + (wRight - w)/2;
  if (x < xRight) x = xRight;
  oled.setCursor(x, y);
  oled.print(text);
}

// ---------- Welcome screen (centered + one blank line) ----------
void drawWelcome() {
  oled.clearBuffer();
  setNormalBrightness();

  const uint8_t* f = u8g2_font_6x12_tf;
  oled.setFont(f);
  int asc = oled.getAscent();
  int dsc = -oled.getDescent();
  int lh  = asc + dsc;

  int total = 4 * lh;                 // 3 lines + 1 blank
  int top   = (64 - total) / 2 + asc;

  int y1 = top;
  int y2 = y1 + lh;
  int y4 = y2 + 2*lh;                 // blank line in between

  drawCenteredH(y1, "Arduino", f);
  drawCenteredH(y2, "Nano Matter AQI", f);
  drawCenteredH(y4, "ENS160+BME680+DS18B20", f);

  oled.sendBuffer();
}

// ---------- Extract "MT:..." payload from Matter URL ----------
String extractQrPayload() {
  String url = Matter.getOnboardingQRCodeUrl();
  int di = url.indexOf("data=");
  if (di < 0) return String();
  String enc = url.substring(di + 5);
  int amp = enc.indexOf('&');
  if (amp >= 0) enc = enc.substring(0, amp);

  String out; out.reserve(enc.length());
  for (size_t i = 0; i < enc.length(); ++i) {
    char c = enc[i];
    if (c == '+') out += ' ';
    else if (c == '%' && i + 2 < enc.length()) {
      auto hexv = [](char h)->int {
        if (h>='0' && h<='9') return h-'0';
        if (h>='A'&&h<='F')   return 10+(h-'A');
        if (h>='a'&&h<='f')   return 10+(h-'a');
        return 0;
      };
      char v = (char)((hexv(enc[i+1]) << 4) | hexv(enc[i+2]));
      out += v; i += 2;
    } else out += c;
  }
  if (!out.startsWith("MT:")) return String();
  return out;
}

/*
  Pixel-accurate QR inside 64×64 frame @ (0,0), with EXACT 1-px pad.
*/
void drawMaxQRIn64Frame_Left(const String& data, const int marginPx = 1) {
  const int frameX = 0, frameY = 0, box = 64;
  oled.drawFrame(frameX, frameY, box, box);
  if (data.length() == 0) return;

  uint8_t qrbuff[qrcode_getBufferSize(10)];
  QRCode  qr;
  bool ok = false;
  for (uint8_t ver = 1; ver <= 10; ++ver) {
    if (qrcode_initText(&qr, qrbuff, ver, ECC_LOW, data.c_str()) == 0) { ok = true; break; }
  }
  if (!ok) qrcode_initText(&qr, qrbuff, 10, ECC_LOW, data.c_str());

  const int size = qr.size;
  const int avail = box - 2*marginPx;
  if (avail <= 0 || size <= 0) return;

  const int base  = avail / size;
  const int extra = avail % size;

  const int ox = frameX + marginPx;
  const int oy = frameY + marginPx;

  static uint16_t xEdge[182], yEdge[182];
  uint16_t sum = 0;
  for (int i = 0; i < size; ++i) { sum += base + (i < extra ? 1 : 0); xEdge[i+1] = sum; }
  sum = 0;
  for (int i = 0; i < size; ++i) { sum += base + (i < extra ? 1 : 0); yEdge[i+1] = sum; }

  oled.setDrawColor(1);
  for (int y = 0; y < size; y++) {
    const int py = oy + yEdge[y];
    const int h  = (int)yEdge[y+1] - (int)yEdge[y];
    for (int x = 0; x < size; x++) {
      if (qrcode_getModule(&qr, x, y)) {
        const int px = ox + xEdge[x];
        const int w  = (int)xEdge[x+1] - (int)xEdge[x];
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

  const int xRight  = 64 + 8;
  const int wRight  = 128 - xRight;

  // Code “123 456” / “789 01”
  String d = pairingPIN; d.replace(" ", "");
  String top6    = (d.length() >= 6 ) ? (d.substring(0,3) + " " + d.substring(3,6)) : d;
  String bottom5 = (d.length() >= 11) ? (d.substring(6,9) + " " + d.substring(9,11))
                                      : (d.length()>6? d.substring(6) : "");

  // Fonts
  oled.setFont(u8g2_font_6x12_tf);
  const int ascBig   = oled.getAscent();
  const int descBig  = -oled.getDescent();
  const int lineHBig = ascBig + descBig;

  oled.setFont(u8g2_font_5x8_mf);
  const int ascSm    = oled.getAscent();
  const int descSm   = -oled.getDescent();
  const int lineHSm  = ascSm + descSm;

  // Status text (two lines while not commissioned)
  bool twoLineStatus = false;
  const char* s1 = nullptr;
  const char* s2 = nullptr;
  if (!Matter.isDeviceCommissioned()) {
    s1 = "Waiting for";
    s2 = "pairing...";
    twoLineStatus = true;
  } else if (!Matter.isDeviceThreadConnected()) {
    s1 = "Thread: connecting";
  } else {
    // *** CHANGED HERE: split to two lines so it fits ***
    s1 = "Thread:";
    s2 = "connected";
    twoLineStatus = true;
  }

  // Vertical center in right panel
  const int blockH = 2*lineHBig + lineHBig + (twoLineStatus ? 2*lineHSm : lineHSm);
  const int topY   = (64 - blockH) / 2;

  // Codes
  const int yCode1 = topY + ascBig;
  const int yCode2 = topY + lineHBig + ascBig;
  drawCenteredInRightPanel(xRight, wRight, yCode1, top6.c_str(),    u8g2_font_6x12_tf);
  drawCenteredInRightPanel(xRight, wRight, yCode2, bottom5.c_str(), u8g2_font_6x12_tf);

  // blank (lineHBig)
  const int yStatusStart = topY + 2*lineHBig + lineHBig;

  // Status (small)
  const int yS1 = yStatusStart + ascSm;
  drawCenteredInRightPanel(xRight, wRight, yS1, s1, u8g2_font_5x8_mf);
  if (twoLineStatus && s2) {
    const int yS2 = yS1 + (ascSm + descSm);
    drawCenteredInRightPanel(xRight, wRight, yS2, s2, u8g2_font_5x8_mf);
  }

  oled.sendBuffer();
}

// ---------- Sensor screen (NEW ORDER) ----------
void drawSensors() {
  oled.clearBuffer();
  setNormalBrightness();

  const uint8_t* f = u8g2_font_6x12_tf;
  oled.setFont(f);
  int asc = oled.getAscent();
  int dsc = -oled.getDescent();
  int lh  = asc + dsc;

  int y1 = asc;
  int y2 = y1 + lh;
  int y3 = y2 + lh;
  int y4 = y3 + lh;
  int y5 = y4 + lh;

  char line[48];

  // 1) eCO2
  if (isnan(eco2_ppm)) snprintf(line, sizeof(line), "eCO2: ---- ppm");
  else                 snprintf(line, sizeof(line), "eCO2: %.0f ppm", roundf(eco2_ppm));
  drawLeftLine(y1, line, f);

  // 2) tVOC
  if (isnan(tvoc_ppb)) snprintf(line, sizeof(line), "tVOC: ---- ppb");
  else                 snprintf(line, sizeof(line), "tVOC: %.0f ppb", roundf(tvoc_ppb));
  drawLeftLine(y2, line, f);

  // 3) Humidity
  if (isnan(humi_pct)) snprintf(line, sizeof(line), "Humidity: -- %%");
  else                 snprintf(line, sizeof(line), "Humidity: %.0f %%", roundf(humi_pct));
  drawLeftLine(y3, line, f);

  // 4) Pressure
  if (isnan(pres_hpa)) snprintf(line, sizeof(line), "Pressure: ---- hPa");
  else                 snprintf(line, sizeof(line), "Pressure: %.0f hPa", roundf(pres_hpa));
  drawLeftLine(y4, line, f);

  // 5) Temperature
  if (isnan(tempC_ds)) snprintf(line, sizeof(line), "Temperature: --.- \xC2\xB0""C");
  else                 snprintf(line, sizeof(line), "Temperature: %.1f \xC2\xB0""C", tempC_ds);
  drawLeftLine(y5, line, f);

  oled.sendBuffer();
}

// ---------- Sensor reading ----------
void readSensors() {
  // DS18B20
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  if (t > -100 && t < 100) tempC_ds = t;

  // BME680
  if (bme.performReading()) {
    humi_pct = bme.humidity;            // %
    pres_hpa = bme.pressure / 100.0f;   // hPa
  }

  // ENS160 (avoid transient zeros)
  ens160.measure(0);
  float tv = ens160.getTVOC();
  float co = ens160.geteCO2();
  if (tv > 0)  { tvoc_ppb = tv; lastTvoc = tv; }
  else if (!isnan(lastTvoc)) tvoc_ppb = lastTvoc;

  if (co > 0)  { eco2_ppm = co; lastEco2 = co; }
  else if (!isnan(lastEco2)) eco2_ppm = lastEco2;

  // Serial in requested order
  Serial.println(F("--- Sensor readings ---"));
  Serial.print(F("eCO2 (ENS160):        ")); if (isnan(eco2_ppm)) Serial.println(F("---- ppm")); else { Serial.print(eco2_ppm,0); Serial.println(F(" ppm")); }
  Serial.print(F("tVOC (ENS160):        ")); if (isnan(tvoc_ppb)) Serial.println(F("---- ppb")); else { Serial.print(tvoc_ppb,0); Serial.println(F(" ppb")); }
  Serial.print(F("Humidity (BME680):    ")); if (isnan(humi_pct)) Serial.println(F("-- %"));  else { Serial.print(humi_pct,0);  Serial.println(F(" %"));  }
  Serial.print(F("Pressure (BME680):    ")); if (isnan(pres_hpa)) Serial.println(F("---- hPa")); else { Serial.print(pres_hpa,0); Serial.println(F(" hPa")); }
  Serial.print(F("Temperature (DS18B20): ")); if (isnan(tempC_ds)) Serial.println(F("--.- C")); else { Serial.print(tempC_ds,1); Serial.println(F(" C")); }
}

// ---------- Matter publish (ONLY those 5) ----------
void matterPublish() {
#ifdef HAS_MATTER_CO2
  if (!isnan(eco2_ppm)) matter_co2.set_measured_value(eco2_ppm);     // ppm
#endif
#ifdef HAS_MATTER_VOC
  if (!isnan(tvoc_ppb)) matter_voc.set_measured_value(tvoc_ppb);     // ppb
#endif
#ifdef HAS_MATTER_HUMI
  if (!isnan(humi_pct)) matter_humi.set_measured_value(humi_pct);    // %
#endif
#ifdef HAS_MATTER_PRES
  if (!isnan(pres_hpa)) matter_pres.set_measured_value(pres_hpa);    // hPa (if your lib expects kPa, use pres_hpa/10.0f)
#endif
#ifdef HAS_MATTER_TEMP
  if (!isnan(tempC_ds)) matter_temp.set_measured_value_celsius(tempC_ds); // °C
#endif
}

// ---------- Button handling ----------
void handleButton() {
  bool now = (digitalRead(BTN_BUILTIN) == LOW); // active LOW
  uint32_t ms = millis();

  if (now && !btnLast) {
    btnDownMs = ms;
  } else if (!now && btnLast) {
    uint32_t held = ms - btnDownMs;
    if (held >= 6000) {
      Serial.println(F("Decommissioning..."));
      oled.clearBuffer();
      drawCenteredH(32, "Decommissioning...", u8g2_font_6x12_tf);
      oled.sendBuffer();
      delay(200);
      Matter.decommission(); // device reboots after
    } else {
      pairing_until_ms = ms + 30000ul;
      screen = SCREEN_PAIRING;
      drawPairing(); // one-shot render (no flicker)
    }
  }
  btnLast = now;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println();
  Serial.println(F("Arduino Nano Matter AQI starting..."));

  pinMode(BTN_BUILTIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();
  Wire.setClock(400000);

  oled.begin();
  oled.setI2CAddress(OLED_I2C_ADDR << 1);
  oled.enableUTF8Print();                 // correct "°"
  setNormalBrightness();
  drawWelcome();
  welcome_until_ms = millis() + 3000;

  // Sensors
  if (!bme.begin(0x76) && !bme.begin(0x77)) {
    Serial.println(F("ERROR: BME680 not found at 0x76/0x77!"));
  } else {
    bme.setTemperatureOversampling(BME680_OS_2X);
    bme.setHumidityOversampling(BME680_OS_4X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 0);
    Serial.println(F("BME680 OK"));
  }

  ds18b20.begin();
  Serial.println(F("DS18B20 OK"));

  if (!ens160.begin()) {
    Serial.println(F("ERROR: ENS160 not found!"));
  } else {
    ens160.setMode(ENS160_OPMODE_STD);
    Serial.println(F("ENS160 OK (warm-up)"));
    delay(3000);
  }

  // Matter
  Matter.begin();

#ifdef HAS_MATTER_TEMP
  matter_temp.begin();
#else
  Serial.println(F("[Matter] Temperature cluster not found in this library build."));
#endif
#ifdef HAS_MATTER_HUMI
  matter_humi.begin();
#else
  Serial.println(F("[Matter] Humidity cluster not found in this library build."));
#endif
#ifdef HAS_MATTER_PRES
  matter_pres.begin();
  Serial.println(F("[Matter] Pressure cluster enabled."));
#else
  Serial.println(F("[Matter] Pressure cluster NOT available in this library build."));
#endif
#ifdef HAS_MATTER_CO2
  matter_co2.begin();
  Serial.println(F("[Matter] CO2 cluster enabled (ppm)."));
#else
  Serial.println(F("[Matter] CO2 cluster NOT available in this library build."));
#endif
#ifdef HAS_MATTER_VOC
  matter_voc.begin();
  Serial.println(F("[Matter] TVOC cluster enabled (ppb)."));
#else
  Serial.println(F("[Matter] TVOC cluster NOT available in this library build."));
#endif

  pairingPIN     = Matter.getManualPairingCode();
  pairingPayload = extractQrPayload();

  if (!Matter.isDeviceCommissioned()) {
    Serial.println(F("Matter device is NOT commissioned."));
    Serial.print(F("Manual pairing code: "));
    Serial.println(pairingPIN);
  }

  screen = SCREEN_WELCOME;
}

// ---------- Loop ----------
void loop() {
  handleButton();

  uint32_t ms = millis();

  if (screen == SCREEN_WELCOME && ms >= welcome_until_ms) {
    screen = SCREEN_SENSORS;
    readSensors();
    drawSensors();
  }

  if (screen == SCREEN_PAIRING && ms >= pairing_until_ms) {
    screen = SCREEN_SENSORS;
    setNormalBrightness();
    drawSensors();
  }

  // Only redraw pairing screen if commissioning state changes (no flicker)
  bool nowPaired = Matter.isDeviceCommissioned();
  bool nowThread = Matter.isDeviceThreadConnected();
  if ((nowPaired != pairedCached || nowThread != threadConnCached) && screen == SCREEN_PAIRING) {
    pairedCached = nowPaired; threadConnCached = nowThread;
    drawPairing();
  } else {
    pairedCached = nowPaired; threadConnCached = nowThread;
  }

  // Periodic sensor updates + publish
  if (screen == SCREEN_SENSORS && (ms - lastSensorMs >= 2000)) {
    lastSensorMs = ms;
    readSensors();
    matterPublish();
    drawSensors();
  }

  // heartbeat LED
  digitalWrite(LED_BUILTIN, (ms/500)%2);
}