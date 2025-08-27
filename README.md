# Nano Matter AQI (Arduino Nano Matter)

Two ready‑to‑flash sketches for an **indoor air‑quality monitor** with:
- **ENS160** (eCO₂ & tVOC),
- **BME680** (humidity & pressure; *gas disabled*),
- **DS18B20** (temperature),
- **SSD1306 128×64 OLED** (U8g2),
- **Matter over Thread** (adds to Apple Home / Google Home, etc.).

Both sketches share the same UI and Matter basics; they differ in **alert LED** behavior and whether an **Air Quality** enum (1..5) is shown.

---

## Hardware

- Arduino **Nano Matter** (EFR32)
- SSD1306 128×64 I²C OLED (0x3C)
- ENS160 gas sensor (0x53)
- BME680 (0x76/**0x77** fallback)
- DS18B20 (on **D5** with 4.7 kΩ pull‑up to 3V3)
- Optional single red LED or on‑board LED for alerts

**I²C:** connect SCL/SDA to the Nano Matter’s I²C pins; 3V3 power.  
**OLED address:** 0x3C. **Wire clock:** 400 kHz.

---

## Sketches (two variants)

### 1) `roomCondNanoMatter.ino`  — *single red alert LED + optional Air Quality tile*
- **Display order (6 lines):** `AQI, eCO₂, tVOC, Humidity, Pressure, Temperature`  
  (AQI is **derived** from eCO₂/tVOC; 1=Good … 5=Poor)
- **Matter publish:** Temperature (°C), Humidity (%), Pressure (hPa), **Air Quality enum** (if `MatterAirQuality.h` exists).
- **Alert LED:** **one RED LED** blinks at 2 Hz when **any** displayed metric is out of its safe range; otherwise OFF.
- **BME680 gas sensing is disabled**.

### 2) `roomCondNanoMatter_Full_HomeAssistant.ino`  — *classic 5‑line display, no AQI*
- **Display order (5 lines):** `eCO₂, tVOC, Humidity, Pressure, Temperature`.
- **Matter publish:** eCO₂ (ppm) & tVOC (ppb) **if your Matter library provides those wrappers**, plus Temperature, Humidity, Pressure.
- **Alert LED:** on‑board heartbeat only (no red‑LED logic in this variant).

> ⚠️ The Silicon Labs Nano Matter **3.0.0** library sometimes omits CO₂/TVOC wrappers.
> This repo detects them at compile time with `__has_include` and prints a Serial note if missing.
> In that case, those values will still show on the OLED and Serial, but won’t be published as Matter clusters.

---

## UI & Controls

**Screens**
- **Welcome:** centered  
  `Arduino`  
  `Nano Matter AQI`  
  *(blank line)*  
  `ENS160+BME680+DS18B20`
- **Pairing:** QR maximized inside a **64×64 frame** with **exact 1‑px pad** on the **left**; pairing code on the **right**, centered both horizontally and vertically.  
  Status lines split to fit:  
  - Not commissioned: `"Waiting for"`, `"pairing..."`  
  - Thread connecting: `"Thread: connecting"`  
  - Thread connected: `"Thread:"`, `"connected"`
  - **Brightness = 36 %** while on pairing screen; normal screens at ~70 %.
- **Sensor screen:** left‑aligned values (as listed per variant). Temperature shows the `°` symbol via U8g2 UTF‑8.

**Button**
- **Short press:** show pairing screen for **30 s** (no flicker; single render).
- **Long press (≥6 s):** **decommission Matter** (`Matter.decommission()`), which resets pairing.

---

## Safe‑range thresholds (used by `roomCondNanoMatter.ino`)

- eCO₂: `> 1000 ppm` → out of range
- tVOC: `> 500 ppb` → out of range
- Humidity: `< 30 %` or `> 70 %` → out of range
- Pressure: `< 950 hPa` or `> 1050 hPa` → out of range
- Temperature: `< 18 °C` or `> 25 °C` → out of range

If **any** metric is out of range, the **red alert LED blinks** at 2 Hz.

---

## Build & Flash

1. **Arduino IDE 2.x** + **Silicon Labs “Nano Matter” core 3.0.0** installed.  
   Board: **Tools → Board → Arduino Nano Matter**.
2. **Install libraries** (Library Manager unless noted):
   - **U8g2** by olikraus
   - **Adafruit BME680 Library** (+ **Adafruit BusIO**, **Adafruit Unified Sensor**)
   - **DallasTemperature** by Miles Burton
   - **OneWire** by Paul Stoffregen
   - **ScioSense_ENS160** (*Adafruit fork*)
   - **QRCode** (Nayuki/Richard Moore port) – header is `<qrcode.h>`
3. Wire the sensors as shown above; plug the board by USB.
4. Open either sketch and **Upload**. Serial monitor at **115200 baud**.

> Tip: If the `°` symbol appears incorrectly, ensure `oled.enableUTF8Print()` is kept and your chosen U8g2 font supports it (these sketches do).

---

## Pairing with Matter

1. Boot the device; press the button **short** to bring up the pairing screen (30 s, dimmed).
2. In Apple Home (or another Matter controller): **Add Accessory** → **More Options** → scan the QR or enter the manual code.
3. After commissioning, Thread state will show `Thread:` / `connected` on the pairing screen (when shown).

**Published endpoints (depending on variant & library availability):**
- Temperature (°C), Humidity (%), Pressure (hPa) – always included.
- **Air Quality (enum 1..5)** – only in `roomCondNanoMatter.ino` and only if `MatterAirQuality.h` exists.
- **eCO₂ (ppm) & tVOC (ppb)** – only in `roomCondNanoMatter_Full_HomeAssistant.ino` *and* when wrappers are available in your Matter core.

> Visibility of some clusters varies by platform/app. Temperature & Humidity always appear; others may show as “Additional Services” or tiles (e.g., Air Quality).

---

## Troubleshooting

- **ENS160 shows zeros:** the sensor may output transient zeros right after power‑up; the code holds the last non‑zero value during warm‑up.
- **QR won’t scan:** ensure the pairing screen is active (short button press), keep 15–30 cm distance, and avoid glare; the code maximizes modules with a 1‑px frame pad for best contrast.
- **No CO₂/TVOC in Matter:** your Nano Matter core likely lacks those wrappers; use the `roomCondNanoMatter.ino` variant to expose an **Air Quality** tile instead.
- **Degree symbol looks wrong:** verify `enableUTF8Print()` is called and you’re using the bundled fonts; avoid changing to non‑UTF‑8 fonts.

---

## Repository layout

```
/roomCondNanoMatter.ino                    # Red-LED alerts + optional Air Quality enum
/roomCondNanoMatter_Full_HomeAssistant.ino # Classic 5-line display; attempts CO₂/TVOC Matter publish
```

---

## License

**MIT License** — see below. You may use this code in commercial, closed‑source, or open‑source projects with attribution and the included license.

```
MIT License

Copyright (c) 2025 Nano Matter AQI contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
