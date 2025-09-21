# Arduino Nano Matter AQI

This project runs on **Arduino Nano Matter** and measures indoor air quality using:

- **ENS160** → eCO₂ and tVOC  
- **BME680** → Humidity & Pressure (gas sensing disabled)  
- **DS18B20** → Temperature  
- **OLED** (128×64 SSD1306) for local display  
- **RGB LED** (or fallback single LED) for air quality alerts  
- **Matter protocol** for HomeKit / Matter integration

---

## Features

- **OLED Display**
  - Shows AQI (1–5 with label), eCO₂, tVOC, Humidity, Pressure, Temperature
- **LED Alerts (simplified)**
  - **tVOC out of safe range → Purple**
  - **eCO₂ out of safe range → Blue**
  - **Both out of safe range → Red**
  - Constant light, **no blinking**
- **Safe Ranges**
  - eCO₂: ≤ 1000 ppm  
  - tVOC: ≤ 500 ppb  
  - Humidity: 30–70 %  
  - Temperature: 18–25 °C  
  - Pressure: 950–1050 hPa
- **Matter Integration**
  - Publishes: Temperature, Humidity, Pressure  
  - Optional: Air Quality index (enum for HomeKit)
- **Pairing Screen**
  - QR code and manual code shown on OLED for quick onboarding

---

## License

This project is licensed under the **MIT License** – free to use, modify, and distribute.

---

## Hardware Setup

- Arduino Nano Matter  
- ENS160 sensor (I²C, addr 0x53)  
- BME680 sensor (I²C, addr 0x76/0x77)  
- DS18B20 sensor (1-Wire, pin D5)  
- SSD1306 OLED 128×64 (I²C, addr 0x3C)  
- RGB LED (active HIGH) or onboard LED

---

## Screenshots

### OLED Sensor Display
Shows live AQI, eCO₂, tVOC, Humidity, Pressure, Temperature.

### OLED Pairing Screen
Shows Matter QR code + manual pairing code, with Thread status.

---

## Notes

- At the moment, iOS / HomeKit only exposes Temperature, Humidity, and Pressure over Matter.  
- Air Quality (enum tile) is experimental and optional.  
- LED indication is simplified to avoid night-time blinking and distraction.

---
