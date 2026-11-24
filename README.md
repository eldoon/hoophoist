# HoopHoist

An **ESP32 + CC1101** Sub-GHz controller with a **Web Bluetooth** interface (`index.html`) and physical button support for automating timed raises/lowers on 316.50 MHz Goal Tender basketball hoop systems.

- **Live Web Bluetooth App:** https://hoophoist.com
  *(Works in Chrome on macOS/Android, or the Bluefy browser on iOS/iPadOS.)*

---

## Hardware Wiring (ESP32 → CC1101)

> **Warning:** Power the CC1101 from **`3V3` only** (never `5V`/`VIN`), and always attach the SMA antenna before transmitting.

| CC1101 Pin | ESP32 Pin | Function |
| :--- | :--- | :--- |
| **GND** | `GND` | Ground |
| **VCC** | `3V3` | 3.3V Power |
| **GDO0** | `P4` (GPIO 4) | RF TX Modulation |
| **CSN** | `P5` (GPIO 5) | SPI Chip Select |
| **SCK** | `P18` (GPIO 18) | SPI Clock |
| **MISO** | `P19` (GPIO 19) | SPI MISO |
| **MOSI** | `P23` (GPIO 23) | SPI MOSI |

### Optional Physical Buttons (Momentary to `GND`)
- **UP:** `P13` → `GND`
- **DOWN:** `P12` → `GND`
- **STOP:** `P14` → `GND` *(Stops immediately when moving; when idle, rotates through Channels 1–9 and blinks the blue LED `N` times for the selected channel)*
- **Onboard `IO0` (`P0`):** Quick press toggles UP/STOP; **hold 5 seconds** to factory-reset Bluetooth pairing.

---

## Features & Security

- **Channels 1–9:** Selectable from the Web UI (persisted in ESP32 flash).
- **Dynamic Duration (`1–900s`):** Defaults to `300s` (5 minutes), adjustable per-run or saved as defaults.
- **Emergency Stop:** Cuts RF transmission immediately from either the Web UI or physical `STOP` button.
- **Encrypted BLE Pairing:**
  - Default 6-digit pairing key: **`000000`**
  - Update the 6-digit pairing key anytime inside the Web UI under **Bluetooth Pairing Key**.
  - **Lost PIN Reset:** Hold the ESP32 **`IO0`** button for **5 seconds** until the blue LED blinks 10 times. This resets the pairing key back to `000000`, clears bonded devices, and reboots.

---

## Flashing Firmware

Open `hoop_controller_ble/hoop_controller_ble.ino` in the Arduino IDE (or `arduino-cli`) with the **ESP32** board package (`esp32:esp32:esp32`) and upload to your ESP32.
