#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

// ============================================================================
// PIN DEFINITIONS (ESP32 "P" Labels)
// ============================================================================
#define CC1101_SCK   18  // P18
#define CC1101_MISO  19  // P19
#define CC1101_MOSI  23  // P23
#define CC1101_CSN   5   // P5
#define CC1101_GDO0  4   // P4 (TX Modulation Pin)

#define BTN_UP_PIN   13  // P13 -> GND = Start UP cycle
#define BTN_DOWN_PIN 12  // P12 -> GND = Start DOWN cycle
#define BTN_STOP_PIN 14  // P14 -> GND = Emergency STOP
#define BTN_BOOT_PIN 0   // Built-in BOOT button (P0)
#define STATUS_LED   2   // Built-in Blue LED (P2)

// ============================================================================
// GOAL TENDER RF CONFIGURATION (316.50 MHz HT6026)
// ============================================================================
#define RF_FREQ_MHZ       316.50
#define HT_SHORT_HIGH_US  622
#define HT_SHORT_LOW_US   4070
#define HT_LONG_HIGH_US   4140
#define HT_LONG_LOW_US    550
#define HT_SYNC_LOW_US    28660

#define MAX_DURATION_SEC  900
#define DEFAULT_DUR_SEC   300

const char* HT_ADDR_BITS = "000000000000";
const char* CHANNEL_BITS[9] = {
  "1111", "1110", "1100",
  "1011", "1010", "1000",
  "0011", "0010", "0000"
};
const char* DIR_UP_BITS   = "11";
const char* DIR_DOWN_BITS = "10";

// ============================================================================
// BLE GATT UUIDs & PIN SECURITY (Match ble_remote.html)
// ============================================================================
#define BLE_DEVICE_NAME   "Gym-Hoop-BLE"
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CMD_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define STATUS_CHAR_UUID  "beb5483f-36e1-4688-b7f5-ea07361b26a8"
#define NAMES_CHAR_UUID   "beb54840-36e1-4688-b7f5-ea07361b26a8"
#define DEFAULT_PASSKEY   0          // Default 6-digit OS Bluetooth pairing passkey ("000000")

// ============================================================================
// CC1101 LOW-LEVEL SPI DRIVER
// ============================================================================
#define CC1101_SRES     0x30
#define CC1101_STX      0x35
#define CC1101_SIDLE    0x36
#define CC1101_PATABLE  0x7E
#define CC1101_VERSION  0xF1

void cc1101WaitMiso() {
  uint32_t start = micros();
  while (digitalRead(CC1101_MISO) == HIGH && (micros() - start) < 50000) {}
}

void cc1101Strobe(uint8_t cmd) {
  digitalWrite(CC1101_CSN, LOW);
  cc1101WaitMiso();
  SPI.transfer(cmd);
  digitalWrite(CC1101_CSN, HIGH);
}

void cc1101WriteReg(uint8_t reg, uint8_t val) {
  digitalWrite(CC1101_CSN, LOW);
  cc1101WaitMiso();
  SPI.transfer(reg);
  SPI.transfer(val);
  digitalWrite(CC1101_CSN, HIGH);
}

uint8_t cc1101ReadStatusReg(uint8_t reg) {
  digitalWrite(CC1101_CSN, LOW);
  cc1101WaitMiso();
  SPI.transfer(reg);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CC1101_CSN, HIGH);
  return val;
}

void cc1101InitOOK(float freqMHz) {
  digitalWrite(CC1101_CSN, LOW);
  delayMicroseconds(10);
  digitalWrite(CC1101_CSN, HIGH);
  delayMicroseconds(40);
  cc1101Strobe(CC1101_SRES);
  delay(2);

  cc1101WriteReg(0x00, 0x2E);
  cc1101WriteReg(0x02, 0x2E);
  cc1101WriteReg(0x08, 0x32);

  uint32_t freqReg = (uint32_t)((freqMHz * 65536.0) / 26.0 + 0.5);
  cc1101WriteReg(0x0D, (freqReg >> 16) & 0xFF);
  cc1101WriteReg(0x0E, (freqReg >> 8) & 0xFF);
  cc1101WriteReg(0x0F, freqReg & 0xFF);

  cc1101WriteReg(0x12, 0x30);
  cc1101WriteReg(0x18, 0x18);
  cc1101WriteReg(0x22, 0x11);

  // TI SmartRF Studio 300-348 MHz VCO calibration registers
  cc1101WriteReg(0x23, 0xE9);
  cc1101WriteReg(0x24, 0x2A);
  cc1101WriteReg(0x25, 0x00);
  cc1101WriteReg(0x26, 0x1F);
  cc1101WriteReg(0x2C, 0x81);
  cc1101WriteReg(0x2D, 0x35);
  cc1101WriteReg(0x2E, 0x09);

  uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  digitalWrite(CC1101_CSN, LOW);
  cc1101WaitMiso();
  SPI.transfer(CC1101_PATABLE);
  for (int i = 0; i < 8; i++) SPI.transfer(paTable[i]);
  digitalWrite(CC1101_CSN, HIGH);
}

// ============================================================================
// CONTROLLER STATE & GLOBALS
// ============================================================================
enum HoopState {
  STATE_IDLE = 0,
  STATE_MOVING_UP = 1,
  STATE_MOVING_DOWN = 2
};

HoopState currentState = STATE_IDLE;
uint32_t stateStartTimeMs = 0;
uint32_t activeDurationMs = 0;

uint8_t  selectedChannel = 1;
uint32_t upDurationSec   = DEFAULT_DUR_SEC;
uint32_t downDurationSec = DEFAULT_DUR_SEC;
uint32_t blePasskey      = DEFAULT_PASSKEY;
uint8_t  cc1101Version   = 0;

char activeFrameBits[19] = "000000000000111111";
Preferences prefs;
uint32_t lastBtnPressMs = 0;
uint32_t lastBleNotifyMs = 0;
uint32_t bootHoldStartMs = 0;

// Non-blocking LED blink state for physical channel selection
uint8_t  ledBlinksRemaining = 0;
bool     ledBlinkOn = false;
uint32_t ledBlinkNextMs = 0;

BLECharacteristic* pStatusChar = nullptr;
BLECharacteristic* pNamesChar  = nullptr;
String hoopNames[9];
bool bleConnected = false;
bool statusDirty = false;

void buildFrameBits(uint8_t ch, bool isUp) {
  if (ch < 1 || ch > 9) ch = 1;
  snprintf(activeFrameBits, sizeof(activeFrameBits), "%s%s%s",
           HT_ADDR_BITS, CHANNEL_BITS[ch - 1], isUp ? DIR_UP_BITS : DIR_DOWN_BITS);
}

void sendHT6026Frame() {
  for (int i = 0; i < 18; i++) {
    bool isOne = (activeFrameBits[i] == '1');
    digitalWrite(CC1101_GDO0, HIGH);
    delayMicroseconds(isOne ? HT_LONG_HIGH_US : HT_SHORT_HIGH_US);
    digitalWrite(CC1101_GDO0, LOW);

    if (i == 17) {
      delay(28);
      delayMicroseconds(HT_SYNC_LOW_US - 28000);
    } else {
      delayMicroseconds(isOne ? HT_LONG_LOW_US : HT_SHORT_LOW_US);
    }
  }
}

void notifyBleStatus() {
  if (!pStatusChar) return;
  uint32_t remSec = 0;
  if (currentState != STATE_IDLE) {
    uint32_t elapsed = millis() - stateStartTimeMs;
    if (elapsed < activeDurationMs) {
      remSec = (activeDurationMs - elapsed + 999) / 1000;
    }
  }
  char json[160];
  snprintf(json, sizeof(json),
           "{\"state\":%d,\"ch\":%u,\"rem\":%lu,\"up\":%lu,\"down\":%lu}",
           (int)currentState, selectedChannel, remSec, upDurationSec, downDurationSec);
  pStatusChar->setValue((uint8_t*)json, strlen(json));
  if (bleConnected) {
    pStatusChar->notify();
  }
  statusDirty = false;
}

void updateNamesCharacteristic(bool doNotify = true) {
  if (!pNamesChar) return;
  String joined = "";
  for (int i = 0; i < 9; i++) {
    if (i > 0) joined += "|";
    joined += hoopNames[i];
  }
  pNamesChar->setValue((uint8_t*)joined.c_str(), joined.length());
  if (doNotify && bleConnected) {
    pNamesChar->notify();
  }
}

void stopTransmission(const char* reason) {
  if (currentState != STATE_IDLE) {
    Serial.printf("[STOP] Stopping RF transmission (%s)\n", reason);
  }
  currentState = STATE_IDLE;
  ledBlinksRemaining = 0;
  ledBlinkOn = false;
  cc1101Strobe(CC1101_SIDLE);
  digitalWrite(CC1101_GDO0, LOW);
  digitalWrite(STATUS_LED, LOW);
  statusDirty = true;
}

void startTransmission(uint8_t newStateIdx, const char* source, uint32_t overrideDurSec = 0) {
  HoopState newState = (HoopState)newStateIdx;
  if (currentState != STATE_IDLE && currentState == newState) {
    stopTransmission("Toggle Stop");
    return;
  }

  uint32_t durSec = overrideDurSec > 0
                      ? overrideDurSec
                      : (newState == STATE_MOVING_UP ? upDurationSec : downDurationSec);
  if (durSec < 1) durSec = 1;
  if (durSec > MAX_DURATION_SEC) durSec = MAX_DURATION_SEC;

  buildFrameBits(selectedChannel, newState == STATE_MOVING_UP);
  currentState = newState;
  stateStartTimeMs = millis();
  activeDurationMs = durSec * 1000UL;
  ledBlinksRemaining = 0;
  ledBlinkOn = false;

  cc1101Strobe(CC1101_STX);
  delayMicroseconds(800);
  digitalWrite(STATUS_LED, HIGH);
  statusDirty = true;

  Serial.printf("[START] Ch %u %s via %s for %lus (Bits: %s)\n",
                selectedChannel,
                newState == STATE_MOVING_UP ? "UP" : "DOWN",
                source, durSec, activeFrameBits);
}

// ============================================================================
// BLE CALLBACKS
// ============================================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    statusDirty = true;
    Serial.println("[BLE] Client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    Serial.println("[BLE] Client disconnected — restarting advertising");
    BLEDevice::startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = pChar->getValue().c_str();
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.printf("[BLE CMD] %s\n", cmd.c_str());

    if (cmd.equalsIgnoreCase("STOP")) {
      stopTransmission("BLE STOP");
      return;
    }

    if (cmd.startsWith("CH:")) {
      int ch = cmd.substring(3).toInt();
      if (ch >= 1 && ch <= 9) {
        if (currentState != STATE_IDLE) stopTransmission("BLE Channel Change");
        selectedChannel = (uint8_t)ch;
        prefs.putUChar("ch", selectedChannel);
        statusDirty = true;
      }
      return;
    }

    if (cmd.startsWith("SET:")) {
      int sep = cmd.indexOf(':', 4);
      if (sep > 4) {
        uint32_t u = cmd.substring(4, sep).toInt();
        uint32_t d = cmd.substring(sep + 1).toInt();
        if (u >= 1 && u <= MAX_DURATION_SEC) {
          upDurationSec = u;
          prefs.putUInt("upSec", upDurationSec);
        }
        if (d >= 1 && d <= MAX_DURATION_SEC) {
          downDurationSec = d;
          prefs.putUInt("downSec", downDurationSec);
        }
        statusDirty = true;
      }
      return;
    }

    if (cmd.startsWith("NAMES:")) {
      String payload = cmd.substring(6);
      int idx = 0;
      int start = 0;
      while (idx < 9 && start <= payload.length()) {
        int sep = payload.indexOf('|', start);
        String part = (sep == -1) ? payload.substring(start) : payload.substring(start, sep);
        part.trim();
        if (part.length() > 18) part = part.substring(0, 18);
        if (part.length() == 0) part = "Hoop " + String(idx + 1);
        hoopNames[idx] = part;
        char kbuf[4];
        snprintf(kbuf, sizeof(kbuf), "n%d", idx);
        prefs.putString(kbuf, hoopNames[idx]);
        idx++;
        if (sep == -1) break;
        start = sep + 1;
      }
      updateNamesCharacteristic(true);
      Serial.println("[BLE] Saved custom hoop names to NVS");
      return;
    }

    if (cmd.startsWith("KEY:")) {
      String k = cmd.substring(4);
      k.trim();
      if (k.length() == 6) {
        bool allDigits = true;
        for (int i = 0; i < 6; i++) {
          if (!isDigit(k[i])) { allDigits = false; break; }
        }
        if (allDigits) {
          blePasskey = (uint32_t)k.toInt();
          prefs.putUInt("passkey", blePasskey);
          BLESecurity::setPassKey(true, blePasskey);
          Serial.printf("[BLE] Updated pairing passkey to %06lu\n", blePasskey);
          statusDirty = true;
        }
      }
      return;
    }

    if (cmd.startsWith("UP:") || cmd.startsWith("DOWN:")) {
      bool isUp = cmd.startsWith("UP:");
      int firstColon = cmd.indexOf(':');
      int secondColon = cmd.indexOf(':', firstColon + 1);
      int ch = selectedChannel;
      uint32_t dur = 0;

      if (secondColon > firstColon) {
        ch = cmd.substring(firstColon + 1, secondColon).toInt();
        dur = cmd.substring(secondColon + 1).toInt();
      } else {
        ch = cmd.substring(firstColon + 1).toInt();
      }

      if (ch >= 1 && ch <= 9) {
        selectedChannel = (uint8_t)ch;
        prefs.putUChar("ch", selectedChannel);
      }
      startTransmission(isUp ? STATE_MOVING_UP : STATE_MOVING_DOWN, "BLE App", dur);
    }
  }
};

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(CC1101_GDO0, OUTPUT);
  digitalWrite(CC1101_GDO0, LOW);

  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  pinMode(BTN_BOOT_PIN, INPUT_PULLUP);

  prefs.begin("hoop", false);
  selectedChannel = prefs.getUChar("ch", 1);
  if (selectedChannel < 1 || selectedChannel > 9) selectedChannel = 1;
  upDurationSec = prefs.getUInt("upSec", DEFAULT_DUR_SEC);
  downDurationSec = prefs.getUInt("downSec", DEFAULT_DUR_SEC);
  blePasskey = prefs.getUInt("passkey", DEFAULT_PASSKEY);
  if (blePasskey > 999999) blePasskey = DEFAULT_PASSKEY;

  for (int i = 0; i < 9; i++) {
    char kbuf[4];
    snprintf(kbuf, sizeof(kbuf), "n%d", i);
    hoopNames[i] = prefs.getString(kbuf, "Hoop " + String(i + 1));
  }

  pinMode(CC1101_CSN, OUTPUT);
  digitalWrite(CC1101_CSN, HIGH);
  SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
  cc1101InitOOK(RF_FREQ_MHZ);
  cc1101Version = cc1101ReadStatusReg(CC1101_VERSION);

  // Initialize BLE Server with Link-Layer MITM Passkey Encryption
  BLEDevice::init(BLE_DEVICE_NAME);
  BLESecurity::setPassKey(true, blePasskey);
  BLESecurity::setAuthenticationMode(true, true, true); // bonding, MITM, Secure Connections
  BLESecurity::setCapability(ESP_IO_CAP_OUT);
  BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLESecurity::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic* pCmdChar = pService->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdChar->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
  pCmdChar->setCallbacks(new CmdCallbacks());

  pStatusChar = pService->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusChar->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);
  BLE2902* p2902 = new BLE2902();
  p2902->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM);
  pStatusChar->addDescriptor(p2902);

  pNamesChar = pService->createCharacteristic(
    NAMES_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pNamesChar->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);
  BLE2902* pNames2902 = new BLE2902();
  pNames2902->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM);
  pNamesChar->addDescriptor(pNames2902);

  pService->start();
  notifyBleStatus();
  updateNamesCharacteristic(false);

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("\n=======================================================");
  Serial.println("   🏀 GOAL TENDER BLE CONTROLLER (316.50 MHz HT6026)   ");
  Serial.println("=======================================================");
  Serial.printf("  CC1101 Status : VERSION 0x%02X (%.2f MHz)\n", cc1101Version, RF_FREQ_MHZ);
  Serial.printf("  BLE Name      : %s\n", BLE_DEVICE_NAME);
  Serial.printf("  Active Channel: %u | Default UP: %lus | DOWN: %lus\n", selectedChannel, upDurationSec, downDurationSec);
  Serial.printf("  Physical Pins : UP=P13, DOWN=P12, STOP=P14 (or BOOT=P0)\n");
  Serial.println("=======================================================\n");
}

void factoryResetPasskey() {
  stopTransmission("Factory Reset");
  blePasskey = DEFAULT_PASSKEY;
  prefs.putUInt("passkey", DEFAULT_PASSKEY);

  // Remove any bonded BLE devices so phones re-pair cleanly with 000000
  int devNum = esp_ble_get_bond_device_num();
  if (devNum > 0) {
    esp_ble_bond_dev_t* devList = (esp_ble_bond_dev_t*)malloc(sizeof(esp_ble_bond_dev_t) * devNum);
    if (devList) {
      esp_ble_get_bond_device_list(&devNum, devList);
      for (int i = 0; i < devNum; i++) {
        esp_ble_remove_bond_device(devList[i].bd_addr);
      }
      free(devList);
    }
  }

  Serial.println("[RESET] Factory reset complete! Pairing passkey reset to 000000. Rebooting...");
  for (int i = 0; i < 10; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(80);
    digitalWrite(STATUS_LED, LOW);
    delay(80);
  }
  ESP.restart();
}

void checkPhysicalButtons() {
  // Detect 5-second continuous hold on built-in BOOT button (P0) for Factory Reset
  if (digitalRead(BTN_BOOT_PIN) == LOW) {
    if (bootHoldStartMs == 0) {
      bootHoldStartMs = millis();
    } else if (millis() - bootHoldStartMs >= 5000) {
      factoryResetPasskey();
      return;
    }
  } else {
    bootHoldStartMs = 0;
  }

  if (millis() - lastBtnPressMs < 250) return;

  if (digitalRead(BTN_STOP_PIN) == LOW) {
    lastBtnPressMs = millis();
    if (currentState != STATE_IDLE) {
      stopTransmission("Physical STOP (P14)");
    } else {
      // Idle: rotate channel 1 -> 2 -> ... -> 9 -> 1 and blink LED N times
      selectedChannel = (selectedChannel % 9) + 1;
      prefs.putUChar("ch", selectedChannel);
      statusDirty = true;
      digitalWrite(STATUS_LED, LOW);
      ledBlinkOn = false;
      ledBlinksRemaining = selectedChannel;
      ledBlinkNextMs = millis() + 250; // Short settle so rapid presses jump cleanly
      Serial.printf("[CHANNEL] Rotated to Channel %u\n", selectedChannel);
    }
    return;
  }
  if (digitalRead(BTN_BOOT_PIN) == LOW) {
    lastBtnPressMs = millis();
    if (currentState != STATE_IDLE) stopTransmission("BOOT Button (P0)");
    else startTransmission(STATE_MOVING_UP, "BOOT Button (P0)");
    return;
  }
  if (digitalRead(BTN_UP_PIN) == LOW) {
    lastBtnPressMs = millis();
    startTransmission(STATE_MOVING_UP, "Physical UP (P13)");
    return;
  }
  if (digitalRead(BTN_DOWN_PIN) == LOW) {
    lastBtnPressMs = millis();
    startTransmission(STATE_MOVING_DOWN, "Physical DOWN (P12)");
    return;
  }
}

void updateLedBlinks() {
  if (currentState != STATE_IDLE || ledBlinksRemaining == 0) return;
  if ((int32_t)(millis() - ledBlinkNextMs) < 0) return;

  if (!ledBlinkOn) {
    ledBlinkOn = true;
    digitalWrite(STATUS_LED, HIGH);
    ledBlinkNextMs = millis() + 140;
  } else {
    ledBlinkOn = false;
    digitalWrite(STATUS_LED, LOW);
    ledBlinksRemaining--;
    ledBlinkNextMs = millis() + 180;
  }
}

void loop() {
  checkPhysicalButtons();
  updateLedBlinks();

  if (currentState != STATE_IDLE) {
    uint32_t elapsed = millis() - stateStartTimeMs;
    if (elapsed >= activeDurationMs) {
      stopTransmission("Timer Completed");
    } else {
      sendHT6026Frame();
    }
  }

  if (statusDirty || (currentState != STATE_IDLE && millis() - lastBleNotifyMs >= 500)) {
    lastBleNotifyMs = millis();
    notifyBleStatus();
  }
}
