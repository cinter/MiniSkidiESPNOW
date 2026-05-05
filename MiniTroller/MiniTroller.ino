#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <esp_system.h>

// =====================
// Debug configuration
// =====================
// Set to 0 for normal use after testing.
#define DEBUG_ENABLED 1

constexpr unsigned long debugInputInterval = 500UL;

#if DEBUG_ENABLED
  #define DBG_BEGIN(baud) Serial.begin(baud)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN(baud) do {} while (false)
  #define DBG_PRINTLN(...) do {} while (false)
  #define DBG_PRINTF(...) do {} while (false)
#endif

Preferences preferences;

// --- Pin Definitions (Seeed Studio XIAO ESP32-C3) ---
constexpr int pinX      = 2;
constexpr int pinY      = 3;
constexpr int btnPower  = 4;
constexpr int btnLights = 5;
constexpr int neoPin    = 6;
constexpr int sw1_A     = 7;
constexpr int sw1_B     = 21;
constexpr int sw2_A     = 20;
constexpr int sw2_B     = 8;
constexpr int sw3_A     = 9;
constexpr int sw3_B     = 10;

Adafruit_NeoPixel pixel(1, neoPin, NEO_GRB + NEO_KHZ800);

// --- System Variables ---
RTC_DATA_ATTR int crashCounter = 0;

enum SystemState { IDLE, PAIRING, PAIRED };
SystemState currentState = IDLE;

uint8_t receiverMAC[6] = {0};
esp_now_peer_info_t peerInfo = {};

portMUX_TYPE pairingMux = portMUX_INITIALIZER_UNLOCKED;
bool pairingComplete = false;
uint8_t pendingPeerMAC[6] = {0};

volatile unsigned long lastVehicleHeartbeatTime = 0;

unsigned long lastActivityTime = 0;
constexpr unsigned long sleepTimeout = 300000UL;
constexpr unsigned long pairingTimeout = 30000UL;
constexpr unsigned long vehicleHeartbeatTimeout = 1500UL;

unsigned long lastSendTime = 0;
unsigned long pairingStartTime = 0;
unsigned long lastLedUpdate = 0;
unsigned long lastDebugInputPrint = 0;

unsigned long btnPowerPressTime = 0;
unsigned long btnLightsPressTime = 0;

bool isPowerBtnHeld = false;
bool isLightsBtnHeld = false;
bool lightsLongPressHandled = false;

bool lightsAreOn = false;
int activeReceiverSlot = 1;

constexpr unsigned long debounceDelay = 25UL;
constexpr unsigned long holdTime = 2000UL;

bool powerBtnRawState = HIGH;
bool powerBtnStableState = HIGH;
bool lastPowerBtnRawState = HIGH;
unsigned long lastPowerBtnDebounceTime = 0;

bool lightsBtnRawState = HIGH;
bool lightsBtnStableState = HIGH;
bool lastLightsBtnRawState = HIGH;
unsigned long lastLightsBtnDebounceTime = 0;

bool lastPowerBtnPressed = false;
bool lastLightsBtnPressed = false;

int centerX = 2048;
int centerY = 2048;
int smoothX = 0;
int smoothY = 0;

constexpr int joystickDeadzone = 40;
constexpr int joystickActivityThreshold = 150;

struct __attribute__((packed)) ControlPacket {
  int16_t x;
  int16_t y;
  uint8_t sw1_A;
  uint8_t sw1_B;
  uint8_t sw2_A;
  uint8_t sw2_B;
  uint8_t sw3_A;
  uint8_t sw3_B;
  uint8_t lightsOn;
};

ControlPacket controlData = {};

const char *stateName(SystemState state) {
  switch (state) {
    case IDLE:    return "IDLE";
    case PAIRING: return "PAIRING";
    case PAIRED:  return "PAIRED";
    default:      return "UNKNOWN";
  }
}

void printMAC(const uint8_t mac[6]) {
#if DEBUG_ENABLED
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
}

void logState(const char *message) {
#if DEBUG_ENABLED
  Serial.printf("[%lu] %s | state=%s slot=%d lights=%s\n",
                millis(), message, stateName(currentState), activeReceiverSlot,
                lightsAreOn ? "ON" : "OFF");
#endif
}

void slotColour(uint8_t slot, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (slot == 1) {
    r = 255;
    g = 150;
    b = 0;
  } else {
    r = 0;
    g = 255;
    b = 0;
  }
}

// --- ESP-NOW Callbacks ---
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  if (recv_info == nullptr || incomingData == nullptr) {
    return;
  }

  if (len == 4 && memcmp(incomingData, "PAIR", 4) == 0) {
    portENTER_CRITICAL(&pairingMux);
    memcpy(pendingPeerMAC, recv_info->src_addr, sizeof(pendingPeerMAC));
    pairingComplete = true;
    portEXIT_CRITICAL(&pairingMux);
    return;
  }

  if (len == 2 && memcmp(incomingData, "HB", 2) == 0) {
    lastVehicleHeartbeatTime = millis();
    return;
  }
}

void OnDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
#if DEBUG_ENABLED
  static esp_now_send_status_t lastStatus = ESP_NOW_SEND_SUCCESS;

  if (status != lastStatus) {
    Serial.printf("[%lu] ESP-NOW send status changed: %s",
                  millis(),
                  status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");

    if (txInfo != nullptr) {
      Serial.print(" peer=");
      printMAC(txInfo->des_addr);
    }

    Serial.println();
    lastStatus = status;
  }
#else
  (void)txInfo;
  (void)status;
#endif
}

// --- Logic Functions ---
void enterPairingMode(unsigned long now) {
  DBG_PRINTF("[%lu] Clearing slot %d and entering pairing mode\n", now, activeReceiverSlot);

  const char *key = (activeReceiverSlot == 1) ? "peer_mac_1" : "peer_mac_2";
  preferences.remove(key);

  if (esp_now_is_peer_exist(receiverMAC)) {
    esp_now_del_peer(receiverMAC);
  }

  memset(receiverMAC, 0, sizeof(receiverMAC));

  portENTER_CRITICAL(&pairingMux);
  pairingComplete = false;
  memset(pendingPeerMAC, 0, sizeof(pendingPeerMAC));
  portEXIT_CRITICAL(&pairingMux);

  lastVehicleHeartbeatTime = 0;
  currentState = PAIRING;
  pairingStartTime = now;
}

void writeStatusLED() {
  const unsigned long now = millis();

  if (now - lastLedUpdate < 50UL) {
    return;
  }
  lastLedUpdate = now;

  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  bool isOn = true;

  slotColour(activeReceiverSlot, r, g, b);

  bool heartbeatTimedOut = false;

  if (currentState == PAIRED) {
    const unsigned long hbTime = lastVehicleHeartbeatTime;

    if (hbTime == 0) {
      heartbeatTimedOut = true;
    } else if (hbTime <= now) {
      heartbeatTimedOut = (now - hbTime > vehicleHeartbeatTimeout);
    } else {
      // millis() wrapped or callback updated hbTime after this loop iteration started.
      // Treat as fresh rather than declaring a false timeout.
      heartbeatTimedOut = false;
    }
  }

  if (currentState == PAIRING) {
    // Fast blink in the selected slot colour while listening for a receiver.
    isOn = ((now / 150UL) % 2UL) != 0UL;
  } else if (currentState == IDLE) {
    // Empty active slot: short periodic blink in the selected slot colour.
    // Slot 1 = amber. Slot 2 = green.
    isOn = (now % 2000UL) < 120UL;
  } else if (currentState == PAIRED && heartbeatTimedOut) {
    // Paired but vehicle heartbeat missing: alternate selected slot colour with red.
    if ((now / 500UL) % 2UL != 0UL) {
      r = 255;
      g = 0;
      b = 0;
    }
  }

  if (isOn) {
    pixel.setPixelColor(0, pixel.Color(r, g, b));
  } else {
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  }

  pixel.show();
}

void rainbowStartup() {
  for (long hue = 0; hue < 3L * 65536L; hue += 512L) {
    pixel.setPixelColor(0, pixel.gamma32(pixel.ColorHSV(hue)));
    pixel.show();
    delay(2);
  }
}

void shutdownSequence() {
  logState("Shutdown requested");

  while (digitalRead(btnPower) == LOW) {
    static int hue = 0;
    pixel.setPixelColor(0, pixel.gamma32(pixel.ColorHSV(hue)));
    pixel.show();
    hue += 1500;
    delay(20);
  }

  pixel.clear();
  pixel.show();
  esp_deep_sleep_start();
}

bool updateDebouncedButton(
  int pin,
  bool &rawState,
  bool &stableState,
  bool &lastRawState,
  unsigned long &lastDebounceTime,
  unsigned long currentMillis
) {
  const bool reading = digitalRead(pin);

  if (reading != lastRawState) {
    lastDebounceTime = currentMillis;
    lastRawState = reading;
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay) {
    stableState = reading;
  }

  rawState = reading;
  return stableState;
}

int readAveragedAnalog(int pin, int samples = 8) {
  long total = 0;
  for (int i = 0; i < samples; ++i) {
    total += analogRead(pin);
    delayMicroseconds(200);
  }
  return static_cast<int>(total / samples);
}

void calibrateJoystickCenter() {
  long totalX = 0;
  long totalY = 0;

  for (int i = 0; i < 64; ++i) {
    totalX += analogRead(pinX);
    totalY += analogRead(pinY);
    delay(5);
  }

  centerX = static_cast<int>(totalX / 64);
  centerY = static_cast<int>(totalY / 64);
  smoothX = centerX;
  smoothY = centerY;

  DBG_PRINTF("Joystick centre calibrated: X=%d Y=%d\n", centerX, centerY);
}

int normalizeAxis(int raw, int center) {
  const int delta = raw - center;

  if (abs(delta) < joystickDeadzone) {
    return 0;
  }

  if (delta < 0) {
    return constrain(map(raw, 0, center, -1000, 0), -1000, 0);
  }

  return constrain(map(raw, center, 4095, 0, 1000), 0, 1000);
}

bool loadReceiverForSlot(int slot) {
  uint8_t tempMAC[6] = {0};
  const char *key = (slot == 1) ? "peer_mac_1" : "peer_mac_2";
  const size_t bytesRead = preferences.getBytes(key, tempMAC, sizeof(tempMAC));

  if (esp_now_is_peer_exist(receiverMAC)) {
    esp_now_del_peer(receiverMAC);
  }

  memset(receiverMAC, 0, sizeof(receiverMAC));

  if (bytesRead == sizeof(tempMAC)) {
    memcpy(receiverMAC, tempMAC, sizeof(receiverMAC));

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverMAC, sizeof(receiverMAC));
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    const esp_err_t addResult = esp_now_add_peer(&peerInfo);
    if (addResult == ESP_OK) {
      currentState = PAIRED;
      lastVehicleHeartbeatTime = millis();  // short grace period until first HB arrives

#if DEBUG_ENABLED
      Serial.printf("[%lu] Loaded receiver for slot %d: ", millis(), slot);
      printMAC(receiverMAC);
      Serial.println();
#endif
      return true;
    }

    DBG_PRINTF("[%lu] Failed to add peer for slot %d, err=%d\n", millis(), slot, static_cast<int>(addResult));
  } else {
    DBG_PRINTF("[%lu] No saved receiver in slot %d\n", millis(), slot);
  }

  currentState = IDLE;
  lastVehicleHeartbeatTime = 0;
  return false;
}

void printDebugInputs(
  unsigned long now,
  int rawX,
  int rawY,
  int joyX,
  int joyY,
  bool pBtn,
  bool lBtn
) {
#if DEBUG_ENABLED
  if (now - lastDebugInputPrint < debugInputInterval) {
    return;
  }
  lastDebugInputPrint = now;

  const unsigned long hbTime = lastVehicleHeartbeatTime;
  const long hbAge = (hbTime == 0 || hbTime > now) ? -1L : static_cast<long>(now - hbTime);

  Serial.printf(
    "[%lu] state=%s slot=%d raw=(%d,%d) smooth=(%d,%d) joy=(%d,%d) "
    "P=%d L=%d sw=%d%d %d%d %d%d lights=%d hbAge=%ldms\n",
    now,
    stateName(currentState),
    activeReceiverSlot,
    rawX,
    rawY,
    smoothX,
    smoothY,
    joyX,
    joyY,
    pBtn == LOW,
    lBtn == LOW,
    digitalRead(sw1_A) == LOW,
    digitalRead(sw1_B) == LOW,
    digitalRead(sw2_A) == LOW,
    digitalRead(sw2_B) == LOW,
    digitalRead(sw3_A) == LOW,
    digitalRead(sw3_B) == LOW,
    lightsAreOn ? 1 : 0,
    hbAge
  );
#else
  (void)now;
  (void)rawX;
  (void)rawY;
  (void)joyX;
  (void)joyY;
  (void)pBtn;
  (void)lBtn;
#endif
}

void setup() {
  DBG_BEGIN(115200);
#if DEBUG_ENABLED
  delay(1000);
#endif

  const esp_reset_reason_t reason = esp_reset_reason();
  DBG_PRINTF("Boot reset reason: %d\n", static_cast<int>(reason));

  if (reason == ESP_RST_BROWNOUT) {
    crashCounter++;
    if (crashCounter > 2) {
      // We crashed 3 times in a row. The battery is probably too low.
      // Go straight to deep sleep. Do NOT turn on the LED or radio.
      esp_deep_sleep_start();
    }
  } else {
    // Normal boot or reset button press; clear the counter.
    crashCounter = 0;
  }

  pixel.begin();
  pixel.setBrightness(5);
  pixel.clear();
  pixel.show();
  rainbowStartup();

  pinMode(btnPower, INPUT_PULLUP);
  pinMode(btnLights, INPUT_PULLUP);
  pinMode(sw1_A, INPUT_PULLUP);
  pinMode(sw1_B, INPUT_PULLUP);
  pinMode(sw2_A, INPUT_PULLUP);
  pinMode(sw2_B, INPUT_PULLUP);
  pinMode(sw3_A, INPUT_PULLUP);
  pinMode(sw3_B, INPUT_PULLUP);

  calibrateJoystickCenter();

  esp_deep_sleep_enable_gpio_wakeup(1ULL << btnPower, ESP_GPIO_WAKEUP_GPIO_LOW);

  WiFi.mode(WIFI_STA);

  // Drop from max power to about 8.5 dBm.
  const esp_err_t txPowerResult = esp_wifi_set_max_tx_power(34); // 34 * 0.25 dBm = 8.5 dBm
  if (txPowerResult != ESP_OK) {
    DBG_PRINTF("esp_wifi_set_max_tx_power failed, err=%d\n", static_cast<int>(txPowerResult));
  }

  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    DBG_PRINTLN("ESP-NOW init failed");
    currentState = IDLE;
    while (true) {
      // Red blink for radio init failure.
      if ((millis() / 250UL) % 2UL == 0UL) {
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      } else {
        pixel.clear();
      }
      pixel.show();
      delay(20);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  if (!preferences.begin("link-data", false)) {
    DBG_PRINTLN("Preferences init failed; saved pairing slots unavailable");
    currentState = IDLE;
  } else {
    activeReceiverSlot = preferences.getInt("active_slot", 1);
    if (activeReceiverSlot != 1 && activeReceiverSlot != 2) {
      activeReceiverSlot = 1;
      preferences.putInt("active_slot", activeReceiverSlot);
    }
    loadReceiverForSlot(activeReceiverSlot);
  }

  lastActivityTime = millis();
  logState("Setup complete");
}

void loop() {
  const unsigned long currentMillis = millis();

  // Joystick & Activity
  const int rawX = readAveragedAnalog(pinX);
  const int rawY = readAveragedAnalog(pinY);

  smoothX = (smoothX * 3 + rawX) / 4;
  smoothY = (smoothY * 3 + rawY) / 4;

  const int joyX = normalizeAxis(smoothX, centerX);
  const int joyY = normalizeAxis(smoothY, centerY);

  // Buttons
  const bool pBtn = updateDebouncedButton(
    btnPower,
    powerBtnRawState,
    powerBtnStableState,
    lastPowerBtnRawState,
    lastPowerBtnDebounceTime,
    currentMillis
  );

  const bool lBtn = updateDebouncedButton(
    btnLights,
    lightsBtnRawState,
    lightsBtnStableState,
    lastLightsBtnRawState,
    lastLightsBtnDebounceTime,
    currentMillis
  );

  printDebugInputs(currentMillis, rawX, rawY, joyX, joyY, pBtn, lBtn);

  if (abs(smoothX - centerX) > joystickActivityThreshold ||
      abs(smoothY - centerY) > joystickActivityThreshold ||
      pBtn == LOW ||
      lBtn == LOW) {
    lastActivityTime = currentMillis;
  }

  if (currentMillis - lastActivityTime > sleepTimeout) {
    shutdownSequence();
  }

  // Power Button (short press = slot swap, long press = sleep)
  if (pBtn == LOW && !lastPowerBtnPressed) {
    isPowerBtnHeld = true;
    btnPowerPressTime = currentMillis;
    logState("Power button pressed");
  }

  if (pBtn == LOW && isPowerBtnHeld && (currentMillis - btnPowerPressTime >= holdTime)) {
    logState("Power button long press");
    shutdownSequence();
  }

  if (pBtn == HIGH && lastPowerBtnPressed) {
    logState("Power button released");
    if (isPowerBtnHeld) {
      activeReceiverSlot = (activeReceiverSlot == 1) ? 2 : 1;
      preferences.putInt("active_slot", activeReceiverSlot);
      loadReceiverForSlot(activeReceiverSlot);
      logState("Active receiver slot changed");
    }
    isPowerBtnHeld = false;
  }

  lastPowerBtnPressed = (pBtn == LOW);

  // Light Button (short press = lights, long press = pair)
  if (lBtn == LOW && !lastLightsBtnPressed) {
    isLightsBtnHeld = true;
    lightsLongPressHandled = false;
    btnLightsPressTime = currentMillis;
    logState("Lights button pressed");
  }

  if (lBtn == LOW && isLightsBtnHeld && !lightsLongPressHandled &&
      (currentMillis - btnLightsPressTime >= holdTime)) {
    enterPairingMode(currentMillis);
    lightsLongPressHandled = true;
    logState("Lights button long press handled");
  }

  if (lBtn == HIGH && lastLightsBtnPressed) {
    logState("Lights button released");
    if (isLightsBtnHeld && !lightsLongPressHandled) {
      lightsAreOn = !lightsAreOn;
      logState(lightsAreOn ? "Lights toggled ON" : "Lights toggled OFF");
    }
    isLightsBtnHeld = false;
  }

  lastLightsBtnPressed = (lBtn == LOW);

  // Handle Pairing Success
  if (currentState == PAIRING) {
    uint8_t pairedMAC[6] = {0};
    bool hasPendingPair = false;

    portENTER_CRITICAL(&pairingMux);
    if (pairingComplete) {
      memcpy(pairedMAC, pendingPeerMAC, sizeof(pairedMAC));
      pairingComplete = false;
      hasPendingPair = true;
    }
    portEXIT_CRITICAL(&pairingMux);

    if (hasPendingPair) {
      memcpy(receiverMAC, pairedMAC, sizeof(receiverMAC));

      const char *key = (activeReceiverSlot == 1) ? "peer_mac_1" : "peer_mac_2";
      preferences.putBytes(key, receiverMAC, sizeof(receiverMAC));

      if (esp_now_is_peer_exist(receiverMAC)) {
        esp_now_del_peer(receiverMAC);
      }

      memset(&peerInfo, 0, sizeof(peerInfo));
      memcpy(peerInfo.peer_addr, receiverMAC, sizeof(receiverMAC));
      peerInfo.channel = 0;
      peerInfo.encrypt = false;

      const esp_err_t addResult = esp_now_add_peer(&peerInfo);
      if (addResult == ESP_OK) {
        currentState = PAIRED;
        lastVehicleHeartbeatTime = millis();  // short grace period until first HB arrives
#if DEBUG_ENABLED
        Serial.print("Paired with receiver: ");
        printMAC(receiverMAC);
        Serial.println();
#endif
        logState("Pairing complete");
      } else {
        currentState = IDLE;
        lastVehicleHeartbeatTime = 0;
        DBG_PRINTF("Pairing failed: esp_now_add_peer err=%d\n", static_cast<int>(addResult));
      }
    }
  }

  if (currentState == PAIRING && (currentMillis - pairingStartTime >= pairingTimeout)) {
    DBG_PRINTF("[%lu] Pairing timed out; reloading slot %d\n", currentMillis, activeReceiverSlot);
    loadReceiverForSlot(activeReceiverSlot);
  }

  // Send Data
  if (currentState == PAIRED && (currentMillis - lastSendTime >= 20UL)) {
    controlData.x = static_cast<int16_t>(joyX);
    controlData.y = static_cast<int16_t>(joyY);
    controlData.sw1_A = (digitalRead(sw1_A) == LOW);
    controlData.sw1_B = (digitalRead(sw1_B) == LOW);
    controlData.sw2_A = (digitalRead(sw2_A) == LOW);
    controlData.sw2_B = (digitalRead(sw2_B) == LOW);
    controlData.sw3_A = (digitalRead(sw3_A) == LOW);
    controlData.sw3_B = (digitalRead(sw3_B) == LOW);
    controlData.lightsOn = lightsAreOn ? 1U : 0U;

    const esp_err_t sendResult = esp_now_send(receiverMAC, reinterpret_cast<uint8_t *>(&controlData), sizeof(controlData));
#if DEBUG_ENABLED
    static esp_err_t lastSendResult = ESP_OK;
    if (sendResult != ESP_OK && sendResult != lastSendResult) {
      Serial.printf("[%lu] esp_now_send failed, err=%d\n", currentMillis, static_cast<int>(sendResult));
    }
    lastSendResult = sendResult;
#endif
    lastSendTime = currentMillis;
  }

  writeStatusLED();
}
