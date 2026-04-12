#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

Preferences preferences;

// --- Pin Definitions (XIAO ESP32-C3) ---
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

// --- ESP-NOW Callbacks ---
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
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

// --- Logic Functions ---
void enterPairingMode(unsigned long now) {
  Serial.printf("Clearing Slot %d and entering Pairing Mode...\n", activeReceiverSlot);

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

  if (activeReceiverSlot == 1) {
    r = 255;
    g = 150;
    b = 0;
  } else {
    r = 0;
    g = 255;
    b = 0;
  }

  bool heartbeatTimedOut = false;

  if (currentState == PAIRED) {
    const unsigned long hbTime = lastVehicleHeartbeatTime;

    if (hbTime == 0) {
      heartbeatTimedOut = true;
    } else if (hbTime <= now) {
      heartbeatTimedOut = (now - hbTime > vehicleHeartbeatTimeout);
    } else {
      // Callback updated hbTime after this loop iteration started.
      // Treat as fresh, not timed out.
      heartbeatTimedOut = false;
    }
  }

  if (currentState == PAIRED && heartbeatTimedOut) {
    if ((now / 500UL) % 2UL != 0UL) {
      r = 255;
      g = 0;
      b = 0;
    }
  }

  if (currentState == PAIRING) {
    isOn = ((now / 150UL) % 2UL) != 0UL;
  } else if (currentState == IDLE) {
    r = 5;
    g = 5;
    b = 5;
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

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      currentState = PAIRED;
      lastVehicleHeartbeatTime = millis();  // short grace period until first HB arrives
      return true;
    }
  }

  currentState = IDLE;
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pixel.begin();
  pixel.setBrightness(40);
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
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    currentState = IDLE;
    while (true) {
      writeStatusLED();
      delay(10);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  preferences.begin("link-data", false);
  activeReceiverSlot = preferences.getInt("active_slot", 1);
  loadReceiverForSlot(activeReceiverSlot);

  lastActivityTime = millis();
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

  if (abs(smoothX - centerX) > joystickActivityThreshold ||
      abs(smoothY - centerY) > joystickActivityThreshold ||
      pBtn == LOW ||
      lBtn == LOW) {
    lastActivityTime = currentMillis;
  }

  if (currentMillis - lastActivityTime > sleepTimeout) {
    shutdownSequence();
  }

  // Power Button (Slot swap / Sleep)
  if (pBtn == LOW && !lastPowerBtnPressed) {
    isPowerBtnHeld = true;
    btnPowerPressTime = currentMillis;
  }

  if (pBtn == LOW && isPowerBtnHeld && (currentMillis - btnPowerPressTime >= holdTime)) {
    shutdownSequence();
  }

  if (pBtn == HIGH && lastPowerBtnPressed) {
    if (isPowerBtnHeld) {
      activeReceiverSlot = (activeReceiverSlot == 1) ? 2 : 1;
      preferences.putInt("active_slot", activeReceiverSlot);
      loadReceiverForSlot(activeReceiverSlot);
    }
    isPowerBtnHeld = false;
  }

  lastPowerBtnPressed = (pBtn == LOW);

  // Light Button (Lights / Pair)
  if (lBtn == LOW && !lastLightsBtnPressed) {
    isLightsBtnHeld = true;
    lightsLongPressHandled = false;
    btnLightsPressTime = currentMillis;
  }

  if (lBtn == LOW && isLightsBtnHeld && !lightsLongPressHandled &&
      (currentMillis - btnLightsPressTime >= holdTime)) {
    enterPairingMode(currentMillis);
    lightsLongPressHandled = true;
  }

  if (lBtn == HIGH && lastLightsBtnPressed) {
    if (isLightsBtnHeld && !lightsLongPressHandled) {
      lightsAreOn = !lightsAreOn;
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

      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        currentState = PAIRED;
        lastVehicleHeartbeatTime = millis();  // short grace period until first HB arrives
      } else {
        currentState = IDLE;
        lastVehicleHeartbeatTime = 0;
      }
    }
  }

  if (currentState == PAIRING && (currentMillis - pairingStartTime >= pairingTimeout)) {
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

    esp_now_send(receiverMAC, reinterpret_cast<uint8_t *>(&controlData), sizeof(controlData));
    lastSendTime = currentMillis;
  }

  writeStatusLED();
}