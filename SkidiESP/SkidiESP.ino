#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <driver/mcpwm.h>
#include <ESP32_MCPWM.h>

// ======================================================
// Pin definitions
// ======================================================
#define bucketServoPin 23
#define auxServoPin    22

#define lightPin1      18
#define lightPin2      5

#define rightMotor0    25   // motor input A
#define rightMotor1    26   // motor input B

#define leftMotor0     33   // motor input A
#define leftMotor1     32   // motor input B

#define armMotor0      21
#define armMotor1      19

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ======================================================
// Shared packet definition
// ======================================================
struct __attribute__((packed)) ControlPacket
{
  int16_t x;      // -1000 to +1000
  int16_t y;      // -1000 to +1000
  uint8_t sw1_A;
  uint8_t sw1_B;
  uint8_t sw2_A;
  uint8_t sw2_B;
  uint8_t sw3_A;
  uint8_t sw3_B;
  uint8_t lightsOn;
};

// ======================================================
// 🔄 INVERSION / POLARITY TUNABLES (Change 1 to -1 to flip)
// ======================================================
const int INVERT_LEFT_MOTOR  = -1;
const int INVERT_RIGHT_MOTOR = -1;
const int INVERT_ARM_MOTOR   = 1;
const int INVERT_BUCKET      = -1;
const int INVERT_AUX         = -1;

// ======================================================
// Tunables
// ======================================================
const uint32_t CONTROL_PERIOD_US     = 20000;  // 50 Hz
const uint32_t SERVO_STEP_PERIOD_US  = 20000;  // 50 Hz servo stepping
const unsigned long DEBUG_PERIOD_MS  = 500;
const unsigned long FAILSAFE_MS      = 150;
const unsigned long PAIR_WINDOW_MS   = 4000;
const unsigned long PAIR_INTERVAL_MS = 250;

const unsigned long HEARTBEAT_INTERVAL_MS      = 250;
const unsigned long HEARTBEAT_ACTIVE_WINDOW_MS = 500;

// Joystick thresholds
const int STOP_DEADZONE   = 80;
const int DRIVE_DEADZONE  = 140;
const int STEER_DEADZONE  = 180;
const int HARD_TURN_ZONE  = 650;

// Servo tuning
const int SERVO_MIN_ANGLE_B    = 5;
const int SERVO_MAX_ANGLE_B    = 150;
const int SERVO_MIN_ANGLE_AUX  = 45;
const int SERVO_MAX_ANGLE_AUX  = 180;
const int SERVO_STEP           = 4;

// Drive tuning
const int MOTOR_INPUT_MAX        = 1000;
const int MOTOR_OUTPUT_DEADZONE  = 35;
const int STEER_GAIN_PERCENT     = 80;

// Ramp / smoothing
const int MOTOR_SLEW_UP_STEP     = 80;  // per control tick
const int MOTOR_SLEW_DOWN_STEP   = 50;  // per control tick

// MCPWM / library tuning
const int MOTOR_PWM_FREQ_HZ       = 20000;
const bool USE_CENTER_ALIGNED_PWM = true;

// ======================================================
// Globals
// ======================================================
Servo bucketServo;
Servo auxServo;

Motor leftDriveMotor;
Motor rightDriveMotor;

portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE peerMux    = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE statusMux  = portMUX_INITIALIZER_UNLOCKED;

ControlPacket controlData = {0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile unsigned long packetCount = 0;
volatile unsigned long lastPacketTime = 0;
volatile bool hasValidPacket = false;

// Communication status / deferred debug work
volatile bool rxPulseRequested = false;
volatile bool wrongRxSizeFlag = false;
volatile int wrongRxSizeLen = 0;

// Controller peer tracking
uint8_t controllerMAC[6] = {0};
uint8_t pendingControllerMAC[6] = {0};
bool controllerPeerKnown = false;
bool pendingControllerMACReady = false;
unsigned long lastHeartbeatSend = 0;

// Servo state
int bucketServoValue = 140;
int auxServoValue = 150;

// Drive state
int leftTargetCmd = 0;
int rightTargetCmd = 0;
int leftCurrentCmd = 0;
int rightCurrentCmd = 0;

// Debug/status
bool debugLedEnabled = true;
unsigned long ledPulseUntil = 0;
unsigned long lastDebugPrint = 0;

// Pairing
const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool pairingWindowActive = true;
unsigned long pairingWindowStart = 0;
unsigned long lastPairSend = 0;

// Scheduler
uint32_t nextControlTickUs = 0;
uint32_t nextServoTickUs = 0;

// ======================================================
// Helpers
// ======================================================
int applyDeadzone(int value, int deadzone)
{
  if (abs(value) < deadzone) return 0;
  return value;
}

int slewToward(int current, int target, int upStep, int downStep)
{
  int maxStep = (abs(target) > abs(current)) ? upStep : downStep;
  int delta = target - current;
  delta = constrain(delta, -maxStep, maxStep);
  return current + delta;
}

void printReceiverMac()
{
  uint8_t mac[6];
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK)
  {
    Serial.printf(
      "Receiver STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
  }
  else
  {
    Serial.println("Receiver STA MAC: unavailable");
  }
}

void pulseDebugLed()
{
  if (!debugLedEnabled) return;
  digitalWrite(LED_BUILTIN, HIGH);
  ledPulseUntil = millis() + 40;
}

void updateDebugLed()
{
  if (!debugLedEnabled) return;

  if (ledPulseUntil != 0 && millis() >= ledPulseUntil)
  {
    digitalWrite(LED_BUILTIN, LOW);
    ledPulseUntil = 0;
  }
}

void processDeferredCommStatus()
{
  bool doPulse = false;
  bool printWrongSize = false;
  int wrongSize = 0;

  portENTER_CRITICAL(&statusMux);
  if (rxPulseRequested)
  {
    doPulse = true;
    rxPulseRequested = false;
  }

  if (wrongRxSizeFlag)
  {
    printWrongSize = true;
    wrongSize = wrongRxSizeLen;
    wrongRxSizeFlag = false;
  }
  portEXIT_CRITICAL(&statusMux);

  if (doPulse)
  {
    pulseDebugLed();
  }

  if (printWrongSize)
  {
    Serial.printf("RX wrong size: got %d expected %u\n", wrongSize, (unsigned)sizeof(ControlPacket));
  }
}

void setLights(bool on)
{
  if (on)
  {
    digitalWrite(lightPin1, HIGH);
    digitalWrite(lightPin2, LOW);
  }
  else
  {
    digitalWrite(lightPin1, LOW);
    digitalWrite(lightPin2, LOW);
  }
}

void writeBucketServo()
{
  bucketServo.write(bucketServoValue);
}

void writeAuxServo()
{
  auxServo.write(auxServoValue);
}

bool macEquals(const uint8_t *a, const uint8_t *b)
{
  return memcmp(a, b, 6) == 0;
}

// ======================================================
// ESP-NOW
// ======================================================
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len)
{
  if (len == (int)sizeof(ControlPacket))
  {
    portENTER_CRITICAL(&controlMux);
    memcpy(&controlData, incomingData, sizeof(ControlPacket));
    lastPacketTime = millis();
    hasValidPacket = true;
    packetCount++;
    portEXIT_CRITICAL(&controlMux);

    portENTER_CRITICAL(&peerMux);
    memcpy(pendingControllerMAC, info->src_addr, sizeof(pendingControllerMAC));
    pendingControllerMACReady = true;
    portEXIT_CRITICAL(&peerMux);

    portENTER_CRITICAL(&statusMux);
    rxPulseRequested = true;
    portEXIT_CRITICAL(&statusMux);
  }
  else
  {
    portENTER_CRITICAL(&statusMux);
    wrongRxSizeLen = len;
    wrongRxSizeFlag = true;
    portEXIT_CRITICAL(&statusMux);
  }
}

void sendPairPacket()
{
  const uint8_t pairPayload[] = {'P', 'A', 'I', 'R'};
  esp_now_send(broadcastAddress, pairPayload, sizeof(pairPayload));
}

void updateStartupPairing()
{
  if (!pairingWindowActive) return;

  unsigned long now = millis();

  if (now - pairingWindowStart >= PAIR_WINDOW_MS)
  {
    pairingWindowActive = false;
    return;
  }

  if (now - lastPairSend >= PAIR_INTERVAL_MS)
  {
    sendPairPacket();
    lastPairSend = now;
  }
}

void processPendingControllerPeer()
{
  uint8_t newMAC[6] = {0};
  bool hasPending = false;

  portENTER_CRITICAL(&peerMux);
  if (pendingControllerMACReady)
  {
    memcpy(newMAC, pendingControllerMAC, sizeof(newMAC));
    pendingControllerMACReady = false;
    hasPending = true;
  }
  portEXIT_CRITICAL(&peerMux);

  if (!hasPending)
  {
    return;
  }

  if (controllerPeerKnown && macEquals(controllerMAC, newMAC))
  {
    return;
  }

  if (controllerPeerKnown && esp_now_is_peer_exist(controllerMAC))
  {
    esp_now_del_peer(controllerMAC);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, newMAC, sizeof(newMAC));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK)
  {
    memcpy(controllerMAC, newMAC, sizeof(controllerMAC));
    controllerPeerKnown = true;

    Serial.printf(
      "Controller peer set to %02X:%02X:%02X:%02X:%02X:%02X\n",
      controllerMAC[0], controllerMAC[1], controllerMAC[2],
      controllerMAC[3], controllerMAC[4], controllerMAC[5]
    );
  }
  else
  {
    controllerPeerKnown = false;
    memset(controllerMAC, 0, sizeof(controllerMAC));
    Serial.println("Controller peer add failed");
  }
}

void sendHeartbeatIfDue()
{
  if (!controllerPeerKnown)
  {
    return;
  }

  const unsigned long now = millis();
  if (now - lastHeartbeatSend < HEARTBEAT_INTERVAL_MS)
  {
    return;
  }

  unsigned long localLastPacketTime;
  bool localHasValidPacket;

  portENTER_CRITICAL(&controlMux);
  localLastPacketTime = lastPacketTime;
  localHasValidPacket = hasValidPacket;
  portEXIT_CRITICAL(&controlMux);

  if (!localHasValidPacket)
  {
    return;
  }

  if (now - localLastPacketTime > HEARTBEAT_ACTIVE_WINDOW_MS)
  {
    return;
  }

  const uint8_t hbPayload[] = {'H', 'B'};
  esp_now_send(controllerMAC, hbPayload, sizeof(hbPayload));
  lastHeartbeatSend = now;
}

// ======================================================
// Motor backend
// ======================================================
void applySignedMotorCommand(Motor &motor, int signedCmd, int polarity)
{
  int cmd = constrain(signedCmd * polarity, -MOTOR_INPUT_MAX, MOTOR_INPUT_MAX);

  if (abs(cmd) <= MOTOR_OUTPUT_DEADZONE)
  {
    motor.setSpeed(0, Dir::CW);
    return;
  }

  if (cmd > 0)
  {
    motor.setSpeed(cmd, Dir::CW);
  }
  else
  {
    motor.setSpeed(-cmd, Dir::CCW);
  }
}

void stopDriveMotors()
{
  leftTargetCmd = 0;
  rightTargetCmd = 0;
  leftCurrentCmd = 0;
  rightCurrentCmd = 0;

  leftDriveMotor.setSpeed(0, Dir::CW);
  rightDriveMotor.setSpeed(0, Dir::CW);
}

void setArmMotor(int direction)
{
  direction = direction * INVERT_ARM_MOTOR;

  if (direction > 0)
  {
    digitalWrite(armMotor0, LOW);
    digitalWrite(armMotor1, HIGH);
  }
  else if (direction < 0)
  {
    digitalWrite(armMotor0, HIGH);
    digitalWrite(armMotor1, LOW);
  }
  else
  {
    digitalWrite(armMotor0, LOW);
    digitalWrite(armMotor1, LOW);
  }
}

void stopArmMotor()
{
  setArmMotor(0);
}

// ======================================================
// Control logic
// ======================================================
void computeDriveTargets(int x, int y, int &leftOut, int &rightOut)
{
  int throttle = applyDeadzone(constrain(y, -1000, 1000), DRIVE_DEADZONE);
  int steering = applyDeadzone(constrain(x, -1000, 1000), STEER_DEADZONE);

  steering = (steering * STEER_GAIN_PERCENT) / 100;

  int leftCmd  = throttle + steering;
  int rightCmd = throttle - steering;

  leftCmd = constrain(leftCmd, -1000, 1000);
  rightCmd = constrain(rightCmd, -1000, 1000);

  if (abs(x) < STOP_DEADZONE && abs(y) < STOP_DEADZONE)
  {
    leftCmd = 0;
    rightCmd = 0;
  }

  if (abs(throttle) < DRIVE_DEADZONE && abs(x) > HARD_TURN_ZONE)
  {
    int pivot = map(abs(x), HARD_TURN_ZONE, 1000, 650, 1000);
    pivot = constrain(pivot, 0, 1000);

    if (x > 0)
    {
      leftCmd  = pivot;
      rightCmd = -pivot;
    }
    else
    {
      leftCmd  = -pivot;
      rightCmd = pivot;
    }
  }

  leftOut = leftCmd;
  rightOut = rightCmd;
}

void updateDriveFromJoystick(int x, int y)
{
  computeDriveTargets(x, y, leftTargetCmd, rightTargetCmd);

  leftCurrentCmd = slewToward(leftCurrentCmd, leftTargetCmd, MOTOR_SLEW_UP_STEP, MOTOR_SLEW_DOWN_STEP);
  rightCurrentCmd = slewToward(rightCurrentCmd, rightTargetCmd, MOTOR_SLEW_UP_STEP, MOTOR_SLEW_DOWN_STEP);

  applySignedMotorCommand(leftDriveMotor, leftCurrentCmd, INVERT_LEFT_MOTOR);
  applySignedMotorCommand(rightDriveMotor, rightCurrentCmd, INVERT_RIGHT_MOTOR);
}

void updateArmFromSwitches(const ControlPacket &data)
{
  if (data.sw1_A && !data.sw1_B)
  {
    setArmMotor(1);
  }
  else if (data.sw1_B && !data.sw1_A)
  {
    setArmMotor(-1);
  }
  else
  {
    stopArmMotor();
  }
}

void updateServosFromSwitches(const ControlPacket &data)
{
  int bucketMove = 0;
  if (data.sw2_A && !data.sw2_B) bucketMove = 1;
  else if (data.sw2_B && !data.sw2_A) bucketMove = -1;

  if (bucketMove != 0)
  {
    bucketServoValue += (bucketMove * SERVO_STEP * INVERT_BUCKET);
    bucketServoValue = constrain(bucketServoValue, SERVO_MIN_ANGLE_B, SERVO_MAX_ANGLE_B);
    writeBucketServo();
  }

  int auxMove = 0;
  if (data.sw3_A && !data.sw3_B) auxMove = 1;
  else if (data.sw3_B && !data.sw3_A) auxMove = -1;

  if (auxMove != 0)
  {
    auxServoValue += (auxMove * SERVO_STEP * INVERT_AUX);
    auxServoValue = constrain(auxServoValue, SERVO_MIN_ANGLE_AUX, SERVO_MAX_ANGLE_AUX);
    writeAuxServo();
  }
}

void applyFailsafe()
{
  stopDriveMotors();
  stopArmMotor();
  // Hold current servo positions
  // Leave lights in last commanded state
}

void readControlSnapshot(ControlPacket &localData,
                         unsigned long &localLastPacketTime,
                         bool &localHasValidPacket)
{
  portENTER_CRITICAL(&controlMux);
  memcpy(&localData, &controlData, sizeof(ControlPacket));
  localLastPacketTime = lastPacketTime;
  localHasValidPacket = hasValidPacket;
  portEXIT_CRITICAL(&controlMux);
}

void updateControl()
{
  ControlPacket localData;
  unsigned long localLastPacketTime;
  bool localHasValidPacket;

  readControlSnapshot(localData, localLastPacketTime, localHasValidPacket);

  if (!localHasValidPacket || (millis() - localLastPacketTime > FAILSAFE_MS))
  {
    applyFailsafe();
    return;
  }

  setLights(localData.lightsOn);
  updateDriveFromJoystick(localData.x, localData.y);
  updateArmFromSwitches(localData);
}

void updateServoControl()
{
  ControlPacket localData;
  unsigned long localLastPacketTime;
  bool localHasValidPacket;

  readControlSnapshot(localData, localLastPacketTime, localHasValidPacket);

  if (!localHasValidPacket || (millis() - localLastPacketTime > FAILSAFE_MS))
  {
    return;
  }

  updateServosFromSwitches(localData);
}

// ======================================================
// Debug
// ======================================================
void printDebugStatus()
{
  if (millis() - lastDebugPrint < DEBUG_PERIOD_MS) return;
  lastDebugPrint = millis();

  ControlPacket localData;
  unsigned long localLastPacketTime;
  bool localHasValidPacket;
  unsigned long localPacketCount;

  portENTER_CRITICAL(&controlMux);
  memcpy(&localData, &controlData, sizeof(ControlPacket));
  localLastPacketTime = lastPacketTime;
  localHasValidPacket = hasValidPacket;
  localPacketCount = packetCount;
  portEXIT_CRITICAL(&controlMux);

  Serial.printf(
    "Packets=%lu | valid=%d | age=%lu ms | x=%d y=%d | LT=%d RT=%d | LC=%d RC=%d | sw1(%u,%u) sw2(%u,%u) sw3(%u,%u) | light=%u | bucket=%d aux=%d | peer=%d\n",
    localPacketCount,
    localHasValidPacket,
    millis() - localLastPacketTime,
    localData.x,
    localData.y,
    leftTargetCmd,
    rightTargetCmd,
    leftCurrentCmd,
    rightCurrentCmd,
    localData.sw1_A, localData.sw1_B,
    localData.sw2_A, localData.sw2_B,
    localData.sw3_A, localData.sw3_B,
    localData.lightsOn,
    bucketServoValue,
    auxServoValue,
    controllerPeerKnown
  );
}

// ======================================================
// Setup helpers
// ======================================================
void setupPins()
{
  pinMode(armMotor0, OUTPUT);
  pinMode(armMotor1, OUTPUT);

  pinMode(lightPin1, OUTPUT);
  pinMode(lightPin2, OUTPUT);

  stopArmMotor();
  setLights(false);
}

void setupServos()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  bucketServo.setPeriodHertz(50);
  auxServo.setPeriodHertz(50);

  bucketServo.attach(bucketServoPin, 500, 2400);
  auxServo.attach(auxServoPin, 500, 2400);

  writeBucketServo();
  writeAuxServo();
}

void setupDriveMotors()
{
  MotorMCPWMConfig rightHw{
    rightMotor0,
    rightMotor1,
    -1,
    MCPWM_UNIT_0,
    MCPWM_TIMER_0,
    MCPWM0A,
    MCPWM0B
  };

  MotorMCPWMConfig leftHw{
    leftMotor0,
    leftMotor1,
    -1,
    MCPWM_UNIT_1,
    MCPWM_TIMER_0,
    MCPWM0A,
    MCPWM0B
  };

  rightHw.pwm_freq_hz = MOTOR_PWM_FREQ_HZ;
  leftHw.pwm_freq_hz  = MOTOR_PWM_FREQ_HZ;

  rightHw.input_max = MOTOR_INPUT_MAX;
  leftHw.input_max  = MOTOR_INPUT_MAX;

  rightHw.counter = USE_CENTER_ALIGNED_PWM ? MCPWM_UP_DOWN_COUNTER : MCPWM_UP_COUNTER;
  leftHw.counter  = USE_CENTER_ALIGNED_PWM ? MCPWM_UP_DOWN_COUNTER : MCPWM_UP_COUNTER;

  rightHw.use_deadtime = false;
  leftHw.use_deadtime  = false;

  MotorBehaviorConfig behavior{
    FreewheelMode::HiZ_Awake,
    60,    // soft_brake_hz
    0,     // dither_pwm
    0,     // default_soft
    600,   // min_phase_us
    true   // dither_coast_hi_z
  };

  leftDriveMotor.setup(leftHw, behavior);
  rightDriveMotor.setup(rightHw, behavior);

  stopDriveMotors();
}

bool setupEspNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  printReceiverMac();
  Serial.printf("ControlPacket size = %u bytes\n", (unsigned)sizeof(ControlPacket));
  Serial.printf("MOTOR_PWM_FREQ_HZ = %d\n", MOTOR_PWM_FREQ_HZ);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return false;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist((uint8_t *)broadcastAddress))
  {
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Broadcast peer add failed");
      return false;
    }
  }

  return true;
}

// ======================================================
// Setup / Loop
// ======================================================
void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  setupPins();
  setupServos();
  setupDriveMotors();

  if (!setupEspNow())
  {
    Serial.println("Receiver setup halted");
    while (true)
    {
      updateDebugLed();
    }
  }

  pairingWindowStart = millis();
  lastPairSend = 0;
  pairingWindowActive = true;
  lastHeartbeatSend = 0;

  nextControlTickUs = micros() + CONTROL_PERIOD_US;
  nextServoTickUs   = micros() + SERVO_STEP_PERIOD_US;

  Serial.println("Receiver ready");
}

void loop()
{
  uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextControlTickUs) >= 0)
  {
    nextControlTickUs += CONTROL_PERIOD_US;
    updateControl();
  }

  if ((int32_t)(nowUs - nextServoTickUs) >= 0)
  {
    nextServoTickUs += SERVO_STEP_PERIOD_US;
    updateServoControl();
  }

  updateStartupPairing();
  processPendingControllerPeer();
  sendHeartbeatIfDue();
  processDeferredCommStatus();
  updateDebugLed();
  printDebugStatus();
}
