/*
 * Pin mapping (XIAO nRF52840, plain):
 *   D8  -> DWM1000 SCK   (fixed HW SPI pin)
 *   D9  -> DWM1000 MISO  (fixed HW SPI pin)
 *   D10 -> DWM1000 MOSI  (fixed HW SPI pin)
 *   D0  -> DWM1000 CSN
 *   D1  -> DWM1000 RSTn
 *   D2  -> DWM1000 IRQ
 *   D4  -> PCA9685 SDA   (HW I2C)
 *   D5  -> PCA9685 SCL   (HW I2C)
 *   PCA9685 CH0..CH4 -> AO3400A gates -> PN-VM102 motors
 *   DWM1000 VCC <- dedicated external 3.3V regulator
 *   GND common across XIAO, DWM1000, PCA9685, regulator, PSU

 */

#include <SPI.h>
#include <DW1000.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_TinyUSB.h>

// connection pins (XIAO nRF52840)
const uint8_t PIN_RST = 1; // D1 - reset
const uint8_t PIN_IRQ = 2; // D2 - irq
const uint8_t PIN_SS  = 0; // D0 - spi select (CSN)

// Antenna delay calibration
#define ANTENNA_DELAY 16384

// room geometry (m), origin at a floor corner, z up; anchor heights are placeholders
#define ANCHOR_COUNT 4
#define ANCHOR_LOW_Z  0.5f
#define ANCHOR_HIGH_Z 2.5f
const float ANCHOR_POS[ANCHOR_COUNT][3] = {
  {0.0f, 0.0f, ANCHOR_LOW_Z},  // anchor 1: wall corner, low
  {3.0f, 0.0f, ANCHOR_HIGH_Z}, // anchor 2: wall corner, high
  {0.0f, 3.0f, ANCHOR_HIGH_Z}, // anchor 3: wall corner, high
  {3.0f, 3.0f, ANCHOR_LOW_Z}   // anchor 4: wall corner, low
};

// ranges outside [0, MAX_VALID_RANGE_M] count as failed
#define MAX_VALID_RANGE_M 6.0f

// wait per anchor before skipping it
#define TAG_TIMEOUT_MS 50

// messages used in the ranging protocol
enum class MessageType : uint8_t {
  POLL = 0,
  POLL_ACK = 1,
  RANGE = 2,
  RANGE_REPORT = 3,
  RANGE_FAILED = 255
};
// message flow state
volatile MessageType expectedMsgId = MessageType::POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer: [0] type, [1..15] timestamps/range, [16] target anchor ID, [17] tag sweep counter
#define LEN_DATA 18
#define ID_BYTE 16
#define SWEEP_BYTE 17
byte data[LEN_DATA];
uint8_t sweepId = 0; // 0..255, wraps
// watchdog
uint32_t lastActivity;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

// sweep state
uint8_t currentAnchor = 0;          // index into ANCHOR_POS of the anchor being polled
float ranges[ANCHOR_COUNT];         // meters, valid where the matching bit of validMask is set
uint8_t validMask = 0;

// motor setup =============================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // A0-A5 all tied to GND

const uint16_t DUTY_MAX = 4095;  // PCA9685 12-bit
const uint16_t DUTY_STEP = 16;
const unsigned long STEP_INTERVAL_MS = 10;

const int RAMP_UP = 0, HOLD_FULL = 1, RAMP_DOWN = 2, PAUSE = 3;

struct Motor {
  int channel;
  unsigned long holdMs, pauseMs, startDelayMs;  // <- tweak per-motor rhythm here

  // runtime state, left at zero-init:
  int state;
  uint16_t duty;
  unsigned long lastStepTime, stateEnteredTime, startAt;
  bool started;
  bool wokeUp;  // set true the instant this motor leaves silence and starts ramping up
  uint16_t sentDuty;  // last duty written to the PCA9685, unchanged channels are skipped
};

const int MOTOR_COUNT = 5;

// channel, holdMs, pauseMs, startDelayMs — placeholders, tune per motor once wired
Motor motors[MOTOR_COUNT] = {
  { 0, 500, 22480, 0 },
  { 1, 500, 22480, 5620 },
  { 2, 500, 22480, 11240 },
  { 3, 500, 22480, 16860 },
  { 4, 500, 22480, 22480 },
};
const char *MOTOR_NAMES[MOTOR_COUNT] = { "motor0", "motor1", "motor2", "motor3", "motor4" };
// =============================================================

void setColor(bool r, bool g, bool b) {
  // XIAO nRF52840 onboard RGB LED is active-LOW (common anode)
  digitalWrite(LED_RED,   r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE,  b ? LOW : HIGH);
}

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  setColor(false, false, true); // blue = booting

  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) delay(10); // don't hang forever if no USB serial monitor is attached
  Serial.println(F("### DW1000-4anchor-tag + motors ###"));

  // motors first
  Wire.begin();

  // I2C check: confirm PCA9685 actually responds at 0x40 
  Wire.beginTransmission(0x40);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("PCA9685 found at 0x40");
  } else {
    Serial.print("PCA9685 NOT found — I2C error code: ");
    Serial.println(error);
  }

  pwm.begin();
  pwm.setPWMFreq(1000);  // 1kHz — within PCA9685's range, avoids the low-frequency whine
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pwm.setPWM(motors[i].channel, 0, 0);  // define every channel at 0% duty before anything else runs
  }

  DW1000.begin(PIN_IRQ, PIN_RST);
  pinMode(PIN_IRQ, INPUT_PULLDOWN);
  DW1000.select(PIN_SS);
  Serial.println("DW1000 initialized ...");

  noInterrupts();
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setAntennaDelay(ANTENNA_DELAY); // must be set before commitConfiguration()
  DW1000.setDeviceAddress(5);            // anchors use 1..4
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  interrupts();

  Serial.println(F("Committed configuration ..."));
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);

  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);

  // motor stagger counts from here, after the UWB setup time
  unsigned long now = millis();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors[i].startAt = now + motors[i].startDelayMs;
  }

  // tag starts by transmitting a POLL message to anchor 1
  receiver();
  transmitPoll();
  noteActivity();
  setColor(false, false, false); // off = idle, polling
}

void loop() {
  updateMotors(millis());
  serviceUwb();
}


void stepMotor(Motor &m, unsigned long now) {
  if (!m.started) {
    if (now >= m.startAt) {
      m.started = true;
      m.stateEnteredTime = now;
      m.lastStepTime = now;
      m.wokeUp = true;  
    } else {
      return;  // still waiting for this motor's staggered start
    }
  }

  switch (m.state) {
    case RAMP_UP:
      if (now - m.lastStepTime >= STEP_INTERVAL_MS) {
        m.lastStepTime = now;
        m.duty = (m.duty + DUTY_STEP >= DUTY_MAX) ? DUTY_MAX : m.duty + DUTY_STEP;
        if (m.duty >= DUTY_MAX) {
          m.state = HOLD_FULL;
          m.stateEnteredTime = now;
        }
      }
      break;

    case HOLD_FULL:
      if (now - m.stateEnteredTime >= m.holdMs) m.state = RAMP_DOWN;
      break;

    case RAMP_DOWN:
      if (now - m.lastStepTime >= STEP_INTERVAL_MS) {
        m.lastStepTime = now;
        m.duty = (m.duty <= DUTY_STEP) ? 0 : m.duty - DUTY_STEP;
        if (m.duty == 0) {
          m.state = PAUSE;
          m.stateEnteredTime = now;
        }
      }
      break;

    case PAUSE:
      if (now - m.stateEnteredTime >= m.pauseMs) {
        m.state = RAMP_UP;
        m.stateEnteredTime = now;
        m.wokeUp = true; 
      }
      break;
  }
}

void updateMotors(unsigned long now) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    stepMotor(motors[i], now);
    if (motors[i].duty != motors[i].sentDuty) {
      pwm.setPWM(motors[i].channel, 0, motors[i].duty);
      motors[i].sentDuty = motors[i].duty;
    }

    // Print only the instant a motor leaves silence and starts ramping up —
    // nothing at HOLD_FULL, RAMP_DOWN, or entering PAUSE.
    if (motors[i].wokeUp) {
      Serial.println(MOTOR_NAMES[i]);
      motors[i].wokeUp = false;
    }
  }
}

void serviceUwb() {
  if (!sentAck && !receivedAck) {
    if (millis() - lastActivity > TAG_TIMEOUT_MS) {
      finishAnchor(false, 0.0f); // no answer from this anchor, skip it
    }
    return;
  }
  if (sentAck) {
    sentAck = false;
    MessageType msgId = static_cast<MessageType>(data[0]);
    if (msgId == MessageType::POLL) {
      DW1000.getTransmitTimestamp(timePollSent);
    } else if (msgId == MessageType::RANGE) {
      DW1000.getTransmitTimestamp(timeRangeSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    DW1000.getData(data, LEN_DATA);
    if (data[ID_BYTE] != currentAnchor + 1) {
      return; // not from the anchor being polled
    }
    MessageType msgId = static_cast<MessageType>(data[0]);
    if (msgId != expectedMsgId) {
      finishAnchor(false, 0.0f);
      return;
    }
    if (msgId == MessageType::POLL_ACK) {
      DW1000.getReceiveTimestamp(timePollAckReceived);
      expectedMsgId = MessageType::RANGE_REPORT;
      transmitRange();
      noteActivity();
    } else if (msgId == MessageType::RANGE_REPORT) {
      float curRange;
      memcpy(&curRange, data + 1, 4);
      finishAnchor(true, curRange);
    } else if (msgId == MessageType::RANGE_FAILED) {
      finishAnchor(false, 0.0f);
    }
  }
}

void noteActivity() {
  lastActivity = millis();
}

void handleSent() {
  sentAck = true;
}

void handleReceived() {
  receivedAck = true;
}

void transmitPoll() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = static_cast<byte>(MessageType::POLL);
  data[ID_BYTE] = currentAnchor + 1;
  data[SWEEP_BYTE] = sweepId;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRange() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = static_cast<byte>(MessageType::RANGE);
  data[ID_BYTE] = currentAnchor + 1;
  data[SWEEP_BYTE] = sweepId;
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  timePollSent.getTimestamp(data + 1);
  timePollAckReceived.getTimestamp(data + 6);
  timeRangeSent.getTimestamp(data + 11);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

// solve position from the four ranges (linearized against anchor 1, Cramer's rule)
bool solvePosition(const float r[ANCHOR_COUNT], float p[3]) {
  float A[3][3];
  float b[3];
  const float* a0 = ANCHOR_POS[0];
  float a0sq = a0[0] * a0[0] + a0[1] * a0[1] + a0[2] * a0[2];
  for (int i = 1; i < ANCHOR_COUNT; i++) {
    const float* ai = ANCHOR_POS[i];
    float aisq = ai[0] * ai[0] + ai[1] * ai[1] + ai[2] * ai[2];
    for (int k = 0; k < 3; k++) {
      A[i - 1][k] = 2.0f * (ai[k] - a0[k]);
    }
    b[i - 1] = aisq - a0sq - r[i] * r[i] + r[0] * r[0];
  }
  float det = det3(A);
  if (fabsf(det) < 1e-6f) {
    return false;
  }
  for (int col = 0; col < 3; col++) {
    float M[3][3];
    for (int row = 0; row < 3; row++) {
      for (int k = 0; k < 3; k++) {
        M[row][k] = (k == col) ? b[row] : A[row][k];
      }
    }
    p[col] = det3(M) / det;
  }
  return true;
}

float det3(float m[3][3]) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// RMS gap between measured ranges and the solved position, near 0 = ranges agree
float residualRms(const float r[ANCHOR_COUNT], const float p[3]) {
  float sum = 0;
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    float dx = p[0] - ANCHOR_POS[i][0];
    float dy = p[1] - ANCHOR_POS[i][1];
    float dz = p[2] - ANCHOR_POS[i][2];
    float e = sqrtf(dx * dx + dy * dy + dz * dz) - r[i];
    sum += e * e;
  }
  return sqrtf(sum / ANCHOR_COUNT);
}

void reportSweep() {
  const uint8_t allValid = (1 << ANCHOR_COUNT) - 1;
  if (validMask != allValid) {
    setColor(true, false, false); // red = at least one anchor missing this sweep
    Serial.print("Sweep incomplete, anchors answered:");
    for (int i = 0; i < ANCHOR_COUNT; i++) {
      if (validMask & (1 << i)) {
        Serial.print(' '); Serial.print(i + 1);
      }
    }
    Serial.println();
    return;
  }
  float p[3];
  if (!solvePosition(ranges, p)) {
    setColor(true, false, false);
    Serial.println(F("Anchor geometry is singular, cannot solve position"));
    return;
  }
  setColor(false, true, false); // green flash = position solved
  Serial.print("Pos x,y,z (m): ");
  Serial.print(p[0], 3); Serial.print(", ");
  Serial.print(p[1], 3); Serial.print(", ");
  Serial.print(p[2], 3);
  Serial.print("\t Ranges:");
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    Serial.print(' '); Serial.print(ranges[i], 3);
  }
  Serial.print("\t Residual RMS (m): ");
  Serial.println(residualRms(ranges, p), 3);
}

// record this anchor's result, move to the next; report after the last one
void finishAnchor(bool ok, float meters) {
  if (ok && meters >= 0.0f && meters <= MAX_VALID_RANGE_M) {
    ranges[currentAnchor] = meters;
    validMask |= (1 << currentAnchor);
  }
  currentAnchor++;
  if (currentAnchor >= ANCHOR_COUNT) {
    currentAnchor = 0;
    reportSweep();
    validMask = 0;
    sweepId++; // next sweep gets a new number
  }
  expectedMsgId = MessageType::POLL_ACK;
  transmitPoll();
  noteActivity();
}
