/*
 * Pin mapping (XIAO nRF52840, plain):
 *   D8  -> DWM1000 SCK   (fixed HW SPI pin)
 *   D9  -> DWM1000 MISO  (fixed HW SPI pin)
 *   D10 -> DWM1000 MOSI  (fixed HW SPI pin)
 *   D0  -> DWM1000 CSN
 *   D1  -> DWM1000 RSTn
 *   D2  -> DWM1000 IRQ
 *   DWM1000 VCC <- dedicated external 3.3V regulator
 *   GND common across XIAO, DWM1000, regulator, PSU

 */

#include <SPI.h>
#include <DW1000.h>

// ---- SET PER BOARD BEFORE FLASHING (1-4) ----
#define ANCHOR_ID 1

// connection pins (XIAO nRF52840)
const uint8_t PIN_RST = 1; // D1 - reset
const uint8_t PIN_IRQ = 2; // D2 - irq
const uint8_t PIN_SS  = 0; // D0 - spi select (CSN)

// Antenna delay calibration
#define ANTENNA_DELAY 16384

// messages used in the ranging protocol
enum class MessageType : uint8_t {
  POLL = 0,
  POLL_ACK = 1,
  RANGE = 2,
  RANGE_REPORT = 3,
  RANGE_FAILED = 255
};
// message flow state
volatile MessageType expectedMsgId = MessageType::POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
// last computed range/time
DW1000Time timeComputedRange;
// data buffer: [0] type, [1..15] timestamps/range, [16] target anchor ID, [17] tag sweep counter
#define LEN_DATA 18
#define ID_BYTE 16
#define SWEEP_BYTE 17
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

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
  while (!Serial && millis() < 3000); // don't hang forever if no USB serial monitor is attached
  Serial.print(F("### DW1000-4anchor-anchor, ID "));
  Serial.print(ANCHOR_ID);
  Serial.println(F(" ###"));

  DW1000.begin(PIN_IRQ, PIN_RST);
  pinMode(PIN_IRQ, INPUT_PULLDOWN);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));

  noInterrupts();
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setAntennaDelay(ANTENNA_DELAY); // must be set before commitConfiguration()
  DW1000.setDeviceAddress(ANCHOR_ID);
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
  // anchor starts in receiving mode, awaiting a ranging poll message
  receiver();
  noteActivity();
  setColor(false, false, false); // off = idle, waiting for a poll
}

void loop() {
  int32_t curMillis = millis();
  if (!sentAck && !receivedAck) {
    if (curMillis - lastActivity > resetPeriod) {
      resetInactive();
    }
    return;
  }
  if (sentAck) {
    sentAck = false;
    MessageType msgId = static_cast<MessageType>(data[0]);
    if (msgId == MessageType::POLL_ACK) {
      DW1000.getTransmitTimestamp(timePollAckSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    DW1000.getData(data, LEN_DATA);
    if (data[ID_BYTE] != ANCHOR_ID) {
      return; // addressed to another anchor, stay silent
    }
    MessageType msgId = static_cast<MessageType>(data[0]);
    if (msgId != expectedMsgId) {
      protocolFailed = true;
    }
    if (msgId == MessageType::POLL) {
      protocolFailed = false;
      DW1000.getReceiveTimestamp(timePollReceived);
      expectedMsgId = MessageType::RANGE;
      transmitPollAck();
      noteActivity();
    }
    else if (msgId == MessageType::RANGE) {
      DW1000.getReceiveTimestamp(timeRangeReceived);
      expectedMsgId = MessageType::POLL;
      if (!protocolFailed) {
        timePollSent.setTimestamp(data + 1);
        timePollAckReceived.setTimestamp(data + 6);
        timeRangeSent.setTimestamp(data + 11);
        computeRangeAsymmetric();
        float distance = timeComputedRange.getAsMeters();
        byte sweep = data[SWEEP_BYTE]; // read before transmitRangeReport() reuses data[]
        transmitRangeReport(distance);
        setColor(false, true, false); // green flash = range computed
        Serial.print("R,"); Serial.print(sweep);
        Serial.print(','); Serial.print(ANCHOR_ID);
        Serial.print(','); Serial.print(distance, 3);
        Serial.print(','); Serial.println(DW1000.getReceivePower(), 1);
      }
      else {
        setColor(true, false, false); // red = protocol failure
        transmitRangeFailed();
      }
      noteActivity();
    }
  }
}


void noteActivity() {
  lastActivity = millis();
}

void resetInactive() {
  expectedMsgId = MessageType::POLL;
  receiver();
  noteActivity();
}

void handleSent() {
  sentAck = true;
}

void handleReceived() {
  receivedAck = true;
}

void transmitPollAck() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = static_cast<byte>(MessageType::POLL_ACK);
  data[ID_BYTE] = ANCHOR_ID;
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRangeReport(float curRangeMeters) {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = static_cast<byte>(MessageType::RANGE_REPORT);
  memcpy(data + 1, &curRangeMeters, 4);
  data[ID_BYTE] = ANCHOR_ID;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRangeFailed() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = static_cast<byte>(MessageType::RANGE_FAILED);
  data[ID_BYTE] = ANCHOR_ID;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void computeRangeAsymmetric() {
  DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
  DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
  DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
  DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
  DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
  timeComputedRange.setTimestamp(tof);
}
