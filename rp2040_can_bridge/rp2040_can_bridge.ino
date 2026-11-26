// Adafruit RP2040 CAN Bus Feather (MCP25625): receives controller state from
// the Heltec over UART and sends it as J1939 joystick messages (BJM1-3) at 250 kbit/s.
//
// Board package: Earle Philhower "Raspberry Pi Pico/RP2040" -> board
//   "Adafruit Feather RP2040 CAN"
// Library: "Adafruit MCP2515" (Library Manager)
//
// The controller-to-J1939 mapping lives in buildAndSendCan().

#include <Adafruit_MCP2515.h>
#include "link_protocol.h"

#define CAN_BAUDRATE    250000  // J1939 standard
#define CAN_PERIOD_MS   20      // joystick messages every 20 ms
#define LINK_TIMEOUT_MS 200     // no valid UART packet for this long -> failsafe

// ---------- J1939 ----------
#define J1939_SA        0x80    // our source address - change to suit your network
#define J1939_PRIORITY  3
#define PGN_BJM1        0xFDD6  // 64982 Basic Joystick Message 1
#define PGN_BJM2        0xFDD8  // 64984 Basic Joystick Message 2
#define PGN_BJM3        0xFDDA  // 64986 Basic Joystick Message 3

#define STICK_DEADBAND  40      // raw counts (of ~512) treated as neutral
#define TRIGGER_DEADBAND 20     // raw counts (of 1023)
#define TRIGGER_BUTTON_THRESHOLD 512   // LT/RT count as "pressed" buttons above this

// 2-bit J1939 states
#define ST_OFF  0x0   // not active / not pressed
#define ST_ON   0x1   // active / pressed
#define ST_ERR  0x2   // error
#define ST_NA   0x3   // not available

// Bluepad32 bit masks
#define BTN_A   0x0001
#define BTN_B   0x0002
#define BTN_X   0x0004
#define BTN_Y   0x0008
#define BTN_LB  0x0010
#define BTN_RB  0x0020
#define BTN_L3  0x0100
#define BTN_R3  0x0200
#define DPAD_UP    0x01
#define DPAD_DOWN  0x02
#define DPAD_RIGHT 0x04
#define DPAD_LEFT  0x08
#define MISC_XBOX  0x01
#define MISC_VIEW  0x02
#define MISC_MENU  0x04
#define MISC_SHARE 0x08

Adafruit_MCP2515 mcp(PIN_CAN_CS);

LinkPacket gState = {};         // last good packet
uint32_t gLastRx = 0;
uint32_t gBadCrc = 0;

// ---------- UART receive (byte-wise state machine) ----------
void pollLink() {
  static uint8_t buf[sizeof(LinkPacket)];
  static uint8_t idx = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (idx == 0 && b != LINK_MAGIC0) continue;
    if (idx == 1 && b != LINK_MAGIC1) { idx = (b == LINK_MAGIC0) ? 1 : 0; continue; }
    buf[idx++] = b;
    if (idx == sizeof(LinkPacket)) {
      idx = 0;
      if (link_crc8(buf, sizeof(LinkPacket) - 1) == buf[sizeof(LinkPacket) - 1]) {
        memcpy(&gState, buf, sizeof(LinkPacket));
        gLastRx = millis();
      } else {
        gBadCrc++;
      }
    }
  }
}

// ---------- CAN helpers ----------
void sendJ1939(uint16_t pgn, const uint8_t *data) {
  uint32_t id = ((uint32_t)J1939_PRIORITY << 26) | ((uint32_t)pgn << 8) | J1939_SA;
  mcp.beginExtendedPacket(id);
  mcp.write(data, 8);
  mcp.endPacket();
}

// Encode one joystick axis into the 2-byte BJM layout:
//   byte0 bits1-2 neutral status, bits3-4 negative (left/back) status,
//         bits5-6 positive (right/forward) status, bits7-8 position LSBs
//   byte1 position MSBs.  Position = 0..1000 (0.1 %/bit) magnitude from neutral.
// `raw` is signed (negative = left/back); `span` is full-scale raw counts.
void encodeAxis(uint8_t *out, int32_t raw, int32_t span, int32_t deadband, bool ok) {
  uint8_t neutral, neg, pos;
  uint16_t mag = 0;
  if (!ok) {
    neutral = neg = pos = ST_ERR;
  } else {
    int32_t a = raw < 0 ? -raw : raw;
    if (a <= deadband) {
      neutral = ST_ON; neg = pos = ST_OFF;
    } else {
      mag = (uint16_t)min<int32_t>(1000, (a - deadband) * 1000 / (span - deadband));
      neutral = ST_OFF;
      neg = raw < 0 ? ST_ON : ST_OFF;
      pos = raw > 0 ? ST_ON : ST_OFF;
    }
  }
  out[0] = neutral | (neg << 2) | (pos << 4) | ((mag & 0x3) << 6);
  out[1] = mag >> 2;
}

// Pack 12 button states (2 bits each) into bytes 6-8 of a BJM.
// Per J1939: byte6 bits7-8 = button 1 ... bits1-2 = button 4; byte7 = 5-8; byte8 = 9-12.
void encodeButtons(uint8_t *out3, const uint8_t st[12]) {
  for (int i = 0; i < 3; i++) {
    out3[i] = (st[i*4] << 6) | (st[i*4+1] << 4) | (st[i*4+2] << 2) | st[i*4+3];
  }
}

static inline uint8_t bs(bool ok, bool pressed) { return ok ? (pressed ? ST_ON : ST_OFF) : ST_ERR; }

// Map controller state -> J1939 joystick messages. `ok` is false when the link
// timed out or the controller disconnected: every axis and button then reports
// the J1939 "error" state so the receiver can react.
//
//   BJM1 (0xFDD6): X/Y = left stick
//                  btn 1-12 = A B X Y LB RB L3 R3 View Menu Xbox Share
//   BJM2 (0xFDD8): X/Y = right stick
//                  btn 1-6 = D-up D-down D-left D-right LT-pressed RT-pressed
//   BJM3 (0xFDDA): X = left trigger (positive/right only), Y = right trigger (positive/forward only)
// Xbox Y axes are up-negative, so they are inverted to make stick-forward = positive.
void buildAndSendCan(const LinkPacket &s, bool ok) {
  uint8_t d[8];
  uint8_t st[12];

  // ---- BJM1: left stick + face/shoulder/system buttons ----
  encodeAxis(&d[0], s.lx, 512, STICK_DEADBAND, ok);
  encodeAxis(&d[2], -(int32_t)s.ly, 512, STICK_DEADBAND, ok);
  d[4] = 0xFF;                       // detent statuses / reserved: not available
  const uint16_t b = s.buttons;
  st[0]  = bs(ok, b & BTN_A);
  st[1]  = bs(ok, b & BTN_B);
  st[2]  = bs(ok, b & BTN_X);
  st[3]  = bs(ok, b & BTN_Y);
  st[4]  = bs(ok, b & BTN_LB);
  st[5]  = bs(ok, b & BTN_RB);
  st[6]  = bs(ok, b & BTN_L3);
  st[7]  = bs(ok, b & BTN_R3);
  st[8]  = bs(ok, s.misc & MISC_VIEW);
  st[9]  = bs(ok, s.misc & MISC_MENU);
  st[10] = bs(ok, s.misc & MISC_XBOX);
  st[11] = bs(ok, s.misc & MISC_SHARE);
  encodeButtons(&d[5], st);
  sendJ1939(PGN_BJM1, d);

  // ---- BJM2: right stick + D-pad + trigger-as-button ----
  encodeAxis(&d[0], s.rx, 512, STICK_DEADBAND, ok);
  encodeAxis(&d[2], -(int32_t)s.ry, 512, STICK_DEADBAND, ok);
  d[4] = 0xFF;
  st[0] = bs(ok, s.dpad & DPAD_UP);
  st[1] = bs(ok, s.dpad & DPAD_DOWN);
  st[2] = bs(ok, s.dpad & DPAD_LEFT);
  st[3] = bs(ok, s.dpad & DPAD_RIGHT);
  st[4] = bs(ok, s.brake    > TRIGGER_BUTTON_THRESHOLD);
  st[5] = bs(ok, s.throttle > TRIGGER_BUTTON_THRESHOLD);
  for (int i = 6; i < 12; i++) st[i] = ST_NA;
  encodeButtons(&d[5], st);
  sendJ1939(PGN_BJM2, d);

  // ---- BJM3: analog triggers ----
  encodeAxis(&d[0], s.brake,    1023, TRIGGER_DEADBAND, ok);
  encodeAxis(&d[2], s.throttle, 1023, TRIGGER_DEADBAND, ok);
  d[4] = 0xFF;
  for (int i = 0; i < 12; i++) st[i] = ST_NA;
  encodeButtons(&d[5], st);
  sendJ1939(PGN_BJM3, d);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(LINK_BAUD);     // Feather TX=GPIO0, RX=GPIO1

#ifdef PIN_CAN_STANDBY
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW);   // transceiver active
#endif
#ifdef PIN_CAN_RESET
  pinMode(PIN_CAN_RESET, OUTPUT);
  digitalWrite(PIN_CAN_RESET, HIGH);    // MCP25625 out of reset
#endif
  pinMode(LED_BUILTIN, OUTPUT);

  if (!mcp.begin(CAN_BAUDRATE)) {
    while (true) {                      // fast blink = CAN controller not found
      Serial.println("MCP25625 init failed");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }
  Serial.println("CAN bridge ready");
}

void loop() {
  static uint32_t lastCan = 0, lastPrint = 0;
  pollLink();

  uint32_t now = millis();
  bool linkOk = (now - gLastRx) < LINK_TIMEOUT_MS;
  bool alive  = linkOk && (gState.flags & LINK_FLAG_CONNECTED);

  if (now - lastCan >= CAN_PERIOD_MS) {
    lastCan = now;
    static const LinkPacket neutral = {};
    buildAndSendCan(alive ? gState : neutral, alive);   // failsafe = J1939 error states
    digitalWrite(LED_BUILTIN, alive);
  }

  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.printf("link=%d ctl=%d LX=%d LY=%d RX=%d RY=%d LT=%u RT=%u btn=0x%04x dpad=0x%02x badcrc=%lu\n",
                  linkOk, alive, gState.lx, gState.ly, gState.rx, gState.ry,
                  gState.brake, gState.throttle, gState.buttons, gState.dpad,
                  (unsigned long)gBadCrc);
  }
}
