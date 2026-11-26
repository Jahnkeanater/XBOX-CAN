// Heltec WiFi LoRa 32 V2 (ESP32): pairs with an Xbox controller over Bluetooth
// using Bluepad32 and streams its state over UART to the RP2040 CAN Feather.
// The on-board OLED shows connection status, battery, sticks, triggers and buttons.
//
// Board package: "esp32_bluepad32" (see README) -> board "ESP32 Dev Module".
// Libraries: Adafruit SSD1306, Adafruit GFX
//
// Wiring:  Heltec GPIO17 (TX)  ->  Feather RX (GPIO1)
//          Heltec GPIO13 (RX)  <-  Feather TX (GPIO0)   (optional, unused for now)
//          Heltec GND          ->  Feather GND

#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "link_protocol.h"

#define LINK_TX_PIN 17
#define LINK_RX_PIN 13
#define LED_PIN     25          // Heltec V2 white LED
#define SEND_PERIOD_MS 10       // 100 Hz

// Heltec V2 on-board OLED (SSD1306 128x64)
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16
#define VEXT_PIN    21          // LOW = Vext power on (feeds the OLED on some V2 revisions)
#define OLED_PERIOD_MS 66       // ~15 fps

Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RST);

ControllerPtr gCtl = nullptr;   // only the first controller is used
uint8_t gSeq = 0;

// Snapshot handed from loop() to the display task.
struct DisplayState {
  LinkPacket pkt;
  uint8_t battery;              // 0 = unknown, 1..255
  uint32_t connectedSince;      // millis() when the controller connected
};
DisplayState gDisp = {};
portMUX_TYPE gDispMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t gConnectedAt = 0;

void onConnectedController(ControllerPtr ctl) {
  if (gCtl == nullptr) {
    gCtl = ctl;
    gConnectedAt = millis();
    ControllerProperties p = ctl->getProperties();
    Serial.printf("Controller connected: %s VID=0x%04x PID=0x%04x\n",
                  ctl->getModelName().c_str(), p.vendor_id, p.product_id);
  } else {
    Serial.println("Extra controller ignored");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (ctl == gCtl) {
    gCtl = nullptr;
    Serial.println("Controller disconnected");
  }
}

void sendPacket() {
  LinkPacket pkt = {};
  pkt.magic0 = LINK_MAGIC0;
  pkt.magic1 = LINK_MAGIC1;
  pkt.seq = gSeq++;
  uint8_t battery = 0;

  // Note: isGamepad() stays false until the first input report arrives, and Xbox
  // controllers only send reports on change - so don't require it. Just reject
  // devices that identified as something else.
  if (gCtl && gCtl->isConnected() &&
      !gCtl->isMouse() && !gCtl->isKeyboard() && !gCtl->isBalanceBoard()) {
    pkt.flags    = LINK_FLAG_CONNECTED;
    pkt.lx       = gCtl->axisX();
    pkt.ly       = gCtl->axisY();
    pkt.rx       = gCtl->axisRX();
    pkt.ry       = gCtl->axisRY();
    pkt.brake    = gCtl->brake();
    pkt.throttle = gCtl->throttle();
    pkt.buttons  = gCtl->buttons();
    pkt.dpad     = gCtl->dpad();
    pkt.misc     = gCtl->miscButtons();
    battery      = gCtl->battery();
  }
  // When disconnected everything stays zero with flags=0 -> RP2040 goes to failsafe.

  pkt.crc = link_crc8((const uint8_t *)&pkt, sizeof(pkt) - 1);
  Serial2.write((const uint8_t *)&pkt, sizeof(pkt));

  portENTER_CRITICAL(&gDispMux);
  gDisp.pkt = pkt;
  gDisp.battery = battery;
  gDisp.connectedSince = gConnectedAt;
  portEXIT_CRITICAL(&gDispMux);
}

// ---------------------------------------------------------------- display ---

// Stick box: 30x30 outline with centre cross and a dot for the stick position.
void drawStick(int x, int y, int16_t ax, int16_t ay) {
  const int s = 30, c = s / 2;
  oled.drawRect(x, y, s, s, SSD1306_WHITE);
  oled.drawPixel(x + c, y + c, SSD1306_WHITE);
  int dx = constrain(ax, -512, 512) * (c - 3) / 512;
  int dy = constrain(ay, -512, 512) * (c - 3) / 512;
  oled.fillCircle(x + c + dx, y + c + dy, 2, SSD1306_WHITE);
}

// Vertical trigger bar, fills from the bottom (0..1023).
void drawTrigger(int x, int y, uint16_t v) {
  const int w = 5, h = 30;
  oled.drawRect(x, y, w, h, SSD1306_WHITE);
  int f = (int)min<uint16_t>(v, 1023) * (h - 2) / 1023;
  if (f > 0) oled.fillRect(x + 1, y + h - 1 - f, w - 2, f, SSD1306_WHITE);
}

void dot(int x, int y, int r, bool on) {
  if (on) oled.fillCircle(x, y, r, SSD1306_WHITE);
  else    oled.drawCircle(x, y, r, SSD1306_WHITE);
}

void box(int x, int y, int w, int h, bool on) {
  if (on) oled.fillRect(x, y, w, h, SSD1306_WHITE);
  else    oled.drawRect(x, y, w, h, SSD1306_WHITE);
}

int pct(int16_t v) { return constrain((int)v, -512, 512) * 100 / 512; }

void drawScreen(const DisplayState &d) {
  const LinkPacket &p = d.pkt;
  const bool connected = p.flags & LINK_FLAG_CONNECTED;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // ---- top bar ----
  oled.setCursor(0, 0);
  if (connected) {
    uint32_t up = (millis() - d.connectedSince) / 1000;
    oled.printf("XBOX %02lu:%02lu", (unsigned long)(up / 60) % 100, (unsigned long)up % 60);
  } else {
    oled.print("XBOX ---");
  }
  oled.setCursor(80, 0);
  if (connected && d.battery) oled.printf("BAT%3d%%", d.battery * 100 / 255);
  else                        oled.print("BAT --");
  oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  if (!connected) {
    // pairing screen
    oled.setTextSize(2);
    oled.setCursor(10, 18);
    oled.print("PAIRING");
    static const char spin[] = "|/-\\";
    oled.print(spin[(millis() / 200) % 4]);
    oled.setTextSize(1);
    oled.setCursor(4, 40);
    oled.print("Hold controller's");
    oled.setCursor(4, 50);
    oled.print("pair button.  CAN:FS");
    oled.display();
    return;
  }

  // ---- live view ----
  const int top = 13;
  drawTrigger(0, top, p.brake);
  drawStick(7, top, p.lx, p.ly);
  drawStick(91, top, p.rx, p.ry);
  drawTrigger(123, top, p.throttle);

  // shoulders
  box(41, top, 14, 5, p.buttons & 0x0010);       // LB
  box(73, top, 14, 5, p.buttons & 0x0020);       // RB

  // D-pad (centre 48,30)
  box(46, 22, 5, 5, p.dpad & 0x01);              // up
  box(46, 34, 5, 5, p.dpad & 0x02);              // down
  box(40, 28, 5, 5, p.dpad & 0x08);              // left
  box(52, 28, 5, 5, p.dpad & 0x04);              // right

  // face buttons (centre 80,30): Y top, A bottom, X left, B right
  dot(80, 24, 3, p.buttons & 0x0008);            // Y
  dot(80, 36, 3, p.buttons & 0x0001);            // A
  dot(74, 30, 3, p.buttons & 0x0004);            // X
  dot(86, 30, 3, p.buttons & 0x0002);            // B

  // view / xbox / menu
  dot(57, 41, 2, p.misc & 0x02);
  dot(64, 41, 3, p.misc & 0x01);
  dot(71, 41, 2, p.misc & 0x04);

  // stick click indicators under the stick boxes
  if (p.buttons & 0x0100) oled.fillRect(7, top + 31, 30, 2, SSD1306_WHITE);   // L3
  if (p.buttons & 0x0200) oled.fillRect(91, top + 31, 30, 2, SSD1306_WHITE);  // R3

  // ---- bottom: stick values in %, Y up = positive ----
  oled.setCursor(0, 56);
  oled.printf("L%4d%4d  R%4d%4d", pct(p.lx), -pct(p.ly), pct(p.rx), -pct(p.ry));
  oled.display();
}

// Runs on its own task so the slow I2C refresh never delays BP32 or UART sends.
void displayTask(void *) {
  DisplayState d;
  for (;;) {
    portENTER_CRITICAL(&gDispMux);
    d = gDisp;
    portEXIT_CRITICAL(&gDispMux);
    drawScreen(d);
    vTaskDelay(pdMS_TO_TICKS(OLED_PERIOD_MS));
  }
}

bool setupDisplay() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(50);
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found - continuing without display");
    return false;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(4, 16);
  oled.print("XBOX->CAN");
  oled.setTextSize(1);
  oled.setCursor(4, 40);
  oled.print("J1939 joystick bridge");
  oled.display();
  return true;
}

// ------------------------------------------------------------------ main ---

void setup() {
  Serial.begin(115200);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  pinMode(LED_PIN, OUTPUT);

  bool haveOled = setupDisplay();

  Serial.printf("Bluepad32 firmware: %s\n", BP32.firmwareVersion());
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  // Uncomment once if pairing gets stuck, then re-flash with it commented out:
  // BP32.forgetBluetoothKeys();
  Serial.println("Hold the pair button on the Xbox controller...");

  if (haveOled) {
    delay(800);   // leave the splash up briefly
    xTaskCreatePinnedToCore(displayTask, "oled", 4096, nullptr, 1, nullptr, 1);
  }
}

void loop() {
  static uint32_t lastSend = 0;
  BP32.update();   // pulls latest controller data into gCtl

  uint32_t now = millis();
  if (now - lastSend >= SEND_PERIOD_MS) {
    lastSend = now;
    sendPacket();
    digitalWrite(LED_PIN, gCtl != nullptr);   // LED on = controller connected
  }
  delay(1);        // let the Bluetooth/idle tasks run
}
