# Xbox controller → CAN bus

```
Xbox controller ──Bluetooth──> Heltec V2 (ESP32, Bluepad32) ──UART 115200──> RP2040 CAN Feather ──CAN──> your device
```

The Heltec does Bluetooth, the Feather does CAN. They talk over a 3-wire serial link.

## Controller compatibility
The Xbox One controller needs Bluetooth: model **1708**, **1914** (Series X|S) or Elite Series 2.
The original 2013 model **1537** has no Bluetooth and won't work here.
Update the controller firmware first (Xbox Accessories app on Windows or an Xbox) — Bluepad32 works best with the latest BLE firmware.

## Wiring
| Heltec V2 | RP2040 CAN Feather |
|-----------|--------------------|
| GPIO17 (TX) | RX (GPIO1) |
| GPIO13 (RX) | TX (GPIO0) — optional |
| GND | GND |

Both boards are 3.3 V logic, so no level shifter is needed. Power each from USB, or feed the Feather's USB pin from the Heltec 5 V pin.

CAN side: Feather terminal block **H → CAN-H**, **L → CAN-L**, **GND → device ground**.
Put a 120 Ω terminator at each end of the bus (the Feather has a solder jumper for its own terminator).

## Heltec V2 setup (Arduino IDE)
1. Preferences → Additional Boards Manager URLs, add:
   `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`
2. Boards Manager → install **esp32_bluepad32**.
3. Board: **esp32_bluepad32 → ESP32 Dev Module**. Open `heltec_controller/heltec_controller.ino` and upload.
4. Library Manager → install **Adafruit SSD1306** (it also installs Adafruit GFX).
5. Open the Serial Monitor at 115200, hold the controller's pair button until the Xbox logo flashes fast. The white LED lights when it's connected.

### OLED screen
- **Before a controller connects:** a `PAIRING` screen with a spinner. `CAN:FS` is a reminder that the CAN output is in failsafe.
- **Once connected, the top bar shows** time connected and controller battery %.
- **Middle, left to right:** LT bar, left stick box, the button group, right stick box, RT bar.
  - Button group: LB/RB on top, D-pad on the left, Y/X/B/A as a diamond on the right, View/Xbox/Menu below. Each button fills in when pressed.
  - A bar under a stick box means that stick is clicked (L3/R3).
- **Bottom row:** stick positions in % (Y up = positive), in the same direction as the J1939 output.

The screen redraws about 15 times a second on its own task, so it never delays the controller data.

## RP2040 CAN Feather setup
1. Add the board URL `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json` and install **Raspberry Pi Pico/RP2040**.
2. Library Manager → install **Adafruit MCP2515**.
3. Board: **Adafruit Feather RP2040 CAN**. Open `rp2040_can_bridge/rp2040_can_bridge.ino`, set `CAN_BAUDRATE`, upload.
4. The Serial Monitor prints the controller values every 500 ms. The red LED lights when controller data is live.

## Command line (arduino-cli)
arduino-cli is installed at `~/.local/bin/arduino-cli`, with both board packages and the library already set up. Replace the port with whatever `arduino-cli board list` shows.
```bash
arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32 heltec_controller
arduino-cli upload  --fqbn esp32-bluepad32:esp32:esp32 -p /dev/ttyUSB0 heltec_controller
arduino-cli compile --fqbn rp2040:rp2040:adafruit_feather_can rp2040_can_bridge
arduino-cli upload  --fqbn rp2040:rp2040:adafruit_feather_can -p /dev/ttyACM0 rp2040_can_bridge
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## CAN output: J1939, 250 kbit/s
Three Basic Joystick Messages (SAE J1939-71) go out every 20 ms, using 29-bit IDs with priority 3 and source address `0x80` (`J1939_SA`).

| Message | PGN | CAN ID | X axis | Y axis | Buttons 1–12 |
|---------|-----|--------|--------|--------|--------------|
| BJM1 | 0xFDD6 (64982) | `0x0CFDD680` | Left stick | Left stick | A, B, X, Y, LB, RB, L3, R3, View, Menu, Xbox, Share |
| BJM2 | 0xFDD8 (64984) | `0x0CFDD880` | Right stick | Right stick | D-up, D-down, D-left, D-right, LT pressed, RT pressed, 7–12 n/a |
| BJM3 | 0xFDDA (64986) | `0x0CFDDA80` | Left trigger (positive only) | Right trigger (positive only) | all n/a |

Each axis uses the standard BJM fields: neutral, negative and positive position status bits, plus a 10-bit position from 0 to 1000 (0.1 %/bit). The position is the distance from neutral, and the status bits give the direction.
- **Direction:** stick forward = Y positive, stick right = X positive.
- **Deadband:** 40 counts out of 512 on the sticks, 20 out of 1023 on the triggers. Positions are rescaled so 0–100 % starts at the edge of the deadband.
- **Button states:** 00 = released, 01 = pressed, 10 = error, 11 = not available.

**Failsafe:** if the controller disconnects or the UART link goes quiet for 200 ms, every axis status and button reports **error (10)** and every position is 0. For an independent second layer, also set a message timeout on these PGNs in IQANdesign.

**DBC file:** `xbox_can_j1939.dbc` describes all three messages: every status, position and button signal, the Off/On/Error/NotAvailable value table, a 20 ms cycle time and the J1939 attributes. Load it in PCAN-View/PCAN-Explorer, SavvyCAN, cantools or Vector tools to decode the bus by name.
Each axis is three 2-bit status signals plus a 0–100 % position. To get a signed value: if `*_Left`/`*_Back` is On, use −Position; otherwise use +Position.

The mapping lives in `buildAndSendCan()` in `rp2040_can_bridge.ino`.

`link_protocol.h` exists in `common/` and in each sketch folder (Arduino IDE needs a copy next to each sketch). If you change it, keep all copies identical.
