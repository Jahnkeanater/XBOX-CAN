// Shared UART packet format between the Heltec V2 (controller side) and the
// RP2040 CAN Feather (CAN side). Copy this file next to each sketch's .ino
// (Arduino IDE only compiles files inside the sketch folder).
#pragma once
#include <stdint.h>

#define LINK_BAUD    115200
#define LINK_MAGIC0  0xA5
#define LINK_MAGIC1  0x5A

#define LINK_FLAG_CONNECTED 0x01   // a controller is paired and sending data

// All multi-byte fields little-endian (both MCUs are little-endian).
typedef struct __attribute__((packed)) {
  uint8_t  magic0;     // LINK_MAGIC0
  uint8_t  magic1;     // LINK_MAGIC1
  uint8_t  seq;        // increments every packet
  uint8_t  flags;      // LINK_FLAG_*
  int16_t  lx, ly;     // left stick,  -511..512 (Y: up is negative)
  int16_t  rx, ry;     // right stick, -511..512
  uint16_t brake;      // left trigger,  0..1023
  uint16_t throttle;   // right trigger, 0..1023
  uint16_t buttons;    // Bluepad32 button bitmask (A=0x01 B=0x02 X=0x04 Y=0x08 LB=0x10 RB=0x20 ...)
  uint8_t  dpad;       // up=0x01 down=0x02 right=0x04 left=0x08
  uint8_t  misc;       // system/xbox=0x01 view=0x02 menu=0x04 share=0x08
  uint8_t  crc;        // CRC-8 over every byte before this one
} LinkPacket;

static inline uint8_t link_crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}
