#pragma once
#include <Arduino.h>
#include <SPI.h>

#include "config.h"

// Bare-metal driver for the 3.2.3 board's JLX19296 LCD (ST75256, 192x96 mono,
// 4-wire SPI mode 0 @ 4MHz — u8g2's rated SCK for this panel; upstream runs it
// at 20MHz via setBusClock, so there is headroom if flush time ever matters). Init byte sequence, addressing offsets (pages
// 8..19, col 0), volume and power values are lifted verbatim from the vendored
// U8g2 source in upstream (u8x8_d_st75256.c, jlx19296 tables) — the init
// sequence is the entire risk in a bare-metal LCD driver, so it is stolen, not
// re-derived. Everything else here is a framebuffer flush.
class St75256 {
 public:
  static const int W = 192, H = 96, PAGES = 12;  // page = 8 rows, LSB = top row

  void init() {
    pinMode(PIN_LCD_CS, OUTPUT);
    digitalWrite(PIN_LCD_CS, HIGH);
    pinMode(PIN_LCD_DC, OUTPUT);
    pinMode(PIN_LCD_RESET, OUTPUT);
    pinMode(PIN_LCD_BACKLIGHT, OUTPUT);
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_LCD_CS);

    digitalWrite(PIN_LCD_RESET, LOW);
    delay(10);
    digitalWrite(PIN_LCD_RESET, HIGH);
    delay(20);

    // --- u8x8_d_st75256_jlx19296_init_seq, transliterated ---
    cmd(0x30);              // command set 00
    cmd(0x94);              // sleep out
    cmd(0xAE);              // display off
    cmd(0x31);              // command set 01
    cmd(0xD7); data(0x9F);  // disable auto read
    cmd(0x32); data(0x00); data(0x01); data(0x03);  // analog set: OSC, booster 6k, bias 1/11
    cmd(0x20);              // gray levels (16 args; harmless in mono mode)
    for (uint8_t g : {0x01,0x03,0x05,0x07,0x09,0x0B,0x0D,0x10,
                      0x11,0x13,0x15,0x17,0x19,0x1B,0x1D,0x1F}) data(g);
    cmd(0x30);
    cmd(0x75); data(0); data(0x4F);   // row range
    cmd(0x15); data(0); data(255);    // col range
    cmd(0xBC); data(0x00);  // scan direction (1 arg)
    cmd(0xA6);              // normal (non-inverted) display — a command, not an arg
    cmd(0x0C);              // data format: LSB top
    cmd(0xCA); data(0x00); data(0x9F); data(0x20);  // display control: 1/160 duty
    cmd(0xF0); data(0x10);  // monochrome mode
    cmd(0x81); data(0x2E); data(0x03);  // volume (contrast) — u8g2 default
    cmd(0x20); data(0x0B);  // power: regulator + follower + booster on
    delay(100);
    cmd(0xAF);              // display on
    digitalWrite(PIN_LCD_BACKLIGHT, HIGH);
  }

  // fb: page-major, PAGES*W bytes, bit0 = top row of page
  void flush(const uint8_t* fb) {
    cmd(0x30);
    cmd(0x75); data(8); data(8 + PAGES - 1);  // rows 8..19: u8g2 jlx19296 sends 8+y_pos (glass COMs start at RAM row 64)
    cmd(0x15); data(0); data(W - 1);          // col window 0..191: pointer must wrap at 192, not 256
    cmd(0x5C);  // write data
    digitalWrite(PIN_LCD_DC, HIGH);
    digitalWrite(PIN_LCD_CS, LOW);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    SPI.writeBytes(fb, PAGES * W);
    SPI.endTransaction();
    digitalWrite(PIN_LCD_CS, HIGH);
  }

  void setContrast(uint16_t v) {  // 9-bit VOP; default 0x03<<6|0x2E = 238
    cmd(0x30);
    cmd(0x81); data(v & 0x3F); data((v >> 6) & 0x07);
  }

  void powerOff() {
    cmd(0x30); cmd(0xAE); cmd(0x95);  // display off, sleep in
    digitalWrite(PIN_LCD_BACKLIGHT, LOW);
  }

 private:
  void write(uint8_t b, bool isData) {
    digitalWrite(PIN_LCD_DC, isData);
    digitalWrite(PIN_LCD_CS, LOW);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(b);
    SPI.endTransaction();
    digitalWrite(PIN_LCD_CS, HIGH);
  }
  void cmd(uint8_t b) { write(b, false); }
  void data(uint8_t b) { write(b, true); }
};
