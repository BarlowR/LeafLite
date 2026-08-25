#pragma once

// ============================================================
// Pin map — Leaf 3.2.3 defaults, lifted from upstream:
//   hardware/configuration.h, Leaf_I2C.h, storage/sd_card.cpp,
//   hardware/lc86g.cpp
// If you're on a different board rev, fix these first.
// ============================================================

// I2C (MS5611 baro @ 0x77)
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

// Piezo speaker (LEDC tone) + 2-bit volume select
#define PIN_SPEAKER 14
#define PIN_SPKR_VOLA 15
#define PIN_SPKR_VOLB 16

// 5-way joystick (directly on GPIO for 3.2.3; active state verified on bench)
#define PIN_BTN_CENTER 2
#define PIN_BTN_LEFT 3
#define PIN_BTN_DOWN 4
#define PIN_BTN_UP 5
#define PIN_BTN_RIGHT 6

// LCD: JLX19296 (ST75256) on hardware SPI (pins from upstream Leaf_SPI.h/display.cpp)
#define PIN_SPI_MOSI 11
#define PIN_SPI_CLK 12
#define PIN_SPI_MISO 13
#define PIN_LCD_CS 10
#define PIN_LCD_DC 17
#define PIN_LCD_RESET 18
#define PIN_LCD_BACKLIGHT 21

// GPS: LC86G on UART0 (GPIO 43/44, the default Serial0 pins)
#define GPS_BAUD 115200
#define PIN_GPS_BACKUP_EN 40  // keep high: hot-start ephemeris retention
#define PIN_GPS_RESET 45

// SD card via SD_MMC (1-bit SDIO)
#define PIN_SDIO_CMD 35
#define PIN_SDIO_CLK 36
#define PIN_SDIO_D0 37
#define PIN_SD_DETECT 26

// Battery charger (semantics from upstream power.cpp):
// I1/I2 are OUTPUTS setting input current: LL=100mA, HL=500mA, LH=~1.35A,
// HH=USB SUSPEND (charging OFF) - never drive both high by accident.
// CHG_GOOD is input, ACTIVE LOW = charging. BATT_SENSE: ADC, mV * 69/41.
#define PIN_CHG_I1 41
#define PIN_CHG_I2 42
#define PIN_CHG_GOOD 47
#define PIN_BATT_SENSE 1
// Soft power latch (upstream power.cpp POWER_LATCH): on battery the 3.3V rail
// is only enabled while the power button is held — drive HIGH at boot to keep
// ourselves alive, LOW to power off. On USB the rail is up regardless.
#define PIN_POWER_LATCH 48

// ============================================================
// Vario tunables — start here when adjusting "feel"
// ============================================================
#define BARO_HZ 20  // sample loop rate; MS5611 OSR4096 needs ~9ms/conversion
// Tone response (frequency/cadence vs climb) lives in tones.h as an
// XCTracer-style 12-point table, overridable at boot by /tones.txt on SD.
