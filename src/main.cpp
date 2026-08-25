// leaflite — minimal vario for Leaf 3.2.3 hardware
// baro + audio + GPS + IGC logging + BLE + minimal display. No WiFi, no FANET, no menus.
//
// Controls:
//   UP / DOWN      volume up/down (chirp confirms; silence = muted)
//   CENTER click   start/stop logging: IGC + raw sensor log (double-chirp =
//                  started, low = stopped; raw log works without a GPS fix)
// IGC logging auto-starts on the first GPS lock (once per boot; a manual stop
// vetoes it) and is closed cleanly by every shutdown path.
//   CENTER 3s hold power off (deep sleep; wake = CENTER press)
//   RIGHT (hold)   show battery page (percent + voltage) while held
//   LEFT 1s hold   reboot into USB drive mode (SD card as USB mass storage;
//                  exit via CENTER or ejecting the drive on the computer)
// Rising double-chirp also announces GPS lock acquisition.

#include <Arduino.h>
#include <USB.h>
#include <USBCDC.h>
#include <USBMSC.h>
#include <Wire.h>
#include <esp_attr.h>

// CDC_ON_BOOT=0 (see platformio.ini), so `Serial` would be UART0 — which the
// GPS owns. Route debug through our own TinyUSB CDC instance instead.
USBCDC USBSerial;
#define Serial USBSerial

#include "ahrs.h"
#include "ble.h"
#include "config.h"
#include "display_ui.h"
#include "fusion.h"
#include "gps.h"
#include "icm20948.h"
#include "igc.h"
#include "ms5611.h"
#include "power.h"
#include "rawlog.h"
#include "speaker.h"
#include "st75256.h"
#include "vario.h"

MS5611 baro;
Vario vario;          // baro-only fallback if IMU is absent/dead
Icm20948 imu;
Ahrs ahrs;
GravityTracker gravityTracker;
KalmanPA fusedKalman;
bool imuOk = false;
float latestAwzG = 1.0f;   // world-frame vertical accel (g), updated at ~100Hz
float latestTotG = 1.0f;   // |accel| magnitude (g)
Speaker speaker;
Gps gps;
IgcLogger igc;
RawLogger rawlog;
BleLink ble;

bool sdOk = false;
St75256 lcd;
USBMSC msc;  // used only in USB drive mode
RTC_NOINIT_ATTR uint32_t bootFlag;               // survives esp_restart()
static const uint32_t BOOT_FLAG_MSC = 0x53445543;  // "reboot into USB drive mode"
static volatile bool mscEjected = false;
PowerMon power;
FrameBuffer fb;
ClimbAverager climbAvg;

static void chirp(uint32_t f, uint16_t ms) {
  ledcWriteTone(PIN_SPEAKER, f);
  delay(ms);
  ledcWriteTone(PIN_SPEAKER, 0);
}

// ---- USB drive mode: expose the SD card as USB mass storage ----
// Entered by holding LEFT (1s while running -> reboot, or held at boot).
// The firmware never touches SD files in this mode, so the host owns the
// filesystem; any exit path is a clean reboot back into the vario.
static int32_t mscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  (void)offset;  // transfers are sector-aligned
  uint32_t sec = SD_MMC.sectorSize();
  for (uint32_t i = 0; i < bufsize / sec; i++)
    if (!SD_MMC.readRAW((uint8_t*)buffer + i * sec, lba + i)) return -1;
  return bufsize;
}
static int32_t mscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  (void)offset;
  uint32_t sec = SD_MMC.sectorSize();
  for (uint32_t i = 0; i < bufsize / sec; i++)
    if (!SD_MMC.writeRAW(buffer + i * sec, lba + i)) return -1;
  return bufsize;
}
static bool mscStartStop(uint8_t powerCondition, bool start, bool loadEject) {
  (void)powerCondition;
  if (loadEject && !start) mscEjected = true;  // host ejected the drive
  return true;
}

// There is no VBUS-sense pin on this board (CHG_GOOD only means "charging",
// which deasserts on a full battery), so "plugged in" is detected from the
// USB stack itself: STARTED fires when a host enumerates us.
static volatile bool usbConnected = false;
static void usbEventCb(void* arg, esp_event_base_t base, int32_t id, void* data) {
  (void)arg; (void)base; (void)data;
  if (id == ARDUINO_USB_STARTED_EVENT || id == ARDUINO_USB_RESUME_EVENT)
    usbConnected = true;
  else if (id == ARDUINO_USB_STOPPED_EVENT || id == ARDUINO_USB_SUSPEND_EVENT)
    usbConnected = false;
}

static void usbDriveMode() {  // never returns
  renderUsbPage(fb);
  lcd.flush(fb.px);
  if (!igc.mountSd()) {  // no card: sad chirp, back to normal boot
    chirp(350, 400);
    esp_restart();
  }
  msc.vendorID("LEAF");
  msc.productID("LEAFLITE SD");
  msc.productRevision("1.0");
  msc.onRead(mscRead);
  msc.onWrite(mscWrite);
  msc.onStartStop(mscStartStop);
  msc.mediaPresent(true);
  msc.isWritable(true);
  msc.begin(SD_MMC.numSectors(), SD_MMC.sectorSize());
  USB.onEvent(usbEventCb);
  USBSerial.begin();
  USB.begin();
  chirp(900, 80); chirp(1200, 80);
  unsigned long connectBy = millis() + 8000;
  while (true) {  // exit: CENTER press, host eject, or no/lost USB connection
    if (digitalRead(PIN_BTN_CENTER) == HIGH || mscEjected) {
      delay(100);
      esp_restart();
    }
    if (!usbConnected && millis() > connectBy) {
      chirp(350, 300);  // not plugged into a computer (or unplugged): back to vario
      esp_restart();
    }
    delay(20);
  }
}

// Optional XCTracer-format tone config from SD (see tones.h for the format).
// Absent/invalid file leaves the built-in profile untouched.
static void loadToneConfig() {
  File f = SD_MMC.open("/tones.txt");
  if (!f) return;
  speaker.tones().beginParse();
  char line[96];
  size_t n = 0;
  while (f.available()) {
    char c = f.read();
    if (c == '\n' || n >= sizeof(line) - 1) {
      line[n] = 0;
      n = 0;
      speaker.tones().parseLine(line);
    } else if (c != '\r') {
      line[n++] = c;
    }
  }
  if (n) { line[n] = 0; speaker.tones().parseLine(line); }
  f.close();
  Serial.printf("tones.txt: %d tone points active\n", speaker.tones().count());
}

// Shutdown melody, then cut the power latch. On battery, power dies at
// latchOff(); the deep-sleep tail only runs on USB (rail stays up), where
// a CENTER press wakes us back into setup().
static void powerOff() {
  igc.stop();
  rawlog.stop();
  lcd.powerOff();
  chirp(600, 100); chirp(450, 100); chirp(300, 200);
  power.latchOff();
  delay(100);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BTN_CENTER, 1);
  esp_deep_sleep_start();
}

void setup() {
  power.init();  // FIRST: drives the power latch high — on battery we die
                 // the moment the user releases the button otherwise
  bool mscBoot = (bootFlag == BOOT_FLAG_MSC);
  bootFlag = 0;

  // Buttons are active-HIGH with pulldowns (joystick common is 3.3V) —
  // matches upstream buttons.cpp: INPUT_PULLDOWN, pressed == HIGH.
  pinMode(PIN_BTN_CENTER, INPUT_PULLDOWN);
  pinMode(PIN_BTN_UP, INPUT_PULLDOWN);
  pinMode(PIN_BTN_DOWN, INPUT_PULLDOWN);
  pinMode(PIN_BTN_LEFT, INPUT_PULLDOWN);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLDOWN);

  speaker.init();
  lcd.init();
  // USB drive mode decides USB interfaces, so branch BEFORE USB.begin():
  // an MSC interface added after the stack starts never enumerates.
  if (mscBoot || digitalRead(PIN_BTN_LEFT) == HIGH) usbDriveMode();  // never returns

  Serial.begin(115200);  // TinyUSB CDC debug
  USB.onEvent(usbEventCb);  // tracks host enumeration (gates low-batt shutdown)
  USB.begin();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  if (!baro.init()) {
    // Baro is the whole point: fail loudly, don't pretend.
    while (true) { chirp(400, 150); delay(850); Serial.println("MS5611 init FAILED"); }
  }
  imuOk = imu.init();
  if (!imuOk) Serial.println("ICM-20948 init failed - falling back to baro-only vario");
  gps.init();
  sdOk = igc.mountSd();
  if (!sdOk) Serial.println("SD mount failed - logging unavailable");
  if (sdOk) loadToneConfig();
  ble.init();
  speaker.playStartup();
}

void loop() {
  static unsigned long lastTick = 0;
  static unsigned long centerDownAt = 0;
  static bool centerWasDown = false;
  static uint8_t bleDiv = 0;
  static bool igcAutoDone = false;  // auto-start used up (or manually vetoed)

  // --- Baro state machine runs as fast as possible; consumes ~9ms conversions
  baro.update();

  // --- IMU + attitude at ~100Hz: keep gravity direction current so vertical
  //     accel is mount-orientation-independent
  static unsigned long lastImuUs = 0;
  if (imuOk) {
    unsigned long nowUs = micros();
    if (nowUs - lastImuUs >= 10000) {
      float dtImu = lastImuUs ? (nowUs - lastImuUs) / 1e6f : 0.01f;
      lastImuUs = nowUs;
      float ax, ay, az, gx, gy, gz;
      if (imu.read(ax, ay, az, gx, gy, gz)) {
        ahrs.update(ax, ay, az, gx, gy, gz, dtImu);
        latestAwzG = ahrs.verticalAccelG(ax, ay, az);
        latestTotG = sqrtf(ax * ax + ay * ay + az * az);
        rawlog.logImu(millis(), ax, ay, az, gx, gy, gz);
      }
    }
  }

  if (baro.hasNewPressure()) {  // ~20Hz cadence set by sensor conversion time
    float altM = Vario::pressureToAltM(baro.pressurePa());
    unsigned long now = millis();
    float dt = lastTick ? (now - lastTick) / 1000.0f : 0.05f;
    lastTick = now;
    rawlog.logBaro(now, baro.pressurePa(), baro.temperatureC());
    vario.update(altM, dt);  // baro-only estimate always runs (fallback)

    int32_t climbCms = vario.climbCms();
    if (imuOk) {
      gravityTracker.setTotal(latestTotG);
      double accelMs2 = gravityTracker.process(latestAwzG, dt) * 9.80665;
      fusedKalman.update(now / 1000.0, altM, accelMs2);
      if (fusedKalman.initialized())
        climbCms = (int32_t)lround(fusedKalman.velocity() * 100.0);
    }
    speaker.update(climbCms);

    // Low-battery shutdown (upstream's 3.20V threshold): 10s sustained below
    // cutoff -> clean power-off instead of a brownout mid-log. Guards against
    // false trips: never while charging or enumerated on a USB host, and the
    // 10s window must be STABLE (<150mV spread) — a dying cell reads smoothly
    // low, while a missing/disconnected battery makes the charger retry-cycle
    // the sense line through huge sawtooth swings that must not power us off.
    static uint16_t lowBattTicks = 0;
    static uint32_t lowBattMin = 0, lowBattMax = 0;
    uint32_t battMv = power.filteredMV();
    if (!power.charging() && !usbConnected && battMv != 0 &&
        battMv < (uint32_t)PowerMon::BATT_SHUTDOWN_MV) {
      if (lowBattTicks == 0) { lowBattMin = battMv; lowBattMax = battMv; }
      if (battMv < lowBattMin) lowBattMin = battMv;
      if (battMv > lowBattMax) lowBattMax = battMv;
      if (++lowBattTicks >= BARO_HZ * 10) {
        if (lowBattMax - lowBattMin < 150) powerOff();
        lowBattTicks = 0;  // unstable reading: not a real battery — rearm
      }
    } else {
      lowBattTicks = 0;
    }

    // Averager consumes the best altitude estimate available at 20Hz
    climbAvg.push((imuOk && fusedKalman.initialized()) ? (float)fusedKalman.position()
                                                       : vario.altM());

    static uint8_t lcdDiv = 0;
    if (++lcdDiv >= 10) {  // 2Hz refresh: plenty for 5s/20s averages
      lcdDiv = 0;
      static uint8_t gpsBlink = 0;
      gpsBlink ^= 1;
      uint8_t gpsIcon = gps.fix().valid ? GPS_ICON_LOCKED
                        : (gpsBlink ? GPS_ICON_SEARCHING : GPS_ICON_NONE);
      if (digitalRead(PIN_BTN_RIGHT) == HIGH)  // battery page only while held
        renderBatteryPage(fb, power.batteryPercent(), power.charging(),
                          (int)power.batteryMV());
      else
        renderVarioPage(fb, climbAvg.avg(5), climbAvg.avg(20), power.charging(),
                        power.batteryPercent(), gpsIcon,
                        igc.active() || rawlog.active());
      lcd.flush(fb.px);
    }

    if (++bleDiv >= 10) {  // 2Hz LK8EX1
      bleDiv = 0;
      float bleAlt = (imuOk && fusedKalman.initialized()) ? (float)fusedKalman.position()
                                                          : vario.altM();
      ble.sendLk8ex1(baro.pressurePa(), bleAlt, climbCms, baro.temperatureC(),
                     power.batteryPercent());
    }
  }

  // --- GPS: drain UART, pass valid NMEA to BLE, log IGC
  gps.poll([](const char* nmea) { ble.sendSentence(nmea); });
  if (igc.active()) igc.update(gps.fix(), vario.altM());
  rawlog.logGps(millis(), gps.fix());

  // Lock-acquired chirp. 5s of lost-fix hysteresis so a flapping RMC valid
  // flag on marginal signal doesn't re-announce constantly.
  static bool gpsHadFix = false;
  static unsigned long gpsLostAt = 0;  // 0 = fix never lost (or never had)
  bool gpsFix = gps.fix().valid;
  if (gpsFix && !gpsHadFix && (gpsLostAt == 0 || millis() - gpsLostAt > 5000)) {
    chirp(1100, 70); chirp(1500, 90);
  }
  if (!gpsFix && gpsHadFix) gpsLostAt = millis();
  gpsHadFix = gpsFix;

  // Auto-start IGC on first GPS lock (needs the RMC date for the filename).
  // Fires once per boot; a manual CENTER stop suppresses it so the pilot's
  // explicit "stop logging" wins. Fix loss does NOT stop the log — the IGC
  // just has a gap, same as upstream. Shutdown paths close it via powerOff().
  if (!igcAutoDone && sdOk && !igc.active() && gpsFix && igc.start(gps.fix())) {
    igcAutoDone = true;
    chirp(1000, 80); delay(40); chirp(1300, 80);
  }

  // --- Buttons (polled; fine at this loop rate)
  bool centerDown = digitalRead(PIN_BTN_CENTER) == HIGH;
  if (centerDown && !centerWasDown) centerDownAt = millis();
  if (centerDown && millis() - centerDownAt > 3000) powerOff();
  if (!centerDown && centerWasDown && millis() - centerDownAt < 600) {  // click
    if (igc.active() || rawlog.active()) {
      igc.stop();
      rawlog.stop();
      igcAutoDone = true;  // manual stop: don't auto-restart on re-lock
      chirp(500, 200);
    } else if (sdOk) {
      // IGC needs a GPS date; the raw log doesn't, so bench captures work
      // without a fix (they get a /raw-NNN.csv name instead).
      bool igcOk = igc.start(gps.fix());
      bool rawOk = rawlog.start(gps.fix());
      if (igcOk || rawOk) { chirp(1000, 80); delay(40); chirp(1300, 80); }
      else chirp(350, 400);
    } else {
      chirp(350, 400);  // no SD card
    }
  }
  centerWasDown = centerDown;

  // LEFT held 1s: reboot into USB drive mode (SD card as USB mass storage)
  static unsigned long leftDownAt = 0;
  if (digitalRead(PIN_BTN_LEFT) != HIGH) {
    leftDownAt = 0;
  } else if (!leftDownAt) {
    leftDownAt = millis();
  } else if (millis() - leftDownAt > 1000) {
    igc.stop();
    rawlog.stop();
    bootFlag = BOOT_FLAG_MSC;
    chirp(900, 80); chirp(1200, 80);
    esp_restart();
  }

  static bool upWas = false, dnWas = false;
  bool up = digitalRead(PIN_BTN_UP) == HIGH;
  bool dn = digitalRead(PIN_BTN_DOWN) == HIGH;
  if (up && !upWas && speaker.volume() < 3) { speaker.setVolume(speaker.volume() + 1); chirp(1200, 60); }
  if (dn && !dnWas && speaker.volume() > 0) { speaker.setVolume(speaker.volume() - 1); if (speaker.volume()) chirp(900, 60); }
  upWas = up; dnWas = dn;

  delay(1);
}
