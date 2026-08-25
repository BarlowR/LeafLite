#pragma once
#include <NimBLEDevice.h>

// BLE for phone apps (XCTrack, XC Guide, SeeYou Navigator, FlySkyHy).
// Standard recipe: Nordic UART Service (NUS) notifying NMEA-style sentences.
// We send $LK8EX1 (pressure/vario/battery) at 2Hz and pass GPS NMEA through
// verbatim so the app gets position without the phone's own GPS.
class BleLink {
 public:
  void init(const char* name = "leaflite") {
    NimBLEDevice::init(name);
    NimBLEServer* server = NimBLEDevice::createServer();
    NimBLEService* nus = server->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    tx_ = nus->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                                    NIMBLE_PROPERTY::NOTIFY);
    nus->start();
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(nus->getUUID());
    // NimBLE 2.x does NOT auto-include the name from NimBLEDevice::init() in
    // the advertisement (1.x did) — without this the device scans as unnamed.
    adv->setName(name);
    adv->enableScanResponse(true);
    adv->start();
  }

  void sendSentence(const char* s) {  // s must already include checksum + CRLF
    if (!tx_) return;
    tx_->setValue((const uint8_t*)s, strlen(s));
    tx_->notify();
  }

  // $LK8EX1,pressure(Pa),altitude(m),vario(cm/s),temp(C),battery,*CS
  // battery field: per LK8EX1 spec, 1000+percent means "percentage".
  void sendLk8ex1(float pressurePa, float altM, int32_t climbCms, float tempC,
                  int battPct) {
    char body[64];
    snprintf(body, sizeof(body), "LK8EX1,%ld,%ld,%ld,%d,%d,", lroundf(pressurePa),
             lroundf(altM), (long)climbCms, (int)lroundf(tempC), 1000 + battPct);
    uint8_t cs = 0;
    for (const char* p = body; *p; p++) cs ^= *p;
    char out[80];
    snprintf(out, sizeof(out), "$%s*%02X\r\n", body, cs);
    sendSentence(out);
  }

 private:
  NimBLECharacteristic* tx_ = nullptr;
};
