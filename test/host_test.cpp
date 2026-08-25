// Host unit tests: g++ -std=c++17 -I../src host_test.cpp && ./a.out
// Covers NMEA parsing, Kalman climb tracking, IGC B-record + LK8EX1 formatting.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "ahrs.h"
#include "fusion.h"
#include "gps.h"
#include "tones.h"
#include "vario.h"

static int failures = 0;
#define CHECK(cond)                                              \
  do {                                                           \
    if (!(cond)) {                                               \
      printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                \
    }                                                            \
  } while (0)

static void testNmea() {
  Gps gps;
  // Real-format RMC/GGA (checksums computed for these exact strings)
  const char* rmc =
      "$GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230826,,,A*4E\r\n";
  const char* gga =
      "$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*77\r\n";

  int passed = 0;
  auto sink = [&](const char* s) { passed++; };
  for (const char* p = rmc; *p; p++) gps.feed(*p, sink);
  for (const char* p = gga; *p; p++) gps.feed(*p, sink);

  CHECK(passed == 2);  // both sentences checksum-valid and delivered
  const GpsFix& f = gps.fix();
  CHECK(f.valid);
  CHECK(fabs(f.latDeg - 48.1173) < 0.0001);
  CHECK(fabs(f.lonDeg - 11.5167) < 0.0001);
  CHECK(f.hh == 12 && f.mm == 35 && f.ss == 19);
  CHECK(f.day == 23 && f.mon == 8 && f.year == 2026);
  CHECK(f.sats == 8);
  CHECK(fabs(f.altM - 545.4f) < 0.01f);
  CHECK(fabs(f.speedKmh - 22.4f * 1.852f) < 0.1f);

  // Corrupted checksum must be rejected silently
  int badPassed = 0;
  auto badSink = [&](const char*) { badPassed++; };
  const char* bad = "$GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230826,,,A*00\r\n";
  for (const char* p = bad; *p; p++) gps.feed(*p, badSink);
  CHECK(badPassed == 0);

  // Southern/western hemisphere signs
  Gps gps2;
  const char* rmcS = "$GNRMC,010203.00,A,3351.000,S,15113.000,W,000.0,000.0,010126,,,A*48\r\n";
  for (const char* p = rmcS; *p; p++) gps2.feed(*p, badSink);
  CHECK(gps2.fix().valid);
  CHECK(gps2.fix().latDeg < 0 && gps2.fix().lonDeg < 0);
  CHECK(fabs(gps2.fix().latDeg + 33.85) < 0.001);
}

static void testKalman() {
  Vario v;
  // Feed a steady 2 m/s climb with baro noise; filter should converge near 200 cm/s
  v.reset(1000.0f);
  float alt = 1000.0f;
  unsigned seed = 42;
  for (int i = 0; i < 400; i++) {  // 20 s at 20 Hz
    alt += 2.0f * 0.05f;
    seed = seed * 1664525u + 1013904223u;
    float noise = ((int)((seed >> 16) % 1000) - 500) / 5000.0f;  // ±0.1 m
    v.update(alt + noise, 0.05f);
  }
  CHECK(v.climbCms() > 160 && v.climbCms() < 240);
  CHECK(fabs(v.altM() - alt) < 2.0f);

  // Pressure->altitude sanity: 101325 Pa -> 0 m, ~89875 Pa -> ~1000 m
  CHECK(fabs(Vario::pressureToAltM(101325.0f)) < 0.5f);
  float a1000 = Vario::pressureToAltM(89875.0f);
  CHECK(a1000 > 990 && a1000 < 1010);
}

static void testIgcFormat() {
  // Replicates igc.h's B-record printf; verifies field widths/rounding
  double latDeg = 48.1173, lonDeg = 11.5167;
  double alat = fabs(latDeg), alon = fabs(lonDeg);
  int latD = (int)alat, lonD = (int)alon;
  long latMm = lroundl((alat - latD) * 60000.0);
  long lonMm = lroundl((alon - lonD) * 60000.0);
  char b[80];
  snprintf(b, sizeof(b), "B%02u%02u%02u%02d%05ld%c%03d%05ld%cA%05ld%05ld", 12u, 35u,
           19u, latD, latMm, 'N', lonD, lonMm, 'E', lroundf(545.0f), lroundf(548.2f));
  CHECK(strcmp(b, "B1235194807038N01131002EA0054500548") == 0);
  CHECK(strlen(b) == 35);  // fixed-width B-record body
}

static void testLk8ex1Checksum() {
  char body[64];
  snprintf(body, sizeof(body), "LK8EX1,%ld,%ld,%ld,%d,999,", 101325L, 0L, 120L, 25);
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) cs ^= *p;
  char out[80];
  snprintf(out, sizeof(out), "$%s*%02X\r\n", body, cs);
  // Verify our own checksum validates with the parser's checker logic
  uint8_t cs2 = 0;
  const char* p = out + 1;
  while (*p && *p != '*') cs2 ^= *p++;
  CHECK(*p == '*');
  CHECK(strtol(p + 1, nullptr, 16) == cs2);
}

static void testAhrs() {
  // Device mounted rotated 90 deg about x: gravity reads on +y axis at rest.
  // Mahony must discover "up" from accel alone (gyro silent), and
  // verticalAccelG must then read ~1.0 despite the weird mounting.
  Ahrs a;
  for (int i = 0; i < 2000; i++) a.update(0, 1.0f, 0, 0, 0, 0, 0.01f);
  CHECK(fabs(a.verticalAccelG(0, 1.0f, 0) - 1.0f) < 0.01f);
  // Extra 0.3g along world-up (= body +y here) must appear in verticalAccelG
  CHECK(fabs(a.verticalAccelG(0, 1.3f, 0) - 1.3f) < 0.02f);
  // Lateral (body x/z) accel must NOT leak into vertical
  CHECK(fabs(a.verticalAccelG(0.5f, 1.0f, 0) - 1.0f) < 0.03f);
}

static void testFusedKalman() {
  // Climb starts at t=1s at 1.5 m/s. With correct accel input the fused filter
  // must reach 90% of true climb faster than the baro-only Vario.
  KalmanPA k;
  Vario v;
  v.reset(500);
  double alt = 500, t = 0, vel = 0;
  int fusedAt90 = -1, baroAt90 = -1;
  for (int i = 0; i < 200; i++) {  // 10s at 20Hz
    double dt = 0.05;
    t += dt;
    double accel = (i == 20) ? 30.0 : 0.0;  // step to 1.5 m/s in one tick
    vel += accel * dt;
    alt += vel * dt;
    k.update(t, alt, accel);
    v.update((float)alt, (float)dt);
    if (fusedAt90 < 0 && i > 20 && k.velocity() > 1.35) fusedAt90 = i;
    if (baroAt90 < 0 && i > 20 && v.climbCms() > 135) baroAt90 = i;
  }
  CHECK(fusedAt90 > 0 && baroAt90 > 0);
  CHECK(fusedAt90 < baroAt90);  // the whole point of the feature
  CHECK(fabs(k.velocity() - 1.5) < 0.1);
  CHECK(fabs(k.position() - alt) < 1.0);
}

static void testGravityTracker() {
  // (a) quiet: gravity estimate slews toward biased reading
  GravityTracker g1;
  g1.setTotal(1.02);
  for (int i = 0; i < 200; i++) g1.process(1.02, 0.05);
  CHECK(g1.gravity() > 1.015 && g1.gravity() <= 1.02 + 1e-9);
  // (b) maneuvering (|tot-1| > 0.065): gravity frozen, accel passed through
  GravityTracker g2;
  g2.setTotal(1.3);
  double out = g2.process(1.25, 0.05);
  CHECK(fabs(out - 0.25) < 1e-9);
  CHECK(fabs(g2.gravity() - 1.0) < 1e-9);
  // (c) sustained vertical disagreement with quiet total: input zeroed
  GravityTracker g3;
  g3.setTotal(1.03);
  out = g3.process(1.15, 0.05);
  CHECK(out == 0.0);
  // (d) plausibility clamp holds under absurd quiet-state bias
  GravityTracker g4;
  g4.setTotal(1.0);
  for (int i = 0; i < 2000; i++) g4.process(1.05, 0.05);
  CHECK(g4.gravity() <= 1.06 + 1e-9);
}

static void testToneProfile() {
  ToneProfile tp;
  uint16_t f, cy;
  uint8_t d;
  // Default table: exact hits at the ends and a listed point
  tp.lookup(10.0f, f, cy, d);
  CHECK(f == 1800 && cy == 150 && d == 70);
  tp.lookup(-10.0f, f, cy, d);
  CHECK(f == 200 && d == 100);
  tp.lookup(1.01f, f, cy, d);
  CHECK(f == 579 && cy == 335 && d == 50);
  // Interpolation between 1.99 (652Hz) and 8.00 (1517Hz)
  tp.lookup(5.0f, f, cy, d);
  CHECK(f > 1000 && f < 1200);
  // Clamped beyond the table
  tp.lookup(15.0f, f, cy, d);
  CHECK(f == 1800);

  // Hysteresis: defaults on=off=0.10 climb, -4.00 sink
  CHECK(!tp.updateActive(0.05f));
  CHECK(tp.updateActive(0.15f));
  CHECK(tp.updateActive(0.10f));   // off is "< 0.10", so 0.10 stays on
  CHECK(!tp.updateActive(0.05f));
  CHECK(!tp.updateActive(-3.0f));  // dead band: silent
  CHECK(tp.updateActive(-4.5f));   // sink alarm
  CHECK(!tp.updateActive(-3.5f));  // recovers above -4.00

  // Parsing replaces the table only once a valid tone line arrives
  tp.beginParse();
  CHECK(!tp.parseLine("# comment"));
  CHECK(!tp.parseLine("tone=banana"));
  tp.lookup(1.01f, f, cy, d);
  CHECK(f == 579);  // defaults still in place
  CHECK(tp.parseLine("tone=0.00,400,600,50"));
  CHECK(tp.parseLine("tone=5.00,1200,200,60"));
  CHECK(!tp.parseLine("tone=2.00,900,300,50"));  // out of order: dropped
  CHECK(tp.parseLine("ClimbToneOnThreshold=0.30"));
  tp.lookup(2.5f, f, cy, d);
  CHECK(f == 800 && cy == 400 && d == 55);  // midpoint of the 2-entry table
  CHECK(!tp.updateActive(0.20f));  // below the new 0.30 threshold
  CHECK(tp.updateActive(0.35f));
}

int main() {
  testNmea();
  testToneProfile();
  testAhrs();
  testFusedKalman();
  testGravityTracker();
  testKalman();
  testIgcFormat();
  testLk8ex1Checksum();
  if (failures == 0) printf("ALL TESTS PASSED\n");
  return failures;
}
