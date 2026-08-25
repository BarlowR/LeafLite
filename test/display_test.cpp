#include <cassert>
#include <cmath>
#include <cstdio>
#include "display_ui.h"

int failures = 0;
#define CHECK(c) do{ if(!(c)){ printf("FAIL %d: %s\n",__LINE__,#c); failures++; } }while(0)

int main() {
  // --- Averager: constant 2 m/s climb at 20Hz ---
  ClimbAverager a;
  for (int i = 0; i < 400; i++) a.push(1000.0f + i * 0.1f);  // 2 m/s for 20s
  CHECK(fabs(a.avg(5) - 2.0f) < 0.02f);
  CHECK(fabs(a.avg(20) - 2.0f) < 0.02f);
  // Climb stops: 5s window decays to ~0 well before the 20s window
  for (int i = 0; i < 100; i++) a.push(1039.9f);  // 5s of level
  CHECK(fabs(a.avg(5)) < 0.05f);
  CHECK(a.avg(20) > 1.0f);
  // Warmup: <1s data reads 0, partial buffer uses available span
  ClimbAverager b;
  for (int i = 0; i < 10; i++) b.push(i * 0.05f);
  CHECK(b.avg(5) == 0.0f);
  for (int i = 10; i < 40; i++) b.push(i * 0.05f);  // 2s at 1 m/s
  CHECK(fabs(b.avg(20) - 1.0f) < 0.05f);

  // --- Rendering: sanity + clipping + visual dump ---
  FrameBuffer fb;
  renderVarioPage(fb, 1.2f, -0.8f);
  int lit = 0;
  for (int i = 0; i < FrameBuffer::PAGES * FrameBuffer::W; i++)
    lit += __builtin_popcount(fb.px[i]);
  CHECK(lit > 500 && lit < 8000);
  renderVarioPage(fb, 99.0f, -99.0f);  // clamp path must not crash or clip
  renderVarioPage(fb, 1.2f, -0.8f);

  // Battery icon: present while charging, absent otherwise
  auto iconLit = [](FrameBuffer& f) {
    int n = 0;
    for (int y = 0; y < 20; y++)
      for (int x = 55; x < 96; x++) {
#if DISPLAY_ROT_CCW
        int nx = 192 - 1 - y, ny = x;
#else
        int nx = y, ny = 96 - 1 - x;
#endif
        if (f.px[(ny/8)*192 + nx] & (1 << (ny%8))) n++;
      }
    return n;
  };
  FrameBuffer fbc;
  renderVarioPage(fbc, 0.0f, 0.0f, false, 50);
  CHECK(iconLit(fbc) == 0);
  renderVarioPage(fbc, 0.0f, 0.0f, true, 50);
  int half = iconLit(fbc);
  CHECK(half > 40);
  renderVarioPage(fbc, 0.0f, 0.0f, true, 100);
  CHECK(iconLit(fbc) > half);         // fuller battery = more lit pixels
  renderVarioPage(fbc, 0.0f, 0.0f, true, 0);
  CHECK(iconLit(fbc) > 30 && iconLit(fbc) < half);  // outline+bolt remain at 0%

  // GPS icon: none < searching (armless antenna) < locked (antenna + arms)
  auto totalLit = [](FrameBuffer& f) {
    int n = 0;
    for (int i = 0; i < FrameBuffer::PAGES * FrameBuffer::W; i++)
      n += __builtin_popcount(f.px[i]);
    return n;
  };
  FrameBuffer fbg;
  renderVarioPage(fbg, 0.0f, 0.0f, false, 0, GPS_ICON_NONE);
  int noGps = totalLit(fbg);
  renderVarioPage(fbg, 0.0f, 0.0f, false, 0, GPS_ICON_SEARCHING);
  int searching = totalLit(fbg);
  renderVarioPage(fbg, 0.0f, 0.0f, false, 0, GPS_ICON_LOCKED);
  int locked = totalLit(fbg);
  CHECK(searching > noGps + 5);
  CHECK(locked > searching + 10);
  renderVarioPage(fbg, 0.0f, 0.0f, false, 0, GPS_ICON_LOCKED, true);
  CHECK(totalLit(fbg) > locked + 30);  // REC dot adds pixels

  // Battery page: renders, fuller battery = more pixels, charging adds bolt
  FrameBuffer fbb;
  renderBatteryPage(fbb, 100, false, 4200);
  int b100 = totalLit(fbb);
  CHECK(b100 > 300);
  renderBatteryPage(fbb, 5, false, 3400);
  int b5 = totalLit(fbb);
  CHECK(b5 < b100);
  renderBatteryPage(fbb, 5, true, 3400);
  CHECK(totalLit(fbb) > b5);

  renderVarioPage(fb, 1.2f, -0.8f, true, 65, GPS_ICON_LOCKED);  // dump WITH icons

  // ASCII dump of LOGICAL portrait space (2x3 px/char): verify readability
  auto lit_at = [&](int lx, int ly) {
#if DISPLAY_ROT_CCW
    int x = 192 - 1 - ly, y = lx;
#else
    int x = ly, y = 96 - 1 - lx;
#endif
    return (fb.px[(y/8)*192 + x] & (1 << (y%8))) != 0;
  };
  for (int y = 0; y < 192; y += 3) {
    for (int x = 0; x < 96; x += 2) putchar(lit_at(x, y) ? '#' : ' ');
    putchar('\n');
  }
  printf(failures ? "FAILURES\n" : "ALL DISPLAY TESTS PASSED\n");
  return failures;
}
