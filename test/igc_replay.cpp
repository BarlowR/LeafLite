#include <cmath>
#include <cstdio>
#include <cstring>
#include "fusion.h"
#include "vario.h"
int main(int argc, char** argv) {
  FILE* f = fopen(argv[1], "r");
  if (!f) return 1;
  char line[256];
  Vario v; KalmanPA k; GravityTracker g;
  bool first = true; double t = 0, minV = 99, maxV = -99, minA = 1e9, maxA = -1e9;
  long n = 0, nanCount = 0;
  int lastHH=-1,lastMM=-1,lastSS=-1;
  while (fgets(line, sizeof(line), f)) {
    if (line[0] != 'B' || strlen(line) < 35) continue;
    if (line[24] != 'A') continue;  // skip no-fix records
    int hh=(line[1]-'0')*10+line[2]-'0', mm=(line[3]-'0')*10+line[4]-'0', ss=(line[5]-'0')*10+line[6]-'0';
    char gbuf[6]={0}; memcpy(gbuf, line+30, 5);      // GPS alt field
    double alt = atof(gbuf);
    if (alt < 1.0) continue;  // GPS-alt dropout in phone logs
    double dt = 1.0;
    if (lastHH>=0) dt = (hh-lastHH)*3600+(mm-lastMM)*60+(ss-lastSS);
    lastHH=hh; lastMM=mm; lastSS=ss;
    if (dt <= 0) continue;
    t += dt;
    if (first) { v.reset(alt); first = false; }
    v.update((float)alt, (float)dt);
    g.setTotal(1.0);
    k.update(t, alt, g.process(1.0, dt) * 9.80665);
    double vel = v.climbCms()/100.0;
    if (std::isnan(vel) || std::isnan(k.velocity())) nanCount++;
    minV = std::fmin(minV, vel); maxV = std::fmax(maxV, vel);
    minA = std::fmin(minA, alt); maxA = std::fmax(maxA, alt);
    n++;
  }
  printf("%ld fixes, %.1f min, alt %.0f..%.0fm, baro-only vel %+.1f..%+.1f m/s, NaN=%ld, kalman final v=%.2f\n",
         n, t/60, minA, maxA, minV, maxV, nanCount, k.velocity());
  return 0;
}
