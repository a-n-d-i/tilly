#ifdef TILLY_DISPLAY
#include "ST7305_U8g2.h"
#include "board_pins.h"
#include "applog.h"
#include <time.h>
#include <math.h>

// ----- SCREEN GEOMETRY (landscape, after U8G2_R1 rotation) -----
static const int DISP_W = RLCD_WIDTH;   // 400
static const int DISP_H = RLCD_HEIGHT;  // 300

struct TillyDisplayState {
  int sats = 0;
  bool ekfOk = false;
  bool magOk = false;
  bool armed = false;
  bool wifiOk = false;
  bool autoMode = false;
  bool nmeaMode = false;  // GUIDED steering driven by the OpenCPN APB bridge
  float speedKn = 0.0f;
  int curHeading = 0;
  int desHeading = 0;
  double lat = 0.0, lon = 0.0;
  float cogDeg = 0.0f;             // GPS course over ground, for the heading/COG deviation row
  uint64_t unixTimeSec = 0;        // from SYSTEM_TIME; 0 = not received yet
  float batteryVolts = -1.0f;      // from SYS_STATUS; -1 = not received yet
  float batteryAmps = -1.0f;       // from SYS_STATUS; -1 = not received yet / not measured
  float pidFF = 0, pidP = 0, pidI = 0, pidD = 0, pidSRate = 0;
};

static ST7305_U8g2 tillyLcd(RLCD_SCK_PIN, RLCD_MOSI_PIN, RLCD_DC_PIN, RLCD_CS_PIN, RLCD_RST_PIN);
static U8G2 *tillyU8g2 = nullptr;

// boxSize is sized to match the label font's height, so the checkbox reads
// as part of the same line as the text rather than a mismatched blob next
// to it. Checked draws a bold (3px-thick) black checkmark, not a filled
// square - each stroke is drawn 3 times, offset by 1px, so it reads as
// solid black instead of a thin/grey-looking single-pixel line.
// Returns the x position right after the checkbox, so callers can pack
// header items tightly one after another instead of using fixed columns.
static int drawCheckbox(int x, int y, int boxSize, bool checked, const char *label) {
  tillyU8g2->setFont(u8g2_font_helvB10_tr);
  tillyU8g2->drawStr(x, y + boxSize, label);
  int bx = x + tillyU8g2->getStrWidth(label) + 8;
  tillyU8g2->drawFrame(bx, y, boxSize, boxSize);
  if (checked) {
    int x0 = bx + boxSize / 8,       y0 = y + boxSize * 5 / 8;
    int x1 = bx + boxSize * 2 / 5,   y1 = y + boxSize - 3;
    int x2 = bx + boxSize - boxSize / 8, y2 = y + boxSize / 8;
    for (int o = -1; o <= 1; o++) {
      tillyU8g2->drawLine(x0, y0 + o, x1, y1 + o);
      tillyU8g2->drawLine(x1, y1 + o, x2, y2 + o);
    }
  }
  return bx + boxSize;
}

// UTC sunrise/sunset time in hours (0-24) at this date/lat/lon, or -1 if the
// sun doesn't rise/set that day at this latitude (polar day/night).
// Low-precision sunrise/sunset formula from the Almanac for Computers,
// 1990, Nautical Almanac Office, US Naval Observatory - good to about a
// minute, computed locally, no network call.
static float sunEventUtcHours(int year, int month, int day, double lat, double lon, bool isSunset) {
  static const int cum[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  int dayOfYear = cum[month - 1] + day + ((leap && month > 2) ? 1 : 0);

  double lngHour = lon / 15.0;
  double t = dayOfYear + (((isSunset ? 18.0 : 6.0) - lngHour) / 24.0);

  double M = (0.9856 * t) - 3.289;
  double Mr = M * DEG_TO_RAD;

  double L = M + (1.916 * sin(Mr)) + (0.020 * sin(2 * Mr)) + 282.634;
  L = fmod(L, 360.0); if (L < 0) L += 360.0;
  double Lr = L * DEG_TO_RAD;

  double RA = atan(0.91764 * tan(Lr)) * RAD_TO_DEG;
  RA = fmod(RA, 360.0); if (RA < 0) RA += 360.0;
  double Lquadrant = floor(L / 90.0) * 90.0;
  double RAquadrant = floor(RA / 90.0) * 90.0;
  RA = (RA + (Lquadrant - RAquadrant)) / 15.0;

  double sinDec = 0.39782 * sin(Lr);
  double cosDec = cos(asin(sinDec));

  double latR = lat * DEG_TO_RAD;
  double cosH = (cos(90.833 * DEG_TO_RAD) - (sinDec * sin(latR))) / (cosDec * cos(latR));
  if (cosH > 1.0 || cosH < -1.0) return -1.0f;  // never sets / never rises here today

  double Hdeg = acos(cosH) * RAD_TO_DEG;
  if (!isSunset) Hdeg = 360.0 - Hdeg;
  double H = Hdeg / 15.0;

  double T = H + RA - (0.06571 * t) - 6.622;
  double UT = fmod(T - lngHour, 24.0);
  if (UT < 0) UT += 24.0;
  return (float)UT;
}

// Synodic phase fraction: 0.0/1.0 = new moon, 0.5 = full moon.
static float moonPhaseFraction(time_t utcSeconds) {
  const double synodicMonth = 29.530588853;
  const double knownNewMoonUnix = 947182440.0;  // 2000-01-06 18:14 UTC
  double daysSince = (utcSeconds - knownNewMoonUnix) / 86400.0;
  double phase = fmod(daysSince, synodicMonth);
  if (phase < 0) phase += synodicMonth;
  return (float)(phase / synodicMonth);
}

static const char *moonPhaseName(float frac) {
  if (frac < 0.03f || frac >= 0.97f) return "New";
  if (frac < 0.22f) return "Wax Cres";
  if (frac < 0.28f) return "1st Qtr";
  if (frac < 0.47f) return "Wax Gib";
  if (frac < 0.53f) return "Full";
  if (frac < 0.72f) return "Wan Gib";
  if (frac < 0.78f) return "Last Qtr";
  return "Wan Cres";
}

static void drawPidBox(int x, int y, int w, int h, const char *label, float value) {
  tillyU8g2->drawFrame(x, y, w, h);
  tillyU8g2->setFont(u8g2_font_6x12_tr);
  tillyU8g2->drawStr(x + 4, y + 13, label);
  char buf[12];
  snprintf(buf, sizeof(buf), "%.2f", value);
  tillyU8g2->setFont(u8g2_font_helvB10_tr);
  tillyU8g2->drawStr(x + 4, y + h - 8, buf);
}

void updateTillyDisplay(const TillyDisplayState &s) {
  tillyU8g2->clearBuffer();

  // ----- Header: sats / EKF / MAG / armed / wifi, packed tightly -----
  // hdrBoxSize matches the helvB10 label font's height so the checkbox sits
  // level with the text. Each item is placed right after the previous one
  // (measured via getStrWidth/the return value of drawCheckbox) instead of
  // fixed columns, so there's no dead space between items.
  const int hdrBoxSize = 14, hdrBoxY = 16, hdrGap = 10;
  tillyU8g2->setFont(u8g2_font_helvB10_tr);
  char satsBuf[16];
  snprintf(satsBuf, sizeof(satsBuf), "SATS: %d", s.sats);
  tillyU8g2->drawStr(8, hdrBoxY + hdrBoxSize, satsBuf);
  int hx = 8 + tillyU8g2->getStrWidth(satsBuf) + hdrGap;

  hx = drawCheckbox(hx, hdrBoxY, hdrBoxSize, s.ekfOk, "EKF:") + hdrGap;
  hx = drawCheckbox(hx, hdrBoxY, hdrBoxSize, s.magOk, "MAG:") + hdrGap;
  hx = drawCheckbox(hx, hdrBoxY, hdrBoxSize, s.armed, "ARM:") + hdrGap;
  drawCheckbox(hx, hdrBoxY, hdrBoxSize, s.wifiOk, "WIFI:");

  tillyU8g2->drawHLine(0, 44, DISP_W);

  // ----- Mode + speed -----
  const char *modeStr = s.nmeaMode ? "NMEA" : (s.autoMode ? "AUTO" : "STANDBY");
  tillyU8g2->setFont(u8g2_font_helvB24_tr);
  tillyU8g2->drawStr(10, 80, modeStr);

  char speedBuf[16];
  snprintf(speedBuf, sizeof(speedBuf), "%.1f kn", s.speedKn);
  int speedW = tillyU8g2->getStrWidth(speedBuf);
  tillyU8g2->drawStr(DISP_W - speedW - 10, 80, speedBuf);

  tillyU8g2->drawHLine(0, 90, DISP_W);

  // ----- Cur / des heading - label and value, same line, same size -----
  char curBuf[8], desBuf[8];
  snprintf(curBuf, sizeof(curBuf), "%d", s.curHeading);
  snprintf(desBuf, sizeof(desBuf), "%d", s.desHeading);

  tillyU8g2->setFont(u8g2_font_helvB24_tr);
  tillyU8g2->drawStr(10, 126, "CUR:");
  int curLabelW = tillyU8g2->getStrWidth("CUR:");
  tillyU8g2->drawStr(10 + curLabelW + 10, 126, curBuf);

  tillyU8g2->drawStr(210, 126, "DES:");
  int desLabelW = tillyU8g2->getStrWidth("DES:");
  tillyU8g2->drawStr(210 + desLabelW + 10, 126, desBuf);

  tillyU8g2->drawHLine(0, 136, DISP_W);

  // ----- Current position, nautical D MM.mm' format, spread full width -----
  double latAbs = fabs(s.lat);
  int latDeg = (int)latAbs;
  double latMin = (latAbs - latDeg) * 60.0;
  char latHemi = (s.lat >= 0) ? 'N' : 'S';

  double lonAbs = fabs(s.lon);
  int lonDeg = (int)lonAbs;
  double lonMin = (lonAbs - lonDeg) * 60.0;
  char lonHemi = (s.lon >= 0) ? 'E' : 'W';

  char latBuf[24], lonBuf[24];
  snprintf(latBuf, sizeof(latBuf), "LAT: %02d %05.2f'%c", latDeg, latMin, latHemi);
  snprintf(lonBuf, sizeof(lonBuf), "LON: %03d %05.2f'%c", lonDeg, lonMin, lonHemi);
  tillyU8g2->setFont(u8g2_font_helvB14_tr);
  tillyU8g2->drawStr(10, 158, latBuf);
  int lonW = tillyU8g2->getStrWidth(lonBuf);
  tillyU8g2->drawStr(DISP_W - lonW - 10, 158, lonBuf);

  tillyU8g2->drawHLine(0, 168, DISP_W);

  // ----- Battery volts/amps + compass deviation -----
  // "Deviation" here is the live gap between compass heading and GPS course
  // over ground - drift/leeway/calibration error, not a fixed declination
  // table value, since that needs no extra param round-trip and updates
  // live like the rest of this screen.
  bool haveTime = (s.unixTimeSec != 0);  // still needed below for sunrise/sunset
  time_t t = (time_t)s.unixTimeSec;
  struct tm tmVal;
  if (haveTime) gmtime_r(&t, &tmVal);

  char battBuf[24];
  if (s.batteryVolts >= 0 && s.batteryAmps >= 0) {
    snprintf(battBuf, sizeof(battBuf), "%.1fV  %.1fA", s.batteryVolts, s.batteryAmps);
  } else if (s.batteryVolts >= 0) {
    snprintf(battBuf, sizeof(battBuf), "%.1fV", s.batteryVolts);
  } else {
    snprintf(battBuf, sizeof(battBuf), "--V  --A");
  }

  float dev = s.curHeading - s.cogDeg;
  while (dev > 180.0f) dev -= 360.0f;
  while (dev < -180.0f) dev += 360.0f;
  char devBuf[16];
  snprintf(devBuf, sizeof(devBuf), "DEV: %+.1f", dev);

  tillyU8g2->setFont(u8g2_font_helvB14_tr);
  tillyU8g2->drawStr(10, 190, battBuf);
  int devW = tillyU8g2->getStrWidth(devBuf);
  tillyU8g2->drawStr(DISP_W - devW - 10, 190, devBuf);

  tillyU8g2->drawHLine(0, 200, DISP_W);

  // ----- Countdown to next sunrise/sunset (hh:mm) + moon phase -----
  bool havePos = haveTime && (s.lat != 0.0 || s.lon != 0.0);
  char sunBuf[20], moonBuf[20];
  if (havePos) {
    double nowH = tmVal.tm_hour + tmVal.tm_min / 60.0 + tmVal.tm_sec / 3600.0;
    float riseH = sunEventUtcHours(tmVal.tm_year + 1900, tmVal.tm_mon + 1, tmVal.tm_mday, s.lat, s.lon, false);
    float setH  = sunEventUtcHours(tmVal.tm_year + 1900, tmVal.tm_mon + 1, tmVal.tm_mday, s.lat, s.lon, true);

    const char *label = nullptr;
    double deltaH = -1;
    if (riseH >= 0 && riseH > nowH) {
      label = "RISE"; deltaH = riseH - nowH;
    } else if (setH >= 0 && setH > nowH) {
      label = "SET"; deltaH = setH - nowH;
    } else {
      // Both of today's events (if any) already passed - next one is
      // tomorrow's sunrise.
      time_t tomorrow = t + 86400;
      struct tm tmTom;
      gmtime_r(&tomorrow, &tmTom);
      float riseTomH = sunEventUtcHours(tmTom.tm_year + 1900, tmTom.tm_mon + 1, tmTom.tm_mday, s.lat, s.lon, false);
      if (riseTomH >= 0) {
        label = "RISE";
        deltaH = (24.0 - nowH) + riseTomH;
      }
    }

    if (label != nullptr) {
      int totalMin = (int)(deltaH * 60.0 + 0.5);
      snprintf(sunBuf, sizeof(sunBuf), "%s: %02dh%02dmin", label, totalMin / 60, totalMin % 60);
    } else {
      snprintf(sunBuf, sizeof(sunBuf), "SUN: n/a");
    }
  } else {
    snprintf(sunBuf, sizeof(sunBuf), "NEXT: --h--min");
  }
  if (haveTime) {
    snprintf(moonBuf, sizeof(moonBuf), "MOON: %s", moonPhaseName(moonPhaseFraction(t)));
  } else {
    snprintf(moonBuf, sizeof(moonBuf), "MOON: --");
  }

  tillyU8g2->setFont(u8g2_font_helvB14_tr);
  tillyU8g2->drawStr(10, 222, sunBuf);
  int moonW = tillyU8g2->getStrWidth(moonBuf);
  tillyU8g2->drawStr(DISP_W - moonW - 10, 222, moonBuf);

  tillyU8g2->drawHLine(0, 232, DISP_W);

  // ----- PID terms -----
  const int boxY = 236, boxH = DISP_H - boxY - 6, gap = 5;
  const int boxW = (DISP_W - 6 * gap) / 5;
  struct { const char *label; float value; } terms[] = {
    {"FF", s.pidFF}, {"P", s.pidP}, {"I", s.pidI}, {"D", s.pidD}, {"SRATE", s.pidSRate},
  };
  for (int i = 0; i < 5; i++) {
    int x = gap + i * (boxW + gap);
    drawPidBox(x, boxY, boxW, boxH, terms[i].label, terms[i].value);
  }

  tillyU8g2->sendBuffer();
}

// Full-screen scrollback view of appLog()'s ring buffer. During setup() this
// is called directly (see wifibuttonbox.ino) so boot progress is visible
// live; afterwards it's toggled by holding buttons 5+6 together.
void updateTillyLogScreen() {
  tillyU8g2->clearBuffer();

  tillyU8g2->setFont(u8g2_font_6x12_tr);
  tillyU8g2->drawStr(4, 11, "LOG - hold 5+6 to exit");
  tillyU8g2->drawHLine(0, 15, DISP_W);

  const int lineH = 12;
  const int top = 15 + lineH;
  const int maxLines = (DISP_H - top) / lineH;

  size_t total = appLogCount();
  size_t shown = (total < (size_t)maxLines) ? total : (size_t)maxLines;
  size_t startIdx = total - shown;  // oldest of the visible window

  int y = top;
  for (size_t i = 0; i < shown; i++) {
    tillyU8g2->drawStr(2, y, appLogLine(startIdx + i));
    y += lineH;
  }

  tillyU8g2->sendBuffer();
}

void initTillyDisplay() {
  tillyLcd.begin(0, U8G2_R1);
  tillyU8g2 = tillyLcd.getU8g2();

  // Show the (still empty) log screen immediately rather than a half-drawn
  // telemetry layout with no data yet - setup() fills it in as it runs.
  updateTillyLogScreen();
}

#endif
