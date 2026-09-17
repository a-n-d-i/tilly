// ============================================================================
// opencpn_bridge.cpp
// See opencpn_bridge.h for usage and design notes.
// ============================================================================

#include "opencpn_bridge.h"
#include <WiFiUdp.h>
#include <WiFi.h>
#include <MAVLink_ardupilotmega.h>

// desired_heading lives in wifibuttonbox.ino - this module writes to it
// directly, same as handleButtons() does for the +1/+10/-1/-10 buttons.
extern int desired_heading;

// sendCustomEvent() also lives in wifibuttonbox.ino - reused here so
// heading changes from OpenCPN show up in the log the same way the
// button-driven ones do.
extern void sendCustomEvent(const char* text, uint8_t severity);// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static WiFiUDP s_udpIn;
static uint16_t s_listenPort = 10111;

static float s_xteGainDegPerNm = 20.0f;
static float s_xteMaxCorrDeg   = 30.0f;
static uint32_t s_watchdogTimeoutMs = 5000;

static bool     s_navValid          = false;
static uint32_t s_lastNavUpdateMs   = 0;
static float    s_xteNm             = 0.0f;
static char     s_steerDir          = 'R';   // 'L' or 'R'
static float    s_bearingToWpDeg    = 0.0f;  // true bearing, present pos -> waypoint
static bool     s_haveDirectHeading = false;
static float    s_directHeadingDeg  = 0.0f;  // APB field 13, if the source populates it

// ---------------------------------------------------------------------------
// NMEA0183 parsing helpers
// ---------------------------------------------------------------------------

// Verifies the "*hh" checksum on a raw sentence. Returns true and strips
// the checksum if valid.
static bool verifyAndStripChecksum(String &line) {
  line.trim();
  int star = line.lastIndexOf('*');
  if (star < 0 || star + 3 > (int)line.length()) return false;

  uint8_t expected = (uint8_t)strtol(line.substring(star + 1, star + 3).c_str(), nullptr, 16);
  uint8_t actual = 0;
  for (int i = 1; i < star; i++) actual ^= (uint8_t)line[i]; // skip leading '$'
  if (actual != expected) return false;

  line = line.substring(0, star);
  return true;
}

static int splitFields(const String &body, String out[], int maxFields) {
  int count = 0;
  int start = 0;
  while (count < maxFields) {
    int comma = body.indexOf(',', start);
    if (comma < 0) {
      out[count++] = body.substring(start);
      break;
    }
    out[count++] = body.substring(start, comma);
    start = comma + 1;
  }
  return count;
}

// Parses "$--APB,A,A,x.x,a,N,A,A,x.x,a,c--c,x.x,a,x.x,a*hh" (talker ID
// varies - GP/EC/II etc). f[0] is "GPAPB" (talker+type), so field N in the
// NMEA spec sits at f[N].
static void parseAPB(const String &body) {
  String f[16];
  int n = splitFields(body, f, 16);
  if (n < 12) return;

  bool loranValid = (f[1] == "A");
  bool cycleValid  = (f[2] == "A");
  if (!loranValid || !cycleValid) return; // sentence itself flags data invalid

  s_xteNm = f[3].toFloat();
  s_steerDir = f[4].length() ? f[4][0] : 'R';
  // f[5] is XTE units, normally "N" for nautical miles - assumed.
  s_bearingToWpDeg = f[11].toFloat(); // field 11: bearing, present pos to destination, true

  s_haveDirectHeading = false;
  if (n >= 14 && f[13].length() > 0) {
    s_directHeadingDeg = f[13].toFloat(); // field 13: heading to steer to destination
    s_haveDirectHeading = true;
  }

  s_navValid = true;
  s_lastNavUpdateMs = millis();
}

static void handleNmeaLine(String line) {
  if (!verifyAndStripChecksum(line)) return; // silently drop malformed/corrupt sentences
  if (line.length() < 6) return;

  String type = line.substring(3, 6); // skip '$' + 2-char talker ID
  if (type == "APB") {
    parseAPB(line.substring(1));
  }
  // Other sentence types OpenCPN might send on this connection (including
  // RMB) are ignored - this module only acts on APB.
}

// Combines bearing-to-waypoint with a simple proportional XTE correction
// into a single steer-to heading. Deliberately simple (not a real
// cross-track PID) - tune gain/clamp with opencpnBridge_setXteGain().
static float computeSteerToHeading() {
  if (s_haveDirectHeading) return s_directHeadingDeg;

  float correction = s_xteNm * s_xteGainDegPerNm;
  if (correction > s_xteMaxCorrDeg) correction = s_xteMaxCorrDeg;

  float heading = s_bearingToWpDeg;
  if (s_steerDir == 'L') heading -= correction;
  else                   heading += correction;

  if (heading < 0) heading += 360.0f;
  if (heading >= 360.0f) heading -= 360.0f;
  return heading;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void opencpnBridge_setup(uint16_t listenPort) {
  s_listenPort = listenPort;
  s_udpIn.begin(s_listenPort);
  Serial.printf("OpenCPN autopilot bridge listening on UDP %d\n", s_listenPort);
}

void opencpnBridge_setXteGain(float degPerNm, float maxCorrectionDeg) {
  s_xteGainDegPerNm = degPerNm;
  s_xteMaxCorrDeg = maxCorrectionDeg;
}

void opencpnBridge_setWatchdogTimeout(uint32_t timeoutMs) {
  s_watchdogTimeoutMs = timeoutMs;
}

void opencpnBridge_update() {
  int packetSize;
  char udpBuf[512];
  while ((packetSize = s_udpIn.parsePacket()) > 0) {
    int len = s_udpIn.read(udpBuf, sizeof(udpBuf) - 1);
    if (len <= 0) continue;
    udpBuf[len] = '\0';

    // A single UDP datagram may contain multiple CR/LF-terminated sentences.
    String chunk(udpBuf);
    int lineStart = 0;
    while (lineStart < (int)chunk.length()) {
      int lineEnd = chunk.indexOf('\n', lineStart);
      String line = (lineEnd < 0) ? chunk.substring(lineStart) : chunk.substring(lineStart, lineEnd);
      if (line.length() > 0) handleNmeaLine(line);
      if (lineEnd < 0) break;
      lineStart = lineEnd + 1;
    }
  }

  uint32_t now = millis();
  if (s_navValid && (now - s_lastNavUpdateMs > s_watchdogTimeoutMs)) {
    s_navValid = false;
    Serial.println("OpenCPN autopilot bridge: watchdog timeout, holding last heading");
  }

  if (s_navValid) {
    int newHeading = (int)roundf(computeSteerToHeading());
    if (newHeading > 359) newHeading -= 360;
    if (newHeading < 0) newHeading += 360;
    if (newHeading != desired_heading) {
      desired_heading = newHeading;
      Serial.printf("OpenCPN autopilot bridge: desired_heading -> %d\n", desired_heading);
      char buf[64];
      snprintf(buf, sizeof(buf), "OpenCPN AP, Heading -> %d", desired_heading);
      sendCustomEvent(buf, MAV_SEVERITY_NOTICE);
    }
  }
}
