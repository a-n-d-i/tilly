// ============================================================================
// opencpn_bridge.cpp
// See opencpn_bridge.h for usage and design notes.
// ============================================================================

#include "opencpn_bridge.h"
#include <WiFiUdp.h>
#include <WiFi.h>
#include <MAVLink_ardupilotmega.h>
#include "applog.h"

// desired_heading lives in wifibuttonbox.ino - this module writes to it
// directly, same as handleButtons() does for the +1/+10/-1/-10 buttons.
extern int desired_heading;

// sendCustomEvent() also lives in wifibuttonbox.ino - reused here so
// heading changes from OpenCPN show up in the log the same way the
// button-driven ones do.
extern void sendCustomEvent(const char* text, uint8_t severity);

// True only while NMEA mode is active (Auto+Standby combo) - APB sentences
// are always parsed/logged below, but only steer the boat while this is true.
extern bool nmeaModeActive();

// Same MANUAL-mode transition as pressing the Standby button - lives in
// wifibuttonbox.ino since it touches pendingModeCmd/pendingModeCmdSentMs.
extern void requestNmeaWatchdogFallback();

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static WiFiUDP s_udpIn;
static uint16_t s_listenPort = 10111;
static bool     s_socketReady = false;
static uint32_t s_lastBindAttemptMs = 0;
static const uint32_t kBindRetryIntervalMs = 2000;

static float s_xteGainDegPerNm = 20.0f;
static float s_xteMaxCorrDeg   = 5.0f;
static uint32_t s_watchdogTimeoutMs = 3000;

static bool     s_navValid          = false;
static uint32_t s_lastNavUpdateMs   = 0;
static float    s_xteNm             = 0.0f;
static char     s_steerDir          = 'R';   // 'L' or 'R'
static float    s_bearingToWpDeg    = 0.0f;  // APB field 11: bearing, present pos -> waypoint
static float    s_bearingOriginToDestDeg = 0.0f; // APB field 8: bearing, origin -> destination (the intended track)
static bool     s_haveDirectHeading = false;
static float    s_directHeadingDeg  = 0.0f;  // APB field 13, if the source populates it

// Set once the watchdog has already triggered a fallback, so it doesn't
// re-send the MANUAL-mode request on every single loop() tick while the
// COMMAND_ACK for that request is still pending (nmeaModeActive() stays
// true - pilotMode only flips once the ack arrives). Cleared as soon as
// NMEA mode isn't active any more (fallback completed, or it was never
// entered in the first place).
static bool s_watchdogFallbackTriggered = false;

// ---------------------------------------------------------------------------
// On-screen APB feed - readable log of recent sentences, shown in place of
// the position/battery/sun rows while NMEA mode is active (see display.h).
// Kept separate from appLog()'s ring buffer: that one logs a terser,
// technical line for the full debug view, this one is phrased for the
// small live-feed area on the main telemetry screen.
// ---------------------------------------------------------------------------
#define APB_LOG_CAPACITY 8
#define APB_LOG_LINE_LEN 56
static char s_apbLog[APB_LOG_CAPACITY][APB_LOG_LINE_LEN];
static size_t s_apbLogCount = 0;
static size_t s_apbLogHead = 0;

// Timestamped the same way as appLog()'s ring buffer ("[millis] ..."), so
// the on-screen feed reads consistently with the rest of the log output.
static void apbLogPush(const char *line) {
  snprintf(s_apbLog[s_apbLogHead], APB_LOG_LINE_LEN, "[%lu] %s", millis(), line);
  s_apbLogHead = (s_apbLogHead + 1) % APB_LOG_CAPACITY;
  if (s_apbLogCount < APB_LOG_CAPACITY) s_apbLogCount++;
}

size_t opencpnBridge_apbLogCount() {
  return s_apbLogCount;
}

const char *opencpnBridge_apbLogLine(size_t index) {
  if (index >= s_apbLogCount) return "";
  size_t oldest = (s_apbLogCount < APB_LOG_CAPACITY) ? 0 : s_apbLogHead;
  return s_apbLog[(oldest + index) % APB_LOG_CAPACITY];
}

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

  // f[1]/f[2] (LORAN-C blink/cycle-lock warning flags) are dead - not parsed.
  s_xteNm = f[3].toFloat();
  s_steerDir = f[4].length() ? f[4][0] : 'R';
  // f[5] is XTE units, normally "N" for nautical miles - assumed.

  s_bearingOriginToDestDeg = f[8].toFloat(); // field 8: bearing, origin to destination (the intended track)
  char originRef = (n >= 10 && f[9].length()) ? f[9][0] : '?'; // field 9: M/T for field 8

  s_bearingToWpDeg = f[11].toFloat(); // field 11: bearing, present pos to destination
  char bearingRef = (n >= 13 && f[12].length()) ? f[12][0] : '?'; // field 12: M/T for field 11

  s_haveDirectHeading = false;
  char headingRef = '?';
  if (n >= 14 && f[13].length() > 0) {
    s_directHeadingDeg = f[13].toFloat(); // field 13: heading to steer to destination
    s_haveDirectHeading = true;
    if (n >= 15 && f[14].length() > 0) headingRef = f[14][0]; // field 14: M/T for field 13
  }

  // Log every heading APB actually supplied, not just the one currently fed
  // to computeSteerToHeading() - the three can legitimately disagree (track
  // bearing vs. direct bearing-to-waypoint vs. a source-computed steer-to
  // heading), and seeing all of them is the only way to tell which one is
  // driving the boat and whether that's the field it should be.
  char readable[APB_LOG_LINE_LEN];
  if (s_haveDirectHeading) {
    appLog("APB: org->dst %.1f%c  pos->dst %.1f%c  steer %.1f%c  xte %.2fnm %c",
           s_bearingOriginToDestDeg, originRef, s_bearingToWpDeg, bearingRef,
           s_directHeadingDeg, headingRef, s_xteNm, s_steerDir);
    snprintf(readable, sizeof(readable), "Steer %d%c  Org %d%c  Pos %d%c",
             (int)lroundf(s_directHeadingDeg), headingRef,
             (int)lroundf(s_bearingOriginToDestDeg), originRef,
             (int)lroundf(s_bearingToWpDeg), bearingRef);
  } else {
    appLog("APB: org->dst %.1f%c  pos->dst %.1f%c  (no steer hdg)  xte %.2fnm %c",
           s_bearingOriginToDestDeg, originRef, s_bearingToWpDeg, bearingRef,
           s_xteNm, s_steerDir);
    snprintf(readable, sizeof(readable), "Org %d%c  Pos %d%c  XTE %.2f%c",
             (int)lroundf(s_bearingOriginToDestDeg), originRef,
             (int)lroundf(s_bearingToWpDeg), bearingRef,
             s_xteNm, s_steerDir);
  }
  apbLogPush(readable);

  s_navValid = true;
  s_lastNavUpdateMs = millis();
}

static void handleNmeaLine(String line) {
  bool looksLikeApb = line.indexOf("APB") >= 0;
  if (!verifyAndStripChecksum(line)) {
    if (looksLikeApb) appLog("APB checksum fail, dropped: %s", line.c_str());
    return;
  }
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
  s_lastBindAttemptMs = millis();
  s_lastNavUpdateMs = millis();  // avoid a stale 0 sentinel, even though s_navValid gates its use
  s_socketReady = s_udpIn.begin(s_listenPort) != 0;
  if (s_socketReady) {
    appLog("OpenCPN autopilot bridge listening on UDP %d", s_listenPort);
  } else {
    appLog("OpenCPN autopilot bridge: UDP bind on port %d failed, will retry", s_listenPort);
  }
}

void opencpnBridge_setXteGain(float degPerNm, float maxCorrectionDeg) {
  s_xteGainDegPerNm = degPerNm;
  s_xteMaxCorrDeg = maxCorrectionDeg;
}

void opencpnBridge_setWatchdogTimeout(uint32_t timeoutMs) {
  s_watchdogTimeoutMs = timeoutMs;
}

void opencpnBridge_update() {
  uint32_t now = millis();

  if (!s_socketReady) {
    if (now - s_lastBindAttemptMs < kBindRetryIntervalMs) return;
    s_lastBindAttemptMs = now;
    s_socketReady = s_udpIn.begin(s_listenPort) != 0;
    if (s_socketReady) {
      appLog("OpenCPN autopilot bridge: UDP bind on port %d succeeded", s_listenPort);
    } else {
      return;
    }
  }

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

  // Re-read millis() rather than reuse the `now` captured at the top: a
  // sentence parsed during the packet-processing loop above just set
  // s_lastNavUpdateMs to a slightly LATER value than that stale `now`,
  // which would underflow this unsigned subtraction into a huge number and
  // trip the watchdog immediately after a perfectly good update.
  now = millis();
  bool navStale = (now - s_lastNavUpdateMs > s_watchdogTimeoutMs);
  if (s_navValid && navStale) {
    s_navValid = false;
  }

  if (nmeaModeActive() && navStale) {
    if (!s_watchdogFallbackTriggered) {
      s_watchdogFallbackTriggered = true;
      appLog("NMEA: no APB for %lus, falling back to STANDBY", (unsigned long)(s_watchdogTimeoutMs / 1000));
      requestNmeaWatchdogFallback();
    }
  } else {
    s_watchdogFallbackTriggered = false;
  }

  if (s_navValid && nmeaModeActive()) {
    int newHeading = (int)roundf(computeSteerToHeading());
    if (newHeading > 359) newHeading -= 360;
    if (newHeading < 0) newHeading += 360;
    if (newHeading != desired_heading) {
      desired_heading = newHeading;
      appLog("OpenCPN autopilot bridge: desired_heading -> %d", desired_heading);
      char buf[64];
      snprintf(buf, sizeof(buf), "OpenCPN AP, Heading -> %d", desired_heading);
      sendCustomEvent(buf, MAV_SEVERITY_NOTICE);
    }
  }
}
