#pragma once
// ============================================================================
// opencpn_bridge.h
//
// Second UDP listener (port 10111 by default) for OpenCPN's classic-autopilot
// output (APB - see OpenCPN's route/waypoint autopilot output setting).
// Parses cross-track error + bearing-to-waypoint and writes the result into
// desired_heading (extern, defined in wifibuttonbox.ino) - the same global
// the +1/+10/-1/-10 buttons already drive in AUTO mode.
//
// APB sentences are always parsed and logged, but only applied to
// desired_heading while NMEA mode is active (nmeaModeActive(), toggled by
// the Auto+Standby button combo) - otherwise OpenCPN could silently steer
// the boat any time it's connected, regardless of what the button box
// thinks it's doing.
//
// Deliberately does NOT send MAVLink itself. It piggybacks on the existing
// sendYawCommandDeg() call in loop() (gated on pilotMode == AUTO || NMEA,
// firing every mavlinkUpdateInterval) instead of adding a second sender on
// ArduPilotSerial. Two independent senders racing to set yaw is the kind of
// thing that causes rudder hunting - one source of truth is safer. If you'd
// rather this send MAVLink directly, that's a small change - flagging the
// reasoning since it's a divergence from "sends MAVLink itself".
//
// Usage in wifibuttonbox.ino:
//
//   #include "opencpn_bridge.h"
//
//   void setup() {
//     ...
//     opencpnBridge_setup(10111);
//   }
//
//   void loop() {
//     ...
//     opencpnBridge_update();
//   }
//
// OpenCPN side: Options -> Connections -> add a Network/UDP connection,
// address = this ESP32's IP, port 10111, direction Output, and enable
// "APB sentence" for the active route/waypoint autopilot output.
// ============================================================================

#include <Arduino.h>

// Call once from setup(), after WiFi is up.
void opencpnBridge_setup(uint16_t listenPort);

// Call every loop() iteration. Non-blocking: drains pending UDP packets,
// parses APB, and updates desired_heading whenever fresh, valid nav
// data arrives (wrapped to 0-359, same convention as handleButtons()).
// Each change also fires a STATUSTEXT via sendCustomEvent(), same as the
// button-driven heading changes, so it shows up in the ArduPilot log.
// A watchdog drops NMEA mode back to STANDBY if nothing valid has arrived
// recently - see opencpnBridge_setWatchdogTimeout(). Without this, NMEA
// mode would happily sit in GUIDED holding whatever heading it last had
// (its entry heading, if no APB ever arrived at all) with no live nav data
// behind it at all - actively "steering" with no input.
void opencpnBridge_update();

// Proportional XTE correction gain, used only when APB doesn't supply a
// direct "heading to steer" field (field 13 - many sources leave it blank):
// degrees of heading correction per nautical mile of cross-track error,
// clamped to maxCorrectionDeg. Default: 20 deg/nm, clamped to 30 deg.
void opencpnBridge_setXteGain(float degPerNm, float maxCorrectionDeg);

// If no valid APB arrives within this many ms while NMEA mode is active,
// fall back to STANDBY (same MANUAL-mode transition as pressing the
// Standby button) and log an error - fail-safe against steering on stale
// or absent nav data. Default 3000 ms.
void opencpnBridge_setWatchdogTimeout(uint32_t timeoutMs);

// Readable, display-friendly log of recent APB sentences (e.g. "Course 123
// deg true, XTE 0.05nm R") - shown in place of the position/battery/sun
// rows while NMEA mode is active (see display.h). Newest last, same
// indexing convention as appLog()'s ring buffer.
size_t opencpnBridge_apbLogCount();
const char *opencpnBridge_apbLogLine(size_t index);
