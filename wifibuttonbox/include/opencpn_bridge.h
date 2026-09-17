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
// Deliberately does NOT send MAVLink itself. It piggybacks on the existing
// sendYawCommandDeg() call in loop() (gated on pilotMode == AUTO, firing
// every mavlinkUpdateInterval) instead of adding a second sender on
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
// A watchdog stops updating desired_heading if nothing valid has arrived
// recently - see opencpnBridge_setWatchdogTimeout().
void opencpnBridge_update();

// Proportional XTE correction gain, used only when APB doesn't supply a
// direct "heading to steer" field (field 13 - many sources leave it blank):
// degrees of heading correction per nautical mile of cross-track error,
// clamped to maxCorrectionDeg. Default: 20 deg/nm, clamped to 30 deg.
void opencpnBridge_setXteGain(float degPerNm, float maxCorrectionDeg);

// If no valid APB arrives within this many ms, stop updating
// desired_heading (fail-safe - button box still works normally, boat just
// holds last commanded heading). Default 5000 ms.
void opencpnBridge_setWatchdogTimeout(uint32_t timeoutMs);
