#pragma once
// ============================================================================
// web_telemetry.h
//
// On-device replacement for tilly_telemetry_backend.py: serves the tuning/
// live-telemetry dashboard (data/index.html, uploaded via `pio run -t
// uploadfs`) over HTTP, and re-broadcasts decoded MAVLink telemetry to it
// over WebSocket - same JSON schema the page's Live Telemetry tab already
// expects (see Telemetry::snapshot() in tilly_telemetry_backend.py).
//
// Usage in wifibuttonbox.ino:
//
//   #include "web_telemetry.h"
//
//   void setup() {
//     ...
//     if (wifi) webTelemetry_setup(ArduPilotSerial);
//   }
//
//   void loop() {
//     ...
//     webTelemetry_update();
//   }
//
// Feed it MAVLink messages the same way the NMEA/OpenCPN bridges are fed:
//
//   webTelemetry_handleMavMessage(msg);
// ============================================================================

#include <Arduino.h>
#include <MAVLink_ardupilotmega.h>

// Call once from setup(), after WiFi and ArduPilotSerial are up. Mounts
// SPIFFS, starts the HTTP server (port 80, serves data/index.html at "/")
// and the WebSocket telemetry server (port 81). Also sets GCS_PID_MASK=1 on
// the vehicle so PID_TUNING (steering axis) data flows, and kicks off the
// first round of tuning-parameter reads.
void webTelemetry_setup(HardwareSerial &mavSerial);

// Call every loop() iteration. Handles HTTP/WebSocket client I/O, re-polls
// tuning parameters periodically, and broadcasts a telemetry snapshot to any
// connected WebSocket clients at a fixed rate.
void webTelemetry_update();

// Feed every parsed MAVLink message here (in addition to the NMEA/OpenCPN
// bridges) so ATTITUDE, VFR_HUD, GLOBAL_POSITION_INT, PID_TUNING,
// SERVO_OUTPUT_RAW and PARAM_VALUE can update the telemetry snapshot.
void webTelemetry_handleMavMessage(const mavlink_message_t &msg);
