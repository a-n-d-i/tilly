#pragma once
// ============================================================================
// mavlink_nmea_bridge.h
//
// Listens to a MAVLink stream (e.g. Serial2 from Pixhawk/ArduPilot) and
// re-broadcasts a subset of the data as NMEA0183 sentences over UDP,
// so any chartplotter / SignalK / OpenCPN instance on the LAN can consume
// autopilot position, heading, and nav-to-waypoint data.
//
// Excluded on purpose: VHW (needs true water speed, not groundspeed) and
// XDR (no clean NMEA0183 slot for battery/generic transducer data here).
//
// Usage in your existing sketch:
//
//   #include "mavlink_nmea_bridge.h"
//
//   void setup() {
//     Serial2.begin(57600, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
//     mavNmeaBridge_setup(Serial2, IPAddress(192,168,1,255), 10110);
//     mavNmeaBridge_setDeclination(3.5f); // degrees E positive, W negative
//   }
//
//   void loop() {
//     mavNmeaBridge_update();
//     // ... rest of your existing loop ...
//   }
//
// Requires WiFi already connected before mavNmeaBridge_setup() sends anything
// (UDP begin() itself doesn't need it, but sends will silently fail without).
// ============================================================================

#include <Arduino.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <MAVLink_ardupilotmega.h>
// Call once from setup(), after mavSerial.begin(...) and ideally after WiFi
// is connected. broadcastIP is typically your subnet broadcast address
// (e.g. 192.168.1.255) so any plotter listening on udpPort picks it up.
void mavNmeaBridge_setup(HardwareSerial &mavSerial, WiFiUDP &udp, IPAddress broadcastIP, uint16_t udpPort);

// Call every loop() iteration. Non-blocking: drains available MAVLink bytes,
// updates cached state, and emits NMEA sentences on their own rate limits.
void mavNmeaBridge_update();

// Optional: magnetic declination in degrees, East positive / West negative.
// Used to derive HDT (true heading) from VFR_HUD's magnetic heading.
// Default is 0.0 if never called.
void mavNmeaBridge_setDeclination(float declinationDeg);

// Optional: change how often each sentence group is sent (default 1000 ms
// for position/heading, 2000 ms for nav-to-waypoint). Safe to ignore.
void mavNmeaBridge_setRates(uint32_t positionMs, uint32_t headingMs, uint32_t navMs);

void handleMavMessage(const mavlink_message_t &msg);
