#pragma once
#include <Arduino.h>

// Ring buffer of recent status/event lines, mirrored to Serial as they
// come in. Feeds the on-screen log view (hold buttons 5+6 together to
// toggle it - see handleButtons() in wifibuttonbox.ino).
#define APPLOG_CAPACITY 64
#define APPLOG_LINE_LEN 72

// printf-style. Each line is timestamped with millis() and echoed to Serial.
void appLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Optional sink that mirrors every appLog() line out over MAVLink (as a
// STATUSTEXT, see wifibuttonbox.ino's mavLogSink()). Unset (nullptr) until
// wifibuttonbox.ino registers it once ArduPilotSerial is up, so early boot
// lines still land in the ring buffer + Serial but don't reach the vehicle.
typedef void (*AppLogMavSink)(const char *text);
void appLogSetMavSink(AppLogMavSink sink);

// Number of lines currently held (<= APPLOG_CAPACITY).
size_t appLogCount();

// Line `index` back from the oldest still-held line (0 = oldest,
// appLogCount()-1 = newest). Empty string if out of range.
const char *appLogLine(size_t index);
