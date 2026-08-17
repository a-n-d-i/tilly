// ============================================================================
// mavlink_nmea_bridge.cpp
// See mavlink_nmea_bridge.h for usage.
//
// Assumes the ArduPilot/MAVLink c_library_v2 headers are already in your
// project (you're already parsing MAVLink for the custom STATUSTEXT event
// injection, so this reuses the same dialect). Adjust the include below if
// your project uses a different path (e.g. "mavlink/common/mavlink.h").
// ============================================================================

#include "mavlink_nmea_bridge.h"
#include <MAVLink_ardupilotmega.h>

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static HardwareSerial *s_mavSerial = nullptr;
static WiFiUDP s_udp;
static IPAddress s_broadcastIP;
static uint16_t s_udpPort = 10110;
static float s_declinationDeg = 0.0f;

static uint32_t s_positionRateMs = 1000;
static uint32_t s_headingRateMs  = 1000;
static uint32_t s_navRateMs      = 2000;

static uint32_t s_lastPositionSend = 0;
static uint32_t s_lastHeadingSend  = 0;
static uint32_t s_lastNavSend      = 0;

// Cached latest values from MAVLink messages
static bool     s_haveGps      = false;
static int32_t  s_lat_e7       = 0;      // degrees * 1e7
static int32_t  s_lon_e7       = 0;      // degrees * 1e7
static float    s_groundSpeedMs = 0.0f;  // m/s
static float    s_courseDeg     = 0.0f;  // degrees true (COG)
static uint8_t  s_fixType       = 0;     // GPS_FIX_TYPE
static uint8_t  s_satsVisible   = 0;
static uint64_t s_timeUnixUsec  = 0;     // for RMC/ZDA date+time

static bool     s_haveHeading  = false;
static float    s_headingMagDeg = 0.0f;  // VFR_HUD.heading

static bool     s_haveNav      = false;
static float    s_wpDistM      = 0.0f;
static float    s_targetBearingDeg = 0.0f;
static float    s_xtrackErrorM = 0.0f;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

// Appends the NMEA "*hh\r\n" checksum + terminator to a sentence that
// already starts with '$' and contains no checksum yet.
static String withChecksum(const String &body) {
  uint8_t cs = 0;
  for (size_t i = 1; i < body.length(); i++) { // skip leading '$'
    cs ^= (uint8_t)body[i];
  }
  char tail[8];
  snprintf(tail, sizeof(tail), "*%02X", cs);
  return body + tail + "\r\n";
}

static void sendSentence(const String &bodyNoChecksum) {
  Serial.println("Sendsentence");
  if (WiFi.status() != WL_CONNECTED) return;
  String out = withChecksum(bodyNoChecksum);
  s_udp.beginPacket(s_broadcastIP, s_udpPort);
  s_udp.write((const uint8_t *)out.c_str(), out.length());
  s_udp.endPacket();
  Serial.println("Sent UDP Package");
}

// Converts signed decimal degrees to NMEA "ddmm.mmmm"/"dddmm.mmmm" + hemisphere.
static void toNmeaLatLon(double latDeg, double lonDeg,
                          char *latOut, size_t latLen, char &latHemi,
                          char *lonOut, size_t lonLen, char &lonHemi) {
  latHemi = (latDeg >= 0) ? 'N' : 'S';
  lonHemi = (lonDeg >= 0) ? 'E' : 'W';
  latDeg = fabs(latDeg);
  lonDeg = fabs(lonDeg);

  int latD = (int)latDeg;
  double latM = (latDeg - latD) * 60.0;
  snprintf(latOut, latLen, "%02d%07.4f", latD, latM);

  int lonD = (int)lonDeg;
  double lonM = (lonDeg - lonD) * 60.0;
  snprintf(lonOut, lonLen, "%03d%07.4f", lonD, lonM);
}

// Splits a unix-epoch-microseconds timestamp into NMEA time (hhmmss.ss)
// and date (ddmmyy) strings.
static void toNmeaTimeDate(uint64_t unixUsec, char *timeOut, size_t timeLen,
                            char *dateOut, size_t dateLen) {
  time_t secs = (time_t)(unixUsec / 1000000ULL);
  struct tm tmv;
  gmtime_r(&secs, &tmv);
  snprintf(timeOut, timeLen, "%02d%02d%02d.00",
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  snprintf(dateOut, dateLen, "%02d%02d%02d",
           tmv.tm_mday, tmv.tm_mon + 1, (tmv.tm_year + 1900) % 100);
}

// ---------------------------------------------------------------------------
// Sentence builders
// ---------------------------------------------------------------------------

static void sendGGA() {
  if (!s_haveGps || s_fixType < 2) return; // no fix, nothing useful to send

  char latStr[16], lonStr[16], timeStr[16], dateStr[8];
  char latHemi, lonHemi;
  toNmeaLatLon(s_lat_e7 / 1e7, s_lon_e7 / 1e7,
               latStr, sizeof(latStr), latHemi,
               lonStr, sizeof(lonStr), lonHemi);
  char dummyDate[8];
  toNmeaTimeDate(s_timeUnixUsec, timeStr, sizeof(timeStr), dummyDate, sizeof(dummyDate));

  // Fix quality: 0=invalid,1=GPS,2=DGPS. ArduPilot fix_type>=3 => 3D fix.
  int quality = (s_fixType >= 3) ? 1 : 0;

  String s = "$GPGGA,";
  s += timeStr; s += ",";
  s += latStr; s += ","; s += latHemi; s += ",";
  s += lonStr; s += ","; s += lonHemi; s += ",";
  s += String(quality); s += ",";
  s += String(s_satsVisible); s += ",";
  s += "1.0,"; // HDOP placeholder (not tracked here)
  s += "0.0,M,0.0,M,,"; // altitude/geoid separation left blank/placeholder
  sendSentence(s);
}

static void sendRMC() {
  if (!s_haveGps) return;

  char latStr[16], lonStr[16], timeStr[16], dateStr[8];
  char latHemi, lonHemi;
  toNmeaLatLon(s_lat_e7 / 1e7, s_lon_e7 / 1e7,
               latStr, sizeof(latStr), latHemi,
               lonStr, sizeof(lonStr), lonHemi);
  toNmeaTimeDate(s_timeUnixUsec, timeStr, sizeof(timeStr), dateStr, sizeof(dateStr));

  float sog_kn = s_groundSpeedMs * 1.9438445f; // m/s -> knots
  char status = (s_fixType >= 3) ? 'A' : 'V';

  String s = "$GPRMC,";
  s += timeStr; s += ",";
  s += status; s += ",";
  s += latStr; s += ","; s += latHemi; s += ",";
  s += lonStr; s += ","; s += lonHemi; s += ",";
  s += String(sog_kn, 1); s += ",";
  s += String(s_courseDeg, 1); s += ",";
  s += dateStr; s += ",,,";
  sendSentence(s);
}

static void sendZDA() {
  if (!s_haveGps || s_timeUnixUsec == 0) return;

  char timeStr[16], dateStr[8];
  toNmeaTimeDate(s_timeUnixUsec, timeStr, sizeof(timeStr), dateStr, sizeof(dateStr));
  // dateStr is ddmmyy; ZDA wants day,month,year separately
  int day = (dateStr[0]-'0')*10 + (dateStr[1]-'0');
  int mon = (dateStr[2]-'0')*10 + (dateStr[3]-'0');
  int yr  = 2000 + (dateStr[4]-'0')*10 + (dateStr[5]-'0');

  String s = "$GPZDA,";
  s += timeStr; s += ",";
  s += String(day); s += ",";
  s += String(mon); s += ",";
  s += String(yr); s += ",00,00";
  sendSentence(s);
}

static void sendHDM() {
  if (!s_haveHeading) return;
  String s = "$GPHDM,";
  s += String(s_headingMagDeg, 1);
  s += ",M";
  sendSentence(s);
}

static void sendHDT() {
  if (!s_haveHeading) return;
  float trueHdg = s_headingMagDeg + s_declinationDeg;
  if (trueHdg < 0) trueHdg += 360.0f;
  if (trueHdg >= 360.0f) trueHdg -= 360.0f;
  String s = "$GPHDT,";
  s += String(trueHdg, 1);
  s += ",T";
  sendSentence(s);
}

static void sendAPB() {
  if (!s_haveNav) return;
  // Simplified APB: status flags fixed to "A,A" (no loran-c blink/cycle
  // warnings), XTE in nautical miles (left/right + steer-to sense),
  // bearing-to-waypoint true, waypoint ID left blank.
  float xteNm = fabs(s_xtrackErrorM) / 1852.0f;
  char steerDir = (s_xtrackErrorM < 0) ? 'L' : 'R'; // sign convention per your autopilot's xtrack_error

  String s = "$GPAPB,A,A,";
  s += String(xteNm, 2); s += ",";
  s += steerDir; s += ",N,V,V,";
  s += String(s_targetBearingDeg, 1); s += ",T,,";
  s += String(s_targetBearingDeg, 1); s += ",T,";
  s += String(s_targetBearingDeg, 1); s += ",T";
  sendSentence(s);
}

static void sendRMB() {
  if (!s_haveNav) return;
  float xteNm = fabs(s_xtrackErrorM) / 1852.0f;
  char steerDir = (s_xtrackErrorM < 0) ? 'L' : 'R';
  float distNm = s_wpDistM / 1852.0f;

  String s = "$GPRMB,A,";
  s += String(xteNm, 2); s += ",";
  s += steerDir; s += ",,,,,,,,";
  s += String(distNm, 2); s += ",";
  s += String(s_targetBearingDeg, 1); s += ",,A";
  sendSentence(s);
}

// ---------------------------------------------------------------------------
// MAVLink message handling
// ---------------------------------------------------------------------------

void handleMavMessage(const mavlink_message_t &msg) {
  Serial.println("Handling Message for bridge");
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t p;
      mavlink_msg_global_position_int_decode(&msg, &p);
      s_lat_e7 = p.lat;
      s_lon_e7 = p.lon;
      // vx/vy are cm/s NED -> derive ground speed + course
      float vx = p.vx / 100.0f, vy = p.vy / 100.0f;
      s_groundSpeedMs = sqrtf(vx * vx + vy * vy);
      float courseRad = atan2f(vy, vx); // 0 = north, clockwise
      s_courseDeg = courseRad * RAD_TO_DEG;
      if (s_courseDeg < 0) s_courseDeg += 360.0f;
      s_haveGps = true;
      Serial.println("GLOBAL_POSITION_INT");
      break;     
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_gps_raw_int_t g;
      mavlink_msg_gps_raw_int_decode(&msg, &g);
      s_fixType = g.fix_type;
      s_satsVisible = g.satellites_visible;
      if (g.time_usec != 0) s_timeUnixUsec = g.time_usec;
      // Prefer GLOBAL_POSITION_INT for lat/lon/course if already set;
      // fall back to this message if it arrives first.
      if (!s_haveGps) {
        s_lat_e7 = g.lat;
        s_lon_e7 = g.lon;
        s_groundSpeedMs = g.vel / 100.0f; // cm/s
        s_courseDeg = g.cog / 100.0f;     // centidegrees
        s_haveGps = true;
      }
      Serial.println("GPS_RAW_INT");
      break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME: {
      mavlink_system_time_t t;
      mavlink_msg_system_time_decode(&msg, &t);
      if (t.time_unix_usec != 0) s_timeUnixUsec = t.time_unix_usec;
      Serial.println("SYSTEM_TIME");
      break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t v;
      mavlink_msg_vfr_hud_decode(&msg, &v);
      s_headingMagDeg = v.heading; // degrees, 0-360
      s_haveHeading = true;
      Serial.println("VFR_HUD");
      break;
    }
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
      mavlink_nav_controller_output_t n;
      mavlink_msg_nav_controller_output_decode(&msg, &n);
      s_wpDistM = n.wp_dist;
      s_targetBearingDeg = n.target_bearing;
      s_xtrackErrorM = n.xtrack_error;
      s_haveNav = true;
      Serial.println("NAV_CONTROLLER_OUTPUT");
      break;
    }
    default:
      break; // everything else (RAW_IMU, RC channels, STATUSTEXT, ...) ignored
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void mavNmeaBridge_setup(HardwareSerial &mavSerial, WiFiUDP &udp, IPAddress broadcastIP, uint16_t udpPort) {
  Serial.println("Setting up NMEA Bridge");
  s_mavSerial = &mavSerial;
  s_broadcastIP = broadcastIP;
  s_udpPort = udpPort;
  s_udp = udp;
}

void mavNmeaBridge_setDeclination(float declinationDeg) {
  s_declinationDeg = declinationDeg;
}

void mavNmeaBridge_setRates(uint32_t positionMs, uint32_t headingMs, uint32_t navMs) {
  s_positionRateMs = positionMs;
  s_headingRateMs  = headingMs;
  s_navRateMs      = navMs;
}

void mavNmeaBridge_update() {

  uint32_t now = millis();

  if (now - s_lastPositionSend >= s_positionRateMs) {
    s_lastPositionSend = now;
    sendGGA();
    sendRMC();
    sendZDA();
  }

  if (now - s_lastHeadingSend >= s_headingRateMs) {
    s_lastHeadingSend = now;
    sendHDM();
    sendHDT();
  }

  if (now - s_lastNavSend >= s_navRateMs) {
    s_lastNavSend = now;
    sendAPB();
    sendRMB();
  }
}
