// ============================================================================
// web_telemetry.cpp
// See web_telemetry.h for usage. Mirrors tilly_telemetry_backend.py's
// Telemetry state + WebSocket broadcast, but reads MAVLink off the same
// ArduPilotSerial stream the rest of the sketch already parses, instead of
// a second, independent MAVLink connection.
// ============================================================================

#include "web_telemetry.h"

#include <SPIFFS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "applog.h"

static const uint16_t HTTP_PORT = 80;
static const uint16_t WS_PORT = 81;
static const uint32_t BROADCAST_INTERVAL_MS = 100;   // 10 Hz, matches the python backend's default
static const uint32_t PARAM_POLL_INTERVAL_MS = 5000; // matches --param-poll default
static const uint8_t GCS_PID_MASK_STEERING = 1;        // bit 0 = Rover steering axis

static WebServer s_http(HTTP_PORT);
static WebSocketsServer s_ws(WS_PORT);
static HardwareSerial *s_mavSerial = nullptr;

// The 17 tuning parameters the dashboard's Signal Flow tab knows about -
// kept in sync with TRACKED_PARAMS in tilly_telemetry_backend.py.
static const char *const TRACKED_PARAMS[] = {
  "ACRO_TURN_RATE", "ATC_STR_ANG_P", "ATC_STR_RAT_MAX", "TURN_RADIUS",
  "ATC_STR_ACC_MAX", "ATC_STR_DEC_MAX",
  "ATC_STR_RAT_FLTT", "ATC_STR_RAT_FLTE", "ATC_STR_RAT_FLTD",
  "ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_D",
  "ATC_STR_RAT_FF", "ATC_STR_RAT_D_FF",
  "ATC_STR_RAT_IMAX", "ATC_STR_RAT_PDMX", "ATC_STR_RAT_SMAX",
};
static const size_t NUM_TRACKED_PARAMS = sizeof(TRACKED_PARAMS) / sizeof(TRACKED_PARAMS[0]);

struct TrackedParam {
  bool have;
  float value;
};
static TrackedParam s_params[NUM_TRACKED_PARAMS];

static int trackedParamIndex(const char *name) {
  for (size_t i = 0; i < NUM_TRACKED_PARAMS; i++) {
    if (strcmp(TRACKED_PARAMS[i], name) == 0) return (int)i;
  }
  return -1;
}

// ---------------------------------------------------------------------------
// Telemetry state - updated from webTelemetry_handleMavMessage(), snapshotted
// into JSON on each broadcast tick.
// ---------------------------------------------------------------------------
static unsigned long s_t0Ms = 0;
static unsigned long s_lastMavMessageMs = 0;
static float s_yawDeg = 0.0f;
static float s_heelDeg = 0.0f;
static float s_speedKn = 0.0f;
static float s_pidTar = 0.0f, s_pidAct = 0.0f;
static float s_pidP = 0.0f, s_pidI = 0.0f, s_pidD = 0.0f, s_pidFF = 0.0f;
static uint16_t s_rcoutUs = 1500;
static bool s_haveOrigin = false;
static double s_originLat = 0.0, s_originLon = 0.0;
static float s_northM = 0.0f, s_eastM = 0.0f;

static unsigned long s_lastBroadcastMs = 0;
static unsigned long s_lastParamPollMs = 0;

// ---------------------------------------------------------------------------
// MAVLink senders
// ---------------------------------------------------------------------------

static void sendToVehicle(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  size_t written = s_mavSerial->write(buf, len);
  if (written != len) {
    appLog("[web] MAVLink write short: %u/%u bytes (msgid %u)", (unsigned)written, (unsigned)len, (unsigned)msg.msgid);
  }
}

static void requestParam(const char *name) {
  mavlink_message_t msg;
  mavlink_msg_param_request_read_pack(250, 1, &msg, 1, 1, name, -1);
  sendToVehicle(msg);
}

static void requestTrackedParams() {
  for (size_t i = 0; i < NUM_TRACKED_PARAMS; i++) {
    requestParam(TRACKED_PARAMS[i]);
  }
}

static void setGcsPidMask() {
  mavlink_message_t msg;
  mavlink_msg_param_set_pack(250, 1, &msg, 1, 1, "GCS_PID_MASK",
                              (float)GCS_PID_MASK_STEERING, MAV_PARAM_TYPE_INT32);
  sendToVehicle(msg);
}

// ---------------------------------------------------------------------------
// MAVLink message handling
// ---------------------------------------------------------------------------

void webTelemetry_handleMavMessage(const mavlink_message_t &msg) {
  s_lastMavMessageMs = millis();

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t a;
      mavlink_msg_attitude_decode(&msg, &a);
      s_heelDeg = a.roll * RAD_TO_DEG;
      break;
    }

    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t hud;
      mavlink_msg_vfr_hud_decode(&msg, &hud);
      s_yawDeg = hud.heading;
      s_speedKn = hud.groundspeed * 1.94384f;
      break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t p;
      mavlink_msg_global_position_int_decode(&msg, &p);
      s_speedKn = hypotf(p.vx, p.vy) / 100.0f * 1.94384f;

      double lat = p.lat / 1e7, lon = p.lon / 1e7;
      if (!s_haveOrigin) {
        s_originLat = lat;
        s_originLon = lon;
        s_haveOrigin = true;
      }
      const double mPerDegLat = 111320.0;
      const double mPerDegLon = 111320.0 * cos(s_originLat * DEG_TO_RAD);
      s_northM = (float)((lat - s_originLat) * mPerDegLat);
      s_eastM = (float)((lon - s_originLon) * mPerDegLon);
      break;
    }

    case MAVLINK_MSG_ID_PID_TUNING: {
      mavlink_pid_tuning_t pid;
      mavlink_msg_pid_tuning_decode(&msg, &pid);
      if (pid.axis == PID_TUNING_STEER) {
        s_pidTar = pid.desired;
        s_pidAct = pid.achieved;
        s_pidP = pid.P;
        s_pidI = pid.I;
        s_pidD = pid.D;
        s_pidFF = pid.FF;
      }
      break;
    }

    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
      mavlink_servo_output_raw_t servo;
      mavlink_msg_servo_output_raw_decode(&msg, &servo);
      s_rcoutUs = servo.servo1_raw; // steering is usually servo1 on Rover
      break;
    }

    case MAVLINK_MSG_ID_PARAM_VALUE: {
      mavlink_param_value_t pv;
      mavlink_msg_param_value_decode(&msg, &pv);
      char name[17];
      memcpy(name, pv.param_id, 16);
      name[16] = '\0';
      int idx = trackedParamIndex(name);
      if (idx >= 0) {
        s_params[idx].have = true;
        s_params[idx].value = pv.param_value;
      }
      break;
    }

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// WebSocket broadcast
// ---------------------------------------------------------------------------

// Writes a tuning parameter back to the vehicle. Only names on the
// TRACKED_PARAMS whitelist are accepted - the dashboard's Signal Flow tab
// also has two synthetic "scenario input" fields (err_yaw, speed) that
// aren't real ArduPilot parameters, and this rejects anything else too.
static void handleSetParam(JsonObjectConst msg) {
  const char *name = msg["name"] | "";
  if (trackedParamIndex(name) < 0) {
    appLog("[web] rejected set_param for untracked param: %s", name);
    return;
  }
  if (!msg["value"].is<float>()) {
    appLog("[web] set_param %s missing/invalid value", name);
    return;
  }
  float value = msg["value"];

  mavlink_message_t setMsg;
  mavlink_msg_param_set_pack(250, 1, &setMsg, 1, 1, name, value, MAV_PARAM_TYPE_REAL32);
  sendToVehicle(setMsg);
  appLog("[web] set %s = %.4f", name, value);

  // Re-read right away instead of waiting for the next 5s poll, so the
  // dashboard sees the vehicle's actual (possibly clamped) value quickly.
  requestParam(name);
}

static void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    appLog("[web] client %u connected (%u total)", num, (unsigned)s_ws.connectedClients());
  } else if (type == WStype_DISCONNECTED) {
    appLog("[web] client %u disconnected (%u total)", num, (unsigned)s_ws.connectedClients());
  } else if (type == WStype_TEXT) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
      appLog("[web] bad JSON from client %u: %s", num, err.c_str());
      return;
    }
    const char *msgType = doc["type"] | "";
    if (strcmp(msgType, "set_param") == 0) {
      handleSetParam(doc.as<JsonObjectConst>());
    }
  }
}

static void broadcastTelemetry() {
  bool connected = (millis() - s_lastMavMessageMs) < 3000;

  JsonDocument doc;
  doc["t"] = (millis() - s_t0Ms) / 1000.0;
  doc["connected"] = connected;
  doc["yaw"] = s_yawDeg;
  doc["heel"] = s_heelDeg;
  doc["speed_kn"] = s_speedKn;

  JsonObject pid = doc["pid"].to<JsonObject>();
  pid["tar"] = s_pidTar;
  pid["act"] = s_pidAct;
  pid["P"] = s_pidP;
  pid["I"] = s_pidI;
  pid["D"] = s_pidD;
  pid["FF"] = s_pidFF;

  doc["rcout_us"] = s_rcoutUs;

  JsonObject params = doc["params"].to<JsonObject>();
  for (size_t i = 0; i < NUM_TRACKED_PARAMS; i++) {
    if (s_params[i].have) params[TRACKED_PARAMS[i]] = s_params[i].value;
  }

  JsonObject pos = doc["pos"].to<JsonObject>();
  pos["north"] = s_northM;
  pos["east"] = s_eastM;

  String out;
  serializeJson(doc, out);

  // broadcastTXT() writes to every connected client's TCP socket in turn.
  // A client that's gone dark without a clean disconnect (out of WiFi
  // range, phone locked) can leave lwIP blocking on that one write for a
  // long time waiting for a TCP timeout - since this all runs on the main
  // loop(), that stalls the display/buttons/everything else too. Flagging
  // any broadcast that takes oddly long is meant to catch that in the act.
  unsigned long t0 = millis();
  s_ws.broadcastTXT(out);
  unsigned long dt = millis() - t0;
  if (dt > 50) {
    appLog("[web] broadcastTXT took %lu ms (%u clients)", dt, (unsigned)s_ws.connectedClients());
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void webTelemetry_setup(HardwareSerial &mavSerial) {
  s_mavSerial = &mavSerial;
  s_t0Ms = millis();

  if (!SPIFFS.begin(true)) {
    appLog("[web] SPIFFS mount failed - upload the filesystem image with `pio run -t uploadfs`");
  }

  s_http.serveStatic("/", SPIFFS, "/index.html");
  s_http.begin();
  appLog("[web] HTTP dashboard on port %u", HTTP_PORT);

  s_ws.begin();
  s_ws.onEvent(wsEvent);
  // A client that drops off WiFi without a clean close still looks
  // "connected" to the socket layer, so broadcastTelemetry() keeps trying
  // to write to it - and blocking the shared main loop() for up to
  // WEBSOCKETS_TCP_TIMEOUT (see platformio.ini) - on every single 100ms
  // tick until something else notices. Heartbeat pings it every 5s and
  // force-disconnects after 2 missed pongs, so the dead client is dropped
  // (and stops being retried) instead of stalling the board indefinitely.
  s_ws.enableHeartbeat(5000, 2000, 2);
  appLog("[web] telemetry WebSocket on port %u", WS_PORT);

  setGcsPidMask();
  requestTrackedParams();
  s_lastParamPollMs = millis();
}

void webTelemetry_update() {
  unsigned long t0 = millis();
  s_http.handleClient();
  unsigned long dt = millis() - t0;
  if (dt > 50) appLog("[web] handleClient took %lu ms", dt);

  t0 = millis();
  s_ws.loop();
  dt = millis() - t0;
  if (dt > 50) appLog("[web] ws.loop took %lu ms", dt);

  unsigned long now = millis();

  if (now - s_lastParamPollMs > PARAM_POLL_INTERVAL_MS) {
    requestTrackedParams();
    s_lastParamPollMs = now;
  }

  if (now - s_lastBroadcastMs > BROADCAST_INTERVAL_MS) {
    s_lastBroadcastMs = now;
    broadcastTelemetry();
  }
}
