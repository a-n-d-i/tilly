/*
  RC Linear Actuator Position Controller - ESP32 DevKit, RMT hardware capture
  -------------------------------------------------------------------------------

  - Reads RC PWM input (1000-2000us) using the ESP32 RMT peripheral in RX mode.
    This is true hardware pulse timing - the RMT unit timestamps edges in
    dedicated silicon, independent of CPU load/interrupt jitter, closest
    ESP32 equivalent to the AVR ICP1 input-capture approach.
  - Reads ADS1115 ADC (I2C) for linear potentiometer position feedback
  - Drives RC ESC (bidirectional, servo-PWM controlled) via ESP32Servo
    (uses the LEDC hardware PWM peripheral)
  - Ramped speed control (accel-limited) to move actuator to commanded position

  COMPATIBILITY NOTE:
  Ported to the new `driver/rmt_rx.h` API (ESP-IDF 5 / arduino-esp32 core 3.x).
  The RMT peripheral is now handled through channel handles and an
  install-once, re-arm-per-frame receive model, rather than the old
  continuous ring buffer. A frame is captured into `rawRmtSymbols`, handed
  off to the main loop through a small FreeRTOS queue from the RX-done
  callback (which runs in ISR context), and the channel is immediately
  re-armed with `rmt_receive()` so the next pulse is not missed. This will
  NOT compile against arduino-esp32 core 2.x (classic `driver/rmt.h`).

  Hardware assumptions:
  - Linear potentiometer: 5 kOhm, 250 mm travel, wired as a voltage divider,
    wiper into ADS1115 A0
  - Max actuator travel speed: 50 mm/s
  - RC input signal on GPIO 16 (RMT RX works on essentially any GPIO)
  - ESC signal output on GPIO 18
  - I2C: SDA = GPIO 21, SCL = GPIO 22 (ESP32 devkit defaults)

  Libraries required (install via Library Manager):
  - ESP32Servo
  - Adafruit_ADS1X15
  (driver/rmt_rx.h is part of the ESP-IDF bundled with arduino-esp32 core
   3.x, no separate install needed)

  *** CALIBRATE BEFORE USE ***
  - ADC_COUNTS_AT_0MM / ADC_COUNTS_AT_250MM: raw ADS1115 readings taken with
    the actuator physically at each end of travel - measure and set.
  - ESC_NEUTRAL_US / ESC_MIN_US / ESC_MAX_US: match your ESC's actual range.
*/

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ---------- Pin definitions ----------
#define RC_INPUT_PIN   GPIO_NUM_16
#define ESC_OUTPUT_PIN 18
#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22

// ---------- RMT configuration ----------
#define RMT_RESOLUTION_HZ    1000000  // 1MHz -> 1 tick = 1us
#define RMT_MEM_BLOCK_SYMS   64       // symbol memory block size (ESP32 granularity)
#define RMT_MIN_NS           1250     // filters glitches shorter than ~1.25us
#define RMT_IDLE_NS          12000000 // 12ms of continuous level ends a capture

// ---------- RC input calibration ----------
const uint16_t RC_MIN_US = 999;
const uint16_t RC_MAX_US = 2000;

// ---------- Actuator / travel parameters ----------
const float TRAVEL_MM       = 250.0;  // total travel
const float MAX_SPEED_MMS   = 55.0;   // max travel speed, mm/s
// const float MAX_ACCEL_MMSS  = 150.0;  // ramp rate, mm/s^2 -- tune to taste
const float MAX_ACCEL_MMSS  = 300.0;  // ramp rate, mm/s^2 -- tune to taste

const float POS_DEADBAND_MM = 1.0;    // stop once within this of target


// ---------- ADS1115 calibration (PLACEHOLDERS - measure and set!) ----------
int16_t ADC_COUNTS_AT_0MM   = 0;      // raw reading at 0mm end-stop
int16_t ADC_COUNTS_AT_250MM = 17000;  // raw reading at 250mm end-stop

// ---------- ESC output calibration ----------
const uint16_t ESC_NEUTRAL_US = 1500;
const uint16_t ESC_MIN_US     = 1000;
const uint16_t ESC_MAX_US     = 2000;

// ---------- Control loop timing ----------
const uint16_t LOOP_INTERVAL_MS = 20; // 50 Hz

// ================= Globals =================
Servo esc;
Adafruit_ADS1115 ads;

rmt_channel_handle_t rmtRxChannel = NULL;
QueueHandle_t rmtRxQueue = NULL;
rmt_symbol_word_t rawRmtSymbols[RMT_MEM_BLOCK_SYMS];
rmt_receive_config_t rmtReceiveConfig = {};

uint16_t rcPulseWidth_us = 1500;
unsigned long rcLastPulseMillis = 0;

float currentSpeed_mms = 0.0;   // ramped/actual commanded speed
float currentPos_mm = 0.0;      // last measured position
unsigned long lastLoopMillis = 0;

// ---------------------------------------------------------
// RMT RX setup (hardware pulse capture) - new driver/rmt_rx.h API
// ---------------------------------------------------------

// Runs in ISR context: just hand the completed frame off to the main loop.
static bool IRAM_ATTR onRmtRxDone(rmt_channel_handle_t channel,
                                   const rmt_rx_done_event_data_t *edata,
                                   void *user_data) {
  BaseType_t highTaskWakeup = pdFALSE;
  xQueueSendFromISR(rmtRxQueue, edata, &highTaskWakeup);
  return highTaskWakeup == pdTRUE;
}

void setupRMTCapture() {
  rmt_rx_channel_config_t rxChannelCfg = {};
  rxChannelCfg.gpio_num = RC_INPUT_PIN;
  rxChannelCfg.clk_src = RMT_CLK_SRC_DEFAULT;
  rxChannelCfg.resolution_hz = RMT_RESOLUTION_HZ;
  rxChannelCfg.mem_block_symbols = RMT_MEM_BLOCK_SYMS;

  rmt_new_rx_channel(&rxChannelCfg, &rmtRxChannel);

  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = onRmtRxDone;
  rmtRxQueue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
  rmt_rx_register_event_callbacks(rmtRxChannel, &cbs, NULL);

  rmt_enable(rmtRxChannel);

  rmtReceiveConfig.signal_range_min_ns = RMT_MIN_NS;
  rmtReceiveConfig.signal_range_max_ns = RMT_IDLE_NS;

  // Arm the first capture; every subsequent capture is re-armed in
  // pollRMTCapture() right after a frame is consumed.
  rmt_receive(rmtRxChannel, rawRmtSymbols, sizeof(rawRmtSymbols), &rmtReceiveConfig);
}

// Non-blocking poll of the RMT done-queue. Call every loop iteration.
void pollRMTCapture() {
  rmt_rx_done_event_data_t rxData;
  if (xQueueReceive(rmtRxQueue, &rxData, 0) != pdTRUE) return;

  for (size_t i = 0; i < rxData.num_symbols; i++) {
    if (rxData.received_symbols[i].level0 == 1) {
      // duration0 = HIGH time in us (resolution_hz gives 1 tick = 1us)
      uint16_t pulse_us = rxData.received_symbols[i].duration0;
      if (pulse_us >= 800 && pulse_us <= 2200) { // sanity filter
        rcPulseWidth_us = pulse_us;
        rcLastPulseMillis = millis();
      }
      break; // only one pulse expected per RC frame
    }
  }

  // Re-arm for the next frame using the same symbol buffer.
  rmt_receive(rmtRxChannel, rawRmtSymbols, sizeof(rawRmtSymbols), &rmtReceiveConfig);
}

// ---------------------------------------------------------
// Helpers
// ---------------------------------------------------------

// Map RC pulse width to a target position (0..TRAVEL_MM)
float rcToTargetPosition(uint16_t pulse_us) {
  pulse_us = constrain(pulse_us, RC_MIN_US, RC_MAX_US);
  return (float)(pulse_us - RC_MIN_US) * TRAVEL_MM / (float)(RC_MAX_US - RC_MIN_US);
}

// Read ADS1115 and convert to position in mm
float readPositionMM() {
  int16_t raw = ads.readADC_SingleEnded(0);
  float pos = (float)(raw - ADC_COUNTS_AT_0MM) * TRAVEL_MM /
              (float)(ADC_COUNTS_AT_250MM - ADC_COUNTS_AT_0MM);
  return constrain(pos, 0.0, TRAVEL_MM);
}

// Convert a speed command (-MAX_SPEED_MMS .. +MAX_SPEED_MMS) to an ESC pulse width
uint16_t speedToESC_us(float speed_mms) {
  speed_mms = constrain(speed_mms, -MAX_SPEED_MMS, MAX_SPEED_MMS);
  if (speed_mms >= 0) {
    return ESC_NEUTRAL_US + (uint16_t)((speed_mms / MAX_SPEED_MMS) * (ESC_MAX_US - ESC_NEUTRAL_US));
  } else {
    return ESC_NEUTRAL_US - (uint16_t)((-speed_mms / MAX_SPEED_MMS) * (ESC_NEUTRAL_US - ESC_MIN_US));
  }
}

// ---------------------------------------------------------
// Setup
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);

  setupRMTCapture();

  ESP32PWM::allocateTimer(0);
  esc.setPeriodHertz(50);
  esc.attach(ESC_OUTPUT_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(ESC_NEUTRAL_US); // arm ESC at neutral
  delay(2000);                           // give ESC time to arm

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!ads.begin()) {
    Serial.println("ADS1115 not found - check wiring!");
    while (1) delay(100);
  }
  ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range, safe headroom for a 5V pot supply

  currentPos_mm = readPositionMM();
  lastLoopMillis = millis();
}

// ---------------------------------------------------------
// Main loop
// ---------------------------------------------------------
void loop() {
  pollRMTCapture(); // always service the RMT done-queue

  unsigned long now = millis();
  if (now - lastLoopMillis < LOOP_INTERVAL_MS) return;
  float dt = (now - lastLoopMillis) / 1000.0;
  lastLoopMillis = now;

  // Failsafe: if no RC pulses received for 200ms, force stop
  bool rcTimeout = (now - rcLastPulseMillis) > 200;

  float targetPos_mm = rcToTargetPosition(rcPulseWidth_us);
  currentPos_mm = readPositionMM();

  float error_mm = targetPos_mm - currentPos_mm;

  float desiredSpeed_mms;
  if (rcTimeout || fabs(error_mm) <= POS_DEADBAND_MM) {
    desiredSpeed_mms = 0.0;
  } else {
    // Simple proportional speed request toward target, capped at max speed.
    // This also gives natural slow-down near the target (reduces overshoot).
    const float kP = 5.0; // mm/s per mm of error - tune as needed
    desiredSpeed_mms = constrain(error_mm * kP, -MAX_SPEED_MMS, MAX_SPEED_MMS);
    if (desiredSpeed_mms < 0) {
      desiredSpeed_mms = constrain(desiredSpeed_mms, -MAX_SPEED_MMS, -20);
      }


    if (desiredSpeed_mms > 0) {
      desiredSpeed_mms = constrain(desiredSpeed_mms, 20, MAX_SPEED_MMS);
      }
    }


  // Ramp currentSpeed_mms toward desiredSpeed_mms, limited by MAX_ACCEL_MMSS
  float maxDeltaV = MAX_ACCEL_MMSS * dt;
  float speedError = constrain(desiredSpeed_mms - currentSpeed_mms, -maxDeltaV, maxDeltaV);
  currentSpeed_mms += speedError;

  uint16_t escPulse = speedToESC_us(currentSpeed_mms);
  esc.writeMicroseconds(escPulse);

  // Debug output - remove/reduce once calibrated
  Serial.print("RC_us:");       Serial.print(rcPulseWidth_us);
  Serial.print(" Target_mm:");  Serial.print(targetPos_mm);
  Serial.print(" Pos_mm:");     Serial.print(currentPos_mm);
  Serial.print(" Speed_mms:");  Serial.print(currentSpeed_mms);
  Serial.print(" ESC_us:");     Serial.println(escPulse);
}
