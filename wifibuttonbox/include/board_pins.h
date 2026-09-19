#pragma once

// Waveshare ESP32-S3-RLCD-4.2 ("eLCD-4.2") pin map for the wifibuttonbox
// hardware. Display + I2C button-expander pins per the board's own
// examples: https://github.com/waveshareteam/ESP32-S3-RLCD-4.2

// ST7305 reflective LCD, driven over SPI.
#define RLCD_WIDTH    400
#define RLCD_HEIGHT   300
#define RLCD_SCK_PIN  11
#define RLCD_MOSI_PIN 12
#define RLCD_DC_PIN   5
#define RLCD_CS_PIN   40
#define RLCD_RST_PIN  41

// I2C bus shared with the board's onboard RTC/sensors.
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 14

// PCF8574T button expander: 6 buttons on P0-P5, active low, INT on GPIO17.
#define PCF8574_ADDR    0x20
#define PCF8574_INT_PIN 17
#define BUTTON_COUNT    6

// ArduPilot MAVLink link (UART2), wired to the P1 expansion header.
#define MAVLINK_RX_PIN 18
#define MAVLINK_TX_PIN 3
