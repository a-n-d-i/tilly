PlatformIO project. Build/upload with `pio run` / `pio run -t upload`, or the
PlatformIO IDE extension. Copy include/config.h.example to include/config.h
and fill in WiFi credentials before building.

The tuning/live-telemetry dashboard (data/index.html) is served straight off
the ESP32 at http://<device-ip>/, with telemetry over a WebSocket on port 81
(auto-connects, no setup). It lives on the SPIFFS filesystem partition, so
after changing data/index.html it needs its own upload step, separate from
the firmware:
    pio run -t uploadfs

Connect serial while OTA updating:
minicom -D /dev/ttyUSB1 -b 115200



Hardware: Waveshare ESP32-S3-RLCD-4.2 ("eLCD"), a 400x300 reflective ST7305
LCD driven over SPI via U8g2, with a PCF8574T I2C button expander (6 buttons,
INT-driven). See include/board_pins.h for the full pin map. The ArduPilot
MAVLink link (UART2) is wired to the board's P1 expansion header on GPIO18
(RX) / GPIO3 (TX) - swap if it comes up backwards.

Buttons (silkscreen numbering): 1 Auto, 2 Standby, 3 -1, 4 +1, 5 -10, 6 +10.




Connect Tilly to Buttonbox:

Tilly Button
Black White GND
Red Brown + 
Orange Green 
yellow yellow



Development: Set the SITL SERIALX Parameters to serial6, 115200

start SITL and forward the serial port of the simulation to the buttonbox
sim_vehicle.py -v Rover -f sailboat -L ANDI -A  "--serial6=uart:/dev/ttyUSB1" --console --map
