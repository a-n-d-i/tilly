Connect serial while OTA updating:
minicom -D /dev/ttyUSB1 -b 115200



Pinout


| Shared With          | I/O  | I/O                  | Shared With          |
|----------------------|------|----------------------| -------------------- |
| n/a                  | 3.3V | GND                  | n/a                  |
| NC/XTAL              | IO32 | IO33                 | NC/XTAL              |
| Auto, Standby        | IO12 | IO13                 | JTAG, microSD        |
| +1 -1                | IO14 | IO27                 | Camera               |
| +10                  | IO26 | IO25                 | Camera, LCD          |
| nn/RX(green)          | IO34 | IO35                 | Camera               |
| Camera               | IO39 | IO36                 | Camera               |
| JTAG                 | EN   | IO23                 | Camera, LCD          |
| -10, LCD             | IO22 | IO21                 | Camera, LCD, microSD |
| Camera, LCD          | IO19 | IO18                 | Camera, LCD          |
| IO5                  | IO17 | PSRAM                | PSRAM                |
| IO16                 | IO4  | LED, Camera, microSD | Camera, LED, Boot    |
| IO0                  | IO2  | LED, microSD         | JTAG, microSD        |
| IO15         (brown) | 5V   |                      |                      |

IO19 tx, fix table...




Connect Tilly to Buttonbox:

Tilly Button
Black White GND
Orange Green 
yellow yellow



Development: Set the SITL SERIALX Parameters to serial6, 115200

start SITL and forward the serial port of the simulation to the buttonbox
sim_vehicle.py -v Rover -f sailboat -L ANDI -A  "--serial6=uart:/dev/ttyUSB1" --console --map
