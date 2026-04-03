# Connections
Raspi - FC
5V from BEC PIN?
TX/RX -> RX5 TX5


Set FC SERIAL.SERIAL5 parameters to 115100 Mavlink2 to 
communicate with the Pi via ttySO serial 

# Serial Ports

| Port     | Function      | Speed    | Protocol      |
|----------|---------------|----------|---------------|
| Serial 1 | ELRS/TBS recv | ---      | RC Controller |
| Serial 2 | free          | Row 2    | Row 2         |
| Serial 3 | BD GPS        | 230400   | GPS           |
| Serial 4 | RTK GPS       | 115200   | GPS           |
| Serial 5 | Buttonbox     | 115200   | MAVlink2      |
| Serial 6 | WIFI/BT       | 115200   | MAVlink2      |