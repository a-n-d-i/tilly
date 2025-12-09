#!/usr/bin/env python3
from pymavlink import mavutil

def main():
    # --- CONNECT TO VEHICLE ---
    # Change to your connection: "udp:127.0.0.1:14550" or serial: "/dev/ttyACM0"
    connection_string = "/dev/ttyACM0"

    print(f"Connecting to MAVLink on {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)

    # Wait until a heartbeat is received to confirm connection
    master.wait_heartbeat()
    print("Heartbeat received. Connected.")

    # --- REQUEST RAW_IMU STREAM ---
    # MAV_DATA_STREAM_ALL = 0, raw sensors included
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        10,     # Hz
        1       # Start streaming
    )

    print("Requested RAW_IMU data stream...")

    # --- READ AND DISPLAY COMPASS DATA ---
    while True:
        msg = master.recv_match(type='RAW_IMU', blocking=True)
        if not msg:
            continue

        # RAW_IMU fields:
        # xmag, ymag, zmag   → raw magnetometer readings (milliGauss)
        print(f"Mag X:{msg.xmag}  Y:{msg.ymag}  Z:{msg.zmag}")

if __name__ == "__main__":
    main()