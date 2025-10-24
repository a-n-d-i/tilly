from pymavlink import mavutil
import time, math

class Tillerpilot:

    m = None

    def connect(self, url='udp:127.0.0.1:14550'):
        self.m = mavutil.mavlink_connection(url)
        self.m.wait_heartbeat()
        print("Heatbeat received")

    def send_heading(self, hdg_deg):
        yaw = math.radians(hdg_deg)
        self.m.mav.set_position_target_local_ned_send(
            int(time.time()),
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            # ignore xyz, accel, yaw_rate; use vx and yaw
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
            0,0,0,                    # x,y,z (ignored)
            0,0,0,                # vx,vy,vz (vy unused)
            0,0,0,                    # ax,ay,az (ignored)
            yaw,0)                    # yaw, yaw_rate

        # maybe submode heading+speed instead?
        # maybe mode simple?


    def init(self):
        self.connect()
        # Get GLOBAL_POSITION_INT message at 2Hz
        self.request_message_interval(33, 1000000)

    def arm(self):
        # ARM as in RC speak
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, # confirmation
            1, # param1 (0 to indicate disarm)
            0, # param2 (all other params meaningless)
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7

    def disarm(self):
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, # confirmation
            0, # param1 (0 to indicate disarm)
            0, # param2 (all other params meaningless)
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0) # param7


    def set_mode(self, mode):
        """
        GUIDED mode means the tilerpilot runs the control loop and actuates the rudder, while this app is
        only pointing it in a direction.
        MANUAL is regular RC operation, move sticks to move servos
        """
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: mode flag
            mode,  # param2: custom mode (15 = GUIDED for Sailboat)
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7

        # Wait for mode change acknowledgment
        ack = self.m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Mode changed to GUIDED")
            else:
                print(f"Mode change failed with result: {ack.result}")
        else:
            print("No acknowledgment received")

    def set_auto_mode(self):
        # connect servo to control loop, 26 is GroundSteering
        self.set_parameter("SERVO1_FUNCTION", 26, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        self.arm()
        # 15 is guided mode for rover
        self.set_mode(15)

    def set_standby_mode(self):
        """
        # there's a bunch of different ways to do this
        # This sets a position and the servo stays there, don't try this with a open loop actuator
        # Since we can't move servos which are used in the control loop, we have to detach the servo from
        # the loop and disarm
        """
        # 0 is manual mode
        self.set_mode(0)
        self.disarm()
        # disconnect servo from loop
        self.set_parameter("SERVO1_FUNCTION", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)

    def request_message_interval(self, message_id, interval_us):
        """
        Mavlink lets us specify which information is sent back to us at which interval
        Very handy to extract IMU data.

        :param message_id:
        :param interval_us:
        :return:
        """
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            message_id,  # param1: Message ID
            interval_us,  # param2: Interval in microseconds
            0, 0, 0, 0, 0  # param3-7: unused
        )
        print(f"Requested message {message_id} at {interval_us/1000000}Hz")

    def get_gps_position(self):
        """
        Get the current GPS position from GLOBAL_POSITION_INT message
        :return: Dictionary with lat, lon, alt or None
        """
        msg = self.m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            return {
                'lat': msg.lat / 1e7,  # Convert from int (degE7) to float degrees
                'lon': msg.lon / 1e7,  # Convert from int (degE7) to float degrees
                'alt': msg.alt / 1000.0,  # Convert from mm to meters
                'relative_alt': msg.relative_alt / 1000.0,  # Alt above ground in meters
                'vx': msg.vx / 100.0,  # Velocity in m/s
                'vy': msg.vy / 100.0,
                'vz': msg.vz / 100.0,
                'hdg': msg.hdg / 100.0  # Heading in degrees
            }
        return None



    def move_servo_absolute(self, increment):
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # confirmation
            1,  # param1: Servo instance (1 for servo1)
            increment,  # param2: PWM value
            0, 0, 0, 0, 0  # param3-7: unused
        )


    def set_parameter(self, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        print(f"Setting parameter: {param_name} = {param_value}")
        self.m.mav.param_set_send(
            self.m.target_system,
            self.m.target_component,
            param_name.encode('utf-8'),
            param_value,
            param_type
        )



