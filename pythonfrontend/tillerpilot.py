from pymavlink import mavutil
import time, math

# TODO: Set gps filtering to something like boat or pedestriean
# TODO: smartrtl deaktivieren


#url='udp:127.0.0.1:14550'
#url='udp:127.0.0.1:14552'
url='/dev/ttyACM0'

class Tillerpilot:

    m = None
    def connect(self, url):
        self.m = mavutil.mavlink_connection(url)
        self.m.wait_heartbeat()
        print("Heatbeat received")

    def send_heading(self, hdg_deg):
        # MAV_CMD_CONDITION_YAW is not supported by rover
        yaw = math.radians(hdg_deg)

        #print("sending heading %s" % hdg_deg)

        # This needs ekf position/origin, doesnt work for indoor testing and without gps fix

        self.m.mav.set_position_target_local_ned_send(
            int(time.time()),
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            # ignore xyz, accel, yaw_rate; just use yaw
            0b100111111111,
            0,0,0,                    # x,y,z (ignored)
            0,0,0,                # vx,vy,vz (vy unused)
            0,0,0,                    # ax,ay,az (ignored)
            yaw,0)                    # yaw, yaw_rate


        # maybe submode heading+speed instead? nope, only accessible via lua, not mavlink
        # mavutil.mavlink.MAV_CMD_CONDITION_YAW not supported
        # maybe mode simple?


    def init(self):
        # TODO: set parameters at startup
        # sim wind/wave
        self.connect(url)
        self.request_message_interval(33, 1000000)
        self.arm()

    def arm(self):
        # ARM as in RC speak
        # self.m.arducopter_arm()
        # self.m.motors_armed_wait()

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


    def set_mode(self, mode, submode):
        """
        GUIDED mode means the tilerpilot runs the control loop and actuates the rudder, while this app is
        only pointing it in a direction.
        MANUAL is regular RC operation, move sticks to move servos
        """
        self.arm()
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: mode flag
            mode,  # param2: custom mode (15 = GUIDED for Sailboat)
            submode,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7

        # Wait for mode change acknowledgment
        ack = self.m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Mode changed ")
            else:
                print(f"Mode change failed with result: {ack.result}")
        else:
            print("No acknowledgment received")

    def set_auto_mode(self):
        self.set_mode(15,1)

    def set_standby_mode(self):
        # 0 is manual mode, 1 acro
        self.set_mode(0,0)


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

    def set_yaw_manual_control(self, yaw_percent):
        """
        Set yaw (rudder) using manual control

        :param yaw_percent: Yaw control -100 to +100 percent
                           -100 = full left rudder
                             0 = center
                           +100 = full right rudder
        """
        # Clamp to valid range
        yaw_percent = max(-100, min(100, yaw_percent))

        # Convert percent to -1000 to +1000 range
        yaw_value = int(yaw_percent * 10)

        print(f"Manual Control: Setting yaw to {yaw_percent}% ({yaw_value})")

        # Send manual control message
        self.m.mav.manual_control_send(
            self.m.target_system,
            0,           # x (pitch) - not used
            0,           # y (roll) - not used
            500,         # z (throttle) - neutral/maintain current
            yaw_value,   # r (yaw/rudder) - -1000 to +1000
            0            # buttons - not used
        )



    def set_servo1_rc_override(self, pwm_value):
        """
        Override servo1 using RC channels override
        This bypasses all autopilot control loops

        :param pwm_value: PWM value 1000-2000, or 0/65535 to release control
        """
        # Clamp to valid PWM range (or allow 0/65535 for release)
        if pwm_value != 0 and pwm_value != 65535:
            pwm_value = max(1000, min(2000, pwm_value))

        print(f"RC Override: Setting servo1 to {pwm_value} PWM")

        # Send RC override command
        # Most systems use channel 1 for servo1
        # this sends a raw command to the channel, not the servo. Expo, deadband, ... seem to apply
        self.m.mav.rc_channels_override_send(
            self.m.target_system,
            self.m.target_component,
            pwm_value,  # channel 1 (servo1)
            0,          # channel 2 (0 = no change)
            0,          # channel 3
            0,          # channel 4
            0,          # channel 5
            0,          # channel 6
            0,          # channel 7
            0           # channel 8
        )


    def move_servo_absolute(self, position):
        self.set_servo1_rc_override(position)


    def set_parameter(self, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        print(f"Setting parameter: {param_name} = {param_value}")
        self.m.mav.param_set_send(
            self.m.target_system,
            self.m.target_component,
            param_name.encode('utf-8'),
            param_value,
            param_type
        )

