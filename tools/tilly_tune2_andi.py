#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from collections import deque
import threading
import socket
import time
from pymavlink import mavutil

# ============================================================================
# CONFIGURATION
# ============================================================================

MAVLINK_CONNECTION = 'udpin:127.0.0.1:14550'
UDP_SENSOR_PORT = 42424
UDP_SENSOR_HOST = '127.0.0.1'
BUFFER_SIZE = 3000
SAMPLING_RATE = 50.0  # Hz
UPDATE_INTERVAL = 50  # milliseconds

# MAVLink Parameter Names
PARAM_NAMES = [
    'ATC_STR_RAT_P',
    'ATC_STR_RAT_I',
    'ATC_STR_RAT_D',
    'ATC_STR_RAT_FF',
    'ATC_STR_RAT_D_FF',
    'ATC_STR_RAT_IMAX',
    'ATC_STR_RAT_PDMX',
    'ATC_STR_RAT_SMAX',
    'ATC_STR_RAT_MAX',
    'ATC_STR_ACC_MAX',
]

# Parameter ranges for sliders
PARAM_RANGES = {
    'ATC_STR_RAT_P': (0.0, 1.0),
    'ATC_STR_RAT_I': (0.0, 1.0),
    'ATC_STR_RAT_D': (0.0, 0.1),
    'ATC_STR_RAT_FF': (0.0, 1.0),
    'ATC_STR_RAT_D_FF': (0.0, 0.1),
    'ATC_STR_RAT_IMAX': (0.0, 5.0),
    'ATC_STR_RAT_PDMX': (0.0, 1.0),
    'ATC_STR_RAT_SMAX': (0.0, 1.0),
    'ATC_STR_RAT_MAX': (0.0, 180.0),
    'ATC_STR_ACC_MAX': (0.0, 180.0),
}

# ============================================================================
# DATA STORAGE
# ============================================================================

sensor_data = {
    'time': deque(maxlen=BUFFER_SIZE),
    'target_pos': deque(maxlen=BUFFER_SIZE),
    'current_pos': deque(maxlen=BUFFER_SIZE),
    'state': deque(maxlen=BUFFER_SIZE),
    'speed': deque(maxlen=BUFFER_SIZE),
    'servo_raw': deque(maxlen=BUFFER_SIZE),
    'output_pwm': deque(maxlen=BUFFER_SIZE),
}

param_values = {name: 0.0 for name in PARAM_NAMES}
param_sliders = {}
mavlink_master = None
last_sensor_time = time.time()

# ============================================================================
# GUI SETUP
# ============================================================================

fig = plt.figure(figsize=(18, 10))
#fig.suptitle('Tilly Tuning', fontsize=16, fontweight='bold')



# Data Plot
ax_timeseries = fig.add_subplot(1, 1, 1)
line_target, = ax_timeseries.plot([], [], 'b-', linewidth=1.5, label='Target Position')
line_current, = ax_timeseries.plot([], [], 'r-', linewidth=1.5, label='Current Position')
line_servo, = ax_timeseries.plot([], [], 'g-', linewidth=1.5, label='Servo Output (normalized)')
ax_timeseries.set_xlabel('Time (s)')
ax_timeseries.set_ylabel('Position / Output (0-150)')
ax_timeseries.set_title('Steering Control Response')
ax_timeseries.legend(loc='upper left', fontsize=9)
ax_timeseries.grid(True, alpha=0.3)
ax_timeseries.set_ylim(-10, 160)


# Make space for sliders
plt.subplots_adjust(left=0.32, right=0.98, top=0.96, bottom=0.08)

slider_axes = []
for i, param_name in enumerate(PARAM_NAMES):
    ax = fig.add_axes([0.09, 0.95 - (i + 1) * 0.088, 0.15, 0.08])
    slider_axes.append(ax)


# ============================================================================
# MAVLink Functions
# ============================================================================

def mavlink_thread_func():
    """Connect to MAVLink and handle parameter updates"""
    global mavlink_master

    try:
        print(f"Connecting to MAVLink at {MAVLINK_CONNECTION}...")
        mavlink_master = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        mavlink_master.wait_heartbeat()
        print("MAVLink connection established!")

        # Request all parameters
        print("Requesting parameters...")
        for param_name in PARAM_NAMES:
            mavlink_master.param_fetch_one(param_name)
            #request_message_interval(mavlink_master, param_name, 1)
            #time.sleep(0.1)

        next_param_request = time.time() + 1

        # Receive parameter updates
        while True:
            # request params every second
            if time.time() > next_param_request:
                next_param_request = time.time() + 1
                for param_name in PARAM_NAMES:
                    mavlink_master.param_fetch_one(param_name)

            msg = mavlink_master.recv_msg()
            if msg and (msg.msgname == "PARAM_VALUE"):
                # Handle both bytes and string param_id
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode('utf-8').strip()
                else:
                    param_id = param_id.strip()

                if param_id in param_values:
                    param_values[param_id] = msg.param_value
                    # Update slider if it exists
                    if param_id in param_sliders:
                        param_sliders[param_id].set_val(msg.param_value)
                    print(f"Updated {param_id}: {msg.param_value}")
            elif msg and msg.msgname == "SERVO_OUTPUT_RAW":
                # Normalize servo output to 0-150
                servo_normalized = (msg.servo1_raw - 1000) / 1000.0 * 150
                sensor_data['servo_raw'].append(servo_normalized)
                print("received servo outout")
                #if len(sensor_data['time']) > 0:
                #    sensor_data['time'][-1] = current_time


            # check if slider values changed. if so, update parameter
            for param_name, slider in param_sliders.items():
                if slider.val != param_values[param_name]:
                    print("got "+ param_name)
                    param_values[param_name] = slider.val
                    send_parameter(param_name, slider.val)

    except Exception as e:
        print(f"MAVLink error: {e}")
        import traceback
        traceback.print_exc()


def send_parameter(param_name, value):
    """Send parameter update via MAVLink"""
    if mavlink_master:
        try:
            mavlink_master.param_set_send(param_name, value)
            print(f"Sent {param_name} = {value}")
        except Exception as e:
            print(f"Error sending parameter: {e}")


def udp_sensor_thread_func():
    """Receive sensor data from UDP"""
    global last_sensor_time

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_SENSOR_HOST, UDP_SENSOR_PORT))
    print(f"UDP sensor listener started on {UDP_SENSOR_HOST}:{UDP_SENSOR_PORT}")

    start_time = time.time()

    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                current_time = time.time() - start_time

                # Parse CSV: Target Position;Current Position;State;Speed;OutputPWM
                msg = data.decode('utf-8').strip()
                parts = msg.split(';')

                if len(parts) >= 5:
                    target_pos = float(parts[0])
                    current_pos = float(parts[1])
                    state = parts[2].strip()
                    speed = float(parts[3])
                    output_pwm = float(parts[4])

                    sensor_data['time'].append(current_time)
                    sensor_data['target_pos'].append(target_pos)
                    sensor_data['current_pos'].append(current_pos)
                    sensor_data['state'].append(state)
                    sensor_data['speed'].append(speed)
                    sensor_data['output_pwm'].append(output_pwm)
                    last_sensor_time = current_time

            except (ValueError, IndexError) as e:
                print(f"UDP parse error: {e}")
                continue

    except Exception as e:
        print(f"UDP error: {e}")
    finally:
        sock.close()


def compute_fft_servo(servo_signal, fs=SAMPLING_RATE):
    """Compute FFT of servo signal in 0.1-4 Hz range"""
    if len(servo_signal) < 32:
        return np.array([]), np.array([])

    signal_array = np.array(servo_signal)
    signal_detrended = signal_array - np.mean(signal_array)
    window = np.hanning(len(signal_detrended))
    signal_windowed = signal_detrended * window

    fft_mag = np.abs(np.fft.rfft(signal_windowed))
    freq = np.fft.rfftfreq(len(signal_windowed), 1.0 / fs)

    # Filter to 0.1-4 Hz range
    mask = (freq >= 0.1) & (freq <= 4.0)
    return freq[mask], fft_mag[mask]


def update_animation(frame):
    """Update all plots"""
    if len(sensor_data['time']) < 2:
        return

    time_window = slider_time.val
    current_time = sensor_data['time'][-1] if sensor_data['time'] else 0

    # Filter data within time window
    time_array = np.array(sensor_data['time'])
    mask = time_array >= (current_time - time_window)

    times = time_array[mask] - current_time + time_window
    target_pos = np.array(sensor_data['target_pos'])[mask]
    current_pos = np.array(sensor_data['current_pos'])[mask]
    servo_raw = np.array(sensor_data['servo_raw'])[mask]
    speed = np.array(sensor_data['speed'])[mask]
    output_pwm = np.array(sensor_data['output_pwm'])[mask]

    # Update time series plot
    line_target.set_data(times, target_pos)
    line_current.set_data(times, current_pos)
    line_servo.set_data(times, servo_raw)

    ax_timeseries.set_xlim(times.min(), times.max())

    # Update FFT plot
    freq, fft_mag = compute_fft_servo(servo_raw)
    if len(freq) > 0:
        line_fft.set_data(freq, fft_mag)
        ax_fft.set_ylim(0, fft_mag.max() * 1.2 if fft_mag.max() > 0 else 1)

    # Update statistics
    if len(servo_raw) > 0:
        error = current_pos - target_pos
        stats_text = (
            f"Parameters:\n"
            f"{'─' * 28}\n"
        )

        for param_name in PARAM_NAMES:
            val = param_values[param_name]
            stats_text += f"{param_name:<20} {val:8.5f}\n"

        stats_text += f"\n{'─' * 28}\n"
        stats_text += f"Servo Stats:\n"
        stats_text += f"  Mean: {np.mean(servo_raw):.2f}\n"
        stats_text += f"  Std: {np.std(servo_raw):.2f}\n"
        stats_text += f"  Min/Max: {np.min(servo_raw):.2f} / {np.max(servo_raw):.2f}\n"
        stats_text += f"\nPosition Error:\n"
        stats_text += f"  Mean: {np.mean(error):.2f}\n"
        stats_text += f"  Std: {np.std(error):.2f}\n"
        stats_text += f"  Max: {np.max(np.abs(error)):.2f}\n"
        stats_text += f"\nOutput PWM:\n"
        stats_text += f"  Mean: {np.mean(output_pwm):.1f}\n"
        stats_text += f"  Range: {np.min(output_pwm):.1f} - {np.max(output_pwm):.1f}\n"
        stats_text += f"\nSpeed: {speed[-1]:.2f} m/s\n"
        stats_text += f"Data Points: {len(sensor_data['time'])}\n"

        text_stats.set_text(stats_text)


# ============================================================================
# INITIALIZE SLIDERS
# ============================================================================

for i, param_name in enumerate(PARAM_NAMES):
    ax = slider_axes[i]
    min_val, max_val = PARAM_RANGES[param_name]

    slider = Slider(
        ax, param_name,
        min_val, max_val,
        valinit=param_values[param_name],
        valstep=0.001 if max_val < 1 else 0.01,
        color='steelblue'
    )

    #slider.on_changed(on_slider_change(param_name))
    param_sliders[param_name] = slider

# ============================================================================
# START THREADS AND ANIMATION
# ============================================================================

# Start MAVLink thread
mav_thread = threading.Thread(target=mavlink_thread_func, daemon=True)
mav_thread.start()

# Start UDP sensor thread
udp_thread = threading.Thread(target=udp_sensor_thread_func, daemon=True)
udp_thread.start()

# Create animation
anim = FuncAnimation(fig, update_animation, interval=UPDATE_INTERVAL,
                     blit=False, repeat=True, cache_frame_data=False)

print("\n" + "="*60)
print("Steering Rate PID Tuning GUI")
print("="*60)
print(f"MAVLink: {MAVLINK_CONNECTION}")
print(f"UDP Sensor: {UDP_SENSOR_HOST}:{UDP_SENSOR_PORT}")
print(f"CSV Format: Target Position;Current Position;State;Speed;OutputPWM")
print("="*60 + "\n")

plt.show()