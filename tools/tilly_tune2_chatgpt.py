import asyncio
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from pymavlink import mavutil

# =============================
# CONFIGURATION
# =============================
MAVLINK_UDP = "udp:127.0.0.1:14550"
CSV_UDP_PORT = 42424

POS_MIN = 0.0
POS_MAX = 150.0
SERVO_MIN = 1000.0
SERVO_MAX = 2000.0

FFT_MIN_HZ = 0.1
FFT_MAX_HZ = 4.0
EST_SAMPLE_RATE = 20.0
MAX_SECONDS = 120

# =============================
# DATA BUFFERS
# =============================
maxlen = int(MAX_SECONDS * EST_SAMPLE_RATE)
t_buf = deque(maxlen=maxlen)
servo_buf = deque(maxlen=maxlen)
target_buf = deque(maxlen=maxlen)
current_buf = deque(maxlen=maxlen)
pwm_buf = deque(maxlen=maxlen)

# =============================
# PARAMETERS
# =============================
PARAM_RANGES = {
    "ATC_STR_RAT_P":    (0.0, 2.0),
    "ATC_STR_RAT_I":    (0.0, 1.0),
    "ATC_STR_RAT_D":    (0.0, 0.5),
    "ATC_STR_RAT_FF":   (0.0, 2.0),
    "ATC_STR_RAT_D_FF": (0.0, 1.0),
    "ATC_STR_RAT_IMAX": (0.0, 1.0),
    "ATC_STR_RAT_PDMX": (0.0, 2.0),
    "ATC_STR_RAT_SMAX": (10.0, 180.0),
    "ATC_STR_RAT_MAX":  (10.0, 180.0),
    "ATC_STR_ACC_MAX":  (10.0, 500.0),
}

# =============================
# MAVLINK ASYNC READER
# =============================
class AsyncMAV:
    def __init__(self, udp_address):
        self.master = mavutil.mavlink_connection(udp_address, source_system=255)
        self.servo_value = SERVO_MIN

    async def read_messages(self):
        while True:
            msg = await asyncio.to_thread(self.master.recv_match, None, False)
            if msg:
                if msg.get_type() == "SERVO_OUTPUT_RAW":
                    self.servo_value = msg.servo1_raw
            await asyncio.sleep(0.001)

    async def get_param(self, name, timeout=2.0):
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            name.encode(),
            -1
        )
        start = time.time()
        while time.time() - start < timeout:
            msg = await asyncio.to_thread(self.master.recv_match, "PARAM_VALUE", False)
            if msg:
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode()
                if param_id.strip("\x00") == name:
                    return msg.param_value
            await asyncio.sleep(0.01)
        return None

    async def set_param(self, name, value):
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode(),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

# =============================
# CSV UDP ASYNC READER
# =============================
async def csv_udp_reader():
    loop = asyncio.get_event_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: CSVProtocol(),
        local_addr=("0.0.0.0", CSV_UDP_PORT)
    )

class CSVProtocol:
    def datagram_received(self, data, addr):
        try:
            line = data.decode().strip()
            parts = line.split(";")
            if len(parts) < 5:
                return
            target = float(parts[0])
            current = float(parts[1])
            pwm = float(parts[4])

            now = time.time()
            t_buf.append(now)
            target_buf.append(target)
            current_buf.append(current)
            pwm_buf.append(pwm)
        except:
            pass

# =============================
# NORMALIZATION
# =============================
def normalize(val, vmin, vmax):
    return 150.0 * (val - vmin) / (vmax - vmin)

# =============================
# GUI SETUP
# =============================
plt.style.use("seaborn-v0_8-darkgrid")
fig = plt.figure(figsize=(14, 8))
gs = fig.add_gridspec(2, 2, width_ratios=[1, 3])

ax_sliders = fig.add_subplot(gs[:, 0])
ax_time = fig.add_subplot(gs[0, 1])
ax_fft = fig.add_subplot(gs[1, 1])
ax_sliders.axis("off")

# Window slider
ax_win = plt.axes([0.15, 0.05, 0.7, 0.03])
window_slider = Slider(ax_win, "Window (s)", 5, 60, valinit=30, valstep=1)

# =============================
# INIT PARAMETER SLIDERS
# =============================
sliders = {}

async def init_sliders(async_mav):
    y = 0.95
    for name, (vmin, vmax) in PARAM_RANGES.items():
        value = await async_mav.get_param(name)
        if value is None:
            value = (vmin + vmax) / 2.0
        ax = plt.axes([0.05, y, 0.25, 0.035])
        s = Slider(ax, name, vmin, vmax, valinit=value)
        s.on_changed(lambda val, n=name: asyncio.create_task(async_mav.set_param(n, val)))
        sliders[name] = s
        y -= 0.045

# =============================
# PLOT LINES
# =============================
line_servo, = ax_time.plot([], [], label="Servo Output")
line_target, = ax_time.plot([], [], label="Target Position")
line_current, = ax_time.plot([], [], label="Current Position")
line_pwm, = ax_time.plot([], [], label="OutputPWM", linestyle='--')
ax_time.set_ylim(0, 150)
ax_time.set_title("Steering / Position (Normalized)")
ax_time.legend(loc="upper right")

line_fft, = ax_fft.plot([], [])
ax_fft.set_xlim(FFT_MIN_HZ, FFT_MAX_HZ)
ax_fft.set_title("Servo Output FFT (0.1 – 4 Hz)")
ax_fft.set_xlabel("Frequency (Hz)")
ax_fft.set_ylabel("Amplitude")

# =============================
# UPDATE FUNCTION
# =============================
async_mav = AsyncMAV(MAVLINK_UDP)

def update(frame):
    if len(t_buf) < 10:
        return
    now = time.time()
    window = window_slider.val
    t = np.array(t_buf)
    mask = t > (now - window)
    t = t[mask] - t[mask][0]

    servo = np.array([async_mav.servo_value]*len(t)) if len(t) > 0 else np.zeros(len(t))
    target = np.array(target_buf)[-len(t):]
    current = np.array(current_buf)[-len(t):]
    pwm = np.array(pwm_buf)[-len(t):] if len(pwm_buf) >= len(t) else np.zeros(len(t))

    servo_n = normalize(servo, SERVO_MIN, SERVO_MAX)
    target_n = np.clip(target, POS_MIN, POS_MAX)
    current_n = np.clip(current, POS_MIN, POS_MAX)
    pwm_n = np.clip(pwm, POS_MIN, POS_MAX)

    line_servo.set_data(t, servo_n)
    line_target.set_data(t, target_n)
    line_current.set_data(t, current_n)
    line_pwm.set_data(t, pwm_n)
    ax_time.set_xlim(0, max(t))

    y_fft = servo_n - np.mean(servo_n)
    yf = np.abs(np.fft.rfft(y_fft))
    xf = np.fft.rfftfreq(len(y_fft), 1.0 / EST_SAMPLE_RATE)
    fmask = (xf >= FFT_MIN_HZ) & (xf <= FFT_MAX_HZ)
    line_fft.set_data(xf[fmask], yf[fmask])
    if np.any(fmask):
        ax_fft.set_ylim(0, np.max(yf[fmask]) * 1.2)

# =============================
# MAIN ASYNC
# =============================
async def main():
    await init_sliders(async_mav)
    asyncio.create_task(async_mav.read_messages())
    await csv_udp_reader()  # runs forever

# Start animation
ani = FuncAnimation(fig, update, interval=100)

# Run asyncio loop in background
loop = asyncio.get_event_loop()
loop.create_task(main())

plt.tight_layout()
plt.show()
