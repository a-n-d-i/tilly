import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ---------- SERIAL CONFIG ----------
PORT = '/dev/ttyUSB0'        # Windows example: 'COM3'
# PORT = '/dev/ttyUSB0'  # Linux
# PORT = '/dev/ttyACM0'  # Linux (Arduino)
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# ---------- PLOT CONFIG ----------
WINDOW_SIZE = 200  # number of points shown

data = [deque(maxlen=WINDOW_SIZE) for _ in range(4)]

fig, ax = plt.subplots()
lines = [ax.plot([], [], label=i)[0] for i in("CommandedSpeed", "Planned", "Input", "Output")]

# https://matplotlib.org/stable/gallery/subplots_axes_and_figures/subplots_demo.html

ax.set_xlim(0, WINDOW_SIZE)
ax.set_ylim(0, 500)   # adjust to your data range
ax.legend()
ax.grid(True)

# ---------- INIT FUNCTION ----------
def init():
    for line in lines:
        line.set_data([], [])
    return lines

# ---------- UPDATE FUNCTION ----------
def update(frame):
    line = ser.readline().decode('utf-8').strip()
    if not line:
        return lines

    try:
        values = [float(v) for v in line.split(',')]
        if len(values) != 4:
            return lines

        for i in range(4):
            data[i].append(values[i])
            lines[i].set_data(range(len(data[i])), data[i])

    except ValueError:
        pass  # ignore malformed lines

    return lines

# ---------- ANIMATION ----------
ani = animation.FuncAnimation(
    fig,
    update,
    init_func=init,
    interval=50,
    blit=True
)

plt.show()
ser.close()
