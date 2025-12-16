import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from collections import deque

# TODO: Make this universial, treat everything as float and just configure name, min, max.
# ---------- SERIAL CONFIG ----------
PORT = '/dev/ttyUSB0'
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)

# ---------- DATA CONFIG ----------
WINDOW = 200

data_speed = [deque(maxlen=WINDOW) for _ in range(2)]
data_pid = [deque(maxlen=WINDOW) for _ in range(1)]

labels_top = ["PWM", "Velocity"]
labels_bottom = ["Position (rad)"]

# ---------- FIGURE LAYOUT ----------
fig = plt.figure(figsize=(10, 8))

gs = fig.add_gridspec(
    nrows=6, ncols=1,
    height_ratios=[3, 3, 0.3, 0.3, 0.3, 0.3]
)

ax_speed = fig.add_subplot(gs[0])
ax_pid = fig.add_subplot(gs[1])

slider_axes = [
    fig.add_subplot(gs[i]) for i in range(2, 5)
]

# ---------- PLOTS ----------
lines_speed = [
    ax_speed.plot([], [], label=labels_top[i])[0]
    for i in range(2)
]

lines_pid = [
    ax_pid.plot([], [], label=labels_bottom[i])[0]
    for i in range(1)
]


ax_speed.set_xlim(0, WINDOW)
ax_speed.set_ylim(-500, 500)     # <<< updated range
ax_speed.grid(True)
ax_speed.legend(loc="upper left")

ax_pid.set_xlim(0, WINDOW)
ax_pid.set_ylim(-1550, 1500)     # <<< updated range
ax_pid.grid(True)
ax_pid.legend(loc="upper left")


# ---------- SLIDERS ----------
sliders = []
labels = [ "Ramp Increment", "Ramp Delay", "Minimum Velocity" ]
valmaxs = [ 3.0, 3.0, 3.0, 1, 1 ]

for i, ax in enumerate(slider_axes):
    s = Slider(
        ax=ax,
        label=labels[i],
        valmin=0.0,
        valmax=valmaxs[i],
        valinit=0.0
    )
    sliders.append(s)

# ---------- SERIAL WRITE ----------
def send_sliders(val=None):
    values = [s.val for s in sliders]
    msg = ",".join(f"{v:.3f}" for v in values) + "\n"
    ser.write(msg.encode("utf-8"))
    #print("Sending "+msg)

for s in sliders:
    s.on_changed(send_sliders)

# ---------- ANIMATION ----------
def update(frame):
    line = ser.readline().decode("utf-8").strip()
    print(line)
    if not line:
        return lines_speed + lines_pid

    try:
        values = [float(v) for v in line.split(";")]
        if len(values) != 6:
            return lines_speed + lines_pid

        for i in range(2):
            data_speed[i].append(values[i])
            lines_speed[i].set_data(range(len(data_speed[i])), data_speed[i])

        for i in range(1):
            data_pid[i].append(values[i + 2])
            lines_pid[i].set_data(range(len(data_pid[i])), data_pid[i])

    except ValueError:
        pass

    return lines_speed + lines_pid

ani = animation.FuncAnimation(
    fig,
    update,
    interval=50,
    blit=True
)

plt.tight_layout()
plt.show()
ser.close()
