import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from collections import deque

# ---------- SERIAL CONFIG ----------
PORT = '/dev/ttyUSB0'
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=1)

# ---------- DATA CONFIG ----------
WINDOW = 200

data_top = [deque(maxlen=WINDOW) for _ in range(4)]
data_bottom = [deque(maxlen=WINDOW) for _ in range(3)]

labels_top = ["CommandedSpeed", "Planned", "Input", "Output"]
labels_bottom = ["Pterm", "Iterm", "Dterm"]

# ---------- FIGURE LAYOUT ----------
fig = plt.figure(figsize=(10, 8))

gs = fig.add_gridspec(
    nrows=8, ncols=1,
    height_ratios=[3, 3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.2]
)

ax_top = fig.add_subplot(gs[0])
ax_bottom = fig.add_subplot(gs[1])

slider_axes = [
    fig.add_subplot(gs[i]) for i in range(2, 7)
]

# ---------- PLOTS ----------
lines_top = [
    ax_top.plot([], [], label=labels_top[i])[0]
    for i in range(4)
]

lines_bottom = [
    ax_bottom.plot([], [], label=labels_bottom[i])[0]
    for i in range(3)
]

for ax in (ax_top, ax_bottom):
    ax.set_xlim(0, WINDOW)
    ax.set_ylim(-1550, 1500)     # <<< updated range
    ax.grid(True)
    ax.legend(loc="upper left")




# ---------- SLIDERS ----------
sliders = []
labels = [ "Kp", "Ki", "Kd", "Pon", "Don" ]
valmaxs = [ 10.0, 10.0, 10.0, 1, 1 ]

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
        return lines_top + lines_bottom

    try:
        values = [float(v) for v in line.split(",")]
        if len(values) != 7:
            return lines_top + lines_bottom

        for i in range(4):
            data_top[i].append(values[i])
            lines_top[i].set_data(range(len(data_top[i])), data_top[i])

        for i in range(3):
            data_bottom[i].append(values[i + 4])
            lines_bottom[i].set_data(range(len(data_bottom[i])), data_bottom[i])

    except ValueError:
        pass

    return lines_top + lines_bottom

ani = animation.FuncAnimation(
    fig,
    update,
    interval=50,
    blit=True
)

plt.tight_layout()
plt.show()
ser.close()
