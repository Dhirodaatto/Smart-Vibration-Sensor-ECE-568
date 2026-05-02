import serial
import matplotlib.pyplot as plt
from collections import deque

# ======================
# CONFIG
# ======================
PORT = "COM3"   # change for your system
BAUD = 115200
WINDOW = 500    # number of samples to show

ser = serial.Serial(PORT, BAUD)

raw_buf = deque([0]*WINDOW, maxlen=WINDOW)
filt_buf = deque([0]*WINDOW, maxlen=WINDOW)

plt.ion()
fig, ax = plt.subplots()

line1, = ax.plot(raw_buf, label="Raw")
line2, = ax.plot(filt_buf, label="Filtered")

ax.legend()
ax.set_ylim(0, 1)

# ======================
# LOOP
# ======================
while True:
    try:
        line = ser.readline().decode().strip()
        x_str, y_str = line.split(',')

        x = float(x_str)
        y = float(y_str)

        raw_buf.append(x)
        filt_buf.append(y)

        line1.set_ydata(raw_buf)
        line2.set_ydata(filt_buf)

        plt.pause(0.001)

    except:
        pass