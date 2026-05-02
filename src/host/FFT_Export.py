# Export FFT. To be used to CSV Export FFT scripts.

import serial
import numpy as np
import matplotlib.pyplot as plt

PORT = "COM5"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=5)

while True:
    line = ser.readline().decode(errors="ignore").strip()

    if line != "FFT_FRAME_START":
        continue

    freqs = []
    mags = []

    ser.readline()  # skip header

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if line == "FFT_FRAME_END":
            break

        parts = line.split(",")
        if len(parts) == 3:
            freqs.append(float(parts[1]))
            mags.append(float(parts[2]))

    freqs = np.array(freqs)
    mags = np.array(mags)

    plt.clf()
    plt.plot(freqs, mags)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude")
    plt.title("FFT Spectrum")
    plt.pause(0.01)