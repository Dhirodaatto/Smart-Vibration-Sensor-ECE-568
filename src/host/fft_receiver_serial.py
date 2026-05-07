
# 
# 
# import serial
# 
# PORT = "COM5"       # change this
# BAUD = 115200       # try 115200 if nothing appears
# 
# ser = serial.Serial(PORT, BAUD, timeout=1)
# 
# print("Listening for ESP32 serial output...\n")
# 
# while True:
#     try:
#         line = ser.readline().decode(errors="ignore").strip()
# 
#         if line == "":
#             continue
# 
#         print(line)
# 
#     except Exception as e:
#         print("Error:", e)
# 
# 




import serial
import numpy as np
import matplotlib.pyplot as plt
# 
# 

# ---------------- CONFIG ----------------
PORT = "COM5"          # change this
BAUD = 115200          # match your ESP32 serial config
FFT_BINS = 256         # IMPORTANT: FFT_SIZE/2 from ESP32

# ---------------- SERIAL ----------------
ser = serial.Serial(PORT, BAUD, timeout=1)

# ---------------- PLOT SETUP ----------------
plt.ion()
fig, ax = plt.subplots()

#
Fs = 500  # must match ESP32 sampling_rate
x = np.linspace(0, Fs / 2, FFT_BINS)
line, = ax.plot(x, np.zeros(FFT_BINS))

ax.set_title("ESP32 Vibration FFT (Real-Time)")
ax.set_xlabel("Frequency Bin")
ax.set_ylabel("Magnitude (dB)")

# ---------------- MAIN LOOP ----------------
while True:
    try:
        raw = ser.readline().decode(errors="ignore").strip()

        if not raw:
            continue

        parts = raw.split(",")

        # Expect: frame_id + 256 FFT bins = 257 values total
        if len(parts) != FFT_BINS + 1:
            continue

        # Frame ID (optional, but useful for debugging)
        try:
            frame_id = int(parts[0])
        except:
            continue

        # FFT data
        fft = np.array(parts[1:], dtype=np.float32)

        # Safety check (prevents silent plotting bugs)
        if fft.shape[0] != FFT_BINS:
            continue

        # Convert to dB scale (standard for vibration analysis)
        fft_db = 20 * np.log10(fft + 1e-6)

        # Update plot
        line.set_ydata(fft_db)

        ax.relim()
        ax.autoscale_view()

        plt.pause(0.01)

    except Exception as e:
        print("Error:", e)