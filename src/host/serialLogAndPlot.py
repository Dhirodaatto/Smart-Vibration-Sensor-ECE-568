# Used to log biquad filter io. Load byquad filt as main onto esp32, change interpreter to local python, run this script. It will wait until you reset
# the esp32 (when you reset the esp32, it begins logging samples and attempting to communicate via com port. This script intercepts those
# values and plots them.)
import serial
import numpy as np
import matplotlib.pyplot as plt
# import time

PORT = "COM5"        # change this
BAUD = 115200
NUM_SAMPLES = 1000

ser = serial.Serial(PORT, BAUD)

print("Waiting for data from ESP32")
with open("data.csv", "w") as f:
    f.write("index,raw,filtered\n")
    
    #=======Accelerometer Logging Additions========================
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "System Ready":
            print("ESP32 ready. Waiting for data header...")
            break
    
    
    # --- Wait for the ESP32 to signal that data is starting ---
#     print("Waiting for header line...")
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "index,raw,filtered":
            print("Header detected. Capturing data...")
#             time.sleep
            break  # discard header, we already wrote our own above  
    #=======Accelerometer Logging Additions========================
    
    
    
    count = 0

    while count < NUM_SAMPLES:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        # filter valid CSV lines only
        if line.count(",") == 2:
            f.write(line + "\n")
            print(line)   # optional live view
            count += 1

print("Capture complete. Saved to data.csv")



# ======================
# LOAD DATA
# ======================
# data = np.loadtxt("data.csv", delimiter=",", skiprows=1, dtype=float)

data = np.genfromtxt("data.csv", delimiter=",", skip_header=1)

# remove any bad rows (NaN from serial noise)
data = data[~np.isnan(data).any(axis=1)]


# t = data[:, 0]
# raw = data[:,1] / 4095.0 - 0.5
# filt = data[:,2] / 4095.0

#Use the below for the 2ndOrd_EMA_Filt_NoTimers_logging.py script
# t = data[:, 0]
# raw = data[:,1] - 0.5
# filt = data[:,2] - 0.5

#Use these withh the 2ndOrd_EMA_And_Accel_Logging.py script
t = data[:, 0]
raw = data[:,1]
filt = data[:,2]

# ======================
# PLOT
# ======================
plt.figure()

plt.plot(t, raw, label="Raw Input", alpha=0.6)
plt.plot(t, filt, label="Filtered Output", linewidth=2)

plt.title("Filter Response (Input vs Output)")
plt.xlabel("Sample Index")
plt.ylabel("Normalized Amplitude")
plt.legend()
plt.grid()

plt.show()