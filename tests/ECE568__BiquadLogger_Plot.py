
import numpy as np
import matplotlib.pyplot as plt

# ======================
# LOAD DATA
# ======================
# data = np.loadtxt("data.csv", delimiter=",", skiprows=1, dtype=float)

data = np.genfromtxt("data.csv", delimiter=",", skip_header=1)

# remove any bad rows (NaN from serial noise)
data = data[~np.isnan(data).any(axis=1)]


t = data[:, 0]
raw = data[:,1] / 4095.0 - 0.5
filt = data[:,2] / 4095.0

# ======================
# PLOT
# ======================
plt.figure()

plt.plot(t, raw, label="Raw Input", alpha=0.6)
plt.plot(t, filt, label="Filtered Output", linewidth=2)

plt.title("IIR Filter Response (Input vs Output)")
plt.xlabel("Sample Index")
plt.ylabel("Normalized Amplitude")
plt.legend()
plt.grid()

plt.show()