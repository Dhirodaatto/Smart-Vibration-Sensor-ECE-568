# EMA_And_Accel_Logging_FIFO.py
# EMA Filter (configurable order) with MPU-6050 FIFO and Logging
# Non-blocking FIFO poll — main loop free for other work between samples
from machine import Pin, I2C
import struct
import math
import time

time.sleep(2)

# ----------------------------
# Filter Order Config
# ----------------------------
FILTER_ORDER = 2    # 1, 2, or 3

# ----------------------------
# I2C / Accelerometer Setup
# ----------------------------
i2c = I2C(scl=Pin(14), sda=Pin(22), freq=400000)

ACCEL_ADDR   = 0x68
PWR_MGMT     = 0x6B

# MPU-6050 FIFO registers
USER_CTRL    = 0x6A
FIFO_EN      = 0x23
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
FIFO_COUNT_H = 0x72
FIFO_R_W     = 0x74

# ----------------------------
# MPU-6050 Init and FIFO Setup
# ----------------------------
# Wake up
i2c.writeto_mem(ACCEL_ADDR, PWR_MGMT,    b'\x00')

# DLPF bandwidth ~94 Hz — hardware pre-filter before EMA
i2c.writeto_mem(ACCEL_ADDR, CONFIG,      b'\x02')

# Sample rate = 1000 / (1 + 0) = 1000 Hz
i2c.writeto_mem(ACCEL_ADDR, SMPLRT_DIV,  b'\x00')

# Reset FIFO
i2c.writeto_mem(ACCEL_ADDR, USER_CTRL,   b'\x04')
time.sleep_ms(10)

# Enable FIFO
i2c.writeto_mem(ACCEL_ADDR, USER_CTRL,   b'\x40')

# Accel X/Y/Z into FIFO (bit 3: ACCEL_FIFO_EN)
i2c.writeto_mem(ACCEL_ADDR, FIFO_EN,     b'\x08')

print("Accelerometer initialized with FIFO at 1 kHz")

# ----------------------------
# Helper Functions
# ----------------------------
def normal_accel(raw):
    # +/- 2g range, 1g = 16384 LSB
    return raw / 16384.0

def get_fifo_count():
    raw = i2c.readfrom_mem(ACCEL_ADDR, FIFO_COUNT_H, 2)
    return (raw[0] << 8) | raw[1]

def rd_fifo_sample():
    # 6 bytes: Xh Xl Yh Yl Zh Zl
    raw = i2c.readfrom_mem(ACCEL_ADDR, FIFO_R_W, 6)
    ax, ay, az = struct.unpack(">hhh", raw)
    return normal_accel(ax), normal_accel(ay), normal_accel(az)

def drain_fifo():
    count = get_fifo_count()
    if count > 0:
        # Read in chunks of 6 (complete samples only)
        complete = (count // 6) * 6
        if complete > 0:
            i2c.readfrom_mem(ACCEL_ADDR, FIFO_R_W, complete)

# ----------------------------
# Placeholder for other processes
# Replace with your actual functions
# ----------------------------
def run_other_processes():
    pass

# ----------------------------
# Calibration
# ----------------------------
print("Calibrating. Do not move sensor.")
sum_x = sum_y = sum_z = 0.0
CAL_SAMPLES = 100

# Drain stale FIFO data before calibrating
time.sleep_ms(50)
drain_fifo()

for _ in range(CAL_SAMPLES):
    # Blocking wait during calibration only — no other processes running yet
    while get_fifo_count() < 6:
        pass
    ax, ay, az = rd_fifo_sample()
    sum_x += ax
    sum_y += ay
    sum_z += az

offset_x = sum_x / CAL_SAMPLES
offset_y = sum_y / CAL_SAMPLES
offset_z = (sum_z / CAL_SAMPLES) - 1.0

print("Calibration complete")
print("Offsets: x={:.4f}  y={:.4f}  z={:.4f}".format(offset_x, offset_y, offset_z))
print("System Ready")
time.sleep(1)

# ----------------------------
# Logging / Sampling Config
# ----------------------------
FS               = 1000
CAPTURE_TIME_SEC = 0.5
TOTAL_SAMPLES    = int(FS * CAPTURE_TIME_SEC)

raw_log  = [0.0] * TOTAL_SAMPLES
filt_log = [0.0] * TOTAL_SAMPLES

# ----------------------------
# Filter Config
# ----------------------------
# Cutoff frequencies at FS = 1000 Hz:
#   1st order: -20 dB/decade roll-off
#   2nd order: -40 dB/decade roll-off
#   3rd order: -60 dB/decade roll-off
#
#   alpha = 0.18  ->  ~31 Hz
#   alpha = 0.27  ->  ~50 Hz
#   alpha = 0.46  -> ~100 Hz
#   alpha = 0.54  -> ~125 Hz
#   alpha = 0.61  -> ~150 Hz
#
ALPHA = 0.18

# # --- Potentiometer alpha mapping (re-enable later) ---
# ALPHA_MIN = 0.17   # ~29 Hz cutoff
# ALPHA_MAX = 0.54   # ~125 Hz cutoff
# pot_adc = ADC(Pin(37))
# pot_adc.atten(ADC.ATTN_11DB)
# alpha = ALPHA_MIN + (pot_adc.read() / 4095.0) * (ALPHA_MAX - ALPHA_MIN)

# ----------------------------
# Filter State
# ----------------------------
y1 = 0.0
y2 = 0.0
y3 = 0.0

# ----------------------------
# Debug Helper
# ----------------------------
def cutoff_from_alpha(alpha, fs):
    return (-math.log(1.0 - alpha) / (2.0 * math.pi)) * fs

print("Filter order: {}  |  alpha: {:.3f}  |  cutoff: {:.1f} Hz".format(
    FILTER_ORDER, ALPHA, cutoff_from_alpha(ALPHA, FS)))

# ----------------------------
# Drain FIFO before capture begins
# ----------------------------
drain_fifo()

# ----------------------------
# Main Capture Loop
# Non-blocking — other processes run when no sample is ready
# ----------------------------
sample_index = 0

while sample_index < TOTAL_SAMPLES:

    fifo_count = get_fifo_count()

    if fifo_count >= 6:
        # --- Sample ready — read and process ---
        ax, ay, az = rd_fifo_sample()
        x = ax - offset_x

        # --- Cascaded EMA ---
        y1 = y1 + ALPHA * (x - y1)
        out = y1

        if FILTER_ORDER >= 2:
            y2 = y2 + ALPHA * (y1 - y2)
            out = y2

        if FILTER_ORDER >= 3:
            y3 = y3 + ALPHA * (y2 - y3)
            out = y3

        raw_log[sample_index]  = x
        filt_log[sample_index] = out
        sample_index += 1

    else:
        # --- No sample ready — run other processes ---
        run_other_processes()

# ----------------------------
# Export over Serial
# ----------------------------
print("index,raw,filtered")
for i in range(TOTAL_SAMPLES):
    print("{},{:.6f},{:.6f}".format(i, raw_log[i], filt_log[i]))

# Hold here — prevents REPL from appearing and conflicting with Thonny
while True:
    time.sleep_ms(1000)