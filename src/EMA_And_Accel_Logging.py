# 2nd Order EMA Filter with MPU-6050 Accelerometer Input and Logging
from machine import Pin, I2C
import struct
import math
import time

# ----------------------------
# I2C / Accelerometer Setup
# ----------------------------
i2c = I2C(scl=Pin(14), sda=Pin(22), freq=400000)

ACCEL_ADDR   = 0x68
PWR_MGMT     = 0x6B
ACCEL_X_HIGH = 0x3B

# Wake up MPU-6050
i2c.writeto_mem(ACCEL_ADDR, PWR_MGMT, b'\x00')
print("Accelerometer initialized")

def rd_imu_data():
    raw_imu_data = i2c.readfrom_mem(ACCEL_ADDR, ACCEL_X_HIGH, 14)
    return struct.unpack(">hhhhhhh", raw_imu_data)

def normal_accel(raw):
    # +/- 2g range, 1g = 16384 LSB
    return raw / 16384.0

def rd_accelerometer():
    ax, ay, az, temp, gx, gy, gz = rd_imu_data()
    return normal_accel(ax), normal_accel(ay), normal_accel(az)

# ----------------------------
# Calibration
# ----------------------------
print("Calibrating. Do not move sensor.")
sum_x = sum_y = sum_z = 0.0
CAL_SAMPLES = 100

for _ in range(CAL_SAMPLES):
    ax, ay, az = rd_accelerometer()
    sum_x += ax
    sum_y += ay
    sum_z += az
    time.sleep(0.05)

offset_x = sum_x / CAL_SAMPLES
offset_y = sum_y / CAL_SAMPLES
offset_z = (sum_z / CAL_SAMPLES) - 1.0   # remove 1 g gravity component

print("Calibration complete")
print("Offsets: x={:.4f}  y={:.4f}  z={:.4f}".format(offset_x, offset_y, offset_z))
print("System Ready")
time.sleep(1)

# ----------------------------
# Logging / Sampling Config
# ----------------------------
FS               = 1000                     # accelerometer native rate (Hz)
SAMPLE_PERIOD_US = int(1e6 / FS)            # 1000 us between samples
CAPTURE_TIME_SEC = 1.0
TOTAL_SAMPLES    = int(FS * CAPTURE_TIME_SEC)

raw_log  = [0.0] * TOTAL_SAMPLES
filt_log = [0.0] * TOTAL_SAMPLES

# Filter Config
# ----------------------------
FILTER_ORDER = 2    # 1, 2, or 3
ALPHA = 0.18          # hard-coded for now, later updated via gui, knob, etc.
# Cutoff frequencies at FS = 1000 Hz:
#   1st order: fc = -ln(1-alpha) / (2*pi) * 1000, (-20 dB/dec)
#   2nd order: same formula, (-40 dB/dec)
#   3rd order: same formula, (-60 dB/dec)
#
#   alpha = 0.18  ->  ~31 Hz  
#   alpha = 0.27  ->  ~50 Hz  
#   alpha = 0.46  -> ~100 Hz 
#   alpha = 0.54  -> ~125 Hz  
#   alpha = 0.61  -> ~150 Hz  


# # --- Potentiometer alpha mapping (re-enable later) ---
# ALPHA_MIN = 0.17 ~29 Hz cutoff
# ALPHA_MAX = 0.54 ~125 Hz cutoff

# Testing infrastructure for varying alpha -> filter cutoff
# -----------------------------
# pot_adc = ADC(Pin(37))
# pot_adc.atten(ADC.ATTN_11DB)
# alpha = ALPHA_MIN + (pot_adc.read() / 4095.0) * (ALPHA_MAX - ALPHA_MIN)

# Filter Init (Cfg-order EMA)
# ----------------------------
y1 = 0.0   # first stage output
y2 = 0.0   # second stage output 
y3 = 0.0   # second stage output 


# Cutoff printout
# ----------------------------
def cutoff_from_alpha(alpha, fs):
    return (-math.log(1.0 - alpha) / (2.0 * math.pi)) * fs


# Main Capture Loop  (X-axis)
# ----------------------------
for i in range(TOTAL_SAMPLES):
    t_start = time.ticks_us()

    # --- Read accelerometer, apply calibration offset ---
    ax, ay, az = rd_accelerometer()
    x = ax - offset_x          # calibrated X-axis value in g

    # --- 2nd-order cascaded EMA ---
    y1 = y1 + ALPHA * (x - y1)
    y2 = y2 + ALPHA * (y1 - y2)
    
    # --- Configurable Order ---
#     y1 = y1 + ALPHA * (x - y1)   # always runs
#     out = y1
# 
#     if FILTER_ORDER >= 2:
#         y2 = y2 + ALPHA * (y1 - y2)
#         out = y2
# 
#     if FILTER_ORDER >= 3:
#         y3 = y3 + ALPHA * (y2 - y3)
#         out = y3

    # --- Log both raw (calibrated) and filtered values ---
    raw_log[i]  = x
    filt_log[i] = y2

    # --- Enforce 1 kHz sampling period ---
    elapsed = time.ticks_diff(time.ticks_us(), t_start)
    delay   = SAMPLE_PERIOD_US - elapsed
    if delay > 0:
        time.sleep_us(delay)

# ----------------------------
# Export over Serial
# ----------------------------
print("index,raw,filtered")
for i in range(TOTAL_SAMPLES):
    print("{},{:.6f},{:.6f}".format(i, raw_log[i], filt_log[i]))