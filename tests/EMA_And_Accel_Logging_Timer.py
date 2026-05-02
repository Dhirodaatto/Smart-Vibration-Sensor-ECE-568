# EMA_And_Accel_Logging_Timer.py
# EMA Filter (configurable order) with MPU-6050 Accelerometer Input and Logging
# Timer-based sampling for consistent 1 kHz sample rate
from machine import Pin, I2C, Timer
import struct
import math
import time

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
offset_z = (sum_z / CAL_SAMPLES) - 1.0

print("Calibration complete")
print("Offsets: x={:.4f}  y={:.4f}  z={:.4f}".format(offset_x, offset_y, offset_z))
print("System Ready")
time.sleep(1)

# ----------------------------
# Logging / Sampling Config
# ----------------------------
FS               = 1000
SAMPLE_PERIOD_MS = int(1000 / FS)      # 1 ms — Timer uses milliseconds
CAPTURE_TIME_SEC = 1.0
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
# Shared State between ISR and main loop
# ----------------------------
sample_index = 0
capture_done = False

# ----------------------------
# Debug Helper
# ----------------------------
def cutoff_from_alpha(alpha, fs):
    return (-math.log(1.0 - alpha) / (2.0 * math.pi)) * fs

print("Filter order: {}  |  alpha: {:.3f}  |  cutoff: {:.1f} Hz".format(
    FILTER_ORDER, ALPHA, cutoff_from_alpha(ALPHA, FS)))

# ----------------------------
# Timer ISR — runs at exactly 1 kHz
# ----------------------------
def sample_isr(timer):
    global y1, y2, y3, sample_index, capture_done

    if capture_done:
        return

    # --- Read accelerometer, apply calibration offset ---
#     ax, ay, az = rd_accelerometer()
#     x = ax - offset_x

    raw = signal_adc.read()
    # --- Normalize input ---
    x = raw / 4095.0

    # --- Cascaded EMA ---
    y1 = y1 + ALPHA * (x - y1)
    out = y1

    if FILTER_ORDER >= 2:
        y2 = y2 + ALPHA * (y1 - y2)
        out = y2

    if FILTER_ORDER >= 3:
        y3 = y3 + ALPHA * (y2 - y3)
        out = y3

    # --- Store ---
    raw_log[sample_index]  = x
    filt_log[sample_index] = out
    sample_index += 1

    if sample_index >= TOTAL_SAMPLES:
        capture_done = True

# ----------------------------
# Start Timer
# ----------------------------
tim = Timer(0)
tim.init(period=SAMPLE_PERIOD_MS, mode=Timer.PERIODIC, callback=sample_isr)

# ----------------------------
# Main Loop — waits for capture to complete
# Other tasks can run here without affecting sample timing
# ----------------------------
while not capture_done:
    time.sleep_ms(10)   # yield; main loop free for other work

# --- Stop timer once capture is complete ---
tim.deinit()

# ----------------------------
# Export over Serial
# ----------------------------
print("index,raw,filtered")
for i in range(TOTAL_SAMPLES):
    print("{},{:.6f},{:.6f}".format(i, raw_log[i], filt_log[i]))
    
# Hold here — prevents REPL from appearing and conflicting with Thonny
while True:
    time.sleep_ms(1000)