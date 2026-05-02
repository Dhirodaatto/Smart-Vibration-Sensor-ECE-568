# Timer ISR Filter — Concurrent Process Test
# Purpose: Verify filter maintains consistent 1 kHz sampling
# while a secondary process runs in the main loop.
# The secondary process is intentionally made slow and variable
# to stress-test the timer's independence.

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

i2c.writeto_mem(ACCEL_ADDR, PWR_MGMT, b'\x00')
print("Accelerometer initialized")

def rd_imu_data():
    raw_imu_data = i2c.readfrom_mem(ACCEL_ADDR, ACCEL_X_HIGH, 14)
    return struct.unpack(">hhhhhhh", raw_imu_data)

def normal_accel(raw):
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
SAMPLE_PERIOD_MS = int(1000 / FS)
CAPTURE_TIME_SEC = 2.0                          # longer capture to stress test more
TOTAL_SAMPLES    = int(FS * CAPTURE_TIME_SEC)

raw_log      = [0.0] * TOTAL_SAMPLES
filt_log     = [0.0] * TOTAL_SAMPLES
timestamp_log = [0] * TOTAL_SAMPLES             # log actual sample timestamps (us)
                                                # so we can verify consistent spacing

# ----------------------------
# Filter Config
# ----------------------------
ALPHA = 0.18

# ----------------------------
# Filter State
# ----------------------------
y1 = 0.0
y2 = 0.0
y3 = 0.0

# ----------------------------
# Shared State
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
# Timer ISR
# ----------------------------
def sample_isr(timer):
    global y1, y2, y3, sample_index, capture_done

    if capture_done:
        return

    # Timestamp this sample so we can verify spacing on the PC side
    timestamp_log[sample_index] = time.ticks_us()

    ax, ay, az = rd_accelerometer()
    x = ax - offset_x

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

    if sample_index >= TOTAL_SAMPLES:
        capture_done = True

# ----------------------------
# Start Timer
# ----------------------------
tim = Timer(0)
tim.init(period=SAMPLE_PERIOD_MS, mode=Timer.PERIODIC, callback=sample_isr)

# ----------------------------
# Secondary Process
# Simulates a slow, variable-duration task running in the main loop.
# Intentionally uses sleep and math to hog the main loop
# and verify it does NOT disturb the ISR timing.
# ----------------------------
main_loop_count = 0
main_loop_log   = []        # log how many main loop iterations completed

while not capture_done:
    # --- Simulate variable-duration work ---
    # Mix of fast and slow iterations to create timing pressure
    if main_loop_count % 3 == 0:
        time.sleep_ms(47)   # slow iteration (~47 ms)
    elif main_loop_count % 3 == 1:
        time.sleep_ms(13)   # medium iteration (~13 ms)
    else:
        time.sleep_ms(3)    # fast iteration (~3 ms)

    # --- Simulate some computation ---
    dummy = 0.0
    for j in range(50):
        dummy += math.sqrt(j + 1.0)

    main_loop_count += 1

tim.deinit()

# ----------------------------
# Analyze Timestamp Spacing
# Compute actual inter-sample intervals to verify
# the ISR fired consistently at 1 kHz despite main loop load
# ----------------------------
print("\n--- Timing Analysis ---")
intervals = []
for i in range(1, TOTAL_SAMPLES):
    dt = time.ticks_diff(timestamp_log[i], timestamp_log[i - 1])
    intervals.append(dt)

min_dt  = min(intervals)
max_dt  = max(intervals)
mean_dt = sum(intervals) / len(intervals)

# Count samples that deviated more than +/- 50 us from ideal (1000 us)
jitter_threshold_us = 50
outliers = sum(1 for dt in intervals if abs(dt - 1000) > jitter_threshold_us)

print("Expected interval : 1000 us")
print("Min interval      : {} us".format(min_dt))
print("Max interval      : {} us".format(max_dt))
print("Mean interval     : {:.1f} us".format(mean_dt))
print("Outliers > +/-{}us: {} / {}".format(
    jitter_threshold_us, outliers, len(intervals)))
print("Main loop iterations completed: {}".format(main_loop_count))
print("-----------------------\n")

# ----------------------------
# Export over Serial
# index, raw, filtered, timestamp_us
# timestamp column lets PC-side script independently verify spacing
# ----------------------------
print("index,raw,filtered,timestamp_us")
for i in range(TOTAL_SAMPLES):
    print("{},{:.6f},{:.6f},{}".format(
        i, raw_log[i], filt_log[i], timestamp_log[i]))