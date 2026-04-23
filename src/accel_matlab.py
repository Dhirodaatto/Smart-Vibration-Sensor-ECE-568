from machine import I2C, Pin
import time
import struct

# -----------------------------
# User settings
# -----------------------------
SCL_PIN = 14
SDA_PIN = 22
I2C_FREQ = 400_000

MPU_ADDR_CANDIDATES = (0x68, 0x69)

FS_HZ = 1000
N_SAMPLES = 4096                 # power of 2 for FFT
OUT_FILE = "accel_capture.csv"

# Accelerometer scale for ±2g
ACCEL_LSB_PER_G = 16384.0

# -----------------------------
# MPU-6050 registers
# -----------------------------
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

# -----------------------------
# Init I2C
# -----------------------------
i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)
devices = i2c.scan()
print("I2C scan:", [hex(x) for x in devices])

mpu_addr = None
for a in MPU_ADDR_CANDIDATES:
    if a in devices:
        mpu_addr = a
        break
if mpu_addr is None:
    raise RuntimeError("MPU-6050 not found at 0x68 or 0x69.")

print("Using MPU addr:", hex(mpu_addr))

# -----------------------------
# Configure for ~1kHz accel capture
# -----------------------------
i2c.writeto_mem(mpu_addr, PWR_MGMT_1, b"\x00")
time.sleep_ms(100)

# DLPF_CFG = 0 (CONFIG=0), SMPLRT_DIV=7 => 8k/(1+7)=1k internal sample rate
# (Accel unique output is still 1kHz max.)
i2c.writeto_mem(mpu_addr, CONFIG, b"\x00")
i2c.writeto_mem(mpu_addr, SMPLRT_DIV, bytes([7]))
i2c.writeto_mem(mpu_addr, ACCEL_CONFIG, b"\x00")  # ±2g

# -----------------------------
# Allocate ring buffer (raw int16 counts)
# -----------------------------
sample_bytes = 6
ring = bytearray(N_SAMPLES * sample_bytes)
mv = memoryview(ring)

# -----------------------------
# Capture
# -----------------------------
period_us = int(1_000_000 // FS_HZ)
t_next = time.ticks_us()

print("Capturing {} samples at {} Hz...".format(N_SAMPLES, FS_HZ))
t0 = time.ticks_us()

for i in range(N_SAMPLES):
    # wait until scheduled time
    while time.ticks_diff(t_next, time.ticks_us()) > 0:
        pass

    off = i * sample_bytes
    i2c.readfrom_mem_into(mpu_addr, ACCEL_XOUT_H, mv[off:off+sample_bytes])
    t_next = time.ticks_add(t_next, period_us)

t1 = time.ticks_us()
dt = time.ticks_diff(t1, t0) / 1_000_000
print("Done. Effective rate: {:.1f} samples/s".format(N_SAMPLES / dt))

# -----------------------------
# Save CSV (convert to g)
# -----------------------------
print("Writing CSV:", OUT_FILE)
with open(OUT_FILE, "w") as f:
    f.write("# Fs_Hz,{}\n".format(FS_HZ))
    f.write("# N,{}\n".format(N_SAMPLES))
    f.write("t_us,ax_g,ay_g,az_g\n")

    for i in range(N_SAMPLES):
        off = i * sample_bytes
        ax_raw, ay_raw, az_raw = struct.unpack(">hhh", mv[off:off+6])  # big-endian int16
        ax = ax_raw / ACCEL_LSB_PER_G
        ay = ay_raw / ACCEL_LSB_PER_G
        az = az_raw / ACCEL_LSB_PER_G
        f.write("{},{:.6f},{:.6f},{:.6f}\n".format(i * period_us, ax, ay, az))

print("CSV saved.")