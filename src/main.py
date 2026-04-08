from machine import Pin, I2C
import time
import struct

# ── I2C setup ─────────────────────────────────────────────
i2c = I2C(id=0, scl=Pin(20), sda=Pin(22), freq=400000)

MPU_ADDR = 0x68

# ── MPU-6050 Registers ───────────────────────────────────
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# ── Wake up MPU-6050 ─────────────────────────────────────
i2c.writeto_mem(MPU_ADDR, PWR_MGMT_1, b'\x00')

print("MPU-6050 initialized")

# ── Helper: read signed 16-bit values ────────────────────
def read_raw_data():
    data = i2c.readfrom_mem(MPU_ADDR, ACCEL_XOUT_H, 14)
    return struct.unpack(">hhhhhhh", data)
    # ax, ay, az, temp, gx, gy, gz

# ── Conversion helpers ───────────────────────────────────
def convert_accel(val):
    return val / 16384.0  # ±2g

def convert_gyro(val):
    return val / 131.0  # ±250°/s

def convert_temp(val):
    return (val / 340.0) + 36.53

# ── Main loop ────────────────────────────────────────────
while True:
    ax, ay, az, temp, gx, gy, gz = read_raw_data()

    ax_g = convert_accel(ax)
    ay_g = convert_accel(ay)
    az_g = convert_accel(az)

    gx_d = convert_gyro(gx)
    gy_d = convert_gyro(gy)
    gz_d = convert_gyro(gz)

    temp_c = convert_temp(temp)

    print("Accel (g): X={:.3f} Y={:.3f} Z={:.3f}".format(ax_g, ay_g, az_g))
    print("Gyro (°/s): X={:.3f} Y={:.3f} Z={:.3f}".format(gx_d, gy_d, gz_d))
    print("Temp: {:.2f} °C".format(temp_c))
    print("-" * 40)

    time.sleep(0.5)
