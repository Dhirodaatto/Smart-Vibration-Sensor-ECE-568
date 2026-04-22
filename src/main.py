from machine import Pin, I2C, lightsleep
import time
import struct
import esp32

# ── I2C setup ─────────────────────────────────────────────
i2c = I2C(id=0, scl=Pin(20), sda=Pin(22), freq=400000)
MPU_ADDR = 0x68

# ── Pin configuration ────────────────────────────────────
INTERRUPT_PIN = 15
motion_interrupt = Pin(INTERRUPT_PIN, Pin.IN)
esp32.wake_on_ext0(pin=motion_interrupt, level=esp32.WAKEUP_ANY_HIGH)

# ── MPU-6050 Registers ───────────────────────────────────
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
INT_ENABLE = 0x38
INT_PIN_CFG = 0x37
INT_STATUS = 0x3A
MOT_THR = 0x1F
MOT_DUR = 0x20
MOT_DETECT_CTRL = 0x69
ACCEL_CONFIG = 0x1C

# ── Helper: Write to MPU registers ───────────────────────
def write_to_register(register, value):
    i2c.writeto_mem(MPU_ADDR, register, bytes([value]))

# ── Wake up MPU-6050 ─────────────────────────────────────
i2c.writeto_mem(MPU_ADDR, PWR_MGMT_1, b'\x00')
print("MPU-6050 initialized")

# ── Helper: Set up MPU motion interrupt ──────────────────
def configure_mpu_motion_interrupt():
    write_to_register(ACCEL_CONFIG, 0x10) # Set accelerometer range +/-8g (optional)
    write_to_register(MOT_THR, 5) # Motion threshold, higher=less sensitive, lower=more sensitive
    write_to_register(MOT_DUR, 5) # Motion duration, in 1ms increments
    write_to_register(MOT_DETECT_CTRL, 0x15) # Motion detection control
    write_to_register(INT_PIN_CFG, 0x20) # Interrupt pin: active high, push-pull, latch until cleared
    write_to_register(INT_ENABLE, 0x40) # Enable motion interrupt behavior
    print("MPU6050 motion interrupt configured.")

# ── Helper: Set up MPU motion interrupt ──────────────────
def clear_mpu_interrupt():
    i2c.readfrom_mem(MPU_ADDR, INT_STATUS, 1) # Interrupt won't clear until we read this register

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
# while True:
#     ax, ay, az, temp, gx, gy, gz = read_raw_data()
# 
#     ax_g = convert_accel(ax)
#     ay_g = convert_accel(ay)
#     az_g = convert_accel(az)
# 
#     temp_c = convert_temp(temp)
# 
#     print("Accel (g): X={:.3f} Y={:.3f} Z={:.3f}".format(ax_g, ay_g, az_g))
# 
#     time.sleep(0.5)

configure_mpu_motion_interrupt()
print("Entering sleep.")
time.sleep(0.1)
lightsleep()
print("Waking up.")
clear_mpu_interrupt()