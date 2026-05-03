from machine import Pin, I2C, deepsleep
import machine
import time
import struct
import esp32

# ── I2C setup ─────────────────────────────────────────────
i2c = I2C(id=0, scl=Pin(20), sda=Pin(22), freq=400000)
MPU_ADDR = 0x68

# ── Pin configuration ────────────────────────────────────
motion_interrupt = Pin(15, Pin.IN)
button = Pin(38, Pin.IN)

# ── MPU-6050 Registers ───────────────────────────────────
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
INT_ENABLE = 0x38
INT_PIN_CFG = 0x37
INT_STATUS = 0x3A
MOT_THR = 0x1F # MOTION THRESHOLD - MAY NEED ADJUSTMENT
MOT_DUR = 0x20 # MOTION DURATION - MAY NEED ADJUSTMENT
MOT_DETECT_CTRL = 0x69
ACCEL_CONFIG = 0x1C

# ── Set Up Wake Sources ──────────────────────────────────
SLEEP_TIME = 30_000
esp32.wake_on_ext0(pin=motion_interrupt, level=esp32.WAKEUP_ANY_HIGH)
esp32.wake_on_ext1(pins=(button,), level=esp32.WAKEUP_ALL_LOW)

# ── Data collection stub functions ───────────────────────
def collect_data_for_motion_interrupt():
    print('motion')
def collect_data_for_button_push():
    print('button')
def collect_data_for_regular_interval():
    print('normal')

# ── Process Wake Source ──────────────────────────────────
POWER_ON_RESET = 0
WAKE_ON_EXT0 = 1
WAKE_ON_EXT1 = 2
WAKE_ON_TIMER = 3
wake_source = machine.wake_reason()

if wake_source == WAKE_ON_EXT0:
    collect_data_for_motion_interrupt()
elif wake_source == WAKE_ON_EXT1:
    collect_data_for_button_push()
elif wake_source == WAKE_ON_TIMER:
    collect_data_for_regular_interval()
else:
    pass

# ── Helper: Write to MPU registers ───────────────────────
def write_to_register(register, value):
    i2c.writeto_mem(MPU_ADDR, register, bytes([value]))

# ── Wake up MPU-6050 ─────────────────────────────────────
i2c.writeto_mem(MPU_ADDR, PWR_MGMT_1, b'\x00')

# ── Helper: Set up MPU motion interrupt ──────────────────
def configure_mpu_motion_interrupt():
    write_to_register(ACCEL_CONFIG, 0x10) # Set accelerometer range +/-8g (optional)
    write_to_register(MOT_THR, 5) # Motion threshold, higher=less sensitive, lower=more sensitive
    write_to_register(MOT_DUR, 5) # Motion duration, in 1ms increments
    write_to_register(MOT_DETECT_CTRL, 0x15) # Motion detection control
    write_to_register(INT_PIN_CFG, 0x20) # Interrupt pin: active high, push-pull, latch until cleared
    write_to_register(INT_ENABLE, 0x40) # Enable motion interrupt behavior

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

# ── Enter deep sleep ─────────────────────────────────────
def enter_sleep():
    deepsleep(SLEEP_TIME)
    
clear_mpu_interrupt()
time.sleep(2)
configure_mpu_motion_interrupt()
enter_sleep()