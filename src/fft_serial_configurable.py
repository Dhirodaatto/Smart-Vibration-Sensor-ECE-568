#Implements input sel (ADC or Accelerometer) -> filt select (passthru, EMA, 2nd Ord Butter, 4th ord Butter) -> FFT -> Serial -> Plot

import time
import math
import array
import network
import socket, struct
import machine
import sys

from ulab import numpy as np
from ulab import utils as utils

from config import SSID, PSWD
from machine import Pin, ADC

## ----------------------- Global Variables ------------------
sampling_rate = 500 # Hz
sample_period = int(1000/sampling_rate) # period in ms

_accel_raw = bytearray(6) # Raw I2C read target — 6 bytes for X, Y, Z

FFT_SIZE = 512

hanning = np.array([0.5 * (1 - math.cos(2 * math.pi * i / (FFT_SIZE - 1))) for i in range(FFT_SIZE)])
coherent_gain = float(np.sum(hanning)) / FFT_SIZE

xf, yf, zf = [0,0], [0,0], [0,0]

# -------------------------------------------------
# EMA CONFIG
# -------------------------------------------------
ALPHA = 0.2
# -------------------------------------------------
# BIQUAD COEFFICIENTS
#
# Fc = 100 Hz
# Fs = 500 Hz
# -------------------------------------------------

B0 = 0.2066
B1 = 0.4131
B2 = 0.2066

A1 = -0.3695
A2 = 0.1958

# -----------------------------
# 2nd-order biquad states
# -----------------------------

bx1 = 0
bx2 = 0
by1 = 0
by2 = 0
bz1 = 0
bz2 = 0

ax1 = 0
ax2 = 0
ay1 = 0
ay2 = 0
az1 = 0
az2 = 0

# -----------------------------
# 4th-order biquad cascade
# Stage A states
# -----------------------------

bx1a = 0
bx2a = 0
by1a = 0
by2a = 0
bz1a = 0
bz2a = 0

ax1a = 0
ax2a = 0
ay1a = 0
ay2a = 0
az1a = 0
az2a = 0

# -----------------------------
# Stage B states
# -----------------------------

bx1b = 0
bx2b = 0
by1b = 0
by2b = 0
bz1b = 0
bz2b = 0

ax1b = 0
ax2b = 0
ay1b = 0
ay2b = 0
az1b = 0
az2b = 0

# -------------------------------------------------

raw_x = [array.array('h', [0] * FFT_SIZE), array.array('h', [0] * FFT_SIZE)]
raw_y = [array.array('h', [0] * FFT_SIZE), array.array('h', [0] * FFT_SIZE)]
raw_z = [array.array('h', [0] * FFT_SIZE), array.array('h', [0] * FFT_SIZE)]

flag_data_ready = False
fft_data_buffer = None
current_buffer = 0
buf_ni = 0

acc_xyz = 0x3B
mpu_addr = None
ACC_SCALE = 16384.0 / 9.81
seq = 100

# ## ----------------------- WiFi ------------------
# def wifi_connect():
#     wlan = network.WLAN(network.STA_IF)
#     wlan.active(True)
#     if not wlan.isconnected():
#         wlan.connect(SSID, PSWD)
#         print(wlan.status())
#         print([ap[0] for ap in wlan.scan()])
#         while not wlan.isconnected():
#             machine.idle()
#     print("Connected")
#     print(wlan.ipconfig('addr4')[0])


## ----------------------- ADC Setup ------------------
adc = ADC(Pin(34))
adc.atten(ADC.ATTN_11DB)

# -------------------------------------------------
# INPUT SELECT
#
# 0 = MPU6050
# 1 = ADC
# -------------------------------------------------

INPUT_TYPE = 1

# -------------------------------------------------
# FILTER SELECT
#
# 0 = BYPASS
# 1 = EMA
# 2 = 2ND ORDER BIQUAD
# 3 = 4TH ORDER BIQUAD CASCADE
# -------------------------------------------------

FILT_TYPE = 0


## ----------------------- ISR ------------------------
@micropython.native
def update_buffers(t):
    global _accel_raw, raw_x, raw_y, raw_z
    global fft_data_buffer, flag_data_ready, current_buffer, buf_ni
    global xf, yf, zf
    
    # 2nd-order biquad states
    global bx1, bx2, by1, by2, bz1, bz2
    global ax1, ax2, ay1, ay2, az1, az2

    # 4th-order cascade stage A
    global bx1a, bx2a, by1a, by2a, bz1a, bz2a
    global ax1a, ax2a, ay1a, ay2a, az1a, az2a

    # 4th-order cascade stage B
    global bx1b, bx2b, by1b, by2b, bz1b, bz2b
    global ax1b, ax2b, ay1b, ay2b, az1b, az2b

#     i2c.readfrom_mem_into(mpu_addr, acc_xyz, _accel_raw)
# 
#     x = (_accel_raw[0] << 8) | _accel_raw[1]
#     if x >= 0x8000: x -= 0x10000
# 
#     y = (_accel_raw[2] << 8) | _accel_raw[3]
#     if y >= 0x8000: y -= 0x10000
# 
#     z = (_accel_raw[4] << 8) | _accel_raw[5]
#     if z >= 0x8000: z -= 0x10000

    
    # -------------------------------------------------
    # INPUT SELECT
    #
    # 0 = MPU6050
    # 1 = ADC
    # -------------------------------------------------

    if INPUT_TYPE == 0:

    # -------------------------
    # MPU6050 INPUT
    # -------------------------

        i2c.readfrom_mem_into(mpu_addr, acc_xyz, _accel_raw)

        x = (_accel_raw[0] << 8) | _accel_raw[1]
        if x >= 0x8000:
            x -= 0x10000

        y = (_accel_raw[2] << 8) | _accel_raw[3]
        if y >= 0x8000:
            y -= 0x10000

        z = (_accel_raw[4] << 8) | _accel_raw[5]
        if z >= 0x8000:
            z -= 0x10000

    else:

        # -------------------------
        # ADC INPUT
        # -------------------------

        adc_val = adc.read()

        # center ADC around zero
        adc_val -= 2048

        # use same signal on all channels
        x = adc_val
        y = adc_val
        z = adc_val
        
        
        
    if FILT_TYPE == 0:

        x_f = x
        y_f = y
        z_f = z

    # -------------------------
    # EMA FILTER
    # -------------------------
    elif FILT_TYPE == 1:

        xf[0] += ALPHA * (x - xf[0])
        xf[1] += ALPHA * (xf[0] - xf[1])

        yf[0] += ALPHA * (y - yf[0])
        yf[1] += ALPHA * (yf[0] - yf[1])

        zf[0] += ALPHA * (z - zf[0])
        zf[1] += ALPHA * (zf[0] - zf[1])

        x_f = xf[1]
        y_f = yf[1]
        z_f = zf[1]

    # -------------------------
    # 2ND ORDER BIQUAD
    # -------------------------
    elif FILT_TYPE == 2:

        # X axis
        x_f = B0*x + B1*bx1 + B2*bx2 - A1*ax1 - A2*ax2
        bx2 = bx1
        bx1 = x
        ax2 = ax1
        ax1 = x_f

        # Y axis
        y_f = B0*y + B1*by1 + B2*by2 - A1*ay1 - A2*ay2
        by2 = by1
        by1 = y
        ay2 = ay1
        ay1 = y_f

        # Z axis
        z_f = B0*z + B1*bz1 + B2*bz2 - A1*az1 - A2*az2
        bz2 = bz1
        bz1 = z
        az2 = az1
        az1 = z_f

    # -------------------------
    # 4TH ORDER BIQUAD CASCADE
    # -------------------------
    else:

        # ----- STAGE A -----

        x1 = B0*x + B1*bx1a + B2*bx2a - A1*ax1a - A2*ax2a
        bx2a = bx1a
        bx1a = x
        ax2a = ax1a
        ax1a = x1

        y1 = B0*y + B1*by1a + B2*by2a - A1*ay1a - A2*ay2a
        by2a = by1a
        by1a = y
        ay2a = ay1a
        ay1a = y1

        z1 = B0*z + B1*bz1a + B2*bz2a - A1*az1a - A2*az2a
        bz2a = bz1a
        bz1a = z
        az2a = az1a
        az1a = z1

        # ----- STAGE B -----

        x_f = B0*x1 + B1*bx1b + B2*bx2b - A1*ax1b - A2*ax2b
        bx2b = bx1b
        bx1b = x1
        ax2b = ax1b
        ax1b = x_f

        y_f = B0*y1 + B1*by1b + B2*by2b - A1*ay1b - A2*ay2b
        by2b = by1b
        by1b = y1
        ay2b = ay1b
        ay1b = y_f

        z_f = B0*z1 + B1*bz1b + B2*bz2b - A1*az1b - A2*az2b
        bz2b = bz1b
        bz1b = z1
        az2b = az1b
        az1b = z_f

    # -------------------------------------------------
    # STORE INTO ACTIVE BUFFER
    # -------------------------------------------------

    raw_x[current_buffer][buf_ni] = int(x_f + 0.5)
    raw_y[current_buffer][buf_ni] = int(y_f + 0.5)
    raw_z[current_buffer][buf_ni] = int(z_f + 0.5)


#         # 2nd order low pass filter
#     xf[0] = xf[0] + ALPHA * (x     - xf[0])
#     xf[1] = xf[1] + ALPHA * (xf[0] - xf[1])
# 
#     yf[0] = yf[0] + ALPHA * (y     - yf[0])
#     yf[1] = yf[1] + ALPHA * (yf[0] - yf[1])
# 
#     zf[0] = zf[0] + ALPHA * (z     - zf[0])
#     zf[1] = zf[1] + ALPHA * (zf[0] - zf[1])
#     
#     
# 
# #     raw_x[current_buffer][buf_ni] = int(xf[1])
# #     raw_y[current_buffer][buf_ni] = int(yf[1])
# #     raw_z[current_buffer][buf_ni] = int(zf[1])
# 
#     raw_x[current_buffer][buf_ni] = x
#     raw_y[current_buffer][buf_ni] = y
#     raw_z[current_buffer][buf_ni] = z
    

    buf_ni += 1

    if buf_ni >= FFT_SIZE:
        fft_data_buffer = current_buffer
        current_buffer = 1 - current_buffer
        buf_ni = 0
        flag_data_ready = True

## ----------------------- FFT + SERIAL OUTPUT ------------------
def process_buffers_and_send(raw_x, raw_y, raw_z):
    global seq

    x_arr = np.array(raw_x, dtype=np.float) / ACC_SCALE
    y_arr = np.array(raw_y, dtype=np.float) / ACC_SCALE
    z_arr = np.array(raw_z, dtype=np.float) / ACC_SCALE

    x_arr = x_arr - np.mean(x_arr)
    y_arr = y_arr - np.mean(y_arr)
    z_arr = z_arr - np.mean(z_arr)

    fft_x = utils.spectrogram(x_arr * hanning) * (2.0 / FFT_SIZE) / coherent_gain
    fft_y = utils.spectrogram(y_arr * hanning) * (2.0 / FFT_SIZE) / coherent_gain
    fft_z = utils.spectrogram(z_arr * hanning) * (2.0 / FFT_SIZE) / coherent_gain

    mag = fft_z[:FFT_SIZE // 2]

    try:
        print(seq, end=",")
        print(",".join(str(v) for v in mag))
        seq += 1
        return True
    except:
        return False

## ----------------------- Setup ------------------
i2c = machine.I2C(0, scl=machine.Pin(14), sda=machine.Pin(22), freq=400000)
mpu_addr = i2c.scan()[0]
i2c.writeto(0x68, bytearray([107,0]))
time.sleep_ms(100)

print("MPU6050 ready")

# wifi_connect()

timer0 = machine.Timer(0)
timer0.init(mode=machine.Timer.PERIODIC, period=sample_period, callback=update_buffers)

## ----------------------- Main Loop ------------------
print("Running...")

while True:
    if flag_data_ready:
        process_buffers_and_send(raw_x[fft_data_buffer],
                                 raw_y[fft_data_buffer],
                                 raw_z[fft_data_buffer])
        flag_data_ready = False

