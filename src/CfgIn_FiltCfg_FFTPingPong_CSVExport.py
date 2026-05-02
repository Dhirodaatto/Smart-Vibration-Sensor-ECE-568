# 05012026 This file implements CfgIn -> filter -> FFT -> Log. It is untested.
# The input to the filter is cfg via constant (ADC or Accel).
# The accel uses the MPU FIFO to prevent samples from being missed.
# The filter is configurable to be either EMA/Biquad/None via FILTER_TYPE/ENABLE
# The FFT is implemented with a ping pong buffer and ulab.
# There are no hardware timers and sampling frequency determinism is enforced in the main loop.
# This version also implments a CSV export to be used in conjunction with a PC-side script
# titled FFT_Export.py

from machine import Pin, I2C, ADC
import time, math
from ulab import numpy as np

# =============================
# CONFIG
# =============================
FS = 1000
FFT_SIZE = 256
DT_US = 1000

INPUT_SOURCE = "ADC"   # "ADC" or "I2C"

FILTER_ENABLE = True
FILTER_TYPE   = "EMA"   # "EMA" or "BIQUAD"

ALPHA = 0.5

# BIQUAD (200 Hz Butterworth)
b0 = 0.391335
b1 = 0.782670
b2 = 0.391335
a1 = -0.369527
a2 = 0.195816

# =============================
# ADC SETUP
# =============================
SIGNAL_PIN = 34
adc = ADC(Pin(SIGNAL_PIN))
adc.atten(ADC.ATTN_11DB)
time.sleep(2)

# =============================
# I2C + MPU FIFO (optional)
# =============================
i2c = I2C(scl=Pin(14), sda=Pin(22), freq=400000)
ADDR = 0x68

i2c.writeto_mem(ADDR, 0x6B, b'\x00')
i2c.writeto_mem(ADDR, 0x1A, b'\x00')
i2c.writeto_mem(ADDR, 0x19, b'\x00')
i2c.writeto_mem(ADDR, 0x6A, b'\x40')
i2c.writeto_mem(ADDR, 0x23, b'\x08')

def fifo_count():
    d = i2c.readfrom_mem(ADDR, 0x72, 2)
    return (d[0] << 8) | d[1]

def read_i2c():
    raw = i2c.readfrom_mem(ADDR, 0x74, 6)
    ax = (raw[0] << 8) | raw[1]
    if ax & 0x8000:
        ax -= 65536
    return ax / 16384.0

# =============================
# INPUT SWITCH
# =============================
def read_input():
    if INPUT_SOURCE == "ADC":
        raw = adc.read()  # 0–4095
        return (raw - 2048) / 2048.0   # center + normalize
    else:
        while fifo_count() < 6:
            pass
        return read_i2c()

# =============================
# FFT PRECOMPUTE
# =============================
hanning = np.array([0.5*(1-math.cos(2*math.pi*i/(FFT_SIZE-1))) for i in range(FFT_SIZE)])
coherent_gain = float(np.sum(hanning)) / FFT_SIZE

# =============================
# BUFFERS
# =============================
buf_a = np.zeros(FFT_SIZE)
buf_b = np.zeros(FFT_SIZE)

# =============================
# FILTER STATE
# =============================
x1 = x2 = 0.0
y1 = y2 = 0.0
y_ema = 0.0

def apply_filter(x):
    global x1, x2, y1, y2, y_ema

    if not FILTER_ENABLE:
        return x

    if FILTER_TYPE == "EMA":
        y_ema = y_ema + ALPHA * (x - y_ema)
        return y_ema

    elif FILTER_TYPE == "BIQUAD":
        y = (b0*x + b1*x1 + b2*x2
             - a1*y1 - a2*y2)
        x2 = x1
        x1 = x
        y2 = y1
        y1 = y
        return y

    return x

# =============================
# FFT PROCESS
# =============================
def fft_process(samples):
    half_N = FFT_SIZE // 2
    freq_res = FS / FFT_SIZE

    windowed = samples * hanning
    fft_res = np.fft.fft(windowed)

    mag = np.sqrt(fft_res.real[:half_N]**2 + fft_res.imag[:half_N]**2)
    mag = (mag * (2.0 / FFT_SIZE)) / coherent_gain
    mag[0] = 0.0

    return mag, freq_res

# =============================
# MAIN LOOP
# =============================
while True:

    active = buf_a
    processing = buf_b

    t_next = time.ticks_us()

    # =============================
    # ACQUIRE BUFFER
    # =============================
    for i in range(FFT_SIZE):

        while time.ticks_diff(time.ticks_us(), t_next) < 0:
            pass
        t_next = time.ticks_add(t_next, DT_US)

        x = read_input()
        y = apply_filter(x)

        active[i] = y

    # =============================
    # SWAP BUFFERS
    # =============================
    active, processing = processing, active

    # =============================
    # FFT
    # =============================
    mag, freq_res = fft_process(processing)

    # =============================
    # SERIAL OUTPUT (CSV)
    # =============================
    print("FFT_FRAME_START")
    print("bin,frequency,amplitude")

    for i in range(len(mag)):
        freq = i * freq_res
        print("{},{:.2f},{:.6f}".format(i, freq, mag[i]))

    print("FFT_FRAME_END")

    # Add back in to see filter input samples on plot
    # print("TIME_FRAME_START")
    # print("index,filtered")

    # for i in range(FFT_SIZE):
    #     print("{},{:.6f}".format(i, processing[i]))

    # print("TIME_FRAME_END")

    time.sleep(1)