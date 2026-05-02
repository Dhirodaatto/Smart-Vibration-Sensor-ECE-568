# 05012026 This file implements accelerometer -> filter -> FFT -> Log. It is untested.
# The accel uses the MPU FIFO to prevent samples from being missed.
# The filter is configurable to be either EMA/Biquad/None via FILTER_TYPE/ENABLE
# The FFT is implemented with a ping pong buffer and ulab.
# There are no hardware timers and sampling frequency determinism is enforced in the main loop.
# This version also implments the MQTTClient. Can ignore/comment it out for now.

from machine import Pin, I2C
import time, math, ujson
from ulab import numpy as np
from umqtt.simple import MQTTClient
import network

# =============================
# CONFIG
# =============================
FS = 1000
FFT_SIZE = 256   # safer with WiFi active
DT_US = 1000

FILTER_ENABLE = True
FILTER_TYPE   = "EMA"   # "EMA" or "BIQUAD"

# EMA
ALPHA = 0.5

# BIQUAD (200 Hz Butterworth @ 1 kHz)
b0 = 0.391335
b1 = 0.782670
b2 = 0.391335
a1 = -0.369527
a2 = 0.195816

# WiFi / MQTT
WIFI_SSID = ""
WIFI_PASSWORD = ""
TB_TOKEN = ""
TB_SERVER = "thingsboard.cloud"
PORT = 1883

# =============================
# I2C + MPU FIFO
# =============================
i2c = I2C(scl=Pin(14), sda=Pin(22), freq=400000)
ADDR = 0x68

i2c.writeto_mem(ADDR, 0x6B, b'\x00')
i2c.writeto_mem(ADDR, 0x1A, b'\x00')   # DLPF off
i2c.writeto_mem(ADDR, 0x19, b'\x00')   # 1 kHz
i2c.writeto_mem(ADDR, 0x6A, b'\x40')   # FIFO enable
i2c.writeto_mem(ADDR, 0x23, b'\x08')   # accel to FIFO

# =============================
# FIFO HELPERS
# =============================
def fifo_count():
    d = i2c.readfrom_mem(ADDR, 0x72, 2)
    return (d[0] << 8) | d[1]

def read_sample():
    raw = i2c.readfrom_mem(ADDR, 0x74, 6)
    ax = (raw[0] << 8) | raw[1]
    if ax & 0x8000: ax -= 65536
    return ax / 16384.0

# =============================
# WIFI
# =============================
def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            time.sleep_ms(100)

# =============================
# FFT PRECOMPUTE
# =============================
hanning = np.array([0.5*(1-math.cos(2*math.pi*i/(FFT_SIZE-1))) for i in range(FFT_SIZE)])
coherent_gain = float(np.sum(hanning)) / FFT_SIZE

# =============================
# BUFFERS (PING-PONG)
# =============================
buf_a = np.zeros(FFT_SIZE)
buf_b = np.zeros(FFT_SIZE)

# =============================
# FILTER STATE
# =============================
x1 = x2 = 0.0
y1 = y2 = 0.0
y_ema = 0.0

# =============================
# FILTER FUNCTION
# =============================
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
# FFT + FEATURE EXTRACTION
# =============================
def fft_process(samples):
    half_N = FFT_SIZE // 2
    freq_res = FS / FFT_SIZE

    windowed = samples * hanning
    fft_res = np.fft.fft(windowed)

    mag = np.sqrt(fft_res.real[:half_N]**2 + fft_res.imag[:half_N]**2)
    mag = (mag * (2.0 / FFT_SIZE)) / coherent_gain
    mag[0] = 0.0

    peak_bin = int(np.argmax(mag))
    peak_freq = peak_bin * freq_res
    peak_amp = float(mag[peak_bin])

    rms = math.sqrt(float(np.sum(samples**2)) / FFT_SIZE)

    return peak_freq, peak_amp, rms

# =============================
# MQTT
# =============================
def publish(payload):
    client = MQTTClient("esp32", TB_SERVER, port=PORT, user=TB_TOKEN)
    try:
        client.connect()
        client.publish("v1/devices/me/telemetry", payload)
        client.disconnect()
    except:
        pass

# =============================
# MAIN LOOP
# =============================
wifi_connect()

while True:

    active = buf_a
    processing = buf_b

    t_next = time.ticks_us()

    # =============================
    # ACQUIRE BUFFER (REAL-TIME SAFE)
    # =============================
    for i in range(FFT_SIZE):

        while time.ticks_diff(time.ticks_us(), t_next) < 0:
            pass
        t_next = time.ticks_add(t_next, DT_US)

        while fifo_count() < 6:
            pass

        x = read_sample()
        y = apply_filter(x)

        active[i] = y

    # =============================
    # SWAP BUFFERS
    # =============================
    active, processing = processing, active

    # =============================
    # FFT + FEATURES
    # =============================
    peak_f, peak_a, rms = fft_process(processing)

    # =============================
    # PAYLOAD
    # =============================
    payload = ujson.dumps({
        "freq": peak_f,
        "amp": peak_a,
        "rms": rms,
        "filter": FILTER_ENABLE
    })

    publish(payload)

    print("freq:", peak_f, "amp:", peak_a, "rms:", rms)