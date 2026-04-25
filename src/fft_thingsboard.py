import machine, network, time, math, ujson
from machine import SoftI2C, Pin, Timer
from mpu6050 import MPU
from umqtt.simple import MQTTClient
from ulab import numpy as np

i2c = SoftI2C(scl=Pin(14), sda=Pin(22))
mpu = MPU(i2c)

# ------Initialization------
WIFI_SSID = "" # Update your wifi SSID here
WIFI_PASSWORD = "" # Update your wifi password here

TB_TOKEN = "" # Update your Thingsboard token here
TB_SERVER = "thingsboard.cloud"
PORT = 1883

SAMPLE_RATE = 100 # sample rate at 100Hz (read MPU6050 every 10ms)
FFT_SIZE = 512 # 512 samples per FFT frame

# Global variables
avg_x, avg_y, avg_z = 0, 0, 0 # average accelerometer x, y, z after calibration

# Hanning window
hanning = np.array([0.5 * (1 - math.cos(2 * math.pi * i / (FFT_SIZE - 1))) for i in range(FFT_SIZE)])
coherent_gain = float(np.sum(hanning)) / FFT_SIZE

# ------Function------
# function for connecting to wifi
def wifi_connect(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    
    # reset wifi to avoid internal errors
    wlan.active(False)
    time.sleep(1)
    wlan.active(True)
    
    if not wlan.isconnected():
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            machine.idle()
            
    print(f'Connected to {ssid}')
    print(f'IP Address: {wlan.ifconfig()[0]}')
    
# function for MPU6050 accelerometer calibration
def calibrate_mpu():
    global avg_x, avg_y, avg_z
    num = 100 # number of samples
    sum_x, sum_y, sum_z = 0, 0, 0
    
    print("MPU6050 sensor calibration in progress, it will take around 25s, please keep the sensor board flat and still.")
    time.sleep(5)
    for _ in range(num): # update the sum of x, y, and z every 0.2s
        x, y, z = mpu.acceleration()
        sum_x += x
        sum_y += y
        sum_z += z
        time.sleep_ms(200)

    avg_x = sum_x / num # x should be 0m/s^2
    avg_y = sum_y / num # y should be 0m/s^2
    avg_z = sum_z / num # z should be 9.8m/s^2

    print(f"MPU6050 sensor calibration complete! Average: X={avg_x:.3f}, Y={avg_y:.3f}, Z={avg_z:.3f}")
    print("Motion detection system is ready!")

# function for collecting data for one fft frame
def collect_fft_frame(mpu, fft_size, sample_rate):
    buf_x = np.zeros(fft_size)
    buf_y = np.zeros(fft_size)
    buf_z = np.zeros(fft_size)
    idx = 0
    ready = False

    def update_buf(t):
        nonlocal idx, ready
        if ready:
            return
        x, y, z = mpu.acceleration()
        buf_x[idx] = x - avg_x
        buf_y[idx] = y - avg_y
        buf_z[idx] = z - avg_z
        idx += 1
        if idx >= fft_size:
            idx = 0
            ready = True
            
    fft_timer = Timer(0)
    fft_timer.init(period=1000 // sample_rate, mode=Timer.PERIODIC, callback=update_buf)
    # block until buffer is full
    while not ready:          
        pass
    fft_timer.deinit()
    return buf_x, buf_y, buf_z

# function for fft on one axis
def fft_per_axis(samples):
    half_N = FFT_SIZE // 2
    freq_res = SAMPLE_RATE / FFT_SIZE
    
    # fft
    fft_results = np.fft.fft(samples * hanning)
    magnitude = np.sqrt(fft_results.real[:half_N]**2 + fft_results.imag[:half_N]**2) * (2.0 / FFT_SIZE ) / coherent_gain
    magnitude[0] = 0.0

    # peak frequency
    peak_bin = int(np.argmax(magnitude))
    peak_freq = peak_bin * freq_res
    peak_amp = float(magnitude[peak_bin])

    # rms in time domain
    rms = math.sqrt(float(np.sum(samples**2)) / FFT_SIZE)
    
    print(f"peak_freq: {peak_freq}")
    print(f"peak_amp: {peak_amp}")
    print(f"rms: {rms}")

    return peak_freq, peak_amp, rms

# function for building Thingsboard payload
def thingsboard_payload(fft_x, fft_y, fft_z):
    
    peak_freq_x, peak_amp_x, rms_x = fft_x
    peak_freq_y, peak_amp_y, rms_y = fft_y
    peak_freq_z, peak_amp_z, rms_z = fft_z

    rms_total = math.sqrt(rms_x**2 + rms_y**2 + rms_z**2)

    if rms_total >= 10:
        vibration_level = "HIGH"
    elif rms_total >= 5:
        vibration_level = "MEDIUM"
    else:
        vibration_level = "LOW"

    payload = {
        "rx": rms_x,
        "ry": rms_y,
        "rz": rms_z,
        "rt": rms_total,
        "lvl": vibration_level,
        "fx": peak_freq_x,
        "ax": peak_amp_x,
        "fy": peak_freq_y,
        "ay": peak_amp_y,
        "fz": peak_freq_z,
        "az": peak_amp_z,
    }
    return ujson.dumps(payload)

# function for publishing to Thingsboard
def publish_results(payload):
    
    client = MQTTClient(
        client_id="vibration_sensor",
        server=TB_SERVER,
        port=PORT,
        user=TB_TOKEN,
        password=""
    )
    connected = False
    try:
        client.connect()
        connected = True
        client.publish("v1/devices/me/telemetry", payload)
        print("Published!")
    except Exception as e:
        print("Fail to connect to ThingsBoard:", e)
    finally:
        if connected:
            client.disconnect()

# ------Main------
calibrate_mpu() # calibrate MPU to find avg_x, avg_y, avg_z
wifi_connect(WIFI_SSID, WIFI_PASSWORD) # connect to wifi

while True:
    print("fft start after 10s:")
    time.sleep(10)
    print("fft start!")

    # collect fft frame
    buff_x, buff_y, buff_z = collect_fft_frame(mpu, FFT_SIZE, SAMPLE_RATE)

    # fft on each axis
    fft_x = fft_per_axis(buff_x)
    fft_y = fft_per_axis(buff_y)
    fft_z = fft_per_axis(buff_z)

    # thingsboard
    payload = thingsboard_payload(fft_x, fft_y, fft_z)
    publish_results(payload)


