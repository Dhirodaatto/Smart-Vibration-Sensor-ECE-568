import time
import math
import array
import network
import socket, struct
import machine
import json
import ntptime
import esp32

from ulab import numpy as np
from ulab import utils as utils
from umqtt.simple import MQTTClient
from config import SSID, PSWD

## ----------------------- Global Variables ------------------
# SSID = "Enter Your SSID"
# PSWD = "Enter Your Password"
TB_BROKER = "mqtt.thingsboard.cloud"
TB_USER   = b"MOvaSPQTLj4mOPLQEGOd"
TB_TOPIC  = b"v1/devices/me/telemetry"
first_message = True

sampling_rate = 500 # Hz
sample_period = int(1000/sampling_rate) # period in ms

_accel_raw = bytearray(6) # Raw I2C read target — 6 bytes for X, Y, Z (big-endian int16 each)

FFT_SIZE = 512 # Number of bins for FFT
hanning = np.array([0.5 * (1 - math.cos(2 * math.pi * i / (FFT_SIZE - 1))) for i in range(FFT_SIZE)]) # Hanning window
coherent_gain = float(np.sum(hanning)) / FFT_SIZE

# Variables for low pass filter
xf, yf, zf = [0,0], [0,0], [0,0]
ALPHA = 0.9 # Please change based on experiments!!!

# Three pairs of ping-pong buffers — one pair per axis
raw_x = [array.array('h', [0] * FFT_SIZE), array.array('h', [0] * FFT_SIZE)]
raw_y = [array.array('h', [0] * FFT_SIZE), array.array('h', [0] * FFT_SIZE)]
raw_z = [array.array('h', [0] * FFT_SIZE), array.array('h', [0] * FFT_SIZE)]

# Variables for passing information when buffer is full
flag_data_ready = False
fft_data_buffer = None
current_buffer = 0
buf_ni = 0

acc_xyz = 0x3B
mpu_addr = None
ACC_SCALE = 16384.0 / 9.81
calibration_offset = (0,0,0)
at_still = (0,0,9.8)

last_duration_us = 0

# ThingsBoard userinput (shared attributes)
amp_threshold = 5.0 # m/s²
freq_low = 50.0 # Hz
freq_high = 100.0 # Hz

# Input configuration
motion_interrupt = machine.Pin(15, machine.Pin.IN)
button = machine.Pin(38, machine.Pin.IN)

# MPU-6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
INT_ENABLE = 0x38
INT_PIN_CFG = 0x37
INT_STATUS = 0x3A
MOT_THR = 0x1F # MOTION THRESHOLD - MAY NEED ADJUSTMENT
MOT_DUR = 0x20 # MOTION DURATION - MAY NEED ADJUSTMENT
MOT_DETECT_CTRL = 0x69
ACCEL_CONFIG = 0x1C

# Wake Sources
WAKE_SOURCE = machine.wake_reason()
POWER_ON_RESET = 0
WAKE_ON_EXT0 = 1
WAKE_ON_EXT1 = 2
WAKE_ON_TIMER = 3

SLEEP_TIME = 30_000
esp32.wake_on_ext0(pin=motion_interrupt, level=esp32.WAKEUP_ANY_HIGH)
esp32.wake_on_ext1(pins=(button,), level=esp32.WAKEUP_ALL_LOW)

## ----------------------- Function Declaration -------------
def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(SSID, PSWD)
        while not wlan.isconnected():
            machine.idle()
    print(f'Connected to {wlan.config("ssid")}')
    print(f'IP address: {wlan.ipconfig("addr4")[0]}\n')
    for attempt in range(5):
        try:
            ntptime.settime()
            print(f"Time synced! UNIX timestamp: {time.time() + 946684800}")
            break
        except Exception as e:
            print(f"NTP sync attempt {attempt + 1} failed: {e}")
            time.sleep(1)

def mqtt_callback(topic, msg):
    global amp_threshold, freq_low, freq_high
    try:
        data = json.loads(msg)
        
        # payload in {"shared": {...}}
        if b'response' in topic:
            data = data.get('shared', data)
        if 'amp_threshold' in data:
            amp_threshold = float(data['amp_threshold'])
            print(f'Updated amp_threshold = {amp_threshold}')
        if 'freq_low' in data:
            freq_low = float(data['freq_low'])
            print(f'Updated freq_low = {freq_low}')
        if 'freq_high' in data:
            freq_high = float(data['freq_high'])
            print(f'Updated freq_high = {freq_high}')
    except Exception as e:
        print(f'MQTT callback error: {e}')
            
def get_calibration_param():
    global calibration_offset
    print("Starting Calibration .....")
    N = 10
    calibration_offset = (0,0,0)
    for i in range(0,N):
        accel_raw = i2c.readfrom_mem(mpu_addr, acc_xyz, 6)
        x,y,z = struct.unpack('>hhh', accel_raw)
        a = x / ACC_SCALE, y / ACC_SCALE, z / ACC_SCALE
        calibration_offset = [calibration_offset[n] + (at_still[n] - a[n]) for n in range(0,3)]
        time.sleep(0.01)
    calibration_offset = [calibration_offset[n]/N for n in range(0,3)]
    print(f'Calibration Complete! Calibration Offset = {calibration_offset}')

@micropython.native
def update_buffers(t):
    global _accel_raw, raw_x, raw_y, raw_z
    global fft_data_buffer, flag_data_ready, current_buffer, buf_ni
    global xf, yf, zf
    global last_duration_us
    t0 = time.ticks_us()

    # Load from memory into var
    i2c.readfrom_mem_into(mpu_addr, acc_xyz, _accel_raw)

    # Get XYZ accelerometer from raw
    x = (_accel_raw[0] << 8) | _accel_raw[1]
    if x >= 0x8000:
        x -= 0x10000
    y = (_accel_raw[2] << 8) | _accel_raw[3]
    if y >= 0x8000:
        y -= 0x10000
    z = (_accel_raw[4] << 8) | _accel_raw[5]
    if z >= 0x8000:
        z -= 0x10000

    # 2nd order low pass filter / exponential moving average
    xf[0] = xf[0] + ALPHA * (x     - xf[0])
    xf[1] = xf[1] + ALPHA * (xf[0] - xf[1])

    yf[0] = yf[0] + ALPHA * (y     - yf[0])
    yf[1] = yf[1] + ALPHA * (yf[0] - yf[1])

    zf[0] = zf[0] + ALPHA * (z     - zf[0])
    zf[1] = zf[1] + ALPHA * (zf[0] - zf[1])

    # Update ping-pong buffers / Variables in comments should replace the current values to disable the low pass filter (NOTE)
    raw_x[current_buffer][buf_ni] = int(xf[1]) #x
    raw_y[current_buffer][buf_ni] = int(yf[1]) #y
    raw_z[current_buffer][buf_ni] = int(zf[1]) #z
    buf_ni += 1

    if buf_ni >= FFT_SIZE:
        fft_data_buffer = current_buffer # indicate data buffer
        current_buffer = 1 - current_buffer # swap buffers
        buf_ni = 0 # reset pointer to start of new buffer
        flag_data_ready = True # flag for processing enable

    last_duration_us = time.ticks_diff(time.ticks_us(), t0)

def process_buffers_and_send(raw_x, raw_y, raw_z):
    # Convert raw buffers to np arrays
    x_arr = np.array(raw_x, dtype=np.float) / ACC_SCALE
    y_arr = np.array(raw_y, dtype=np.float) / ACC_SCALE
    z_arr = np.array(raw_z, dtype=np.float) / ACC_SCALE

    # Adjust via calibration offset
    x_arr_corr = x_arr + calibration_offset[0]
    y_arr_corr = y_arr + calibration_offset[1]
    z_arr_corr = z_arr + calibration_offset[2]

    # Remove DC component before FFT
    x_arr = x_arr - np.mean(x_arr)
    y_arr = y_arr - np.mean(y_arr)
    z_arr = z_arr - np.mean(z_arr)

    # FFT with corrections for hanning window
    fft_x = utils.spectrogram(x_arr * hanning) * (2.0 / FFT_SIZE) / coherent_gain
    fft_y = utils.spectrogram(y_arr * hanning) * (2.0 / FFT_SIZE) / coherent_gain
    fft_z = utils.spectrogram(z_arr * hanning) * (2.0 / FFT_SIZE) / coherent_gain

    peakmag_x, peakfreq_x = np.max(fft_x), int(np.argmax(fft_x)) * sampling_rate / FFT_SIZE
    peakmag_y, peakfreq_y = np.max(fft_y), int(np.argmax(fft_y)) * sampling_rate / FFT_SIZE
    peakmag_z, peakfreq_z = np.max(fft_z), int(np.argmax(fft_z)) * sampling_rate / FFT_SIZE

    rms_x = math.sqrt(float(np.sum(x_arr**2)) / FFT_SIZE)
    rms_y = math.sqrt(float(np.sum(y_arr**2)) / FFT_SIZE)
    rms_z = math.sqrt(float(np.sum(z_arr**2)) / FFT_SIZE)
    total_rms = math.sqrt(rms_x**2 + rms_y**2 + rms_z**2)

    # Build payload lists — only the first half of the FFT (positive frequencies)
    freq_bins  = [round(k * sampling_rate / FFT_SIZE, 3) for k in range(FFT_SIZE // 2)]
    amplitudes = [round(float(v), 6) for v in fft_z[:FFT_SIZE // 2].tolist()]

    global first_message

    # Check against shared attributes
#     amp_detected = 'Y' if peakmag_z >= amp_threshold else 'N'
#     freq_detected = 'Y' if freq_low <= peakfreq_z <= freq_high else 'N'
    vib_detected = 'Y' if any(fft_z[i] >= amp_threshold for i, f in enumerate(freq_bins) if freq_low <= f <= freq_high) else 'N'

    payload_dict = {
        "amplitudes": amplitudes,
        "freq_bins": freq_bins,
        "total_rms": round(total_rms, 6),
        "fz": round(peakfreq_z, 4),
        "az": round(peakmag_z, 4),
#         "checkamp":  amp_detected,
#         "checkfreq": freq_detected,
        "checkvib": vib_detected,
    }

    if first_message:
        unix_epoch_offset = 946684800  # seconds between 1970 and 2000
        payload_dict["start_ts"] = (time.time() + unix_epoch_offset) * 1000
        first_message = False

    payload = json.dumps(payload_dict)

    # MQTT upload
    try:
        client.publish(TB_TOPIC, payload)
        print(f'Uploaded to ThingsBoard | total_rms={total_rms:.4f} peak_amp={peakmag_z:.4f} peak_freq={peakfreq_z:.4f}')
        print(f'Uploaded to ThingsBoard | vibration in detection range-{vib_detected}') 
        return True
    except OSError as e:
        print("Network error (OSError):", e)
    except TypeError as e:
        print("Handshake failed (NoneType error). Server didn't respond in time.")
    except Exception as e:
        print("Unexpected error:", e)
    return False

def clear_mpu_interrupt():
    i2c.readfrom_mem(mpu_addr, INT_STATUS, 1) # Interrupt won't clear until we read this register
    
def write_to_register(register, value):
    i2c.writeto_mem(mpu_addr, register, bytes([value]))

def configure_mpu_motion_interrupt():
    write_to_register(ACCEL_CONFIG, 0x10) # Set accelerometer range +/-8g (optional)
    write_to_register(MOT_THR, 5) # Motion threshold, higher=less sensitive, lower=more sensitive
    write_to_register(MOT_DUR, 5) # Motion duration, in 1ms increments
    write_to_register(MOT_DETECT_CTRL, 0x15) # Motion detection control
    write_to_register(INT_PIN_CFG, 0x20) # Interrupt pin: active high, push-pull, latch until cleared
    write_to_register(INT_ENABLE, 0x40) # Enable motion interrupt behavior
    
def collect_and_send_samples(number_of_samples=0):
    global flag_data_ready
    global last_duration_us
    client.check_msg()   # attribute updates
    for i in range(number_of_samples):
        if flag_data_ready:
            #send_success = process_buffers_and_send(
            #    raw_x[fft_data_buffer], raw_y[fft_data_buffer], raw_z[fft_data_buffer])
            print(f'Execution Time = {last_duration_us} us')
            if not send_success:
                print('Send Failed!!')
            flag_data_ready = False
    
#     payload_dict = {
#         "amplitudes": amplitudes,
#         "freq_bins":  freq_bins,
#         "total_rms":  round(total_rms, 6)
#     }
# 
#     if first_message:
#         unix_epoch_offset = 946684800  # seconds between 1970 and 2000
#         payload_dict["start_ts"] = (time.time() + unix_epoch_offset) * 1000
#         first_message = False
# 
#     payload = json.dumps(payload_dict)
# 
#     # MQTT upload
#     try:
#         client.connect()
#         client.publish(TB_TOPIC, payload)
#         client.disconnect()
#         print(f'Uploaded to ThingsBoard | total_rms={total_rms:.4f}')
#         return True
#     except OSError as e:
#         print("Network error (OSError):", e)
#     except TypeError as e:
#         print("Handshake failed (NoneType error). Server didn't respond in time.")
#     except Exception as e:
#         print("Unexpected error:", e)
#     return False


## ----------------------- Core Logic ------------------------
# Setup I2C communication
i2c = machine.I2C(0, scl=machine.Pin(20), sda=machine.Pin(22), freq=400000)
mpu_addr = i2c.scan()[0]
i2c.writeto(mpu_addr, bytearray([107, 0])) # wakeup command
time.sleep_ms(100)
clear_mpu_interrupt()
time.sleep(2)
print('MPU6050 is ready!')
configure_mpu_motion_interrupt()
get_calibration_param()

wifi_connect()

# client = MQTTClient(client_id=b"ESP32_Vibration", server=TB_BROKER, port=1883, user=TB_USER, password=b"")
client = MQTTClient(client_id=b"ESP32_Vibration", server=TB_BROKER, port=1883, user=TB_USER, password=b"", keepalive=60)
client.set_callback(mqtt_callback)
client.connect()
client.subscribe(b'v1/devices/me/attributes')
client.subscribe(b'v1/devices/me/attributes/response/+')
client.publish(b'v1/devices/me/attributes/request/1',
               b'{"sharedKeys":"amp_threshold,freq_low,freq_high"}')
print('MQTT connected to ThingsBoard!')

timer0 = machine.Timer(0)
timer0.init(mode=machine.Timer.PERIODIC, period=sample_period, callback=update_buffers)

# Main loop
print('Setup complete. Main Loop has started running ...')
collect_and_send_samples(3)
