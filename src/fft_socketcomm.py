import time
import math
import array
import network
import socket, struct
import machine

from ulab import numpy as np
from ulab import utils as utils

from config import SSID, PSWD

## ----------------------- Global Variables ------------------
dest = 'IP_HERE', 5005 # IP, Port
sock = None

sampling_rate = 500 # Hz
sample_period = int(1000/sampling_rate) # period in ms

_accel_raw = bytearray(6) # Raw I2C read target — 6 bytes for X, Y, Z (big-endian int16 each)

FFT_SIZE = 512 # Number of bins for FFT
hanning = np.array([0.5 * (1 - math.cos(2 * math.pi * i / (FFT_SIZE - 1))) for i in range(FFT_SIZE)]) # Hanning window
coherent_gain = float(np.sum(hanning)) / FFT_SIZE

# Variables for low pass filter
xf, yf, zf = [0,0], [0,0], [0,0]
ALPHA = 0.2

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
seq = 100

last_duration_us = 0

## ----------------------- Function Decalaration -------------
def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(SSID, PSWD)
        while not wlan.isconnected():
            machine.idle()
    print(f'Connected to {wlan.config('ssid')}')
    print(f'IP address: {wlan.ipconfig('addr4')[0]}\n')

@micropython.native
def update_buffers(t):
    global _accel_raw, raw_x, raw_y, raw_z
    global fft_data_buffer, flag_data_ready, current_buffer, buf_ni
    global xf, yf, zf
    global last_duration_us
    t0 = time.ticks_us()

    # Load from memory into var
    i2c.readfrom_mem_into(mpu_addr, acc_xyz, _accel_raw)
#     _accel_raw = i2c.readfrom_mem(mpu_addr, acc_xyz, 6)
    
    # Get XYZ accelerometer from raw
#     x,y,z = struct.unpack('>hhh', _accel_raw)
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
    
    # Update ping-pong buffers / Filtering not enabled yet
    raw_x[current_buffer][buf_ni] = x
    raw_y[current_buffer][buf_ni] = y
    raw_z[current_buffer][buf_ni] = z
    buf_ni += 1

    if buf_ni >= FFT_SIZE:
        fft_data_buffer = current_buffer # indicate data buffer
        current_buffer = 1 - current_buffer # swap buffers
        buf_ni = 0 # reset pointer to start of new buffer
        flag_data_ready = True # flag for processing enable
    
    last_duration_us = time.ticks_diff(time.ticks_us(), t0)
#     print(f"x={x/ACC_SCALE:.3f} g, y={y/ACC_SCALE:.3f} g, z={z/ACC_SCALE:.3f} g")

def process_buffers_and_send(raw_x, raw_y, raw_z):
#     global last_duration_us
#     t0 = time.ticks_us()
    # Convert raw buffers to np arrays
    x_arr = np.array(raw_x, dtype=np.float) / ACC_SCALE
    y_arr = np.array(raw_y, dtype=np.float) / ACC_SCALE
    z_arr = np.array(raw_z, dtype=np.float) / ACC_SCALE
    
    # Remove DC component in FFT
    x_arr, y_arr, z_arr = x_arr - np.mean(x_arr), y_arr - np.mean(y_arr), z_arr - np.mean(z_arr)
    
    # FFT with corrections for hanning window
    fft_x = utils.spectrogram(x_arr * hanning) * (2.0 / FFT_SIZE ) / coherent_gain
    fft_y = utils.spectrogram(y_arr * hanning) * (2.0 / FFT_SIZE ) / coherent_gain
    fft_z = utils.spectrogram(z_arr * hanning) * (2.0 / FFT_SIZE ) / coherent_gain
#     fft_x[0], fft_y[0], fft_z[0] = 0,0,0 # remove DC component in FFT

    fft_data_packet = [[k*sampling_rate/FFT_SIZE for k in range(FFT_SIZE//2)], fft_z.tolist()] # Data packet to send to thingsboard?
    
    # TODO: Other FFT Processing Tasks as required
    
    # TODO: Thingsboard send logic (Currently websockets are being used)
    global seq, sock, dest
    payload = struct.pack('<I', int(seq)) + fft_z[:FFT_SIZE//2].tobytes()
    
    try:
        sock.sendto(payload, dest)
#         last_duration_us = time.ticks_diff(time.ticks_us(), t0)
#         print(f'Execution Time = {last_duration_us} us')
        return True
    except:
        return False
    
    
## ----------------------- Core Logic ------------------------
# Setup i2c communication
i2c = machine.I2C(0, scl=machine.Pin(14), sda=machine.Pin(22), freq=400000)
mpu_addr = i2c.scan()[0] # Get addr
i2c.writeto(0x68, bytearray([107,0])) # wakeup command
time.sleep_ms(100)
print('MPU6050 is ready!')
# TODO: Add Calibration

wifi_connect()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

timer0 = machine.Timer(0)
timer0.init(mode=machine.Timer.PERIODIC, period=sample_period, callback=update_buffers)

# Main loop
print('Setup complete. Main Loop has started running ...')
while True:
    if flag_data_ready:
        send_success = process_buffers_and_send(raw_x[fft_data_buffer], raw_y[fft_data_buffer], raw_z[fft_data_buffer])
        print(f'Execution Time = {last_duration_us} us')
        if not send_success:
            print('Send Failed!!')
        flag_data_ready = False
