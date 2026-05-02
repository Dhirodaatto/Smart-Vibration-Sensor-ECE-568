#Implements entire adc -> filter -> log  in one ISR. ISR should take around ~75-145 us <<< 1 ms to complete
#so blocking of other loop components due to constant ISR triggering should not be an issue.


from machine import Pin, ADC, Timer
import array
import math
import time

# ======================
# CONFIG
# ======================
SIGNAL_PIN = 34

FS = 1000
CAPTURE_TIME_SEC = 1.0
TOTAL_SAMPLES = int(FS * CAPTURE_TIME_SEC)

# ======================
# SETUP
# ======================
adc = ADC(Pin(SIGNAL_PIN))
adc.atten(ADC.ATTN_11DB)

# ======================
# FIR FILTER (Circular Buffer)
# ======================
class FIR:
    def __init__(self, coeffs):
        self.h = coeffs
        self.N = len(coeffs)
        self.x = [0.0] * self.N
        self.index = 0

    def process(self, x_new):
        self.x[self.index] = x_new

        y = 0.0
        j = self.index

        for i in range(self.N):
            y += self.h[i] * self.x[j]
            j -= 1
            if j < 0:
                j = self.N - 1

        self.index += 1
        if self.index >= self.N:
            self.index = 0

        return y

# 31-tap FIR (~150 Hz cutoff @ 1 kHz)
fir = FIR([
 -0.0010, -0.0020, -0.0030, -0.0020,  0.0020,
  0.0100,  0.0200,  0.0300,  0.0350,  0.0300,
  0.0100, -0.0200, -0.0600, -0.1100, -0.1500,
  0.7200,
 -0.1500, -0.1100, -0.0600, -0.0200,  0.0100,
  0.0300,  0.0350,  0.0300,  0.0200,  0.0100,
  0.0020, -0.0020, -0.0030, -0.0020, -0.0010
])

# ======================
# STORAGE
# ======================
raw_log  = array.array('H', [0]*TOTAL_SAMPLES)
filt_log = array.array('H', [0]*TOTAL_SAMPLES)

sample_idx = 0
done = False

# ======================
# OPTIONAL SAFE TEST SIGNAL (DISABLED)
# ======================
USE_TEST_SIGNAL = False
phase = 0.0
omega = 2 * math.pi * 50 / FS  # 50 Hz test tone

# ======================
# ISR
# ======================
def sample_isr(timer):
    global sample_idx, done, phase

    if sample_idx >= TOTAL_SAMPLES:
        done = True
        return

    # --- INPUT ---
    if USE_TEST_SIGNAL:
        # SAFE sine (bounded, no drift, no division)
        x = math.sin(phase)
        phase += omega
        if phase >= 2 * math.pi:
            phase -= 2 * math.pi

        raw = int((x + 0.5) * 4095)  # fake ADC for logging

    else:
        raw = adc.read()
        x = (raw / 4095.0) - 0.5

    # --- FILTER ---
    y = fir.process(x)

    # --- RE-BIAS FOR LOGGING ---
    y_out = y + 0.5
    if y_out < 0: y_out = 0
    if y_out > 1: y_out = 1

    # --- LOG ---
    raw_log[sample_idx]  = raw
    filt_log[sample_idx] = int(y_out * 4095)

    sample_idx += 1

# ======================
# START SAMPLING
# ======================
timer = Timer(0)
timer.init(freq=FS, mode=Timer.PERIODIC, callback=sample_isr)

# ======================
# WAIT (NON-BLOCKING)
# ======================
while not done:
    time.sleep_us(50)   # prevents CPU lockup

timer.deinit()

# ======================
# EXPORT
# ======================
print("index,raw,filtered")
for i in range(TOTAL_SAMPLES):
    print("{},{},{}".format(i, raw_log[i], filt_log[i]))