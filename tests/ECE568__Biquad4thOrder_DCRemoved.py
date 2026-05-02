from machine import Pin, ADC, DAC
import time
import array

# ======================
# CONFIG
# ======================
SIGNAL_PIN = 34
DAC_PIN = 25

FS = 1000
SAMPLE_PERIOD_US = int(1e6 / FS)

# NUM_CYCLES = 100
# TEST_FREQ = 100
CAPTURE_TIME_SEC = 0.5
TOTAL_SAMPLES = int(FS * CAPTURE_TIME_SEC)

# ======================
# SETUP
# ======================
adc = ADC(Pin(SIGNAL_PIN))
adc.atten(ADC.ATTN_11DB)

dac = DAC(Pin(DAC_PIN))

# ======================
# BIQUAD
# ======================
# class Biquad:
#     def __init__(self):
#         self.b0 = 0.2929
#         self.b1 = 0.5858
#         self.b2 = 0.2929
#         self.a1 = 0.0
#         self.a2 = 0.1716
#         self.reset()
# 
#     def reset(self):
#         self.x1 = self.x2 = 0.0
#         self.y1 = self.y2 = 0.0
# 
#     def process(self, x):
#         y = (self.b0 * x +
#              self.b1 * self.x1 +
#              self.b2 * self.x2 -
#              self.a1 * self.y1 -
#              self.a2 * self.y2)
# 
#         self.x2 = self.x1
#         self.x1 = x
#         self.y2 = self.y1
#         self.y1 = y
#         return y

# biquad = Biquad()



class Biquad:
    def __init__(self, b0, b1, b2, a1, a2):
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.a1 = a1
        self.a2 = a2
        
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

    def process(self, x):
        y = (self.b0 * x +
             self.b1 * self.x1 +
             self.b2 * self.x2 -
             self.a1 * self.y1 -
             self.a2 * self.y2)

        # Shift state
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y

        return y

#2nd order with 200 Hz cutoff
biquad = Biquad(
    0.206572, 0.413144, 0.206572,
   -0.369527, 0.195816
)


# =========================
# HARDCODED COEFFICIENTS
# 4th-order Butterworth
# fc = 250 Hz, fs = 1000 Hz
# =========================

#250 Hz cutoff
# Section 1 (lower Q)
# biquad1 = Biquad(
#     0.097631, 0.195262, 0.097631,
#    -0.942809, 0.333333
# )
# 
# # Section 2 (higher Q)
# biquad2 = Biquad(
#     0.097631, 0.195262, 0.097631,
#    -0.333333, 0.333333
# )

#200 Hz cutoff
biquad1 = Biquad(
    0.160209, 0.320418, 0.160209,
   -0.577240, 0.421787
)

biquad2 = Biquad(
    0.210302, 0.420604, 0.210302,
   -1.048600, 0.295050
)

# ======================
# STORAGE (INTEGER ARRAYS)
# ======================
raw_log  = array.array('H', [0]*TOTAL_SAMPLES)
filt_log = array.array('H', [0]*TOTAL_SAMPLES)

# ======================
# ACQUISITION LOOP
# ======================
for i in range(TOTAL_SAMPLES):
    t0 = time.ticks_us()

    raw = adc.read()
    x = (raw / 4095.0) - 0.5

    y = biquad.process(x)

#     y1 = biquad1.process(x)
#     y  = biquad2.process(y1)

    y = y + 0.5
    y = max(0.0, min(1.0, y))

    dac.write(int(y * 255))

    # STORE AS INTEGERS
    raw_log[i]  = raw
    filt_log[i] = int(y * 4095)

    while time.ticks_diff(time.ticks_us(), t0) < SAMPLE_PERIOD_US:
        pass

# ======================
# EXPORT
# ======================
print("index,raw,filtered")

for i in range(TOTAL_SAMPLES):
    print("{},{},{}".format(i, raw_log[i], filt_log[i]))