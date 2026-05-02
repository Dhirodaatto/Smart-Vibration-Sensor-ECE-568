from machine import Pin, ADC, DAC
import time

# ======================
# CONFIG
# ======================
SIGNAL_PIN = 34
DAC_PIN = 25

FS = 1000
SAMPLE_PERIOD_US = int(1e6 / FS)

NUM_CYCLES = 100
TEST_FREQ = 100  # Hz sine assumption for cycle counting

# total samples = cycles * fs / f
TOTAL_SAMPLES = int(NUM_CYCLES * FS / TEST_FREQ)

# ======================
# SETUP
# ======================
adc = ADC(Pin(SIGNAL_PIN))
adc.atten(ADC.ATTN_11DB)

dac = DAC(Pin(DAC_PIN))

# ======================
# BIQUAD
# ======================
class Biquad:
    def __init__(self):
        # example coefficients (replace with yours)
        self.b0 = 0.2929
        self.b1 = 0.5858
        self.b2 = 0.2929
        self.a1 = 0.0
        self.a2 = 0.1716
        self.reset()

    def reset(self):
        self.x1 = self.x2 = 0.0
        self.y1 = self.y2 = 0.0

    def process(self, x):
        y = (self.b0 * x +
             self.b1 * self.x1 +
             self.b2 * self.x2 -
             self.a1 * self.y1 -
             self.a2 * self.y2)

        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        return y

biquad = Biquad()

# ======================
# STORAGE
# ======================
raw_log = []
filt_log = []

# ======================
# ACQUISITION LOOP
# ======================
for i in range(TOTAL_SAMPLES):
    t0 = time.ticks_us()

    raw = adc.read()
    x = (raw / 4095.0) - 0.5  #removed dc

    # ---- Filter (cascade) ----
#     y1 = biquad1.process(x)
#     y  = biquad2.process(y1)

    y  = biquad.process(x)
    
    y = y + 0.5 # recenter for DAC

    y = max(0.0, min(1.0, y)) #clamp before sending to DAC
    # DAC output
    dac.write(int(max(0.0, min(1.0, y)) * 255))

    # log
    raw_log.append(x)
    filt_log.append(y)

    # timing control
    while time.ticks_diff(time.ticks_us(), t0) < SAMPLE_PERIOD_US:
        pass

# ======================
# EXPORT AS CSV
# ======================
print("index,raw,filtered")

for i in range(len(raw_log)):
    print("{},{:.6f},{:.6f}".format(i, raw_log[i], filt_log[i]))