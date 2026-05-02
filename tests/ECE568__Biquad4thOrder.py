from machine import Pin, ADC, DAC
import time

# =========================
# CONFIGURATION
# =========================
SIGNAL_PIN = 34
DAC_PIN    = 25

FS = 1000
SAMPLE_PERIOD_US = int(1e6 / FS)

# =========================
# ADC + DAC SETUP
# =========================
signal_adc = ADC(Pin(SIGNAL_PIN))
signal_adc.atten(ADC.ATTN_11DB)

dac = DAC(Pin(DAC_PIN))

# =========================
# BIQUAD CLASS
# =========================
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

# =========================
# HARDCODED COEFFICIENTS
# 4th-order Butterworth
# fc = 250 Hz, fs = 1000 Hz
# =========================

# Section 1 (lower Q)
biquad1 = Biquad(
    0.097631, 0.195262, 0.097631,
   -0.942809, 0.333333
)

# Section 2 (higher Q)
biquad2 = Biquad(
    0.097631, 0.195262, 0.097631,
   -0.333333, 0.333333
)

# # Single, 2nd order biquad
# biquad = Biquad(
#     0.2929, 0.5858, 0.2929,
#    -0.0000, 0.1716
# )


# =========================
# MAIN LOOP
# =========================
while True:
    t_start = time.ticks_us()

    # ---- Read ADC ----
    raw = signal_adc.read()
#     x = raw / 4095.0
    x = (raw / 4095.0) - 0.5  #removed dc

    # ---- Filter (cascade) ----
    y1 = biquad1.process(x)
    y  = biquad2.process(y1)

    y = y + 0.5 # recenter for DAC

    y = max(0.0, min(1.0, y)) #clamp before sending to DAC
#     y  = biquad.process(x)

    # ---- Output ----
    dac.write(int(y * 255))

    # ---- Enforce 1 kHz ----
    while time.ticks_diff(time.ticks_us(), t_start) < SAMPLE_PERIOD_US:
        pass