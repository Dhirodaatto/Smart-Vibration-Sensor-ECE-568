#Implements entire adc -> filter -> log -> dac in one ISR. ISR should take around ~75-145 us <<< 1 ms to complete
#so blocking of other loop components due to constant ISR triggering should not be an issue.

from machine import Pin, ADC, DAC, Timer
import array
import math

# ======================
# CONFIG
# ======================
SIGNAL_PIN = 34
DAC_PIN = 25

FS = 1000
F_TEST = 50 #ideal tone for filter only testing

CAPTURE_TIME_SEC = 1.0#0.5
TOTAL_SAMPLES = int(FS * CAPTURE_TIME_SEC)

# ======================
# SETUP
# ======================
adc = ADC(Pin(SIGNAL_PIN))
adc.atten(ADC.ATTN_11DB)

dac = DAC(Pin(DAC_PIN))

# ======================
# BIQUAD (200 Hz cutoff)
# ======================
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

        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y

        return y

biquad = Biquad(
    0.206572, 0.413144, 0.206572,
   -0.369527, 0.195816
)


class FIR:
    def __init__(self, coeffs):
        self.h = coeffs
        self.N = len(coeffs)
        self.x = [0.0] * self.N

    def process(self, x_new):
        # Shift buffer (manual, ISR-safe)
        for i in range(self.N - 1, 0, -1):
            self.x[i] = self.x[i-1]
        self.x[0] = x_new

        # Convolution
        y = 0.0
        for i in range(self.N):
            y += self.h[i] * self.x[i]

        return y
    
    
fir = FIR([
 -0.0012, -0.0020, -0.0025,  0.0000,  0.0065,
  0.0162,  0.0285,  0.0402,  0.0469,  0.0435,
  0.0250, -0.0100, -0.0600, -0.1150, -0.1600,
  0.8200,
 -0.1600, -0.1150, -0.0600, -0.0100,  0.0250,
  0.0435,  0.0469,  0.0402,  0.0285,  0.0162,
  0.0065,  0.0000, -0.0025, -0.0020, -0.0012
])


# ======================
# STORAGE
# ======================
raw_log  = array.array('H', [0]*TOTAL_SAMPLES)
filt_log = array.array('H', [0]*TOTAL_SAMPLES)

sample_idx = 0
done = False

#Ideal test signal to test filter only

# ======================
# ISR
# ======================
def sample_isr(timer):
    global sample_idx, done

    if sample_idx >= TOTAL_SAMPLES:
        done = True
        return

    # --- ADC ---
    raw = adc.read()

    # Convert to centered float
    
    x = (raw / 4095.0) - 0.5
    
    #test input to rule out adc noise for jagged input adc. put the above line back and remove these two after testing.

    # --- FILTER ---
#     y = biquad.process(x)

      y = fir.process(x)
      
#     # Re-bias for DAC
    y = y + 0.5
    if y < 0: y = 0
    if y > 1: y = 1
# 
#     # --- DAC ---
#     dac.write(int(y * 255))

    # --- LOG ---
    raw_log[sample_idx]  = raw
    filt_log[sample_idx] = int(y * 4095)

    sample_idx += 1

# ======================
# START SAMPLING
# ======================
timer = Timer(0)
timer.init(freq=FS, mode=Timer.PERIODIC, callback=sample_isr)

# ======================
# WAIT FOR COMPLETION
# ======================
while not done:
    pass

timer.deinit()

# ======================
# EXPORT
# ======================
print("index,raw,filtered")
for i in range(TOTAL_SAMPLES):
    print("{},{},{}".format(i, raw_log[i], filt_log[i]))