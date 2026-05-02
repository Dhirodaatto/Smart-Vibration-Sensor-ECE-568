# 2nd Order EMA Filter no timers, with logging

from machine import Pin, ADC, DAC
import math
import time
import array

# ----------------------------
# Pin configuration
# ----------------------------
SIGNAL_PIN = 34
POT_PIN    = 37
DAC_PIN    = 25

# ----------------------------
# Initialize peripherals
# ----------------------------
signal_adc = ADC(Pin(SIGNAL_PIN))
pot_adc    = ADC(Pin(POT_PIN))
dac        = DAC(Pin(DAC_PIN))

signal_adc.atten(ADC.ATTN_11DB)
pot_adc.atten(ADC.ATTN_11DB)

# ----------------------------
# Filter state (2nd order EMA)
# ----------------------------
y1 = 0.0   # first stage output
y2 = 0.0   # second stage output (final output)
y3 = 0.0   # Optional third stage output (optional final output)

# Logging infra
FS = 1000
SAMPLE_PERIOD_US = int(1e6 / FS)

CAPTURE_TIME_SEC = 1.0
TOTAL_SAMPLES = int(FS * CAPTURE_TIME_SEC)
# ======================
# STORAGE
# ======================
raw_log  = [0]*TOTAL_SAMPLES
filt_log = [0]*TOTAL_SAMPLES



# ----------------------------
# Sampling configuration
# ----------------------------
fs = 5000.0
Ts_us = int(1e6 / fs)   # ~200 us

# ----------------------------
# Alpha range (~150–250 Hz cutoff)
# ----------------------------
ALPHA_MIN = 0.17
ALPHA_MAX = 0.27

# ----------------------------
# Debug helper
# ----------------------------
def cutoff_from_alpha(alpha, fs):
    return (-math.log(1.0 - alpha) / (2.0 * math.pi)) * fs

# ----------------------------
# Main loop
# ----------------------------
last_print = time.ticks_ms()

for i in range(TOTAL_SAMPLES):
    t_start = time.ticks_us()

    # --- Read ADCs ---
    raw = signal_adc.read()
    pot = pot_adc.read()

    # --- Normalize input ---
    x = raw / 4095.0

    # --- Map pot → alpha ---
    alpha = ALPHA_MIN + (pot / 4095.0) * (ALPHA_MAX - ALPHA_MIN)

    # =====================================================
    # 2nd-order EMA (cascaded single-pole filters)
    # =====================================================

    # Stage 1
    y1 = y1 + alpha * (x - y1)

    # Stage 2
    y2 = y2 + alpha * (y1 - y2)
    
    # Optional Stage 3
    y3 = y3 + alpha * (y2 - y3)

    # --- Output to DAC ---
#     dac.write(int(y3 * 255))

    # STORE AS INTEGERS
#     raw_log[i]  = raw
#     filt_log[i] = int(y2 * 4095)
    raw_log[i]  = x
    filt_log[i] = y3

    # ----------------------------
    # Debug print (slow, non-blocking)
#     # ----------------------------
#     if time.ticks_diff(time.ticks_ms(), last_print) > 500:
#         fc = cutoff_from_alpha(alpha, fs)
#         #print("alpha:", round(alpha, 3), "cutoff:", round(fc, 1), "Hz")
#         last_print = time.ticks_ms()

    # ----------------------------
    # Enforce sampling period
    # ----------------------------
    elapsed = time.ticks_diff(time.ticks_us(), t_start)
    delay = Ts_us - elapsed

    if delay > 0:
        time.sleep_us(delay)
        
        
# ======================
# EXPORT
# ======================
print("index,raw,filtered")

for i in range(TOTAL_SAMPLES):
    print("{},{},{}".format(i, raw_log[i], filt_log[i]))
