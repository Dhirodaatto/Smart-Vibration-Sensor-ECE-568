# EMA Filter no timers
# Project : EMA Filter (No Timer Version)

from machine import Pin, ADC, DAC
import math
import time

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
# Filter state
# ----------------------------
y = 0.0

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
# Main loop (sampling + filtering)
# ----------------------------
last_print = time.ticks_ms()

while True:
    t_start = time.ticks_us()

    # --- Read ADCs ---
    raw = signal_adc.read()
    pot = pot_adc.read()

    # --- Normalize input ---
    x = raw / 4095.0

    # --- Map pot → alpha ---
    alpha = ALPHA_MIN + (pot / 4095.0) * (ALPHA_MAX - ALPHA_MIN)

    # --- EMA filter ---
    y = y + alpha * (x - y)

    # --- Output to DAC ---
    dac.write(int(y * 255))

    # ----------------------------
    # Debug print (slow, non-blocking)
    # ----------------------------
    if time.ticks_diff(time.ticks_ms(), last_print) > 500:
        fc = cutoff_from_alpha(alpha, fs)
        print("alpha:", round(alpha, 3), "cutoff:", round(fc, 1), "Hz")
        last_print = time.ticks_ms()

    # ----------------------------
    # Enforce sampling period
    # ----------------------------
    elapsed = time.ticks_diff(time.ticks_us(), t_start)
    delay = Ts_us - elapsed

    if delay > 0:
        time.sleep_us(delay)