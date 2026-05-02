# Project : Exponential Moving Average Filter Proof of Concept

from machine import Pin, ADC, DAC, Timer
import math

# Pin configuration
SIGNAL_PIN = 34
POT_PIN    = 37
DAC_PIN    = 25

# Initialize peripherals
signal_adc = ADC(Pin(SIGNAL_PIN))
pot_adc    = ADC(Pin(POT_PIN))
dac        = DAC(Pin(DAC_PIN))

signal_adc.atten(ADC.ATTN_11DB)
pot_adc.atten(ADC.ATTN_11DB)

# Filter state
y = 0.0

# Sampling configuration
fs = 5000.0  # Hz

# Alpha range (~150–250 Hz cutoff)
ALPHA_MIN = 0.17
ALPHA_MAX = 0.27

# Timer callback: sample, filter, DAC output
def sample_and_filter(timer):
    global y

    # Read ADCs
    raw = signal_adc.read()
    pot = pot_adc.read()

    # Normalize input
    x = raw / 4095.0

    # Map pot → alpha
    alpha = ALPHA_MIN + (pot / 4095.0) * (ALPHA_MAX - ALPHA_MIN)

    # EMA filter
    y = y + alpha * (x - y)

    # Output to DAC
    dac_val = int(y * 255)
    dac.write(dac_val)

# Configure hardware timer: 200 us → 5 kHz
timer = Timer(0)
timer.init(period=200, mode=Timer.PERIODIC, callback=sample_and_filter)

# Optional debug loop (runs outside ISR)
def cutoff_from_alpha(alpha, fs):
    return (-math.log(1.0 - alpha) / (2.0 * math.pi)) * fs

while True:
    pot = pot_adc.read()
    alpha = ALPHA_MIN + (pot / 4095.0) * (ALPHA_MAX - ALPHA_MIN)
    fc = cutoff_from_alpha(alpha, fs)
    print("alpha:", round(alpha, 3), "cutoff:", round(fc, 1), "Hz")
    import time
    time.sleep(0.5)