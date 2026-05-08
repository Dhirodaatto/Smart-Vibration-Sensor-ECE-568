import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket, struct
import numpy as np
import matplotlib.pyplot as plt

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5005))

sampling_rate = 500 # Hz
sample_period = int(1000/sampling_rate) # period in ms
N = 512
freqL = np.linspace(0, sampling_rate/2, N//2)

fig,ax = plt.subplots(1,1,figsize=(8,6))
fftplot, = ax.plot(freqL, np.zeros(N//2), label='FFT results')
ax.set_ylabel('Magnitude')
ax.set_xlabel('Frequencies [Hz]')
ax.set_ylim(0, 10)
ax.grid()

fig.suptitle(f"STFT (Short Time Fourier Transform) Implementation, N = {N}")

def update(frame):
    pkt, addr = sock.recvfrom(2048)
    seq = struct.unpack('<I', pkt[:4])[0]
    mag = np.frombuffer(pkt[4:], dtype=np.float32)

    fftplot.set_ydata(mag)
    
    if np.max(mag) > 4:
        print(f'Peak frequency = {int(np.argmax(mag)) * sampling_rate / N}')
    return fftplot

ani = FuncAnimation(fig, update, interval=1024, cache_frame_data=False)
plt.legend(loc='upper right')
plt.show()
