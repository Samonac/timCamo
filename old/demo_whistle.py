import pyaudio
import numpy as np
import struct
from scipy.signal import butter, lfilter
from dtw import dtw
import matplotlib.pyplot as plt
import wave
from datetime import datetime, timedelta
import os

# ==== CONFIGURATION ====
ENABLE_PLOT = False
ENABLE_DEBUG_RECORD = True
DEBUG_PATTERN_LENGTH = 100  # Number of tones to capture for pattern learning
SAVE_DIR = "saved_patterns"
# ========================

# Audio Params
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 2048
FREQ_RANGE = (1000, 4000)
AMPLITUDE_THRESHOLD = 1200
PATTERN_MATCH_THRESHOLD = 150
MAX_BUFFER_TIME = timedelta(seconds=3)

# Initial Whistle Patterns
patterns = {
    'pattern1': [261, 329, 392],  # Example
    'pattern2': [392, 329, 261],
    'pattern3': [392, 523, 659],
}

peak_buffer = []
debug_freq_buffer = []
debug_audio_buffer = []

# --- Bandpass Filter ---
def butter_bandpass(lowcut, highcut, fs, order=5):
    return butter(order, [lowcut / (0.5 * fs), highcut / (0.5 * fs)], btype='band')

def apply_bandpass(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order)
    return lfilter(b, a, data)

# --- Save WAV for debug ---
def save_wave(filename, audio_frames):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(audio_frames))
    wf.close()

# --- Real-time Plotting Setup ---
def init_plot():
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_ylim(0, 5000)
    ax.set_xlim(FREQ_RANGE[0], FREQ_RANGE[1])
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Amplitude")

    # ??? Force initial draw to cache the renderer
    fig.canvas.draw()
    fig.canvas.flush_events()

    return fig, ax, line

from scipy.signal import find_peaks

def get_dominant_frequencies(magnitudes, freqs, num_peaks=5, min_prominence=1000):
    # Find peaks with minimum prominence
    peaks, properties = find_peaks(magnitudes, prominence=min_prominence)
    # Sort by prominence
    sorted_peaks = sorted(zip(peaks, properties['prominences']), key=lambda x: -x[1])
    # Return top N peak frequencies and amplitudes
    return [(freqs[i], magnitudes[i]) for i, _ in sorted_peaks[:num_peaks]]


# --- Main Loop ---
def detect_whistle():
    global peak_buffer, debug_freq_buffer, debug_audio_buffer

    print("?? Listening for whistles...")

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    if ENABLE_PLOT:
        fig, ax, line = init_plot()

    os.makedirs(SAVE_DIR, exist_ok=True)

    while True:
        data = stream.read(CHUNK)
        debug_audio_buffer.append(data)  # Raw bytes for WAV saving

        data_int = np.array(struct.unpack(str(2 * CHUNK) + 'B', data), dtype='b')[::2]
        filtered = apply_bandpass(data_int, FREQ_RANGE[0], FREQ_RANGE[1], RATE)

        fft = np.fft.fft(filtered)
        freqs = np.fft.fftfreq(len(fft), 1.0 / RATE)
        magnitudes = np.abs(fft[:CHUNK // 2])
        freqs = freqs[:CHUNK // 2]

        
        dominant_freqs = get_dominant_frequencies(magnitudes, freqs)
        for dominant_freq, amplitude in dominant_freqs:
            print(f"New Code : {dominant_freq:.1f} Hz @ {amplitude:.1f}")
            if amplitude > AMPLITUDE_THRESHOLD and FREQ_RANGE[0] <= dominant_freq <= FREQ_RANGE[1]:
              print('great whistle?')

        dominant_idx = np.argmax(magnitudes)
        dominant_freq = freqs[dominant_idx]
        amplitude = magnitudes[dominant_idx]
        
        # Real-time Plot
        if ENABLE_PLOT:
            line.set_xdata(freqs)
            line.set_ydata(magnitudes)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()


        if FREQ_RANGE[0] <= dominant_freq <= FREQ_RANGE[1] and amplitude > AMPLITUDE_THRESHOLD:
            now = datetime.now()
            peak_buffer.append((now, dominant_freq))
            debug_freq_buffer.append(dominant_freq)
            print(f"?? {dominant_freq:.1f} Hz @ {amplitude:.1f}")

            # Remove old entries
            peak_buffer = [(t, f) for (t, f) in peak_buffer if now - t < MAX_BUFFER_TIME]

            # --- Pattern Matching ---
            if len(peak_buffer) >= 3:
                freqs_only = [f for (_, f) in peak_buffer]
                for name, pattern in patterns.items():
                    dist, _, _, _ = dtw(pattern, freqs_only, dist=lambda x, y: abs(x - y))
                    if dist < PATTERN_MATCH_THRESHOLD:
                        print(f"?? Whistle pattern matched: {name} (dist={dist:.1f})")
                        peak_buffer.clear()
                        break

            # --- Debug: Save Pattern ---
            if ENABLE_DEBUG_RECORD and len(debug_freq_buffer) >= DEBUG_PATTERN_LENGTH:
                now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                wav_path = os.path.join(SAVE_DIR, f"whistle_{now_str}.wav")
                save_wave(wav_path, debug_audio_buffer)
                print(f"? Saved new whistle WAV: {wav_path}")
                print(f"?? Pattern frequencies to copy:\n    {debug_freq_buffer}")
                # Reset debug buffers
                debug_freq_buffer.clear()
                debug_audio_buffer.clear()

    stream.stop_stream()
    stream.close()
    p.terminate()

    
if __name__ == '__main__':
    print('Launching')
    detect_whistle()
