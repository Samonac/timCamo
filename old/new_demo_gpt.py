import pyaudio
import struct
import numpy as np
from dtw import dtw
from datetime import datetime, timedelta
import os
import wave

# Define known whistle patterns
patterns = {
    'coucou_a': [2368.65, 2045.65, 2024.12],
    'coucou_b': [2411.72, 2454.79, 2088.72, 2088.72],
    'coucou_1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6761.43, 5512.5, 5490.97, 0.0, 0.0, 0.0, 0.0, 0.0]
,  # Coucou
    'coucou_2': [0.0, 0.0, 0.0, 0.0, 4285.11, 4371.24, 3639.11, 3596.04, 3596.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Reversed
    'coucou_3': [0.0, 0.0, 0.0, 0.0, 5900.1, 5943.16, 4694.24, 4844.97, 4521.97, 4780.37, 4435.84, 0.0, 0.0, 0.0, 0.0],
    'coucou_4': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3660.64, 3725.24, 5297.17, 5275.63, 5103.37, 0.0, 0.0, 0.0],
    'coucou_5': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2627.05, 2648.58, 2648.58, 2648.58, 2239.45, 2196.39, 2153.32, 2131.79, 2174.85, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
,  # G major chord
}

# Audio parameters
THRESHOLD = 5000  # Amplitude threshold
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 2048
PEAK_THRESHOLD = 10000000  # Minimum power of peak to be considered
GROUP_TIME_GAP = 1.0  # seconds between whistles
MAX_WHISTLE_DURATION = 2.0  # Max time to record whistle
MAX_PEAKS_PER_WHISTLE = 3

SAVE_PATH = "saved_patterns"
os.makedirs(SAVE_PATH, exist_ok=True)

def save_wav(filename, frames):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

def detect_whistle():
    print('[INFO] Starting whistle detection...')
    
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    recent_peaks = []  # (timestamp, frequency, power)
    frames = []
    whistle_start_time = datetime.now()

    try:
        while True:
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            data_int = np.array(struct.unpack(str(CHUNK) + 'h', data))

            fft = np.fft.rfft(data_int)
            freq = np.fft.rfftfreq(len(data_int), 1.0 / RATE)
            magnitude = np.abs(fft)

            peak_idx = np.argmax(magnitude)
            peak_freq = freq[peak_idx]
            peak_power = magnitude[peak_idx]

            now = datetime.now()

            if peak_power > PEAK_THRESHOLD:
                recent_peaks.append((now, peak_freq, peak_power))
                if f"{peak_freq:.1f}" != '0.0':
                  print(f" {peak_freq:.1f} Hz @ {peak_power:.1f}")

            # Time to save whistle
            if (now - whistle_start_time).total_seconds() > MAX_WHISTLE_DURATION:
                if recent_peaks:
                    group_freqs = group_peaks_by_time(recent_peaks)
                    for i, group in enumerate(group_freqs):
                        print(f"\n Pattern frequencies to copy:\n    {[round(freq, 2) for freq in group]}")
                        match_whistle_to_pattern(group)

                    timestamp = now.strftime("%Y%m%d_%H%M%S")
                    # save_wav(os.path.join(SAVE_PATH, f"whistle_{timestamp}.wav"), frames)
                    # print(f"? Saved new whistle WAV: {SAVE_PATH}/whistle_{timestamp}.wav")

                recent_peaks = []
                frames = []
                whistle_start_time = datetime.now()

    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


def group_peaks_by_time(peaks):
    grouped = []
    current_group = []

    for i, (timestamp, freq, power) in enumerate(peaks):
        if i == 0:
            current_group.append(freq)
        else:
            prev_time = peaks[i - 1][0]
            if (timestamp - prev_time).total_seconds() < GROUP_TIME_GAP:
                current_group.append(freq)
            else:
                if current_group:
                    grouped.append(current_group)
                current_group = [freq]

    if current_group:
        grouped.append(current_group)

    return grouped


def match_whistle_to_pattern(freq_group):
    for pattern_name, pattern_freqs in patterns.items():
        dist, _, _, _ = dtw(pattern_freqs, freq_group[:len(pattern_freqs)], dist=lambda x, y: abs(x - y))
        print('dist :', dist)
        if dist < 2000:
            print(f"\n\n !!!! Detected pattern: {pattern_name} (DTW distance {dist:.1f})\n\n")
            return pattern_name
    print("? No matching pattern found.")
    return None


if __name__ == '__main__':
    detect_whistle()
