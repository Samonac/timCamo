import pyaudio
import struct
import numpy as np
from dtw import dtw
from datetime import datetime, timedelta
import os
import wave
import threading
import time
import subprocess
from pathlib import Path
# Set up libraries and overall settings
import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
import camoServo

GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout



# Define known whistle patterns
patterns = {
    #'coucou_a': [2368.65, 2045.65, 2024.12],
    'coucou_b': [2411.72, 2454.79, 2088.72, 2088.72],
    'coucou_c': [6998.29, 7105.96, 5878.56, 5878.56],
    'coucou_d': [5964.7, 5943.16, 5211.04, 4974.17],

    'coucou_1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6761.43, 5512.5, 5490.97, 0.0, 0.0, 0.0, 0.0, 0.0],
    'coucou_2': [0.0, 0.0, 0.0, 0.0, 4285.11, 4371.24, 3639.11, 3596.04, 3596.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'coucou_3': [0.0, 0.0, 0.0, 0.0, 5900.1, 5943.16, 4694.24, 4844.97, 4521.97, 4780.37, 4435.84, 0.0, 0.0, 0.0, 0.0],
    'coucou_4': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3660.64, 3725.24, 5297.17, 5275.63, 5103.37, 0.0, 0.0, 0.0],
    'coucou_5': [0.0]*28 + [2627.05, 2648.58, 2648.58, 2648.58, 2239.45, 2196.39, 2153.32, 2131.79, 2174.85] + [0.0]*8,
    'goodnight_1': [3639.11, 3639.11, 3596.04, 3574.51, 3531.45, 3488.38, 3380.71, 3316.11],
    'goodnight_2': [3488.38, 3509.91, 3509.91, 3509.91]
}

# Audio parameters
THRESHOLD = 5000
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 2048
PEAK_THRESHOLD = 1000000
GROUP_TIME_GAP = 1.0
MAX_WHISTLE_DURATION = 2.0
MAX_PEAKS_PER_WHISTLE = 3
MAX_WHISTLE_DISTANCE_TOLERANCE = 1000

# Video parameters
SAVE_PATH = "saved_patterns"
RECORDINGS_PATH = "recordings"
VIDEO_DURATION=33
VIDEO_FRAMERATE=30
VIDEO_SIZE='1280x720'
os.makedirs(SAVE_PATH, exist_ok=True)
os.makedirs(RECORDINGS_PATH, exist_ok=True)

def save_wav(filename, frames):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

import subprocess

def record_whistle_video(duration=VIDEO_DURATION, output_dir=RECORDINGS_PATH):
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = f"{output_dir}/whistle_capture_{timestamp}.mp4"

    command = [
    "ffmpeg",
    # Video input: use rawvideo (YUYV) from v4l2, 640x480, 25 fps
    "-f", "v4l2",
    "-input_format", "yuyv422",  # Use YUYV instead of MJPEG to work with hw encoder
    "-framerate", "{}".format(VIDEO_FRAMERATE),
    "-video_size", VIDEO_SIZE,
    # "-itsoffset", "3.84",
    # "-map", "1.a",
    # "-codec:v", "h264_omx",
    # "-b:v", "2048k",
    "-i", "/dev/video0",
    # Audio input: ALSA, card 1 device 0 (from 'arecord -l' output)
    "-f", "alsa",
    "-ac", "1",              # Force mono audio input (your mic is mono)
    "-i", "hw:1,0",
    # Video encoding: hardware H264 encoder on Raspberry Pi
    "-c:v", "h264_v4l2m2m",
    "-preset", "ultrafast",
    "-strict", "experimental",
    # Audio encoding: AAC at 128kbps mono
    "-c:a", "aac",
    "-b:a", "128k",
    # Duration of recording in seconds
    "-t", str(duration),
    # Output filename
    output_filename]
    
    print(f"Recording to: {output_filename}")
    
    subprocess.run(command, check=True)
    
    print("Recording complete.")
    
    return output_filename

def record_av_ffmpeg(duration_seconds=33, output_dir=RECORDINGS_PATH, camera_index=0):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(output_dir, f"whistle_capture_{timestamp}.mp4")

    # FFmpeg command for Linux (adjust input devices if needed)
    cmd = [
        "ffmpeg",
        "-y",  # Overwrite without asking
        "-t", str(duration_seconds),
        "-f", "v4l2",
        "-framerate", "20",
        "-video_size", "640x480",
        "-i", f"/dev/video{camera_index}",  # USB camera
        "-f", "alsa",
        "-i", "default",  # Default microphone
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-c:a", "aac",
        "-strict", "experimental",
        output_file
    ]

    print(f" Starting FFmpeg audio/video capture: {output_file}")
    try:
        subprocess.run(cmd, check=True)
        print(" AV recording completed.")
    except subprocess.CalledProcessError as e:
        print(f" FFmpeg error: {e}")

def execute_matched_pattern(pattern_id, stream, p):
    print('In execute_matched_pattern , for pattern number ', pattern_id)
    # sleep 3 ; xset dpms force off
    stream.stop_stream()
    stream.close()
    p.terminate()
    sleep(2.5)
    record_thread = threading.Thread(target=record_whistle_video, args=(VIDEO_DURATION,))
    record_thread.start()
    print('starting to record video, sleeping a few seconds before opening servos')
    #moveServo(start=True)  # need to try to get the recording started prior to opening
    time.sleep(5)
    print('opening servos')
    servo_thread = threading.Thread(target=camoServo.moveServoThread, args=(VIDEO_DURATION,))
    servo_thread.start()
    record_thread.join()
    servo_thread.join()
    #print('sleeping 2s before closing servos')
    #time.sleep(2)
    #sleep(VIDEO_DURATION+2)
    #print('closing servos')
    
    #servo_thread2 = threading.Thread(target=moveServo, args=(False,))
    
    #servo_thread2.start()
    #servo_thread2.join()
    #moveServo(start=False)
    #sleep(0.5)
    # subprocess.run(["sleep 1 ; xset dpms force off"]) # Turn RaspberryPi screen off, cause why not
    
    return detect_whistle()


def detect_whistle():

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    recent_peaks = []  # (timestamp, frequency, power)
    frames = []
    whistle_start_time = datetime.now()

    print('[INFO] Starting whistle detection...')
    camoServo.jiggleServo()
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

            if (now - whistle_start_time).total_seconds() > MAX_WHISTLE_DURATION:
                if recent_peaks:
                    group_freqs = group_peaks_by_time(recent_peaks)
                    for i, group in enumerate(group_freqs):
                        print(f"\n Pattern frequencies to copy:\n    {[round(freq, 2) for freq in group]}")
                        matched_pattern = match_whistle_to_pattern(group)

                        if matched_pattern:
                            print(f"\n[TRIGGER] Launching video capture for pattern '{matched_pattern}'\n")
                            execute_matched_pattern(matched_pattern, stream, p)
                            time.sleep(1)
                            return detect_whistle()

                    timestamp = now.strftime("%Y%m%d_%H%M%S")
                    # save_wav(os.path.join(SAVE_PATH, f"whistle_{timestamp}.wav"), frames)
                    # print(f"? Saved new whistle WAV: {SAVE_PATH}/whistle_{timestamp}.wav")

                recent_peaks = []
                frames = []
                whistle_start_time = datetime.now()

    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
    finally:
        camoServo.clean()
        stream.stop_stream()
        stream.close()
        p.terminate()
        print('Going to sleep now. Cya later!')
        return True

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
        # print('dist :', dist)
        if dist < MAX_WHISTLE_DISTANCE_TOLERANCE:
            print(f"\n\n !!!! Detected pattern: {pattern_name} (DTW distance {dist:.1f})\n\n")
            camoServo.jiggleServo()
            return pattern_name
    print(" [X] No matching pattern found.")
    return None
            
if __name__ == '__main__':
    detect_whistle()
