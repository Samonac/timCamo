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
#import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
import camoServo
# GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout



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


# duty cycle, calibrate if needed
MIN_DUTY = 2.9
MAX_DUTY = 97.1

 # 32 = Right // 33 = Left
servo_signal_pin_R = 32
servo_signal_pin_L = 33
    
# GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
# Set up pin 11 for PWM

#GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
#GPIO.setwarnings(True)
#GPIO.setup(servo_signal_pin_R,GPIO.OUT)  # Sets up pin 32 to an output (instead of an input)
#GPIO.setup(servo_signal_pin_L,GPIO.OUT)  # Sets up pin 33 to an output (instead of an input)
#p_32 = GPIO.PWM(servo_signal_pin_R, 50)     # Sets up pin 32 as a PWM pin
#p_33 = GPIO.PWM(servo_signal_pin_L, 50)     # Sets up pin 33 as a PWM pin

# Max degrees
MIN_DEGREE = 7
MAX_DEGREE = 15


pastDeg1 = MAX_DEGREE # = 32 = RIGHT
pastDeg2 = MIN_DEGREE # = 33 = LEFT
# TODO : Might need to read and save these values from a JSON file in case of an unexpected reboot


# For jiggling purposes
dutyCycle32 = 10.5
dutyCycle33 = 6.5
jiggleDelta = 0.5

# Video parameters
SAVE_PATH = "saved_patterns"
RECORDINGS_PATH = "recordings"
VIDEO_DURATION=33
VIDEO_FRAMERATE=15
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
    print('In execute_matched_pattern , for pattern n°', pattern_id)
    # sleep 3 ; xset dpms force off
    
    blink_thread = threading.Thread(target=camoServo.blinkByHour, args=(None,))
    blink_thread.start()
    blink_thread.join()
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
    jiggleServo()
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
        cleanServo()
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
            jiggleServo()
            return pattern_name
    print(" [X] No matching pattern found.")
    return None
            
def deg_to_duty(deg):
    return 1.0*((deg - 0) * (MAX_DUTY- MIN_DUTY) / 180 + MIN_DUTY)

def jiggleServo():
    print('jiggleServo with dutyCycle32 : ', dutyCycle32, ' and dutyCycle33 : ', dutyCycle33)
    camoServo.jiggleServo()
    
    
def moveServo_old(start=True, servoList = [], minAngleInput = MIN_DEGREE, maxAngleInput = MAX_DEGREE):

    # Past degrees (not in a CV way ^^)
    global p_32
    global p_33
    global pastDeg1
    global pastDeg2
    
    minAngle = minAngleInput
    maxAngle = maxAngleInput
    print('\n\n\n\n\n =#=#>> moveServo {} with mode_start = {} , for degrees [{}, {}]'.format(servoList, start, minAngle, maxAngle))
    
    if minAngle > MAX_DEGREE or minAngle < MIN_DEGREE or maxAngle > MAX_DEGREE or maxAngle < MIN_DEGREE:
      print(' ERROR for angle input : [minAngle, maxAngle] = [{}, {}] must be inside [{}, {}] = [MIN_DEGREE, MAX_DEGREE]'.format(minAngle, maxAngle, MIN_DEGREE, MAX_DEGREE))
      minAngle = MIN_DEGREE
      maxAngle = MAX_DEGREE
      print('Defaulting to opening mode')
      
    index = 1
    # Move the servo either back or forth
    if start:
      index = -1
      minAngle = maxAngleInput
      maxAngle = minAngleInput
    else:
      minAngle = minAngleInput
      maxAngle = maxAngleInput
      if maxAngleInput < minAngleInput and not start:
        print('Angles are inversed while closing ? , switching')
        minAngle = maxAngleInput
        maxAngle = minAngleInput
    
    # 'skip' either servos in order to facilitate opening and closing (as little friction as possible)
    skipR = False
    skipL = False  
    
    deg = minAngle
    print('ready to loop from deg = {} in [{}, {}]'.format(deg, minAngle, maxAngle))

    # loop from startAngle = minAngle to endAngle = maxAngle
    while (deg != maxAngle + index and deg >= MIN_DEGREE and deg <= MAX_DEGREE):
        print('Deg : ', deg)
        deg1 = deg
        deg2 = minAngle+maxAngle-deg
        print('\n  ==> New round ! index:{}  //  Deg1(R) = {} ; Deg2(L) = {}'.format(index, deg1, deg2))
        print('pastDeg1 = ', pastDeg1)
        print('pastDeg2 = ', pastDeg2)
        
        if 'R' in servoList and 'L' in servoList: 
          print('Both servos ! Better be careful')
          skipR = False
          skipL = False
          if start:
            if deg2 < pastDeg2: # We've already opened the left one previously
              print(' in second-opening mode with deg2 < pastDeg2 ({}<{}) so skipping Left'.format(deg2, pastDeg2))
              skipL = True
          else:
            if deg1 < pastDeg1: # We've already closed the right one previously
              print(' in second-closing mode with deg1 < pastDeg1 ({}<{}) so skipping Right'.format(deg1, pastDeg1))
              skipR = True
          
        if 'R' in servoList:
          if not skipR: 
            print(' >(R)< Right mode : deg : ', deg)
            duty_cycle = deg_to_duty(deg)    
            print('duty_cycle : ', duty_cycle)
            p_32.ChangeDutyCycle(duty_cycle)
            pastDeg1 = deg
          else:
            print('Should skip R since closing, but want to keep going..') # should only happen during closing, so deg going from 7 to 15
            deg1 = pastDeg1 + index
            if deg1 > MAX_DEGREE or deg1 < MIN_DEGREE:
              print('We seemed to have reached the end with deg1 : {} so skipping for real this time'.format(deg1))
            else:
              print(' > (R) new deg1 : ', deg1)
              duty_cycle = deg_to_duty(deg1)    
              print('duty_cycle : ', duty_cycle)
              p_32.ChangeDutyCycle(duty_cycle)
              pastDeg1 = deg1
          
        if 'L' in servoList:  
          if not skipL: 
            print(' >(L)< Left mode : deg2 : ', deg2)
            duty_cycle2 = deg_to_duty(deg2)    
            print('duty_cycle2 : ', duty_cycle2)
            p_33.ChangeDutyCycle(duty_cycle2)
            pastDeg2 = deg2
          else:
            print('Should skip L since opening, but want to keep going..') # should only happen during opening, so deg2 going from 7 to 15
            deg2 = pastDeg2 - index
            if deg2 > MAX_DEGREE or deg2 < MIN_DEGREE:
              print('We seemed to have reached the end with deg2 : {} so skipping for real this time'.format(deg2))
            else:
              print(' > (L) new deg2 : ', deg2)
              duty_cycle2 = deg_to_duty(deg2)    
              print('duty_cycle2 : ', duty_cycle2)
              p_33.ChangeDutyCycle(duty_cycle2)
              pastDeg2 = deg2
        
          
        deg += index
        
        if abs(deg) - int(MIN_DEGREE+MAX_DEGREE/4) < int(MIN_DEGREE+MAX_DEGREE/4) or abs(deg) > MIN_DEGREE+MAX_DEGREE - int(MIN_DEGREE+MAX_DEGREE/4):
          print(' -> SLOW ! sleeping 0.1')
          time.sleep(0.1)
        else:
          print('FAST ! sleeping 0.05')
          time.sleep(0.05)   
    # GPIO.setup(servo_signal_pin_R,GPIO.IN)  # Temporarily Sets up pin 32 to an input (to avoid jittering)
    # GPIO.setup(servo_signal_pin_L,GPIO.IN)  # Temporarily Sets up pin 33 to an int (to avoid jittering)

    # extra

def moveServoThread_old(VIDEO_DURATION):
    p_32.start(0)               # Starts running PWM on the pin and sets it to 0
    p_33.start(0)               # Starts running PWM on the pin and sets it to 0
    time.sleep(0.3)
    
    print('Start opening')
    moveServo(start=True, servoList = ['L'], minAngleInput = MIN_DEGREE, maxAngleInput = 10)
    moveServo(start=True, servoList = ['L', 'R'], minAngleInput = MIN_DEGREE, maxAngleInput = MAX_DEGREE)
    print(' > Done !')
    print('   CAPTURING VIDEO : Stopping {}s before doing closing..'.format(VIDEO_DURATION))
    p_32.stop()                   # At the end of the program, stop the PWM
    p_33.stop()  
    
    #p_32 = GPIO.PWM(servo_signal_pin_R, 50)     # Sets up pin 32 as a PWM pin
    #p_33 = GPIO.PWM(servo_signal_pin_L, 50)     # Sets up pin 33 as a PWM pin
    
    #p_32.start(0)               # Starts running PWM on the pin and sets it to 0
    #p_33.start(0)               # Starts running PWM on the pin and sets it to 0
    if VIDEO_DURATION > 8:
      time.sleep(VIDEO_DURATION-8)
    p_32.start(0)               # Starts running PWM on the pin and sets it to 0
    p_33.start(0)               # Starts running PWM on the pin and sets it to 0
    print('Start closing')
    time.sleep(0.3)
    moveServo(start=False, servoList = ['R'], minAngleInput = MIN_DEGREE, maxAngleInput = 10)
    moveServo(start=False, servoList = ['L', 'R'], minAngleInput = MIN_DEGREE, maxAngleInput = MAX_DEGREE)
    print(' > Done done !')
    p_32.stop()                   # At the end of the program, stop the PWM
    p_33.stop()  
    
    return True
    
    


def cleanServo():
    # Clean up everything
    # p_32.stop()                 # At the end of the program, stop the PWM
    # p_33.stop()                 # At the end of the program, stop the PWM
    # GPIO.cleanup()           # Resets the GPIO pins back to defaults
    camoServo.clean()
    print(" Servos have been cleaned")
    return None

if __name__ == '__main__':
    #cleanServo()
    detect_whistle()
