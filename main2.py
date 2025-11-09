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
import servoCamo
import camoLight

import subprocess

# GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout

# os.environ["PYTHONWARNINGS"] = "ignore"  # try to ignore ALSA warnings
os.environ["SDL_AUDIODRIVER"] = "dummy" # Try to stop ALSA from trying to PLAY audio, just record it.
# try:
#     subprocess.run(["xset dpms force off"], check=True) # sleep 3 ; xset dpms force off
#     print("[INFO] Screen turned off successfully.")
# except Exception as e:
#     print(f"[ERROR] Failed to turn off screen : {e}")
#     # exit(1)

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
    'goodnight_2': [3488.38, 3509.91, 3509.91, 3509.91],
    'curious': [3488.38, 3509.91, 3509.92, 3509.91]
}

# Audio parameters
THRESHOLD = 5000
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
CHUNK = 8192
DEVICE_INDEX = 0 
PEAK_THRESHOLD = 3000000
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
VIDEO_FRAMERATE=30
# VIDEO_SIZE='1280x720'
VIDEO_SIZE='640x480'
os.makedirs(SAVE_PATH, exist_ok=True)
os.makedirs(RECORDINGS_PATH, exist_ok=True)

def save_wav(filename, frames):
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()


def record_whistle_video(duration=VIDEO_DURATION, output_dir=RECORDINGS_PATH):
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = f"{output_dir}/whistle_capture_{timestamp}.mp4"

    command = [
        "ffmpeg",
        # Apply itsoffset to video only. Increasing value makes sound occur sooner
        "-itsoffset", "-0.5", # sound is slightly too early at offset 0 ; trying -0.5
        "-f", "v4l2",
        "-input_format", "yuyv422",
        "-framerate", str(VIDEO_FRAMERATE),
        "-video_size", VIDEO_SIZE,
        "-i", "/dev/video0",

        # Audio input (offset at the beginning)
        "-thread_queue_size", "512",
        "-f", "alsa",
        "-ac", "1",
        "-ar", "48000",
        "-i", "hw:1,0",

        # Output encoding
        "-c:v", "h264_v4l2m2m",   # Avoid preset/gop/bframes
        "-c:a", "aac",
        "-b:a", "128k",
        "-b:v", "5000k",

        # Stream mapping
        "-map", "0:v:0",
        "-map", "1:a:0",
        "-max_muxing_queue_size",  "4096",

        # Duration and output
        "-t", str(duration),
        output_filename
    ]

    print(f"Recording to: {output_filename}")
    subprocess.run(command, check=True)
    print("Recording complete.")
    return output_filename


def execute_matched_pattern(pattern_id):
    print(f'Executing pattern: {pattern_id}')
    # sleep 3 ; xset dpms force off  # command to turn the raspberry pi screen off (saves battery)
    # lsof /dev/snd/* # lists all PIDs that are using sound devices 
    #v
    # sudo killall pigpiod
    # sudo pigpiod
    #^² reboots pigpiod , when servomotors are no longer responding at boot.
    camoLight.blinkByHour(newSleepTime=0.3, setHour=1)

    record_thread = threading.Thread(target=record_whistle_video, args=(VIDEO_DURATION,))
    
    record_thread.start()
    time.sleep(4)
    servo_thread = threading.Thread(target=servoCamo.moveServoThread, args=(VIDEO_DURATION,))
    
    servo_thread.start()

    record_thread.join()
    servo_thread.join()

    camoLight.blinkByHour(newSleepTime=0.2, setHour=1)
    time.sleep(2)
    print('Done with execute_matched_pattern')

def moveMouse():
    print('TODO : Moving mouse over microphone icon with PyAutoGui ; would that be helpful ?')
    

def detect_whistle():
    print('In detect_whistle(), starting in ~15s, for pulseAudio to start')
    global p
    global stream
    
    break_out = False
    complete_break = False
    firstStart = True
    time.sleep(5)
    print('Starting first loop')
    while not complete_break:
        print('In first while True')
        print('Complete_break = ', complete_break)
        print('break_out = ', break_out)
        # os.system("arecord -D hw:1,0 -d 1 /dev/null")
        # This could be a way to force ALSA to resync
        #time.sleep(3)
        time.sleep(5)
        if not firstStart:
            servoCamo.resetXset(wait=False)
        firstStart = False
        moveMouse()
        print('(re)launching pyAudio')
        time.sleep(10)
        try:
            p = pyaudio.PyAudio()
            time.sleep(1)
            
                        
            print("Available input devices:")
            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    print(f"Device {i}: {info['name']} | Channels: {info['maxInputChannels']}")
                    print("Device info:", info)
            
            
            stream = p.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=RATE,
                            input=True,
                            frames_per_buffer=CHUNK)
            
            time.sleep(5)
            jiggleServo()
            time.sleep(1)
            camoLight.blinkByHour(newSleepTime=0.1, setHour=3)
            frames = []
            recent_peaks = []
            whistle_start_time = datetime.now()
            whistle_initial_time = datetime.now()
            indexGlow = 0 # so as not to glow EVERY time
            maxGlowSteps = 3
            break_out = False
            detected_something = False
            print('[INFO] Starting whistle detection...')


            while (not complete_break) and (not break_out):
                time.sleep(0)
                # whistle_current_time = datetime.now()
                # print("(whistle_current_time-whistle_initial_time).total_seconds() = ", (whistle_current_time-whistle_initial_time).total_seconds())
                # if (whistle_current_time-whistle_initial_time).total_seconds() > 30.0:
                #     print('30seconds since starting .. ')
                #     if not detected_something:
                #         print('Not detected anything, trying to reboot')
                #         camoLight.blinkByHour(newSleepTime=0.3, setHour=5)
                #         break_out = True
                #     else:
                #         whistle_initial_time = datetime.now()
                    
                #print('In second While True')
                if break_out:
                    print('Breaking out from second loop!')
                    # return
                    # complete_break = True  #  This would stop the loop completely, maybe rebooted by force by systemctl ?
                    # break # This breaks stuff
                else:
                  data = ''
                  try:
                      data = stream.read(CHUNK, exception_on_overflow=False) # TODO : was False, trying to detect the errors.
                  except Exception as e:
                      print(f"[ERROR] : Audio stream error while reading : {e}")
                      time.sleep(0.1)
                  if not data.strip(b'\x00'):
                      print("[WARN] Received empty/zeroed data")
                  # print(f"Raw data sample: {data[:10]}")
                  frames.append(data)
                  data_int = np.array(struct.unpack(str(CHUNK) + 'h', data))
                  fft = np.fft.rfft(data_int)
                  freq = np.fft.rfftfreq(len(data_int), 1.0 / RATE)
                  magnitude = np.abs(fft)
  
                  peak_idx = np.argmax(magnitude)
                  peak_freq = freq[peak_idx]
                  peak_power = magnitude[peak_idx]
                  now = datetime.now()
  
                  # print(f" {peak_freq:.1f} Hz @ {peak_power:.1f}")
  
                  if peak_power > PEAK_THRESHOLD:
                      print(f" (!) {peak_freq:.1f} Hz @ {peak_power:.1f}")
                      detected_something = True
                      recent_peaks.append((now, peak_freq, peak_power))
                      if peak_freq > 4000:
                          print('Peak freq seems unusually high; reducing it by 4000')
                          print('IndexGlow : ', indexGlow)
                          indexGlow += 1
                          if indexGlow%maxGlowSteps == 0:
                              camoLight.blinkByHour(newSleepTime=0.1, setHour=1)
                              print('Glowed enough, resetting before glowing again')
                          
                          # peak_freq = max(0, peak_freq - 4000)
  
                  if (now - whistle_start_time).total_seconds() > MAX_WHISTLE_DURATION:
                      if recent_peaks:
                          group_freqs = group_peaks_by_time(recent_peaks)
                          for group in group_freqs:
                              matched_pattern = match_whistle_to_pattern(group)
                              if matched_pattern or indexGlow >= 33: # or indexGlow >= 11 # Might not want this behaviour.. 
                                  if indexGlow >=33:
                                      print('Glowed way too many times ; Tim is curious and will adopt such pattern')
                                      matched_pattern = 'curious'
                                  indexGlow = 0
                                  print(f"\n[TRIGGER] Matched pattern '{matched_pattern}', terminating pyAudio stream and then launching video\n")
  
                                  # Stop PyAudio BEFORE recording
                                  try:
                                      print(' Starting stream.stop_stream() at {}'.format(datetime.now()))
                                      stream.stop_stream()
                                      print(' Done with stream.stop_stream() at {}'.format(datetime.now()))
                                      print(' Starting stream.close() at {}'.format(datetime.now()))
                                      stream.close()
                                      print(' Done with stream.close() at {}'.format(datetime.now()))
                                  except Exception as e:
                                      print('ERROR while stoping stream !!', e)
                                  try:
                                      print(' Starting p.terminate() at {}'.format(datetime.now()))
                                      p.terminate()
                                      print(' Done with p.terminate() at {}'.format(datetime.now()))
                                  except Exception as e:
                                      print('ERROR while terminating stream !!', e)
                                  print('PyAudio should have been stopped')
                                  time.sleep(10)
                                  print('Waited 10s for PyAudio to really stop, hopefully. Launching execute_matched_pattern')
                                  # Trigger video and servo
                                  execute_matched_pattern(matched_pattern)
  
                                  # Allow USB mic to reset
                                  time.sleep(10)
                                  print('Slept 10s after executing matched_pattern, Breaking out')
                                  break_out = True
                                  # Break out and restart new stream
                                  # break
                      # else:
                      #     print('No recent_peaks!')
  
                      # Reset
                      recent_peaks = []
                      frames = []
                      whistle_start_time = datetime.now()
            print('At {} : Broke out from second loop ! Hopefully restarting first one.'.format(datetime.now()))
            print('break_out : ', break_out)
            print('complete_break : ', complete_break)
            
        
        except Exception as e:
            print("\n[EXCEPTION-INFO] (maybe?) KeyboardInterrupt -> Exiting properly ...")
            print(e)
            try:
              stream.stop_stream()
              time.sleep(1)
              stream.close()
              time.sleep(1)
              p.terminate()
            except Exception as e2:
              print('... new exception, but passing : ', e2)
              pass
            time.sleep(1)
            servoCamo.clean()
            print('Complete_break = True')
            complete_break = True
        #finally:
            #print('finally : Going to sleep now. Cya later!')
            #complete_break = True
            #return True
        
    print('At {} : Done with detect_whistle ! Hope all is closed correctly !  Returning False (to try to trigger systemCtl)'.format(datetime.now()))
    return False
    #except Exception as e:
    #  print(f"[ERROR] Big Global Error: {e}")
      
    #  return False





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
    blinkOnce = False
    for pattern_name, pattern_freqs in patterns.items():
        dist, _, _, _ = dtw(pattern_freqs, freq_group[:len(pattern_freqs)], dist=lambda x, y: abs(x - y))
        # print('dist :', dist)
        # TODO : Here blink if the whistle is 'close enough'
        if dist < MAX_WHISTLE_DISTANCE_TOLERANCE:
            print(f"\n\n !!!! Detected pattern: {pattern_name} (DTW distance {dist:.1f})\n\n")
            camoLight.blinkByHour(newSleepTime=0.1, setHour=2)
            jiggleServo()
            #camoLight.blinkByHour(newSleepTime=0.3, setHour=1)
            return pattern_name
        elif dist < 1.3*MAX_WHISTLE_DISTANCE_TOLERANCE:
            print('Pattern is somewhat close to {} ! dist:{} < 1.3*{}:MAX_WHISTLE_DISTANCE_TOLERANCE={}'.format(pattern_name, dist, MAX_WHISTLE_DISTANCE_TOLERANCE, 1.3*MAX_WHISTLE_DISTANCE_TOLERANCE))
            blinkOnce = True
            
    print(" [X] No matching pattern found...")
    if blinkOnce:
        print('But blinking once for effort !')
        camoLight.blinkByHour(newSleepTime=0.1, setHour=1)
    return None
            
def deg_to_duty(deg):
    return 1.0*((deg - 0) * (MAX_DUTY- MIN_DUTY) / 180 + MIN_DUTY)

def jiggleServo():
    print('\n -> Launching servoCamo.jiggleServo')
    servoCamo.jiggleServo()
    print(' -> Done with servoCamo.jiggleServo ! \n')
    
    return True
    
    

if __name__ == '__main__':
    #cleanServo()
    print('Ready to start, just sleeping 7s first')
    camoLight.clearLight() # Moved to start to turn off light immediately
    time.sleep(7)
    print('Lets reset xset !')
    # os.system("sh /home/samonac/xset-blank.sh") # Forcing screensaver to 10 seconds
    servoCamo.resetXset(wait=True)
    print('(should have launched xset-blank.sh)') 
    print('At {} : Starting Main ! Good luck Tim !'.format(datetime.now()))
    try:
        detect_whistle()
    except Exception as e:
        print('Detect_Whistle Exception : ', e)
        camoLight.clearLight() # Moved to start to turn off light immediately
        pass
    
    print('At {} : Done with detect_whistle() in main ! Hope it was all OK !'.format(datetime.now()))
    servoCamo.clean()
    print('At {} : Done with servoCamo.clean()'.format(datetime.now()))
    camoLight.clean()
    print('At {} : Done with camoLight.clean() and therefore main ! Maybe should return something here to trigger systemCtl ? Anyway, goodnight Tim ! (for now)'.format(datetime.now()))
