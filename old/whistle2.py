import pyaudio
import struct
import numpy as np
from dtw import dtw
import subprocess
# import datetime
from datetime import datetime, timedelta
import random
import threading
import math
import time
#import pyautogui
#import pyperclip
import os
from random import randrange
import datetime

# Define the known whistle patterns
patterns = {
    'pattern1': [261, 329, 392],  # C major chord
    'pattern2': [392, 329, 261],  # C major chord (reversed)
    'pattern3': [392, 523, 659],  # G major chord
}
whistleList = []

# Parameters for audio capture and processing
THRESHOLD = 8000  # adjust this value to set the sensitivity of the detector
FREQ_THRESHOLD = 20000
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 2048

wandCapturePath = 'C:/Users/samon/PycharmProjects/wandCapture/main.py'
lightControlPath = 'C:/Users/samon/PycharmProjects/wandCapture/lights_control.py'
defaultPythonParam = ['--param1', 'value1', '--param2', 'value2']


def openPython(script_path=wandCapturePath, params=defaultPythonParam):
    # Specify the path to the Python script you want to run
    # script_path =

    # Specify any command line parameters for the script
    # params =

    # Asynchronously open the script with the specified parameters
    # subprocess.Popen(['python', script_path] + params)
    print('Here, I would launch the script located at : {}'.format(script_path))


def detect_whistle():
    print('Detecting Whistling now !')
    global whistleList
    totalWhistle = 0
    lightControlIndex = 0
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    while True:
        data = stream.read(CHUNK)
        data_int = np.array(struct.unpack(str(2 * CHUNK) + 'B', data), dtype='b')[::2]
        fft = np.fft.fft(data_int)
        freq = np.fft.fftfreq(CHUNK, 1.0 / RATE)
        idx = np.argmax(np.abs(fft))
        if freq[idx] > FREQ_THRESHOLD:
            print('high freq : {}'.format(freq[idx]))
            print("fft[idxy]:", fft[idx])
            
        if freq[idx] > FREQ_THRESHOLD and np.abs(fft[idx]) > THRESHOLD:
            detected_freq = freq[idx]
            print("\n\n => Whistle detected at frequency:", detected_freq)
            print("fft[idxy]:", fft[idx])
            # import datetime

            whistleTime = datetime.datetime.now()

            # whistleTime = today.strftime("%Y-%m-%d %H:%M:%S.%f")
            print("Whistle detected at whistleTime:", whistleTime)
            # whistleTime = datetime.datetime("%Y-%m-%d %H:%M:%S.%f")
            whistleTemp = {'frequency' : detected_freq, 'timeDetected': whistleTime}
            whistleList.append(whistleTemp)
            print(f'whistleList : {whistleList}')
            newWhistleListTemp = []
            for indexList, whistlePast in enumerate(whistleList):
                # print(indexList)
                # print(f' => whistleList index : {indexList}')
                # print(f'whistleTime : {whistleTime}')
                # print(f'whistlePast : {whistlePast}')
                # fulldate = fulldate + datetime.timedelta(milliseconds=500)
                # delta33 = datetime.datetime.strptime(whistlePast['timeDetected'], "%Y-%m-%d %H:%M:%S.%f")
                delta33 = whistlePast['timeDetected']

                if (whistlePast['timeDetected'] < whistleTime - timedelta(seconds=33)) or (whistlePast['timeDetected'] > whistleTime - timedelta(microseconds=3333) and whistlePast['timeDetected'] < whistleTime):
                    print(f"whistlePast does not work : {whistlePast}")
                    print(f"whistlePast['timeDetected'] : {whistlePast['timeDetected']}")
                    # print(f"whistleTime - datetime.second(33) : {whistleTime - datetime.second(33)}")
                    # print(f"whistleTime - datetime.microsecond(3333) : {whistleTime - datetime.microsecond(3333)}")

                    # whistleList = whistleList[1:]
                    # whistleList.
                else:
                    print(f'Keeping this whistlePast : {whistlePast}')
                    newWhistleListTemp.append(whistlePast)
            # print(f'whistleList (1) will be replaced by newWhistleListTemp (2) : \n(1) : {whistleList}\n(2) : {newWhistleListTemp}')
            whistleList = newWhistleListTemp

            if detected_freq >= 11000:
                print(f'(!) very high frequency @ {whistleTime}')
                print(f'len(whistleList) <3 ? {len(whistleList)} < 3 = {len(whistleList) < 3}')
                if len(whistleList) > 3:
                    print('3 very high past frequencies ! Switching lights')
                    print(f"lightControlIndex : {lightControlIndex}")

                    openPython(lightControlPath, [['full', 'off'][lightControlIndex]])
                    lightControlIndex = (lightControlIndex + 1)%2
                    # print(f"lightControlIndex : {lightControlIndex}")

            if detected_freq >= 7000:
                totalWhistle += 1
            if totalWhistle >= 10 and len(whistleList) > 3:
                print(f'opening wandCapture because 10 <= {totalWhistle} and 3 < {len(whistleList)} : len({whistleList})')
                totalWhistle = 0
                openPython(wandCapturePath)
            else:
                print(f'not opening wandCapture because totalWhistle ({totalWhistle}) < 10 or len(whistleList) ({len(whistleList)}) <= 3)')
            print("Total whistle:", totalWhistle)
            for pattern_name, pattern_freqs in patterns.items():
                dist, cost, acc, path = dtw(pattern_freqs, [detected_freq], dist=lambda x, y: np.abs(x - y))
                if dist < 100:
                    print("Detected pattern:", pattern_name)

    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == '__main__':
    print('Launching')
    detect_whistle()
