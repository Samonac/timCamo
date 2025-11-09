import subprocess
import pigpio
import time
import datetime
import os

# Step 1: Restart pigpiod via shell script
print("[INFO] Restarting pigpiod & resetting screensaver via shell script...")
try:
    subprocess.run(["/bin/bash", "/home/samonac/git_projects/timCamo/resetServo.sh"], check=True)
    print("[INFO] pigpiod restart script executed successfully.")
except subprocess.CalledProcessError as e:
    print(f"[ERROR] Failed to restart pigpiod : {e}")
    exit(1)

# Step 2: Short wait to allow pigpiod to fully initialize
time.sleep(5)

# Step 3: Connect to pigpio daemon
pi = pigpio.pi()
if not pi.connected:
    raise Exception("\n\n\n (!!!) pigpio daemon is not running or failed to connect!\n\n\n")

# ================== Your existing code ==================
# Servo control: PWM pulsewidth range
MIN_PW = 500     # in microseconds (~2.5%)
MAX_PW = 2500    # in microseconds (~12.5%)

# GPIO Pins
servo_signal_pin_R = 12
servo_signal_pin_L = 13

# Angle limits
MIN_DEGREE = 0
MAX_DEGREE = 90
indexDegree = 9
deltaDegree = 10

# Jiggle setup
dutyCycle32 = 0
dutyCycle33 = 0
jiggleDelta = 90 #degrees of jiggle

# Global past states
pastDeg1 = MAX_DEGREE
pastDeg2 = MIN_DEGREE
SLOW_SLEEP_TIME=0.1
FAST_SLEEP_TIME=0.05


def resetXset():
    try:
        subprocess.run(["/bin/bash", "/home/samonac/git_projects/timCamo/xset-blank.sh"], check=True)
        print("[INFO] xset-blank restart script executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to launch xset-blank : {e}")
        exit(1)
def deg_to_pulsewidth(deg):
    return int((deg / 180) * (MAX_PW - MIN_PW) + MIN_PW)

def moveServoThread(VIDEO_DURATION=10, degInputMin=MIN_DEGREE, degInputMax=MAX_DEGREE, servoList=[]):
    print('in cycleServos')
    sleepTime=0.1
    pi.set_servo_pulsewidth(servo_signal_pin_R, 0)
    pi.set_servo_pulsewidth(servo_signal_pin_L, 0)
    deg = 0
    while deg <= degInputMax:
        print('\n -> deg :', deg)
        pw32 = deg_to_pulsewidth(deg)
        pw33 = deg_to_pulsewidth(deg)
        print('pw32 (R) : ', MAX_PW - pw32, ' and pw33 (L) : ', pw33)

        pi.set_servo_pulsewidth(servo_signal_pin_L, pw33)
        pi.set_servo_pulsewidth(servo_signal_pin_R, MAX_PW - pw32)
        time.sleep(sleepTime)
        deg += deltaDegree

    if VIDEO_DURATION > 8:
        print('Now sleeping VIDEO_DURATION : ', VIDEO_DURATION)
        time.sleep(VIDEO_DURATION-5)

    print('Now reversing !')
    deg -= deltaDegree
    while deg >= degInputMin:
        print('\n -> deg :', deg)
        pw32 = deg_to_pulsewidth(deg)
        pw33 = deg_to_pulsewidth(deg)
        print('pw32 (R) : ', MAX_PW - pw32, ' and pw33 (L) : ', pw33)

        pi.set_servo_pulsewidth(servo_signal_pin_L, pw33)
        pi.set_servo_pulsewidth(servo_signal_pin_R, MAX_PW - pw32)
        time.sleep(sleepTime)
        deg -= deltaDegree

    print('Resetting to pw 0')
    pi.set_servo_pulsewidth(servo_signal_pin_R, 0)
    pi.set_servo_pulsewidth(servo_signal_pin_L, 0)
    time.sleep(sleepTime)
    print('Done with cycleServos!')

def jiggleServo():
    print('jiggleServo with dutyCycle32 : ', dutyCycle32, ' and dutyCycle33 : ', dutyCycle33)
    pi.set_servo_pulsewidth(servo_signal_pin_R, 0)
    pi.set_servo_pulsewidth(servo_signal_pin_L, 0)
    moveServoThread(VIDEO_DURATION=0, degInputMin=MIN_DEGREE, degInputMax=MIN_DEGREE+20)
    print('Done with jiggling!')
    time.sleep(1)
    pi.set_servo_pulsewidth(servo_signal_pin_R, 0)
    pi.set_servo_pulsewidth(servo_signal_pin_L, 0)

def clean():
    pi.set_servo_pulsewidth(servo_signal_pin_R, 0)
    pi.set_servo_pulsewidth(servo_signal_pin_L, 0)
    print('  -> camoServo clean() complete')

if __name__ == "__main__":
    print('\nStarting camoServo\n')
    try:
        jiggleServo()
        print('Done jiggling 1')
        moveServoThread()
        print('Done cycling')
        time.sleep(1)
    finally:
        clean()
        print('\nCamoServo All done! Goodnight!\n ')
