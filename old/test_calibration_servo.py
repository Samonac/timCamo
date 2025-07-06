import RPi.GPIO as GPIO
import time

# duty cycle, calibrate if needed
MIN_DUTY = 2.9
MAX_DUTY = 97.1

 # 32 = Right // 33 = Left
servo_signal_pin_R = 32
#servo_signal_pin_L = 33

# Max degrees
MIN_DEGREE = 7
MAX_DEGREE = 20

def deg_to_duty(deg):
    return 1.0*((deg - 0) * (MAX_DUTY- MIN_DUTY) / 180 + MIN_DUTY)


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)


def moveServo(start=True):
    print('moveServo with mode_start =', start, 'for degrees [{}, {}]'.format(MIN_DEGREE, MAX_DEGREE))
    
    # GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
    # Set up pin 11 for PWM
    GPIO.setup(servo_signal_pin_R,GPIO.OUT)  # Sets up pin 32 to an output (instead of an input)
    # GPIO.setup(servo_signal_pin_L,GPIO.OUT)  # Sets up pin 33 to an output (instead of an input)
    p_32 = GPIO.PWM(servo_signal_pin_R, 50)     # Sets up pin 32 as a PWM pin
    # p_33 = GPIO.PWM(servo_signal_pin_L, 50)     # Sets up pin 33 as a PWM pin
    p_32.start(0)               # Starts running PWM on the pin and sets it to 0
    # p_33.start(0)               # Starts running PWM on the pin and sets it to 0

    minAngle = 0
    maxAngle = 0
    index = 1
    # Move the servo back and forth
    if start:
      index = -1
      minAngle = MAX_DEGREE
      maxAngle = MIN_DEGREE
    else:
      minAngle = MIN_DEGREE
      maxAngle = MAX_DEGREE
    # loop from 180 to 0
    deg = minAngle
    while deg != maxAngle:
        print('deg : ', deg)
        duty_cycle = deg_to_duty(deg)    
        print('duty_cycle : ', duty_cycle)
        p_32.ChangeDutyCycle(duty_cycle)
        
        deg2 = minAngle+maxAngle-deg
        # print('deg2 : ', deg2)
        duty_cycle2 = deg_to_duty(deg2)    
        # print('duty_cycle2 : ', duty_cycle2)
        # p_33.ChangeDutyCycle(duty_cycle2)
        deg += index
        if abs(deg) - int(minAngle+maxAngle/4) < int(minAngle+maxAngle/4) or abs(deg) > minAngle+maxAngle - int(minAngle+maxAngle/4):
          print(' -> SLOW ! sleeping 0.1')
          time.sleep(0.1)
        else:
          print('FAST ! sleeping 0.05')
          time.sleep(0.05)   
    

    # GPIO.setup(servo_signal_pin_R,GPIO.IN)  # Temporarily Sets up pin 32 to an input (to avoid jittering, but didn't work)
    # GPIO.setup(servo_signal_pin_L,GPIO.IN)  # Temporarily Sets up pin 33 to an int (to avoid jittering, but didn't work)
    p_32.stop()                 # At the end of the program, stop the PWM    
    # p_33.stop()  
    

    
if __name__ == "__main__":
    moveServo(True)
    time.sleep(4)
    moveServo(False)
    
    GPIO.cleanup()           # Resets the GPIO pins back to defaults