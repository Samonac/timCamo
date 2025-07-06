import RPi.GPIO as GPIO
import time
import datetime

# duty cycle, calibrate if needed
MIN_DUTY = 2.9
MAX_DUTY = 97.1

 # 32 = Right // 33 = Left
servo_signal_pin_R = 32
servo_signal_pin_L = 33
light_pin = 12
    
# GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
# Set up pin 11 for PWM

GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
GPIO.setup(servo_signal_pin_R,GPIO.OUT)  # Sets up pin 32 to an output (instead of an input)
GPIO.setup(servo_signal_pin_L,GPIO.OUT)  # Sets up pin 33 to an output (instead of an input)

p_32 = GPIO.PWM(servo_signal_pin_R, 50)     # Sets up pin 32 as a PWM pin
p_33 = GPIO.PWM(servo_signal_pin_L, 50)     # Sets up pin 33 as a PWM pin

# Light setup
# light_pwn=''  # Temporary ; creates flashing apparently
GPIO.setup(light_pin,GPIO.OUT)  # Sets up light pin 12 to an output (instead of an input)
light_pwn=GPIO.PWM(light_pin, 50)
      
# Max degrees
MIN_DEGREE = 7
MAX_DEGREE = 15

# Max Light (overheats!)
MAX_LIGHT = 50

pastDeg1 = MAX_DEGREE # = 32 = RIGHT
pastDeg2 = MIN_DEGREE # = 33 = LEFT
# TODO : Might need to read and save these values from a JSON file in case of an unexpected reboot


# For jiggling purposes
dutyCycle32 = 10.5
dutyCycle33 = 6.5
jiggleDelta = 0.5

def deg_to_duty(deg):
    return 1.0*((deg - 0) * (MAX_DUTY- MIN_DUTY) / 180 + MIN_DUTY)


GPIO.setwarnings(True)
GPIO.setmode(GPIO.BOARD)


def clearLight():
    global light_pwn
    light_pwn.start(100) 
    light_pwn.ChangeDutyCycle(100)
    light_pwn.stop()

def jiggleServo():
    print('jiggleServo with dutyCycle32 : ', dutyCycle32, ' and dutyCycle33 : ', dutyCycle33)
        
    global p_32
    global p_33
    
    p_32.start(0)               # Starts running PWM on the pin and sets it to 0
    p_33.start(0)               # Starts running PWM on the pin and sets it to 0
    time.sleep(0.1)
    p_33.ChangeDutyCycle(dutyCycle33)
    time.sleep(0.1)
    p_32.ChangeDutyCycle(dutyCycle32)
    time.sleep(0.1)
    # dutyCycle1$
    p_33.ChangeDutyCycle(dutyCycle33+jiggleDelta)
    time.sleep(0.1)
    # dutyCycle1
    p_32.ChangeDutyCycle(dutyCycle32-jiggleDelta)
    time.sleep(0.1)
    
    p_33.ChangeDutyCycle(dutyCycle33)
    time.sleep(0.1)
    p_32.ChangeDutyCycle(dutyCycle32)
    time.sleep(0.3)
    #p_32.stop()               # Starts running PWM on the pin and sets it to 0
    #p_33.stop()               # Starts running PWM on the pin and sets it to 0
    


def moveServo(start=True, servoList = [], minAngleInput = MIN_DEGREE, maxAngleInput = MAX_DEGREE):

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

    

def moveServoThread(VIDEO_DURATION=10):
    p_32.start(0)               # Starts running PWM on the pin and sets it to 0
    p_33.start(0)               # Starts running PWM on the pin and sets it to 0
    # changeLight(33, start=True, stop=False, sleepTime=0.5)
    #time.sleep(0.1)
    #print('   Sleeping 1s then jiggling servos')
    #jiggleServo()
    #print('   Sleeping 1s then starting for real')
    #time.sleep(1)
    
    
    changeLight(0, start=True, stop=True, sleepTime=0.5) 
    print('Start opening')
    moveServo(start=True, servoList = ['L'], minAngleInput = MIN_DEGREE, maxAngleInput = 10)
    moveServo(start=True, servoList = ['L', 'R'], minAngleInput = MIN_DEGREE, maxAngleInput = MAX_DEGREE)
    print(' > Done !')
    print('  Video will be filming shortly ; Sleeping {}s before closing..'.format(VIDEO_DURATION))
    changeLight(30, start=True, stop=True, sleepTime=0.5) 
    #p_32.stop()                   # At the end of the program, stop the PWM
    #p_33.stop()  
    
    #p_32 = GPIO.PWM(servo_signal_pin_R, 50)     # Sets up pin 32 as a PWM pin
    #p_33 = GPIO.PWM(servo_signal_pin_L, 50)     # Sets up pin 33 as a PWM pin
    
    #p_32.start(0)               # Starts running PWM on the pin and sets it to 0
    #p_33.start(0)               # Starts running PWM on the pin and sets it to 0
    if VIDEO_DURATION > 8:
      time.sleep(VIDEO_DURATION-8)
      print('Sleeping done!')
      
      #changeLight(50, start=False, stop=False, sleepTime=0.5)
      #changeLight(33, start=False, stop=False, sleepTime=0.5)
      
      #changeLight(50, start=False, stop=False, sleepTime=0.5)
      #changeLight(33, start=False, stop=False, sleepTime=0.5)
      
      #changeLight(50, start=False, stop=False, sleepTime=1)
      #changeLight(33, start=False, stop=False, sleepTime=1)
    
      
    changeLight(0, start=True, stop=False, sleepTime=0.5) 
    print('Start closing')
    moveServo(start=False, servoList = ['R'], minAngleInput = MIN_DEGREE, maxAngleInput = 10)
    moveServo(start=False, servoList = ['L', 'R'], minAngleInput = MIN_DEGREE, maxAngleInput = MAX_DEGREE)
    print(' > Done done !')
    time.sleep(0.1)
    clean()
    
    
def clean():
    p_32.stop()                   # At the end of the program, stop the PWM
    p_33.stop()  
    if light_pwn != '':
      print('Light stopped in clean()')
      light_pwn.stop()
    GPIO.cleanup()           # Resets the GPIO pins back to defaults
    print('Cleaning done ! Goodnight !')
        
        
def changeLight(percent, start=False, stop=False, sleepTime=1):
    global light_pwn
    print('\n (!) In changeLight with percent : ', percent)
    actualPercent = 100-percent
    if actualPercent == 0:
      actualPercent = 1
    if actualPercent > MAX_LIGHT:
      acutalPercent = MAX_LIGHT
    print('ActualPercent : ', actualPercent)
    if start:
      print('Starting light')
      #  These 2 lines below create flashing lights...
      # GPIO.setup(light_pin,GPIO.OUT)  # Sets up light pin 12 to an output (instead of an input)
      # light_pwn=GPIO.PWM(light_pin, 1)
      
      light_pwn.start(actualPercent) 
      
    light_pwn.ChangeDutyCycle(actualPercent)
    print('Sleeping ', sleepTime)
    time.sleep(sleepTime)
    
    if stop:
      print('Stopping light')
      # light_pwn.ChangeDutyCycle(100) # Equivalent to turning it off
      if light_pwn != '':
        light_pwn.stop()
        print('Stopped')
    print('Done with changeLight!\n')
    
    
def blinkByHour(newSleepTime=1):
    print('In blinkByHour for newSleepTime : ', newSleepTime)
    currentHour = datetime.datetime.now().hour
    if currentHour > 12:
      currentHour = currentHour-12
    print('Current hour is : ', currentHour)
    
    indexHour = 0
    while indexHour < currentHour:
      changeLight(100, start=True, stop=True, sleepTime=0.5)
      changeLight(50, start=True, stop=True, sleepTime=0.5)
      indexHour += 1
    
    changeLight(0, start=True, stop=False, sleepTime=0.5)  
    # clean()

changeLight(0, start=True, stop=False, sleepTime=0.5)  
if __name__ == "__main__":

    # changeLight(0, start=True, stop=False, sleepTime=0.5)  
    tempIndex = 0
    maxIndex = 1
    # maxIndex = 1
    #jiggleServo()
    print('Done jiggling')
    #while tempIndex < maxIndex:
    time.sleep(1)
    print('attempt n ', tempIndex)
    print('Hello ! Starting main !')
    moveServoThread()
    print('Done moving ! ')
    time.sleep(0.1)
    
    #moveServoThread()
    #print('Done moving x2 ! ')
    
    #tempIndex += 1
    print('Goodnight !')