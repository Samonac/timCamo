import RPi.GPIO as GPIO
import time


 # 32 = Right // 33 = Left
light_pin = 12
    
# GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
# Set up pin 11 for PWM

GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
GPIO.setup(light_pin,GPIO.OUT)  # Sets up pin 31 to an output (instead of an input)

my_pwm=GPIO.PWM(light_pin,100)

my_pwm.start(50)
time.sleep(2)

my_pwm.ChangeDutyCycle(100)
time.sleep(1)
my_pwm.ChangeDutyCycle(90)
time.sleep(1)
my_pwm.ChangeDutyCycle(80)
time.sleep(1)
my_pwm.ChangeDutyCycle(70)
time.sleep(1)
my_pwm.ChangeDutyCycle(60)
time.sleep(1)
# my_pwm.ChangeDutyCycle(50)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(40)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(30)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(20)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(10)

time.sleep(6)

# my_pwm.ChangeDutyCycle(10)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(20)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(30)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(40)
# time.sleep(1)
# my_pwm.ChangeDutyCycle(50)
time.sleep(1)
my_pwm.ChangeDutyCycle(60)
time.sleep(1)
my_pwm.ChangeDutyCycle(70)
time.sleep(1)
my_pwm.ChangeDutyCycle(80)
time.sleep(1)
my_pwm.ChangeDutyCycle(90)
time.sleep(1)
my_pwm.ChangeDutyCycle(100)
time.sleep(2)
my_pwm.stop()
GPIO.cleanup()
print('done')