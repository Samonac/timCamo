# Set up libraries and overall settings
import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout

servo_signal_pin_32 = 32
servo_signal_pin_33 = 33

# Set up pin 11 for PWM
GPIO.setup(servo_signal_pin_32,GPIO.OUT)  # Sets up pin 32 to an output (instead of an input)
#GPIO.setup(servo_signal_pin_33,GPIO.OUT)  # Sets up pin 33 to an output (instead of an input)

p_32 = GPIO.PWM(servo_signal_pin_32, 50)     # Sets up pin 32 as a PWM pin
#p_33 = GPIO.PWM(servo_signal_pin_33, 50)     # Sets up pin 33 as a PWM pin

p_32.start(0)               # Starts running PWM on the pin and sets it to 0
#p_33.start(0)               # Starts running PWM on the pin and sets it to 0

# Move the servo back and forth
p_32.ChangeDutyCycle(3)     # Changes the pulse width to 3 (so moves the servo)
#p_33.ChangeDutyCycle(3)     # Changes the pulse width to 3 (so moves the servo)
sleep(1)                 # Wait 1 second

p_32.ChangeDutyCycle(12)    # Changes the pulse width to 12 (so moves the servo)
#p_33.ChangeDutyCycle(12)    # Changes the pulse width to 12 (so moves the servo)
sleep(1)

# Clean up everything
p_32.stop()                 # At the end of the program, stop the PWM
#p_33.stop()                 # At the end of the program, stop the PWM
GPIO.cleanup()           # Resets the GPIO pins back to defaults