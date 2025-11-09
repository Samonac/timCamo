import RPi.GPIO as GPIO
import time
import datetime

light_pin = 12
    
# GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
# Set up pin 11 for PWM

GPIO.cleanup()           # Resets the GPIO pins back to defaults
GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout
# Light setup
# light_pwn=''  # Temporary ; creates flashing apparently
GPIO.setup(light_pin,GPIO.OUT)  # Sets up light pin 12 to an output (instead of an input)
light_pwn=GPIO.PWM(light_pin, 50)

# Max Light (overheats!)
MAX_LIGHT = 50

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
    

        
        
def changeLight(percent=0, start=True, stop=False, sleepTime=1):
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
        #light_pwn.stop()
        clearLight()
        print('Stopped')
    print('Done with changeLight!\n')
    
    
def blinkByHour(newSleepTime=0.3, setHour=33):
    
    print('In blinkByHour for newSleepTime : ', newSleepTime)
    currentHour = datetime.datetime.now().hour
    if currentHour > 12:
      currentHour = currentHour-12
    if setHour != 33:
      currentHour = setHour
      print('setHour not 33 ; overriding current hour and setting new current hour')
    print('Current hour is : ', currentHour)
    
    indexHour = 0
    while indexHour < currentHour:
      changeLight(5, start=True, stop=False, sleepTime=0.4)
      changeLight(33, start=True, stop=False, sleepTime=0.6)
      indexHour += 1
      time.sleep(newSleepTime)
      # changeLight(0, start=True, stop=False, sleepTime=0.5)  
    
    changeLight(0, start=True, stop=False, sleepTime=0.5)  
    # clean()

def clearLight():
    print('In clearLight()')
    changeLight(0, start=True, stop=False, sleepTime=0) 

clearLight()


def clean():
    print('CamoLight : Launching clean()')
    changeLight(percent=0, start=True, stop=False, sleepTime=0)  
    #print('launching GPIO.cleaning soon')
    #time.sleep(7)
    GPIO.cleanup()           # Resets the GPIO pins back to defaults
    print('Cleaning done ! Goodnight !')
 
 
def run():
    print('in run')
    changeLight(percent=5, start=True, stop=False, sleepTime=2) 
    changeLight(percent=15, start=True, stop=False, sleepTime=2) 
    changeLight(percent=45, start=True, stop=False, sleepTime=2) 
    changeLight(percent=0, start=True, stop=False, sleepTime=0.5)  
    time.sleep(0.5)
    
if __name__ == "__main__":
    print('In main camoLight')    
    blinkByHour(newSleepTime=0.1)
    #run()
    #time.sleep(1)
    #run()
    changeLight(percent=0, start=True, stop=False, sleepTime=0.5)  
    
    time.sleep(5)
    clean()
    #tempIndex += 1
    print('All done with camoLight ! Goodnight !')