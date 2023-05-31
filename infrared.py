import RPi.GPIO as GPIO

import time

#make sure programm is running
print("Hello-there")

#configure pi by Board pinout 
GPIO.setmode(GPIO.BOARD)
sensor= 16
led = 18


# sensor is input
GPIO.setup(sensor,GPIO.IN)

# led is output
GPIO.setup(led,GPIO.OUT)


try:
    while True:
        if GPIO.input(sensor): # if there is not object
            GPIO.output(led,True) # LED is on
        else:
            GPIO.output(led,False)  # object in front LED off
except KeyboardInterrupt:
     GPIO.cleanup()

    
