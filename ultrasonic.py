## code taken from https://tutorials-raspberrypi.com/



import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

T = 23
R = 24
while (True):
    GPIO.setmode(GPIO.BCM)  
    T = 23
    R = 24
    print("Distance Measurement in Progress")
    GPIO.setup(T,GPIO.OUT)
    GPIO.setup(R,GPIO.IN)

    GPIO.output(T,False)
    print("Waiting for Sensor to settle")
    time.sleep(2)

    GPIO.output(T,True)
    time.sleep(0.00001)
    GPIO.output(T,False)

    while GPIO.input(R) == 0:
        pulse_start = time.time()

    while GPIO.input(R) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration *17150

    distance = round(distance,2)

    print(f"Distance {distance} cm")

    GPIO.cleanup()