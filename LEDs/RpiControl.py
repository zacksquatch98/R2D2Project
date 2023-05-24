#!/usr/bin/env python3
import serial

def pick_led_state():
    color = input('What color should I send to Arduino? ')
    if color.lower().strip() == 'quit':
        return -1
    elif color.lower().strip() == 'yellow':
        return 1
    elif color.lower().strip() == 'green':
        return 2
    elif color.lower().strip() == 'red':
        return 3
    elif color.lower().strip() == 'blue':
        return 4
    else:
        print('Not a valid color. . .')
        return 0

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    led_state = 0

    while True:
        led_state = pick_led_state()
        if led_state == -1:
            print('Quitting. . .')
            ser.write(str(0).encode('utf-8'))
            break
        print('Sending number ' + str(led_state) + ' to Arduino')
        ser.write(str(led_state).encode('utf-8'))