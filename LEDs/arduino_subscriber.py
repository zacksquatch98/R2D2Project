#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String

class SerialServer(Node):

    def __init__(self):
        super().__init__('arduino_subscriber')

        self.subscription = self.create_subscription(
            String,
            'arduino',
            self.listener_callback,
            10
        )
        self.subscription   # prevent unused variable warning

        self.get_logger().info("Serial Server Running")

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.reset_input_buffer()

        # self.create_timer(1.0, self.receive_msg)

    def listener_callback(self, msg):
        self.get_logger().info('Arduino message "%s"' % msg.data)
        if msg.data == "HELLO":
            self.red_led_on()
            self.receive_msg()
        elif msg.data == "GOODBYE":
            self.turn_off()
        elif msg.data == "WAVE":
            self.toggle_leds()


    def receive_msg(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info(line)
        else:
            self.get_logger().info('Nothing incoming')
    
    def red_led_on(self):
        self.get_logger().info('Turning on RED LED...')
        self.ser.write(str(3).encode('utf-8'))

    def turn_off(self):
        self.get_logger().info('Turning off lights!')
        self.ser.write(str(-1).encode('utf-8'))

    def toggle_leds(self):
        self.get_logger().info('Enjoy the light show!')
        led = 3
        for i in range(10):
            self.ser.write(str(led).encode('utf-8'))
            led = led % 2 + 3
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()
    rclpy.spin(serial_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
