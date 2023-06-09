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
            self.send_string('mf\n')
            time.sleep(1)
            self.send_string('mb\n')
            time.sleep(1)
            self.send_string('ml\n')
            time.sleep(1)
            self.send_string('mr\n')
            time.sleep(1)
            self.send_string('ms\n')
            time.sleep(1)
            self.send_string('mf\n')
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('mb\n')
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('sd\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('su\n')
            time.sleep(1)
            self.send_string('ms\n')
        elif msg.data == "REACH":
            self.send_string('lr\n')
            time.sleep(3)
            self.send_string('lb\n')

        elif msg.data == "WAVE":
            self.send_string('d\n')
        


        

    def receive_msg(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info(line)
        else:
            self.get_logger().info('Nothing incoming')
    
    def send_char(self, instruction):
        self.get_logger().info('Sending to Arduino ' + instruction + '...')
        self.ser.write(instruction.encode('utf-8'))

    def send_string(self, instruction):
        self.get_logger().info('Send String to Arduino')
        self.ser.write(instruction.encode('utf-8'))

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

    def move(self, direction):
        self.get_logger().info(f'Tell Arduino to move {direction}...')
        self.ser.write(direction.encode('utf-8'))
def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()
    rclpy.spin(serial_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()