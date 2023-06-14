import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import String

LFIR_PIN = 10
RFIR_PIN = 9
CFIR_PIN = 11
LH_PIN = 20
RH_PIN = 21

SENSOR_PINS = [LFIR_PIN, RFIR_PIN, CFIR_PIN, LH_PIN, RH_PIN]

class InfraredPublisher(Node):
    last_string = "None"
    def __init__(self):
        super().__init__('ir_publisher')
        # self.publisher_ = self.create_publisher(String, 'voice', 10)
        self.get_logger().info("Infrared node running!")
        self.publisher_ = self.create_publisher(String, 'sensor', 10)

        GPIO.setmode(GPIO.BCM)
        self.setup_sensors()

        try:
            while True:
                for pin in SENSOR_PINS:
                    self.check_sensor(pin)
        except KeyboardInterrupt:
            GPIO.cleanup()

    def publish_sensor(self, sensor):
        msg = String()
        msg.data = "DETECTION FROM " + str(sensor)
        self.publisher_.publish(msg)

    def setup_sensors(self):
        GPIO.setup(LFIR_PIN, GPIO.IN)
        GPIO.setup(RFIR_PIN, GPIO.IN)
        GPIO.setup(CFIR_PIN, GPIO.IN)
        GPIO.setup(LH_PIN, GPIO.IN)
        GPIO.setup(RH_PIN, GPIO.IN)
    
    def check_sensor(self, pin):
        # set to not for testing, will send signal when close to object rather than far away
        if not GPIO.input(pin):
            self.get_logger().info("Infrared Sensed Something")
            self.publish_sensor(pin)
        

def main(args=None):
    rclpy.init(args=args)
    ir_publisher = InfraredPublisher()
               
    rclpy.spin(ir_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
