#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.servo_pin = 5
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.angle = 0
        self.increment = 1
        self.timer = self.create_timer(0.05, self.timer_callback)

    def servo_pulse(self, angle):
        pulsewidth = (angle * 11) + 500
        GPIO.output(self.servo_pin, GPIO.HIGH)
        time.sleep(pulsewidth / 1000000.0)
        GPIO.output(self.servo_pin, GPIO.LOW)
        time.sleep(20.0 / 1000 - pulsewidth / 1000000.0)

    def timer_callback(self):
        self.servo_pulse(self.angle)
        self.angle += self.increment
        if self.angle >= 180 or self.angle <= 0:
            self.increment *= -1

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
