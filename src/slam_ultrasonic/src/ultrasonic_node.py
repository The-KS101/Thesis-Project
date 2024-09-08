#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import RPi.GPIO as GPIO
import time
import math
from filterpy.kalman import KalmanFilter
import numpy as np

class UltrasonicSensor(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust timer as needed

        self.scan_msg = LaserScan()
        self.scan_msg.header = Header()
        self.scan_msg.header.frame_id = 'laser_frame'  # Frame ID from URDF
        self.scan_msg.angle_min = -math.pi / 2  # Start angle of the scan
        self.scan_msg.angle_max = math.pi / 2   # End angle of the scan
        self.scan_msg.angle_increment = math.pi / 90.0  # Reduce increment to cover fewer angles
        self.scan_msg.time_increment = 0.0  # Time between measurements (not applicable)
        self.scan_msg.scan_time = 48.0  # Time taken for each scan (adjust as needed)
        self.scan_msg.range_min = 0.02  # Minimum range value (in meters) - 2 cm
        self.scan_msg.range_max = 4.0  # Maximum range value (in meters) - 400 cm
        number_of_readings = int((self.scan_msg.angle_max - self.scan_msg.angle_min) / self.scan_msg.angle_increment) + 1
        self.scan_msg.ranges = [float('inf')] * number_of_readings

        self.trigger_pin = 9  # GPIO pin for Trigger
        self.echo_pin = 8  # GPIO pin for Echo
        self.servo_pin = 5  # GPIO pin for Servo

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        self.servo = GPIO.PWM(self.servo_pin, 50)  # 50Hz PWM frequency
        self.servo.start(0)

        # Initialize the Kalman Filter
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.kf.x = np.array([0., 0.])       # initial state (location and velocity)
        self.kf.F = np.array([[1., 1.],
                              [0., 1.]])     # state transition matrix
        self.kf.H = np.array([[1., 0.]])     # measurement function
        self.kf.P *= 1000.                   # covariance matrix
        self.kf.R = 5                        # measurement noise
        self.kf.Q = np.array([[1., 0.],
                              [0., 1.]])     # process noise

    def skip(self):
        pass

    def timer_callback(self):
        distances = self.measure_distances()
        if distances:
            filtered_distances = [self.kalman_filter(d) for d in distances]
            self.publish_scan(filtered_distances)

    def measure_distances(self):
        try:
            distances = []
            angles = [self.scan_msg.angle_min + i * self.scan_msg.angle_increment for i in range(len(self.scan_msg.ranges))]
            for angle in angles:
                self.set_servo_angle(angle)
                time.sleep(0.5)  # Wait for the servo to reach the position
                distance = self.measure_distance()
                distances.append(distance)
            return distances
        except Exception as e:
            self.get_logger().error(f'Error measuring distances: {e}')
            return None

    def set_servo_angle(self, angle):
        duty_cycle = ((angle + math.pi / 2) / math.pi * 10 + 2.57 ) - math.pi/2 # Convert angle to duty cycle
        
        self.get_logger().info(f'Angle calc at {duty_cycle} cycle')

        self.servo.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f'Set servo angle to {math.degrees(angle)} degrees')

    def measure_distance(self):
        try:
            # Set Trigger to HIGH
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, False)

            start_time = time.time()
            stop_time = time.time()

            while GPIO.input(self.echo_pin) == 0:
                start_time = time.time()

            while GPIO.input(self.echo_pin) == 1:
                stop_time = time.time()

            time_elapsed = stop_time - start_time
            distance = (time_elapsed * 34300) / 2  # Speed of sound is 343 m/s

            distance = distance / 100.0 # in meters

            if distance < self.scan_msg.range_min:
                distance = self.scan_msg.range_min
            elif distance > self.scan_msg.range_max:
                distance = self.scan_msg.range_max

            self.get_logger().info(f'Distance calculated is {distance}')

            return distance
        except Exception as e:
            self.get_logger().error(f'Error measuring distance: {e}')
            return float('inf')  # Return infinite distance if measurement fails

    def kalman_filter(self, measurement):
        self.kf.predict()
        self.kf.update(measurement)
        return self.kf.x[0]

    def publish_scan(self, distances):
        self.scan_msg.ranges = distances
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.scan_msg)

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_sensor = UltrasonicSensor()
    # ultrasonic_sensor.set_servo_angle(-math.pi/10)
    try:
        rclpy.spin(ultrasonic_sensor)
    except KeyboardInterrupt:
        pass
    finally:
        ultrasonic_sensor.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
