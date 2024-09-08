#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import RPi.GPIO as GPIO
import time
import math
from filterpy.kalman import KalmanFilter
import numpy as np

class UltrasonicSensor(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')
        self.publisher_ = self.create_publisher(Twist, 'diff_cont/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust timer as needed
        self.safeDistance = 0.4
        self.dangerDistance = 0.2
        self.min_dist = 0.02
        self.max_dist = 4

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

    def timer_callback(self):
        self.avoid_obstacles()

    def avoid_obstacles(self):
        self.set_servo_angle(0)
        right = math.pi/4
        left = -math.pi/4
        rightPublish = -1.0
        leftPublish = 1.0
        
        try:
            while True:
                distance = self.measure_distance()
                self.get_logger().info(f'Distance is {distance} m')
                print("hold")
                if distance >= self.safeDistance:
                    time.sleep(1)
                    self.publish_twist(0.0, True)
                elif distance >= self.dangerDistance:
                    self.set_servo_angle(right)
                    time.sleep(2)
                    print("Looking right")
                    newDistance = self.measure_distance()
                    if newDistance >= self.safeDistance:
                        time.sleep(2)
                        print("Go right")
                        self.publish_twist(rightPublish, True)
                        self.set_servo_angle(0)
                        time.sleep(2)
                        # Correct position by goig back left same distance                    
                    else:
                        self.set_servo_angle(left)
                        time.sleep(2)
                        print("Looking left")

                        newDistance = self.measure_distance()
                        if newDistance >= self.safeDistance:
                            time.sleep(2)
                            self.publish_twist(leftPublish, True)
                            print("Go left")
                            self.set_servo_angle(0)
                            time.sleep(2)
                            # Correct position by going back right same distance
                        else:
                            self.set_servo_angle(0)
                            time.sleep(2)
                            self.publish_twist(0.0, False)
                else: 
                    # Send reverse command and left/right/uturn
                    time.sleep(2)
                    self.publish_twist(0.0, False)
                right *= -1
                left *= -1
                leftPublish *= -1
                rightPublish *= -1


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

            if distance < self.min_dist:
                distance = self.min_dist
            elif distance > self.max_dist:
                distance = self.max_dist

            self.get_logger().info(f'Distance calculated is {distance}')

            return distance
        except Exception as e:
            self.get_logger().error(f'Error measuring distance: {e}')
            return float('inf')  # Return infinite distance if measurement fails

    def publish_twist(self, angle, forward):
        drive_msg = Twist()
        if forward and angle == 0.0:
            drive_msg.linear.x = 0.5
        elif not forward and angle == 0.0:
            drive_msg.linear.x = -0.5
        self.get_logger().info(f'Publishing message {drive_msg} at angle {angle}')

        drive_msg.angular.z = angle
        self.get_logger().info(f'Publishing message {drive_msg}')
        self.publisher_.publish(drive_msg)

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
