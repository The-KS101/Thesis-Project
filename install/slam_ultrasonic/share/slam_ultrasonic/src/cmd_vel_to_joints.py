#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

class CmdVelToJoints(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joints')
        self.declare_parameter('wheel_separation', 0.145)
        self.declare_parameter('wheel_radius', 0.03)

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Failed to initialize pigpio')
            rclpy.shutdown()
            return

        # Define motor control pins
        self.L_IN1 = 22
        self.L_IN2 = 23
        self.L_PWM1 = 1
        self.R_IN1 = 24
        self.R_IN2 = 25
        self.R_PWM1 = 12
        self.R_IN3 = 26
        self.R_IN4 = 27
        self.R_PWM2 = 13


        

        # Set up motor control pins
        self.pi.set_mode(self.L_IN1, pigpio.OUTPUT)
        self.pi.set_mode(self.L_IN2, pigpio.OUTPUT)
        self.pi.set_mode(self.L_PWM1, pigpio.OUTPUT)
        self.pi.set_mode(self.R_IN1, pigpio.OUTPUT)
        self.pi.set_mode(self.R_IN2, pigpio.OUTPUT)
        self.pi.set_mode(self.R_PWM1, pigpio.OUTPUT)
        self.pi.set_mode(self.R_IN3, pigpio.OUTPUT)
        self.pi.set_mode(self.R_IN4, pigpio.OUTPUT)
        self.pi.set_mode(self.R_PWM2, pigpio.OUTPUT)
        
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z


        wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        left_velocity = (linear_velocity - angular_velocity * wheel_separation / 2.0) / wheel_radius
        right_velocity = (linear_velocity + angular_velocity * wheel_separation / 2.0) / wheel_radius

        self.control_motors(left_velocity, right_velocity)

    def control_motors(self, left_velocity, right_velocity):
        max_vel = max(abs(left_velocity), abs(right_velocity))
        if max_vel > 1.0:
            left_velocity /= max_vel
            right_velocity /= max_vel

        left_speed = int(left_velocity * 100)
        right_speed = int(right_velocity * 85)

        print("LEFT SPEED", left_speed)
        print("RIGHT SPEED", right_speed)
        # Control left motor
        if left_speed >= 0:
            self.pi.write(self.L_IN1, 1)
            self.pi.write(self.L_IN2, 0)

        else:
            self.pi.write(self.L_IN1, 0)
            self.pi.write(self.L_IN2, 1)

        self.pi.set_PWM_dutycycle(self.L_PWM1, abs(left_speed))

        # Control right motor
        if right_speed >= 0:
            self.pi.write(self.R_IN1, 1)
            self.pi.write(self.R_IN2, 0)
            self.pi.write(self.R_IN3, 0)
            self.pi.write(self.R_IN4, 1)
        else:
            self.pi.write(self.R_IN1, 0)
            self.pi.write(self.R_IN2, 1)
            self.pi.write(self.R_IN3, 1)
            self.pi.write(self.R_IN4, 0)
            

        self.pi.set_PWM_dutycycle(self.R_PWM1, abs(right_speed))
        self.pi.set_PWM_dutycycle(self.R_PWM2, abs(right_speed))



    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

    def cleanup(self):
        self.pi.write(self.L_IN1, 0)
        self.pi.write(self.L_IN2, 0)
        self.pi.write(self.L_PWM1, 0)
        self.pi.write(self.R_IN1, 0)
        self.pi.write(self.R_IN2, 0)
        self.pi.write(self.R_PWM1, 0)
        self.pi.write(self.R_IN3, 0)
        self.pi.write(self.R_IN4, 0)
        self.pi.write(self.R_PWM2, 0)
        self.pi.stop()
        self.get_logger().info('GPIO cleanup completed successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import pigpio

# class CmdVelToJoints(Node):
#     def __init__(self):
#         super().__init__('cmd_vel_to_joints')
#         self.declare_parameter('wheel_separation', 0.145)
#         self.declare_parameter('wheel_radius', 0.03)
        
#         self.cmd_vel_subscriber = self.create_subscription(
#             Twist,
#             'cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )

#         # Initialize pigpio
#         self.pi = pigpio.pi()
#         if not self.pi.connected:
#             self.get_logger().error('Failed to initialize pigpio')
#             rclpy.shutdown()
#             return

#         # Define motor control pins
#         self.L_IN1 = 20
#         self.L_IN2 = 21
#         self.L_PWM1 = 1
#         self.R_IN1 = 24
#         self.R_IN2 = 25
#         self.R_PWM1 = 12

#         # Set up motor control pins
#         self.pi.set_mode(self.L_IN1, pigpio.OUTPUT)
#         self.pi.set_mode(self.L_IN2, pigpio.OUTPUT)
#         self.pi.set_mode(self.L_PWM1, pigpio.OUTPUT)
#         self.pi.set_mode(self.R_IN1, pigpio.OUTPUT)
#         self.pi.set_mode(self.R_IN2, pigpio.OUTPUT)
#         self.pi.set_mode(self.R_PWM1, pigpio.OUTPUT)

#     def cmd_vel_callback(self, msg):
#         linear_velocity = msg.linear.x
#         angular_velocity = msg.angular.z

#         wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
#         wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

#         left_velocity = (linear_velocity - angular_velocity * wheel_separation / 2.0) / wheel_radius
#         right_velocity = (linear_velocity + angular_velocity * wheel_separation / 2.0) / wheel_radius

#         left_velocity = left_velocity * wheel_radius # in m/s
#         right_velocity = right_velocity * wheel_radius
#         self.control_motors(left_velocity, right_velocity)

#     def control_motors(self, left_velocity, right_velocity, v_max=0.628):
#         # Scale velocities directly
#         left_speed = int((left_velocity / v_max) * 255)
#         right_speed = int((right_velocity / v_max) * 255)

#         # Ensure the speeds are within the 0-255 range
#         left_speed = max(min(left_speed, 255), 0)
#         right_speed = max(min(right_speed, 255), 0)

#         # Control left motor
#         if left_velocity >= 0:
#             self.pi.write(self.L_IN1, 1)
#             self.pi.write(self.L_IN2, 0)
#         else:
#             self.pi.write(self.L_IN1, 0)
#             self.pi.write(self.L_IN2, 1)
#         self.pi.set_PWM_dutycycle(self.L_PWM1, abs(left_speed))

#         # Control right motor
#         if right_velocity >= 0:
#             self.pi.write(self.R_IN1, 1)
#             self.pi.write(self.R_IN2, 0)
#         else:
#             self.pi.write(self.R_IN1, 0)
#             self.pi.write(self.R_IN2, 1)
#         self.pi.set_PWM_dutycycle(self.R_PWM1, abs(right_speed))

#     def destroy_node(self):
#         self.cleanup()
#         super().destroy_node()

#     def cleanup(self):
#         self.pi.write(self.L_IN1, 0)
#         self.pi.write(self.L_IN2, 0)
#         self.pi.write(self.L_PWM1, 0)
#         self.pi.write(self.R_IN1, 0)
#         self.pi.write(self.R_IN2, 0)
#         self.pi.write(self.R_PWM1, 0)
#         self.pi.stop()
#         self.get_logger().info('GPIO cleanup completed successfully.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = CmdVelToJoints()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
