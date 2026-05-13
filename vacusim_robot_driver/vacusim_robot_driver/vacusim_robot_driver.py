'''
Author: Mark O. Mints (mmints@uni-koblenz.de)
'''

import rclpy
import time
from math import pi
from geometry_msgs.msg import Twist
from vacusim_robot_interfaces.srv import Turn
from vacusim_robot_interfaces.srv import Drive
from vacusim_robot_interfaces.msg import Distance
from vacusim_robot_interfaces.msg import Bumper

AXLE_LENGTH = 0.2978
HALF_DISTANCE_BETWEEN_WHEELS = AXLE_LENGTH / 2.0
WHEEL_RADIUS = 0.031
WHEEL_PEREOHERY = 2.0 * WHEEL_RADIUS * pi

MAX_SPEED = 16
MIN_SPEED = -16
NULL_SPEED = 0

HALF_SPEED = 8
QUATER_SPEED = 4
EIGHTH_SPEED = 2


class RobotDriver():

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__bumper_left = self.__robot.getDevice('bumper_left')
        self.__bumper_left.enable(int(self.__robot.getBasicTimeStep()))

        self.__bumper_right = self.__robot.getDevice('bumper_right')
        self.__bumper_right.enable(int(self.__robot.getBasicTimeStep()))

        self.__distance_left = self.__robot.getDevice('ds_left')
        self.__distance_left.enable(int(self.__robot.getBasicTimeStep()))

        self.__distance_center = self.__robot.getDevice('ds_center')
        self.__distance_center.enable(int(self.__robot.getBasicTimeStep()))

        self.__distance_right = self.__robot.getDevice('ds_right')
        self.__distance_right.enable(int(self.__robot.getBasicTimeStep()))

        self.__position_sensor_left = self.__robot.getDevice('left wheel sensor')
        self.__position_sensor_left.enable(int(self.__robot.getBasicTimeStep()))

        self.__position_sensor_right = self.__robot.getDevice('right wheel sensor')
        self.__position_sensor_right.enable(int(self.__robot.getBasicTimeStep()))

        rclpy.init(args=None)
        self.__node = rclpy.create_node('roomba_driver')

        self.__node.create_subscription(
            Twist,
            'cmd_vel',
            self.__cmd_vel_callback,
            1
        )

        self.__bumper_left_publisher = self.__node.create_publisher(Bumper, 'bumper', 10)
        self.__bumper_timer = self.__node.create_timer(0.005, self.__bumper_callback)

        self.__distance_publisher = self.__node.create_publisher(Distance, 'distance', 5)
        self.__distance_timer = self.__node.create_timer(0.125, self.__distance_callback)

        self.__turn_srv = self.__node.create_service(Turn, 'turn', self.__turn_callback)
        self.__drive_srv = self.__node.create_service(Drive, 'drive', self.__drive_callback)

        self.__last_cmd_vel_time = time.time()
        self.__linear_x = 0.0
        self.__angular_z = 0.0
        self.__cmd_vel_timeout = 0.2

    def __cmd_vel_callback(self, twist_msg):
        self.__linear_x = twist_msg.linear.x
        self.__angular_z = twist_msg.angular.z
        self.__last_cmd_vel_time = time.time()

    def __distance_callback(self):
        distance_msg = Distance()
        distance_msg.left = self.__distance_left.getValue()
        distance_msg.center = self.__distance_center.getValue()
        distance_msg.right = self.__distance_right.getValue()
        self.__distance_publisher.publish(distance_msg)

    def __bumper_callback(self):
        bumper_msg = Bumper()

        bumper_msg.left = self.__bumper_left.getValue() > 0.0
        bumper_msg.right = self.__bumper_right.getValue() > 0.0

        self.__bumper_left_publisher.publish(bumper_msg)

    def __turn_callback(self, request, response):
        self.__node.get_logger().info('Turn for: ' + str(request.angle) + ' radian.')

        l_offset = self.__position_sensor_left.getValue()
        r_offset = self.__position_sensor_right.getValue()

        neg = -1.0 if request.angle < 0.0 else 1.0

        self.__left_motor.setVelocity(-neg * QUATER_SPEED)
        self.__right_motor.setVelocity(neg * QUATER_SPEED)

        break_threshold = 0.025

        while True:
            self.__robot.step(int(self.__robot.getBasicTimeStep()))

            l = self.__position_sensor_left.getValue() - l_offset
            r = self.__position_sensor_right.getValue() - r_offset

            dl = l * WHEEL_RADIUS
            dr = r * WHEEL_RADIUS
            orientation = neg * (dr - dl) / AXLE_LENGTH

            remaining_angle = abs(request.angle) - abs(orientation)

            if remaining_angle <= break_threshold:
                factor = max(0.001, abs(remaining_angle / break_threshold))
                self.__left_motor.setVelocity(factor * (-neg * EIGHTH_SPEED))
                self.__right_motor.setVelocity(factor * (neg * EIGHTH_SPEED))

            if orientation >= (neg * request.angle):
                break

        self.__stop()
        response.done = True
        return response

    def __drive_callback(self, request, response):
        self.__node.get_logger().info('Drive for: ' + str(request.distance) + ' meter.')

        l_offset = self.__position_sensor_left.getValue()

        neg = -1.0 if request.distance < 0.0 else 1.0

        self.__left_motor.setVelocity(neg * HALF_SPEED)
        self.__right_motor.setVelocity(neg * HALF_SPEED)

        break_threshold = 0.005

        while True:
            self.__robot.step(int(self.__robot.getBasicTimeStep()))

            l = self.__position_sensor_left.getValue() - l_offset
            dl = l * WHEEL_RADIUS

            remaining_distance = abs(request.distance) - abs(dl)

            if remaining_distance <= break_threshold:
                factor = max(0.0001, abs(remaining_distance / break_threshold))
                self.__left_motor.setVelocity(factor * (neg * MAX_SPEED))
                self.__right_motor.setVelocity(factor * (neg * MAX_SPEED))

            if abs(dl) >= abs(request.distance):
                break

        self.__stop()
        response.done = True
        return response

    def __stop(self):
        self.__left_motor.setVelocity(0.0)
        self.__right_motor.setVelocity(0.0)
        self.__robot.step(int(self.__robot.getBasicTimeStep()))

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        now = time.time()

        if now - self.__last_cmd_vel_time > self.__cmd_vel_timeout:
            forward_speed = 0.0
            angular_speed = 0.0
        else:
            forward_speed = self.__linear_x
            angular_speed = self.__angular_z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / (2 * WHEEL_RADIUS)
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / (2 * WHEEL_RADIUS)

        command_motor_left = max(MIN_SPEED, min(MAX_SPEED, command_motor_left))
        command_motor_right = max(MIN_SPEED, min(MAX_SPEED, command_motor_right))

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)