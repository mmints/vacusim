'''
Author: Mark O. Mints (mmints@uni-koblenz.de)
'''

import rclpy
from math import pi
from geometry_msgs.msg import Twist
from vacusim_robot_interfaces.srv import Turn
from vacusim_robot_interfaces.srv import Drive
from vacusim_robot_interfaces.msg import Distance
from vacusim_robot_interfaces.msg import Bumper

# From default C-lang Controller
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

        # ROBOT INTERFACES

        # Motor Left
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__left_motor.setPosition(float('inf'))        
        self.__left_motor.setVelocity(0)

        # Motor Right
        self.__right_motor = self.__robot.getDevice('right wheel motor')        
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        # Bumper Left
        self.__bumper_left = self.__robot.getDevice('bumper_left')
        self.__bumper_left.enable(int(self.__robot.getBasicTimeStep()))

        # Bumper Right
        self.__bumper_right = self.__robot.getDevice('bumper_right')
        self.__bumper_right.enable(int(self.__robot.getBasicTimeStep()))
        
        # Distance Sensor Left
        self.__distance_left = self.__robot.getDevice('ds_left')
        self.__distance_left.enable(int(self.__robot.getBasicTimeStep()))

        # Distance Sensor Center
        self.__distance_center = self.__robot.getDevice('ds_center')
        self.__distance_center.enable(int(self.__robot.getBasicTimeStep()))

        # Distance Sensor Right
        self.__distance_right = self.__robot.getDevice('ds_right')
        self.__distance_right.enable(int(self.__robot.getBasicTimeStep()))

        # Wheel Position Sensor Left (not published)
        self.__position_sensor_left = self.__robot.getDevice('left wheel sensor')
        self.__position_sensor_left.enable(int(self.__robot.getBasicTimeStep()))

        # Wheel Position Sensor Right (not published)
        self.__position_sensor_right = self.__robot.getDevice('right wheel sensor')
        self.__position_sensor_right.enable(int(self.__robot.getBasicTimeStep()))

        rclpy.init(args=None)
        self.__node = rclpy.create_node('roomba_driver')

        # Command Velocity Subscription
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        # Bumper Publisher
        self.__bumper_left_publisher = self.__node.create_publisher(Bumper, 'bumper', 5)
        self__bumper_pub_timer = self.__node.create_timer(0.25, self.__bumper_callback)

        # Distance Sensor Publisher
        self.__distance_publisher = self.__node.create_publisher(Distance, 'distance', 5)
        self__bumper_pub_timer = self.__node.create_timer(0.125, self.__distance_callback)

        # Movement Services
        self.__turn_srv = self.__node.create_service(Turn, 'turn', self.__turn_callback)
        self.__drive_srv = self.__node.create_service(Drive, 'drive', self.__drive_callback)


    def __cmd_vel_callback(self, twist_msg):
        forward_speed = twist_msg.linear.x
        angular_speed = twist_msg.angular.z

        command_motor_left = round((forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / (2* WHEEL_RADIUS), 3)
        command_motor_right = round((forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / (2* WHEEL_RADIUS), 3)
        
        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
        self.__robot.step(int(self.__robot.getBasicTimeStep()))


    def __distance_callback(self):
        distance_msg = Distance()
        distance_msg.left = self.__distance_left.getValue()
        distance_msg.center = self.__distance_center.getValue()
        distance_msg.right = self.__distance_right.getValue()
        self.__distance_publisher.publish(distance_msg)


    def __bumper_callback(self):
        bumper_msg = Bumper()

        # Left Bumper
        if self.__bumper_left.getValue() > 0.0:
            bumper_msg.left = True
        else:
            bumper_msg.left = False

        # Right Bumper
        if self.__bumper_right.getValue() > 0.0:
            bumper_msg.right = True
        else:
            bumper_msg.right = False

        self.__bumper_left_publisher.publish(bumper_msg)


    # Turn Server Callback 
    def __turn_callback(self, request, response):    
        self.__node.get_logger().info('Turn for: ' + str(request.angle) + ' radian.')

        # Set Offset for Distance Calculation
        l_offset = self.__position_sensor_left.getValue()
        r_offset = self.__position_sensor_right.getValue()

        # Define Turning Direction
        neg = 0.0
        if request.angle < 0.0:
            neg = -1.0
        else:
            neg = 1.0

        # Set Speed to Hard Velocity
        self.__left_motor.setVelocity(-neg * QUATER_SPEED)
        self.__right_motor.setVelocity(neg * QUATER_SPEED)

        # Set Breaking Point for Final Slow Down
        remaining_angle = abs(request.angle)
        break_threshold = 0.025

        while True:
            self.__robot.step(int(self.__robot.getBasicTimeStep()))

            # Calculate Relative Wheel Position
            l = self.__position_sensor_left.getValue() - l_offset
            r = self.__position_sensor_right.getValue() - r_offset

            dl = l * WHEEL_RADIUS # distance covered by left wheel in meter
            dr = r * WHEEL_RADIUS # distance covered by left wheel in meter
            orientation = neg * (dr - dl) / AXLE_LENGTH # delta orientation in radian
            
            # Check if Turing Movement Should Be Slowed Down
            remaining_angle = abs(request.angle) - abs(orientation)
            if remaining_angle <= break_threshold:
                factor = abs(remaining_angle / break_threshold)
                if factor <= 0.001:
                    factor = 0.001
                self.__left_motor.setVelocity(factor * (-neg * EIGHTH_SPEED))
                self.__right_motor.setVelocity(factor * (neg * EIGHTH_SPEED))
            
            # Do While Python Hack
            if orientation >= (neg * request.angle):
                self.__robot.step(int(self.__robot.getBasicTimeStep()))
                break
            
        # Finalize Turning
        self.__stop()
        self.__robot.step(int(self.__robot.getBasicTimeStep()))
        self.__node.get_logger().info('Turning done! Turned for ' + str(request.angle) + ' radian.')
        response.done = True
        return response
    

    # Drive Server Callback 
    def __drive_callback(self, request, response):
        self.__node.get_logger().info('Drive for: ' + str(request.distance) + ' meter.')

        # Set Offset for Distance Calculation
        l_offset = self.__position_sensor_left.getValue() # We only need one wheel

        # Set Direction
        neg = 0.0
        if request.distance < 0.0:
            neg = -1.0
        else:
            neg = 1.0

        # Drive with Fixed Speed
        self.__left_motor.setVelocity(neg * HALF_SPEED)
        self.__right_motor.setVelocity(neg * HALF_SPEED)

        # Set Breaking Point for Slow Down
        break_threshold = 0.005
        remaining_distance = abs(request.distance)

        while True:
            self.__robot.step(int(self.__robot.getBasicTimeStep()))

            # Calculate Driven Distance
            l = self.__position_sensor_left.getValue() - l_offset            
            dl = l * WHEEL_RADIUS # distance covered by left wheel in meter
            
            # Slow Down Before Destination is Reached
            remaining_distance = abs(request.distance) - abs(dl)
            if remaining_distance <= break_threshold:
                factor = abs(remaining_distance / break_threshold)
                if factor <= 0.0001:
                    factor = 0.0001
                self.__left_motor.setVelocity(factor * (neg * QUATER_SPEED))
                self.__right_motor.setVelocity(factor * (neg * QUATER_SPEED))

            # Do While Python Hack
            if abs(dl) >= abs(request.distance):
                self.__robot.step(int(self.__robot.getBasicTimeStep()))
                break

        # Finalize Driving
        self.__stop()
        self.__robot.step(int(self.__robot.getBasicTimeStep()))
        self.__node.get_logger().info('Driving Done! Driven for: ' + str(request.distance) + ' meters.')
        response.done = True
        return response

    # Stop all Movement
    def __stop(self):
        self.__left_motor.setVelocity(0.0)
        self.__right_motor.setVelocity(0.0)
        self.__robot.step(int(self.__robot.getBasicTimeStep()))

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)


