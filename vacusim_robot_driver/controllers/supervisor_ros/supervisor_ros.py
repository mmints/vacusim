#!/usr/bin/env python3.10

'''
Author: Mark O. Mints (mmints@uni-koblenz.de)
'''

import rclpy
from geometry_msgs.msg import Pose2D

from controller import Supervisor

TIME_STEP = 32

supervisor = Supervisor()

robot_node = supervisor.getFromDef("CUSTOM_ROOMBA")
if robot_node is None:
    sys.stderr.write("No DEF CUSTOM_ROOMBA node found in the current world file\n")
    sys.exit(1)

supervisor_node = supervisor.getFromDef("SUPERVISOR")
if supervisor_node is None:
    sys.stderr.write("No DEF SUPERVISOR node found in the current world file\n")
    sys.exit(1)

trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")

supervisor_trans_field = supervisor_node.getField("translation")
dim = supervisor_trans_field.getSFVec3f()


def main():
    rclpy.init()

    node = rclpy.create_node('supervisor_pose_publisher')
    tile_publisher = node.create_publisher(Pose2D, '/pose/tile', 10)
    raw_publisher = node.create_publisher(Pose2D, '/pose/raw', 10)

    tile_pose_msg = Pose2D()
    raw_pose_msg = Pose2D()

    while supervisor.step(TIME_STEP) != -1 and rclpy.ok():

        translation = trans_field.getSFVec3f()
        rotation = rot_field.getSFRotation()

        tile_cord_x = (translation[0] + (dim[0]*dim[2]) / 2) / dim[2]
        tile_cord_y = (- translation[1] + (dim[1]*dim[2]) / 2) / dim[2]
        raw_cord_x = (translation[0] + (dim[0]*dim[2]) / 2)
        raw_cord_y = (- translation[1] + (dim[1]*dim[2]) / 2)

        tile_pose_msg.x = tile_cord_x
        tile_pose_msg.y = tile_cord_y
        tile_pose_msg.theta = rotation[3]

        raw_pose_msg.x = raw_cord_x
        raw_pose_msg.y = raw_cord_y
        raw_pose_msg.theta = rotation[3]

        tile_publisher.publish(tile_pose_msg)
        raw_publisher.publish(raw_pose_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
