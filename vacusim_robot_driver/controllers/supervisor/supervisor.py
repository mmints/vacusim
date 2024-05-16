import sys
import numpy as np

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

while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly

    translation = trans_field.getSFVec3f()
    rotation = rot_field.getSFRotation()

    tile_cord_x = int(dim[0] * (translation[0] + (dim[0]*dim[2]) / 2) / (dim[0]*dim[2]))
    tile_cord_y = int(dim[1] * (- translation[1] + (dim[1]*dim[2]) / 2) / (dim[1]*dim[2]))
    tile_pose = np.array([tile_cord_x, tile_cord_y, rotation[3]])

    raw_cord_x = (translation[0] + (dim[0]*dim[2]) / 2) / (dim[0]*dim[2])
    raw_cord_y = (- translation[1] + (dim[1]*dim[2]) / 2) / (dim[1]*dim[2])
    raw_pose = np.array([raw_cord_x, raw_cord_y, rotation[3]])

    np.save('tile_pose.npy', tile_pose)
    np.save('raw_pose.npy', raw_pose)