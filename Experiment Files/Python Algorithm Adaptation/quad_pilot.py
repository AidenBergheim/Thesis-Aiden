"""
This file contains wrappers for the crazyflie library commands. The
functions take in the indexes of the crazyflies so that if a drone is switched
the only thing that needs to change in the code is the list of agents in qgv.
Functions for taking off and landing are also specified.

Author: originally written by Juri Hemmi and heavily adopted by Donglin Sui
"""


import time
import quad_global_variables as qgv
import quad_utilities as qu


def _get_cf_from_index(cf_index):
    return qgv.cfs[qgv.AGENTS[cf_index]]


def set_thrust(cf_index, thrust:int):
    _get_cf_from_index(cf_index).commander.send_setpoint(0, 0, 0, thrust)


def set_attitude(cf_index, roll, pitch, yaw, thrust:int):
    _get_cf_from_index(cf_index).commander.send_setpoint(roll, pitch, yaw, thrust)


def set_velocity(cf_index, vx, vy, vz):
    _get_cf_from_index(cf_index).commander.send_velocity_world_setpoint(vx, vy, vz, 0)


# will run at 0.5
def set_2D_velocity(cf_index, vx, vy, z):
    _get_cf_from_index(cf_index).commander.send_hover_setpoint(vx, vy, 0, z)


def set_position(cf_index, x, y, z):
    _get_cf_from_index(cf_index).commander.send_position_setpoint(x, y, z, 0)


# will run at 0.5
def set_2D_position(cf_index, x, y):
    _get_cf_from_index(cf_index).commander.send_position_setpoint(x, y, 0.5, 0)


def stop_motors(cf_index):
    _get_cf_from_index(cf_index).commander.send_stop_setpoint()


# This function waits until the takeoff is complete (2s)
# It takes in a vector of indexes for qgv.AGENTS
def takeoff(cf_index, height=qgv.FLY_HEIGHT):
    # obtain the position of the crazyflies
    ps = {}
    ps[cf_index] = qu.get_2D_pos(cf_index)

    # Linear rise from ground
    rise_time = 1.5
    t = 0
    while t <= rise_time:
        t += 0.1
        set_position(cf_index,
                        ps[cf_index][0],
                        ps[cf_index][1],
                        height * t / rise_time)
        time.sleep(0.1)

    # stabilise at height
    t = 0
    while t <= 0.5: # wait for 0.5 sec
        t += 0.1
        set_position(cf_index,
                        ps[cf_index][0],
                        ps[cf_index][1],
                        height)
        time.sleep(0.1)


# It takes in a vector of indexes for qgv.AGENTS
# and an optional argument containing the desired landing positions.
# If the pos_2D_dict is not given then the cfs will land at their
# current location
# The motors are shut off after landing (2s)
def land(cf_index, height=qgv.FLY_HEIGHT, pos_2D_dict=None):
    # obtain the position of the crazyflies if not given
    ps = {}
    if pos_2D_dict == None:
        ps[cf_index] = qu.get_2D_pos(cf_index)
    else:
        ps = pos_2D_dict

    # Make sure cfs are at the specified positions
    t = 0
    while t <= 0.2: # wait for 0.2 sec
        t += 0.1
        set_position(cf_index,
                        ps[cf_index][0],
                        ps[cf_index][1],
                        height)
        time.sleep(0.1)

    # Linear descent to ground
    descent_time = 1.5
    t = 0
    while t <= descent_time:
        t += 0.1
        set_position(cf_index,
                        ps[cf_index][0],
                        ps[cf_index][1],
                        height - height * t / descent_time)
        time.sleep(0.1)

    # # Make sure cfs are at the specified positions
    # t = 0
    # while t <= 0.3: # wait for 0.3 sec
    #     t += 0.1
    #     for cf in cf_indexes:
    #         set_position(cf,
    #                      ps[cf][0],
    #                      ps[cf][1],
    #                      0)
    #     time.sleep(0.1)

    stop_motors(cf_index)
    time.sleep(0.1)
