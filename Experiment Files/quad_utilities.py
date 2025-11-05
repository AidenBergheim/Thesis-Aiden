"""
This file contains all utility functions that are related to
 - establishing and shutting down connections
 - streaming positions, velocities, etc
 - initializing various components
This file does not contain any function that directly commands
the motion of the quadcopter. For functions that pilot the 
drone, go to quad_pilot.py file.
"""
# //////////////////////////////////////////////////////////////////////////////
# //                           Required Modules                               //
# //////////////////////////////////////////////////////////////////////////////
# Do not modify the following modules!
# Do not remove any unused module! 

# --------------- Python Libraries --------------------------------------------
import logging
import time
import datetime
import sys
from threading import Event
import csv
import os
import numpy as np
import matplotlib.pyplot as plt  
import math
import keyboard 

# --------------- Crazyflie Libraries -----------------------------------------
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import commander

# --------------- Customary Modules -------------------------------------------
import quad_global_variables as qgv

# --------------- OptiTrack Modules -------------------------------------------
from optitrack_NatNetClient import NatNetClient


# //////////////////////////////////////////////////////////////////////////////
# //                            Magic Numbers                                 //
# //////////////////////////////////////////////////////////////////////////////

TIME_BUFFER = 0.1   # in seconds
MOTIVE_FREQ = 120   # Update frequency of Motive system in Hz


# //////////////////////////////////////////////////////////////////////////////
# //                      Set up position streaming                           //
# //////////////////////////////////////////////////////////////////////////////
# The following functions are originally written by Juri Hemmi

# This function sends the full pose to a given crazyflie
# The rotation is in quaternions
def send_pose_to_cf(cf, pos, rot):
    #print(".", end='')
    cf.extpos.send_extpose(pos[0], pos[1], pos[2], 
                           rot[0], rot[1], rot[2], rot[3])

# This function sends only the position to the given crazyflie
def send_pos_to_cf(cf, pos):
    #print(".", end='')
    cf.extpos.send_extpos(pos[0], pos[1], pos[2])

# This function gets passed to the NatNetClient and is called every
# time that a rigid bodies position is received. 
# The pose is then sent to the crazyflie. 
prev_time, prev_position = [None, None]
def receive_rigid_body_frame(new_id, position, rotation ):
    #print( "Received frame for rigid body", new_id )
    global prev_time, prev_position
    for agent in qgv.AGENTS:
        cf = qgv.cfs[agent]
        
        if new_id == agent:
            x, y, z = position
            q1, q2, q3, q4 = rotation

            # ===================================================
            #                 Velocity Calculation
            # ===================================================
            # The velocity of the crazyflie is indirectly 
            # obtained from position measurements using 
            #           v = (d(t2) - d(t1)) / (t2 - t1)
            # The dt = t2 - t1 should equal the update rate of 
            # Motive which is 120 Hz and is fixed.
            # TODO: check if it makes a difference by replacing 
            # the real dt = t-prev_time with dt = 1/MOTIVE_FREQ
            #
            # NOTE: Velocity/acceleration should not be 
            # calculated inside the control loop as the update
            # rate there differs from that in the NatNet client
            #
            # Contributed by Chris Dutkiewicz
        
            t = time.time()
            if (prev_time == None or prev_position == None) :
                xdot, ydot, zdot = [0.0, 0.0, 0.0]
            else:
                if (t - prev_time == 0):
                     xdot, ydot, zdot = [0.0, 0.0, 0.0]
                
                elif (t - prev_time) < 0.075:
                    xdot = (x - prev_position[0]) / (1/MOTIVE_FREQ)
                    ydot = (y - prev_position[1]) / (1/MOTIVE_FREQ)
                    zdot = (z - prev_position[2]) / (1/MOTIVE_FREQ)

                else:
                    xdot = (x - prev_position[0]) / (t - prev_time)
                    ydot = (y - prev_position[1]) / (t - prev_time)
                    zdot = (z - prev_position[2]) / (t - prev_time)

            prev_time = t
            prev_position = [x, y, z]

            qgv.poses[agent] = {"position": [x, y, z],
                            "rotation": [q1, q2, q3, q4],
                            "velocity": [xdot, ydot, zdot]}
            # print(".", end="")
            # print(qgv.poses)

            if qgv.cfs_connected:
                if qgv.USE_FULL_POSE:
                    send_pose_to_cf(cf, position, rotation)
                else:
                    send_pos_to_cf(cf, position)

def receive_new_frame(data_dict):
    pass

# This function sets up the position streaming 
def setup_pos_stream():
    qgv.pos_stream = NatNetClient()
    # These functions will be called in the background whenever the relevant
    # frame is recieved from motive.
    qgv.pos_stream.new_frame_listener = receive_new_frame
    qgv.pos_stream.rigid_body_listener = receive_rigid_body_frame

    if not qgv.pos_stream.run():
        print("Could not get Rigid Body positions")
        exit(1)

# This is a helper function to get the x-y (2D) position of a crazyflie
def get_2D_pos(cf_index):
    return [qgv.poses[qgv.AGENTS[cf_index]]["position"][0], 
            qgv.poses[qgv.AGENTS[cf_index]]["position"][1]]

# This is a helper function to get the height of a crazyflie
def get_height(cf_index):
    return qgv.poses[qgv.AGENTS[cf_index]]["position"][2]

# This is a helper function to get rotation values
# Contributed by Chris Dutkiewicz
def get_rotation(cf_index):
    return qgv.poses[qgv.AGENTS[cf_index]]["rotation"]

# This is a helper function to get velocity values
# Contributed by Chris Dutkiewicz
def get_velocities(cf_index):
    return qgv.poses[qgv.AGENTS[cf_index]]["velocity"]


# //////////////////////////////////////////////////////////////////////////////
# //                         Quadcopter Initializer                           //
# //////////////////////////////////////////////////////////////////////////////
# The following functions are adopted and adapted from Juri Hemmi

# This function set up the link with crazyflies and initializes all 
# system parameters
def quad_initializer():
    
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    print('=========== Initializing ===========',end='\n')
    
    print('|| Initializing radios ...')
    for agent in qgv.AGENTS:
        totalSteps = 4
        print_ProgressBar(0, totalSteps, prefix='| Agent '+ str(agent), suffix='[Initializing...]')
        time.sleep(TIME_BUFFER)
        print_ProgressBar(1, totalSteps, prefix='| Agent '+ str(agent), suffix='[Matching URI...]')

        uri = uri_helper.uri_from_env(default=(qgv.URIs[agent]))
        time.sleep(TIME_BUFFER)

        scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
        time.sleep(TIME_BUFFER)

        scf.open_link()
        time.sleep(TIME_BUFFER)

        qgv.cfs[agent] = scf.cf
        qgv.scfs[agent] = scf
        time.sleep(TIME_BUFFER)
        print_ProgressBar(2, totalSteps, prefix='| Agent '+ str(agent), suffix='[Checking Params]')

        scf.wait_for_params()
        time.sleep(TIME_BUFFER)

        time.sleep(TIME_BUFFER)

        qgv.cfs[agent].param.set_value('stabilizer.estimator', '2')
        qgv.cfs[agent].param.set_value('locSrv.extQuatStdDev', 0.6)
        qgv.cfs[agent].param.set_value('commander.enHighLevel', '1')

    
        time.sleep(TIME_BUFFER)
        print_ProgressBar(3, totalSteps, prefix='| Agent '+ str(agent), suffix='[Connecting.....]')


        print_ProgressBar(4, totalSteps, prefix='| Agent '+ str(agent), suffix='[Radio Setup Completed]')
    

    qgv.cfs_connected = True
    print('|| Initializing estimators ...')

    # The below two for-loops CANNOT be combined!
    for agent in qgv.AGENTS:
        time.sleep(TIME_BUFFER)
        reset_estimator(qgv.scfs[agent])
        time.sleep(TIME_BUFFER)

    totalSteps = 2
    for agent in qgv.AGENTS:
        print_ProgressBar(0, totalSteps, prefix='| Agent '+ str(agent), suffix='[Resetting Est...]')
        time.sleep(TIME_BUFFER)
        print_ProgressBar(1, totalSteps, prefix='| Agent '+ str(agent), suffix='[Waiting Est.....]')
        wait_for_position_estimator(qgv.scfs[agent])
        time.sleep(TIME_BUFFER)
        print_ProgressBar(2, totalSteps, prefix='| Agent '+ str(agent), suffix='[Estimator Setup Complete]')

    print('============ Initialization Completed! ============',end='\n')


# //////////////////////////////////////////////////////////////////////////////
# //                       Quadcopter Shutting Down                           //
# //////////////////////////////////////////////////////////////////////////////

# This function closes the link between the Crazyradio PA and the crazyflie
def quad_shutDown():
    
    print("Closing link and shutting down.")

    for agent in qgv.AGENTS:
        qgv.scfs[agent].close_link()
    
    time.sleep(TIME_BUFFER)

    qgv.pos_stream.shutdown()

# //////////////////////////////////////////////////////////////////////////////
# //                    On-Board Sensor Configuration                         //
# //////////////////////////////////////////////////////////////////////////////

# This function saves log data to global variables so that although the sensor
# block is not synchronous, the drone still gets synchronous reading of the 
# battery information
def log_bat_callback(timestamp,data,logconf,agent):
    qgv.battery_Log[agent]["battery_level"] = data['pm.batteryLevel']
    qgv.battery_Log[agent]["battery_voltage"] = data['pm.vbat']

    # Uncomment the below lines to debug
    # print("From callback:")
    # print("agent = "+str(agent))
    # print("batter_Log[agent][battery_voltage]=")
    # print(qgv.battery_Log[agent]["battery_voltage"])

# This function initializes the sensor and log entry.
# The TIME_BUFFER is required, do not remove
# This function cannot be combined with the other function, 
# "sensor_Configuration" because the log block cannot be 
# created directly in a loop (for reasons I do not know)
def sensorLog_initializer():
    for agent in qgv.AGENTS:
        sensor_Configuration(agent)
        time.sleep(TIME_BUFFER)

# This function configures the log block for a single drone
def sensor_Configuration(agent):
    qgv.logconf[agent] = LogConfig(name='Battery',period_in_ms=10)
    qgv.logconf[agent].add_variable('pm.batteryLevel','uint8_t')
    qgv.logconf[agent].add_variable('pm.vbat','float')
    qgv.cfs[agent].log.add_config(qgv.logconf[agent])
    time.sleep(TIME_BUFFER)
    if qgv.logconf[agent].valid:
        # To access the log entry in each drone, it seems that the only way to do it is to
        # setup an asynchronous log block for each drone. However, since the default 
        # add_callback function in cflib does not allow us to pass on the identity of a 
        # drone, we need to use the python "lambda" function so that we can call the
        # custom log_callback function with extra argument
        qgv.logconf[agent].data_received_cb.add_callback(lambda timestamp, data, logconf: log_bat_callback(timestamp,data,logconf,agent))
        qgv.logconf[agent].start()
    else:
        print("Could not setup log configuration for battery information after connection!")

# This is a helper function to get the almost-synchronous
# battery information
def get_battery_info(cf_index):
    bat_lvl = qgv.battery_Log[qgv.AGENTS[cf_index]]["battery_level"]
    bat_volt = qgv.battery_Log[qgv.AGENTS[cf_index]]["battery_voltage"]

    # Uncomment the below lines to debug
    print(f"Battery level: {bat_lvl}% ({bat_volt}V)")

    return [bat_lvl, bat_volt]
    
# This function closed the log block
def sensorLog_shutDown():
    for agent in qgv.AGENTS:
        qgv.logconf[agent].stop()
        time.sleep(TIME_BUFFER)


# //////////////////////////////////////////////////////////////////////////////
# //                       Other Utility Functions                            //
# //////////////////////////////////////////////////////////////////////////////

# This function prints and updates an iteration progress bar in the terminal
# Taken from stackoverflow.com/questions/3173320/...
# .../text-progress-bar-in-terminal-with-block-characters/13685020
def print_ProgressBar(iteration, total, prefix='', suffix='', decimals = 1, length = 100, fill = '\u2588', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration       - Required  :   current iteration (Int)
        total           - Required  :   total iteration (Int)
        prefix          - Optional  :   prefix string (Str)
        suffix          - Optional  :   suffix string (Str)
        decimals        - Optional  :   positive number of decimals in percent complete (Int)
        length          - Optional  :   bar fill character (Str)
        printEnd        - Optional  :   end character (e.g. "\r", "\r\n)"(Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100* (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total:
        print()



# The following functions are taken from
# github.com/bitcraze/crazyflie-lib-python/blob/master/examples/...
# .../autonomy/autonomousSequence.py
# with minor modifications

# This function waits until the crazyfile has a good estimate of its position.
def wait_for_position_estimator(scf):
    # print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

# This function resets the position estimator of a given crazyflie and
# then waits until the position is found.
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(TIME_BUFFER)
    cf.param.set_value('kalman.resetEstimation', '0')

