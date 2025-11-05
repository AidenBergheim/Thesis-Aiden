"""
This is the main file that can be used to control Crazyflie drones in 
combination with the OptiTrack-Motive motion capture system. 

The main function also sets up the NatNetClient to receive the streamed 
rigid body data and pass the positions to each Crazyflie.

The Motive software must be kept openned and streaming the Crazyflies' 
(rigid bodies) positions while running this python file.

System variables are mostly stored in quad_global_variables.py

The control law should be implemented in a separate file, for example,
'quad_controller_someFancyControlLaw.py', and then called in this file.

The provided sample controller, 'quad_controller_PID.py', controls a 
single crazyflie quadcopter to hover at a prescribed height while not 
drifting on a horizontal plane. However, the provided Python files
can be easily modified to accommdate multiple Crazyflies and serve as
a experimental platform for multi-agent-system-related projects.

Authors:    originally written by Juri Hemmi,
            and heavily adopted by Donglin Sui
"""

# --------------- Required Modules --------------------------------------------
# Do not modify the following modules!
# Do not remove any unused module! 
import logging
import time
import datetime
import sys
from threading import Event

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.crazyflie import commander

import quad_utilities as qu
import quad_global_variables as qgv
import quad_pilot as qp
import quad_controller_ASMC as qc

import csv
import os
import numpy as np
import matplotlib.pyplot as plt  
import math
import keyboard 

from optitrack_NatNetClient import NatNetClient


# --------------- Main Function -----------------------------------------------
def main():

    # initialize the system 
    qu.setup_pos_stream() # Obtain position reading from OptiTrack

    qu.quad_initializer() # Setup communication links with the crazyflies

    qu.sensorLog_initializer() # Creates sensor log block

    # input your controller here
    qc.my_height_control()  # Controls a single crazyflie to localize and
                            # circumnavigate around another stationary 
                            # crazyflie

    # shutting down
    qu.sensorLog_shutDown() # Destroy sensor log block
    qu.quad_shutDown()      # Close communication linkss with the crazyflies

 
if __name__ == "__main__":
    main()
