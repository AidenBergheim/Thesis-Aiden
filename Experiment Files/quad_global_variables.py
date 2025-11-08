"""
This file contains user defined variables as well as variables that the system
uses at runtime.
"""

import quad_utilities as qu
from cflib.utils import uri_helper
from math import sqrt 
import numpy as np

pos_stream = []

AGENTS = [6,0]      # 1st agent is moving, 2nd agent is target
FLY_HEIGHT = 0.5 # default fly height for the drones

RUN_TIME = 60 # Doesn't include takeoff and landing time
  
USE_FULL_POSE = False

URIs = {0: uri_helper.uri_from_env(default=("radio://0/90/2M/E7E7E7E7E0")),
        1: uri_helper.uri_from_env(default=("radio://0/90/2M/E7E7E7E7E1")),
        2: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E2")),
        3: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E3")),
        4: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E4")),
        5: uri_helper.uri_from_env(default=("radio://0/80/2M/E7E7E7E7E5")), 
        6: uri_helper.uri_from_env(default=("radio://0/100/2M/E7E7E7E7E6")), 
        7: uri_helper.uri_from_env(default=("radio://0/100/2M/E7E7E7E7E7")),
        8: uri_helper.uri_from_env(default=("radio://0/100/2M/E7E7E7E7E8")), 
        9: uri_helper.uri_from_env(default=("radio://0/100/2M/E7E7E7E7E9"))}

# Positions
poses = {}
for agent in AGENTS:
    poses[agent] = {"position": [],
                    "rotation": []}

# Batteries
logconf = {}
for agent in AGENTS:
    logconf[agent] = {None}

battery_Log = {}
for agent in AGENTS:
    battery_Log[agent] = {"battery_level": [],
                          "battery_voltage": []}

# crazyflie class and synchronous crazyflie class
cfs = {}
for agent in AGENTS:
    cfs[agent] = []

scfs = {}
for agent in AGENTS:
    scfs[agent] = []

cfs_connected = False



     