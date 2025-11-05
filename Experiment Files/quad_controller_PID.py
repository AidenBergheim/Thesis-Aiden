# Author: Donglin Sui

# --------------- Required Modules --------------------------------------------
# Do not remove any module even if it is not used! These modules will be called
# in the imported files.
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

import quad_global_variables as qgv
import quad_pilot as qp
import quad_utilities as qu

import csv
import os
import numpy as np
import matplotlib.pyplot as plt  
import math
import keyboard 

# --------------- Main Function -----------------------------------------------
def my_height_control():

    print("------------ RUNNING CONTROLLER -------------")
    cf_index = 0            # Crazyfly index, legacy from multi-agent implementation
                            # Don't change!

    # Useful constants
    drone_mass = 1.5        # mass of the BigQuad drone
    gravity    = 9.81       # gravitational constant
    N2ctu      = 169895     # converting Newton to ctu
    ctu2N      = 1/169895   # converting ctu to Newton

    # scaling factor for N2ctu
    magic_scaling_number = 0.01

    # --------------- Set up Clock --------------------------------------------
    t = time.time()       # get current time (in Unix time stamp)
    start_time = t        # record starting time (for plotting)
    run_time = 60         # the duration in secs of the flying mission
    time_interval = 0.01  # Importat: This is a MAGIC number that you should 
                          # NEVER TOUCH!

    z = qu.get_height(cf_index)    # get current altitude 
    z_des = qgv.FLY_HEIGHT   # get desired altitude
    e_z_int = 0             # integrated error along z-axis at t = 0
    prev_e_z = z - z_des    # position error along z-axis at t = 0

    # The following variables are used in the PD control for 2D planar motion
    prev_e_y = 0            # position error along y-axis at t = 0
    prev_e_x = 0            # position error along x-axis at t = 0

    yawrate = 0

    # //////////////////////////////////////////////////////////////////////////////
    # //                         Plot Iniitialization                             //
    # //////////////////////////////////////////////////////////////////////////////
    # --------------- Initialize arrays to store data -------------------------
    # The following arrays are used to generate plots
    e_z_data = []      # e_data     stores position errors (m) in z-direction
    e_z_dot_data = []  # e_dot_data stores velocity errors (m/s) in z-direction 
    z_data = []        # z_data     stores the altitude (m) of the crazyflie
    z_des_data = []    # z_des_data stores the desired altitude (m)
    y_data = []
    x_data = []

    # --------------- Initialize plot handles  --------------------------------
    myFig, axs = plt.subplots(1, 2)

    # --------------- Unlock Crazyflie FW Safety Lock ------------------------- 
    # record start time
    start_time = time.time()

    # Upon first running, the setpoints fed to the drone must be 0, 
    # otherwise, all forthcoming setpoints will be ignored by 
    # the crazyflie firmware (FM) because there is a safety lock 
    # in the FW such that a zero setpoint must first be sent to unlock it.
    roll0 = 0
    pitch0 = 0
    yawrate0 = 0
    thrust0 = 0
    while time.time() - start_time < 5:
        qp.set_attitude(cf_index,
                        roll0,          # roll should be a float type
                        pitch0,         # pitch should be a float type 
                        yawrate0,       # yawrate should be a float type
                        int(thrust0))   # thrust should be an int type
                                        # The set_attitude() function 
                                        # sends the control signals to
                                        # the crazyflie
        time.sleep(time_interval)



    # --------------- Open csv ------------------------------------------------
    cwd = os.path.realpath(os.path.dirname(__file__))   # the os.getcwd() does 
                                                        # not work in VS Code
    current_date = datetime.datetime.now()
    myFileName = 'BigQuad_PID_' + str(current_date.hour) + 'h_' + str(current_date.minute) + 'm_' + str(current_date.day) + '_' + str(current_date.month) + '_' + str(current_date.year)
    myFile = open(os.path.join(cwd, myFileName+'.csv'),"w+")
    csvWriter = csv.writer(myFile)
    csvWriter.writerow(['time','thrust','z(t)','e_z_dot(t)'])

    ########################## Main Loop ################################################
    cf_index = 0

    while time.time() - start_time < qgv.RUN_TIME:
        # --------------- Landing Procedure -------------------------------------------
        # If spacebar is pressed on keyboard, then stop flying and continue to land
        if keyboard.is_pressed('space'):
            break

        # //////////////////////////////////////////////////////////////////////////////
        # //                              Controller                                  //
        # //////////////////////////////////////////////////////////////////////////////
        dt = time.time() - t    # time interval between the previous loop and the 
                                # current loop]
        z = qu.get_height(cf_index)   # current altitude of the drone
        
        # Implement your control law below
        e_z = z - z_des     # error in position along z-axis
        e_z_int += dt * e_z # integrated error in position along z-axis

        # error in velocity along z-axis
        if dt == 0:
            e_z_dot = (e_z - prev_e_z) / time_interval
        else:
            e_z_dot = (e_z - prev_e_z) / dt

        # Gain 3_0 (good)
        k_p = 6.5       # proportional gain
        k_i = 5.0       # integral gain 
        k_d = 3.25      # derivative gain

        # Gain 3_1
        # k_p = 5.5       # proportional gain
        # k_i = 5.0       # integral gain 
        # k_d = 3.25      # derivative gain

        # # Gain 3_2 (not good!)
        # k_p = 3.5       # proportional gain
        # k_i = 3       # integral gain 
        # k_d = 1.5      # derivative gain

        z_ddot = 0      # assume to be zero
        P = k_p * e_z
        I = k_i * e_z_int
        D = k_d * e_z_dot

        # Control law
        # Note that the final control signal needs to be converted
        # to Crazyflie-Thrust-Unit (ctu) by multipling the signal
        # in SI unit (which is Newton here) with N2ctu
        u1 =  magic_scaling_number * drone_mass * (z_ddot - P - I - D + gravity) * N2ctu

        # --------------- Thrust Saturation -------------------------------------------
        # Do NOT modify
        if u1 < 0:
            print('negative thrust', u1)
            u1 = 0
            print('new value of thrust', u1)
        elif u1 > 0xFFFF:
            print('saturating thrust', u1)
            u1 = 0xFFFF
            print('new value of thrust', u1)

        
        # --------------- Planar (x-y) PD Control -------------------------------------
        x, y = qu.get_2D_pos(0)
        #        P                D
        roll =  10*(y)    + 200*(y - prev_e_y)
        pitch = 10*(-x)   - 200*(x - prev_e_x)

        # Saturate at 10deg
        if abs(roll) > 10:
            print('Saturating roll to 10 deg.')
            roll = 10 * np.sign(roll)
        if abs(pitch) > 10:
            print('Saturating pitch to 10 deg.')
            pitch = 10 * np.sign(pitch)

        
        # Send control signals to drone
        qp.set_attitude(cf_index, roll, pitch, yawrate, int(u1))

        # update time & errors
        t = time.time()
        prev_e_z = e_z
        prev_e_y = y
        prev_e_x = x
        time.sleep(time_interval)

        #  --------------- Update data for plots -------------------------------------
        e_z_data.append(e_z)
        e_z_dot_data.append(e_z_dot)
        z_data.append(z)
        z_des_data.append(z_des)
        x_data.append(x)
        y_data.append(y)

        # Record values to csv file
        csvWriter.writerow([t,u1,z,e_z_dot])
 
    print("------------ LANDING -------------")
    qp.land(0)

    # --------------- Close csv -----------------------------------------------
    myFile.close()

    # --------------- Generate plots ------------------------------------------
    # Calculate total flight time
    end_time = time.time()
    time_elapsed = end_time - start_time

    # get time between each interval
    y_time = np.linspace(0,time_elapsed,len(e_z_data))

    # Subplot 1: error variables
    axs[0].plot(y_time, e_z_data, 'g-', 
                markersize=4, linewidth=1, 
                label='e_z(t)')  # plot error signal
    axs[0].set_title('(a) Error Plot')
    axs[0].set_ylim(-1,2)
    axs[0].legend(loc='upper left')
    axs[0].set_xlabel('time (sec)')
    axs[0].set_ylabel('Position Error (m)')
    axs[0].grid()

    ax1_sub_1 = axs[0].twinx()
    ax1_sub_1.plot(y_time, e_z_dot_data, 'b-', 
                   markersize=4, linewidth=1, 
                   label='e_z_dot(t)')  # plot error diff signal
    ax1_sub_1.set_ylim(-1,2)
    ax1_sub_1.legend(loc='upper right')
    ax1_sub_1.set_ylabel('Velocity Error (m/s)')

    # Subplot 2: altitude against time
    axs[1].plot(y_time, z_data, 'r-',
                markersize=4,linewidth=1,
                label='z(t)')
    axs[1].plot(y_time, z_des_data, 'k-',
                markersize=4,linewidth=1,
                label='z_des')
    axs[1].set_ylim(-1,2)
    axs[1].legend(loc='upper left')
    axs[1].set_title('(b) Position Plot')
    axs[1].set_xlabel('time (sec)')
    axs[1].set_ylabel('Altitude (m)')
    axs[1].grid()

    ax1_sub_2 = axs[1].twinx()
    ax1_sub_2.plot(y_time, x_data, 'b-',
                   markersize=4,linewidth=1,
                   label='x(t)')
    ax1_sub_2.plot(y_time, y_data, 'g-',
                   markersize=4,linewidth=1,
                   label='y(t)')
    ax1_sub_2.set_ylim(-1,2)
    ax1_sub_2.set_ylabel('Position (m)')
    ax1_sub_2.legend(loc='upper right')

    plt.show()

