''' 
This code implements the control algorithm for 
a single agent to perform the Bearing-only Target Localization 
and Circumnavigation (BoTLC) task under wind disturbance.

This code is intended to be used to produce experimental results
for the corresponding paper.

Author: Donglin Sui
Date:   14/05/2024
'''

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
def my_PDT_BoTLC_control():

    print("------------ RUNNING CONTROLLER -------------")
    agent_index = 0         # Crazyfly index, legacy from multi-agent implementation
                            # Don't change!
                            # This means that the first agent is viewed as the agent
                            # undertaking the WD BoTLC mission
    target_index = 1        # The second agent in Agents is the stationary target

    # --------------- Set up Clock --------------------------------------------
    t = time.time()       # get current time (in Unix time stamp)
    start_time = t        # record starting time (for plotting)
    time_interval = 0.01  # Importat: This is a MAGIC number that you should 
                          # NEVER TOUCH!

    z = qu.get_height(agent_index)    # get current altitude 
    z_des = qgv.FLY_HEIGHT   # get desired altitude

    # //////////////////////////////////////////////////////////////////////////////
    # //                         Plot Iniitialization                             //
    # //////////////////////////////////////////////////////////////////////////////
    # --------------- Initialize arrays to store data -------------------------
    # The following arrays are used to generate plots
    varrho_data = []        # Aux variable, defined as varrho_i = d_i(t) - d_i_hat(t)
    delta_data = []         # Aux variable, defined as  delta_i = d_i(t) - d_des
    x_tilde_norm_data = []
    Agent_y_data = []
    Agent_x_data = []
    Target_y_data = []
    Target_x_data = []

    # --------------- Initialize plot handles  --------------------------------
    myFig, axs = plt.subplots(2, 2)

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
    while time.time() - start_time < 1:
        qp.set_attitude(agent_index,
                        roll0,          # roll should be a float type
                        pitch0,         # pitch should be a float type 
                        yawrate0,       # yawrate should be a float type
                        int(thrust0))   # thrust should be an int type
                                        # The set_attitude() function 
                                        # sends the control signals to
                                        # the crazyflie
        time.sleep(time_interval)

    # //////////////////////////////////////////////////////////////////////////////
    # //                   PDT Target Estimator Initialization                    //
    # //////////////////////////////////////////////////////////////////////////////

    # ------------------ Control Gains ---------------
    alpha_1 = 0.5  
    T_c1 = 1

    # Initialize variables
    n = 50000  # Upper estimate for the number of time steps

    q = np.zeros((2, n))         # Regressor vector q
    P = np.zeros((2, 2, n))      # Matrix P history
    q_dot = np.zeros((2, n))     # Time derivative of q
    P_dot = np.zeros((2, 2, n))  # Time derivative of P
    xi = np.zeros((2, n))        # History of xi
    
    # Initialize pT_hat with a user-defined initial condition
    pT_hat_x0 = 0.5
    pT_hat_y0 = 0.75
    pT_hat_initial = np.array([pT_hat_x0, pT_hat_y0])  # User-defined initial condition
    pT_hat = np.zeros((2, n))            # Estimated state \hat{x}
    pT_hat[:, 0] = pT_hat_initial        # Set initial condition for \hat{x}

    pT_hat_dot = np.zeros((2, n))  # Time derivative of \hat{x}

    # Singular threshold for the determinant of P
    det_threshold = 1e-6  # Adjust this value as needed


    # //////////////////////////////////////////////////////////////////////////////
    # //               PDT Circumnavigation Controller Initialization             //
    # //////////////////////////////////////////////////////////////////////////////
    alpha_2 = 0.5  # Control gain, alpha_2 ∈ (0, 1]
    T_c2 = 2       # Predefined time constant, T_c2 > T_c1
    k_omega = 1    # Control gain, k_omega > 0
    # Desired circumnavigation scale
    d_des = 0.6

    # Initialize the target position history array
    p_T_x_0, p_T_y_0 = qu.get_2D_pos(target_index)

    # CVT
    # v_T_x = -0.02
    # v_T_y = -0.01
    p_T = np.zeros((2, n))  # Store the target's x and y positions
    # Set the initial position of the target
    p_T[0, 0] = p_T_x_0  # Initial x-coordinate
    p_T[1, 0] = p_T_y_0  # Initial y-coordinate

    # For convergence
    delta_eps = 0.025
    varrho_eps = 0.025

    varrho_Flag = 0
    delta_Flag = 0

    # takeoff flag
    Up_in_the_Sky = False

    # integral term for discrete-time SSE cancellation
    f_int = [0]

    # //////////////////////////////////////////////////////////////////////////////
    # //                             CSV Data Output                              //
    # //////////////////////////////////////////////////////////////////////////////
    # --------------- Open csv ------------------------------------------------
    cwd = os.path.realpath(os.path.dirname(__file__))   # the os.getcwd() does 
                                                        # not work in VS Code
    current_date = datetime.datetime.now()
    myFileName = 'quad_PDT_BoTLC_v3f_TVVT' + str(current_date.hour) + 'h_' + str(current_date.minute) + 'm_' + str(current_date.day) + '_' + str(current_date.month) + '_' + str(current_date.year)
    myFile = open(os.path.join(cwd, myFileName+'.csv'),"w+")
    csvWriter = csv.writer(myFile)

    # --------------- Record control gains and parameters ---------------------
    csvWriter.writerow(['k_omega' , 'd_des' , 'alpha_1', 'T_c1' , 'alpha_2' , 'T_c2' , 'p_T_x_0', 'p_T_y_0' ,  'z'])
    csvWriter.writerow([ k_omega  ,  d_des  ,  alpha_1  , T_c1  ,  alpha_2  ,  T_c2  ,  p_T_x_0 ,  p_T_y_0  ,   z ])

    # --------------- set up header for recorded data -------------------------
    csvWriter.writerow([
        't',                                                # time in secs
        'tStep',                                            # time step
        'p_a_x(t)', 'p_a_y(t)',                             # Agent's current position [p_a_x(t), p_a_y(t)]
        'p_T_x(t)', 'p_T_y(t)',                             # Target's current position 
        'pT_hat_x(t)', 'pT_hat_y(t)',                       # Estimated target position 
        'P_11(t)' , 'P_12(t)' , 'P_21(t)' , 'P_22(t)' ,     # regressor P(t)
        'q_1(t)' , 'q_2(t)' ,                               # regressor q(t)
        'xi_1(t)' ,  'xi_2(t)' ,                            # aux variable xi
        'varphi_x(t)' , 'varphi_y(t)' ,                     # unit bearing vector varphi(t)
        'u_x(t)' , 'u_y(t)' ,                               # control input
        'z'                                                 # current altitude
    ])                                                  

    
    #####################################################################################
    ########################## Main Loop ################################################
    #####################################################################################
    # integer time step
    tStep = 0
    while time.time() - start_time < qgv.RUN_TIME:
        # --------------- Landing Procedure -------------------------------------------
        # If spacebar is pressed on keyboard, then stop flying and continue to land
        if keyboard.is_pressed('space'):
            break

        # //////////////////////////////////////////////////////////////////////////////
        # //                              Controller                                  //
        # //////////////////////////////////////////////////////////////////////////////
        dt = time.time() - t    # time interval between the previous loop and the 
                                # current loop
        

        # --------------- Get Measurements ---------------------------------------------

        # Obtain agent's own position
        p_a_x, p_a_y = qu.get_2D_pos(agent_index)

        # Obtain the target location
        # p_T_x, p_T_y = qu.get_2D_pos(target_index)  
        p_T_x = p_T[0, tStep]
        p_T_y = p_T[1, tStep]

        # Get the actual agent-to-target distance
        d = math.dist([p_a_x,p_a_y],[p_T_x,p_T_y]) 

        # Calculate unit bearing vector from the agent to the target
        varphi_x = 1/d * ( - p_a_x + p_T_x )
        varphi_y = 1/d * ( - p_a_y + p_T_y )

        varphi = np.array([varphi_x, varphi_y])        # \varphi(t)

        # Calculate the orthogonal bearing vector bar_varphi
        # phi_iT_bar is phi_iT rotated counterclockwise by pi/2
        bar_varphi_x = - varphi_y
        bar_varphi_y =   varphi_x

        # Bar \varphi(t) vector
        bar_varphi = np.array([bar_varphi_x, bar_varphi_y])

        
        # //////////////////////////////////////////////////////////////////////////////
        # //                Calculate PDT Target Estimator Initialization             //
        # //////////////////////////////////////////////////////////////////////////////

        # Update P_dot (matrix)
        P_dot[:, :, tStep] = -P[:, :, tStep] + np.outer(bar_varphi, bar_varphi)

        # Update q_dot (vector)
        y_t = np.array([p_a_x, p_a_y])  # y(t)
        q_dot[:, tStep] = -q[:, tStep] + np.outer(bar_varphi, bar_varphi) @ y_t

        # Euler integration to update P and q
        P[:, :, tStep+1] = P[:, :, tStep] + P_dot[:, :, tStep] * dt
        q[:, tStep+1] = q[:, tStep] + q_dot[:, tStep] * dt

        # Check the determinant of P to handle singularity
        det_P = np.linalg.det(P[:, :, tStep])

        if det_P > det_threshold:
            # Calculate \xi(t) only if the determinant is large enough
            xi[:, tStep] = np.linalg.inv(P[:, :, tStep]) @ (P[:, :, tStep] @ pT_hat[:, tStep] - q[:, tStep])

            # Calculate \hat{x} dot
            norm_xi = np.linalg.norm(xi[:, tStep])
            if norm_xi > 1e-6:  # Avoid division by zero
                psi = norm_xi ** (-alpha_1) * xi[:, tStep]
            else:
                psi = np.zeros(2)

            pT_hat_dot[:, tStep] = -(1 / (alpha_1 * T_c1)) * np.exp(norm_xi ** alpha_1) * psi

        else:
            # If the determinant is too small, set \dot{\hat{x}} = 0
            xi[:, tStep] = np.zeros(2)  # Set \xi(t) to zero to reflect that we're not using P inverse
            pT_hat_dot[:, tStep] = np.zeros(2)

        # Euler integration to update \hat{x}
        pT_hat[:, tStep+1] = pT_hat[:, tStep] + pT_hat_dot[:, tStep] * dt

        pT_hat_x = pT_hat[0, tStep]  # Estimated x-coordinate
        pT_hat_y = pT_hat[1, tStep]  # Estimated y-coordinate

        d_hat = math.dist([p_a_x, p_a_y],[pT_hat_x,pT_hat_y])
        d_tilde = d_hat - d_des



        # --------------- Define auxiliary variables for plotting ----------------------
        varrho = d - d_hat
        delta = d - d_des
        varrho_data.append(varrho)
        delta_data.append(delta)

        x_tilde_norm = math.dist([p_T_x, p_T_y],[pT_hat_x,pT_hat_y])
        x_tilde_norm_data.append(x_tilde_norm)


        # --------------- Height Control Setup ---------------------------
        z = qu.get_height(agent_index)   # current altitude of the drone
        
        # get height error
        z_err = z - z_des     # error in position along z-axis
        z_err_tol = 0.1       # error tolerance

        # --------------- Sending Control Signal --------------------------
        if tStep >= 100:
            if Up_in_the_Sky: 

                # //////////////////////////////////////////////////////////////////////////////
                # //                Calculate PDT Circumnavigation Controller                 //
                # //////////////////////////////////////////////////////////////////////////////
                # Centripetal term: exponential and sig^(1 - alpha_2)
                # Calculate sig^(1 - alpha_2) for scalar d_tilde
                sig_term = np.sign(d_tilde) * np.abs(d_tilde) ** (1 - alpha_2)

                # Compute the first term of the control law
                centripetal_term = ((np.exp(np.abs(d_tilde) ** alpha_2) / (alpha_2 * T_c2)) * sig_term + f_int[tStep] ) * varphi

                # Tangential term: k_omega * bar_varphi
                tangential_term = k_omega * bar_varphi

                # Control input u(t) is the sum of the first and second terms
                u_a = centripetal_term + tangential_term

                # u_t is the control input at time t
                v_a_x = u_a[0]  # x-component of the control input
                v_a_y = u_a[1]  # y-component of the control input

                qp.set_2D_velocity(agent_index, v_a_x, v_a_y, z_des)

                # //////////////////////////////////////////////////////////////////////////////
                # //                Update target's position using the virtual velocity       //
                # //////////////////////////////////////////////////////////////////////////////
                # Calculate time t relative to the simulation start time
                elapsed_time = time.time() - start_time

                # Test 1
                # v_T_x = - 0.025 - 0.025 * np.exp(-0.05 * elapsed_time) * abs(np.sin(2 * np.pi * 0.03 * elapsed_time))
                # v_T_y = -0.05 * np.exp(-0.1 * elapsed_time) * abs(np.cos(2 * np.pi * 0.03 * elapsed_time))
                # p_T[0, tStep+1] = p_T[0, tStep] +  v_T_x * dt  # Update x position
                # p_T[1, tStep+1] = p_T[1, tStep] +  v_T_y * dt  # Update y position

                # Test 2
                # v_T_x = - 0.025 - 0.025 * np.exp(-0.05 * elapsed_time) * abs(np.sin(2 * np.pi * 0.03 * elapsed_time))
                # v_T_y = -0.1 * np.exp(-0.1 * elapsed_time) * abs(np.cos(2 * np.pi * 0.03 * elapsed_time))
                # p_T[0, tStep+1] = p_T[0, tStep] + 0.5 * v_T_x * dt  # Update x position
                # p_T[1, tStep+1] = p_T[1, tStep] + 0.5 * v_T_y * dt  # Update y position

                # Test 3_1 & 3_2
                # v_T_x = - 0.025 - 0.025 * np.exp(-0.05 * elapsed_time) * abs(np.sin(2 * np.pi * 0.03 * elapsed_time))
                # v_T_y = -0.5 * np.exp(-0.1 * elapsed_time) * abs(np.cos(2 * np.pi * 0.03 * elapsed_time))
                # p_T[0, tStep+1] = p_T[0, tStep] + 0.5 * v_T_x * dt  # Update x position
                # p_T[1, tStep+1] = p_T[1, tStep] + 0.5 * v_T_y * dt  # Update y position

                # Test 4
                # v_T_x = - 0.025 - 0.025 * np.exp(-0.05 * elapsed_time) * abs(np.sin(2 * np.pi * 0.03 * elapsed_time))
                # v_T_y = -0.3 * np.exp(-0.1 * elapsed_time) * abs(np.cos(2 * np.pi * 0.03 * elapsed_time))
                # p_T[0, tStep+1] = p_T[0, tStep] + 0.5 * v_T_x * dt  # Update x position
                # p_T[1, tStep+1] = p_T[1, tStep] + 0.5 * v_T_y * dt  # Update y position

                # Test 5
                # v_T_x = - 0.025 - 0.025 * np.exp(-0.05 * elapsed_time) * abs(np.sin(2 * np.pi * 0.03 * elapsed_time))
                # v_T_y = -0.15 * np.exp(-0.1 * elapsed_time) * abs(np.cos(2 * np.pi * 0.03 * elapsed_time))
                # p_T[0, tStep+1] = p_T[0, tStep] + 0.5 * v_T_x * dt  # Update x position
                # p_T[1, tStep+1] = p_T[1, tStep] + 0.5 * v_T_y * dt  # Update y position

                # Test 6
                v_T_x = - 0.025 - 0.025 * np.exp(-0.05 * elapsed_time) * abs(np.sin(2 * np.pi * 0.03 * elapsed_time))
                v_T_y = -0.2 * np.exp(-0.1 * elapsed_time) * abs(np.cos(2 * np.pi * 0.03 * elapsed_time))
                p_T[0, tStep+1] = p_T[0, tStep] + 0.5 * v_T_x * dt  # Update x position
                p_T[1, tStep+1] = p_T[1, tStep] + 0.5 * v_T_y * dt  # Update y position

            else:
                v_a_x = 0
                v_a_y = 0
                qp.set_2D_velocity(agent_index, v_a_x, v_a_y, z_des)

                print("Taking off ...")
                if abs(z_err) < z_err_tol:
                    Up_in_the_Sky = True
                    print("Cruising height reached")

                # //////////////////////////////////////////////////////////////////////////////
                # //                Update target's position using the virtual velocity       //
                # //////////////////////////////////////////////////////////////////////////////
                # If not, keep the position unchanged
                p_T[0, tStep+1] = p_T[0, tStep]
                p_T[1, tStep+1] = p_T[1, tStep]

        else:
            v_a_x = 0 
            v_a_y = 0

            # //////////////////////////////////////////////////////////////////////////////
            # //                Update target's position using the virtual velocity       //
            # //////////////////////////////////////////////////////////////////////////////
            # If not, keep the position unchanged
            p_T[0, tStep+1] = p_T[0, tStep]
            p_T[1, tStep+1] = p_T[1, tStep]

        # Check Convergence in target estimation
        if varrho < varrho_eps and varrho_Flag == 0:
            print('Target estimation error is sufficiently small')
            varrho_Flag = 1
        
        if delta < delta_eps and delta_Flag == 0:
            print('Tracking error is sufficiently small')
            delta_Flag = 1 

        #  --------------- Record and Update data for plots ----------------------------
        Agent_x_data.append(p_a_x)
        Agent_y_data.append(p_a_y)

        # for the discrete-time SSE elimination
        f_int.append(f_int[tStep]+d_tilde*dt)

        # Record values to csv file
        # Record data to the CSV file
        csvWriter.writerow([
            t - start_time,  # Time (t)
            tStep,                        # Current time step
            p_a_x, p_a_y,                 # Agent's current position [p_a_x(t), p_a_y(t)]
            p_T[0, tStep], p_T[1, tStep], # Target's position
            pT_hat[0, tStep],             # Estimated target position x (pT_hat_x(t))
            pT_hat[1, tStep],             # Estimated target position y (pT_hat_y(t))
            P[0, 0, tStep],               # P_11(t)
            P[0, 1, tStep],               # P_12(t)
            P[1, 0, tStep],               # P_21(t)
            P[1, 1, tStep],               # P_22(t)
            q[0, tStep],                  # q_1(t)
            q[1, tStep],                  # q_2(t)
            xi[0, tStep],                 # xi_1(t)
            xi[1, tStep],                 # xi_2(t)
            varphi_x, varphi_y,           # Current \varphi vector [varphi_x(t), varphi_y(t)]
            v_a_x, v_a_y,                 # Control input components [u_x(t), u_y(t)]
            z                             # Control gain or additional variable
        ])

        # update time & errors
        t = time.time()
        time.sleep(time_interval)

        tStep = tStep + 1
  
    print("------------ LANDING -------------")
    qp.land(agent_index)

    # --------------- Close csv -----------------------------------------------
    myFile.close()

   # --------------- Generate plots ------------------------------------------
    # Calculate total flight time
    end_time = time.time()
    time_elapsed = end_time - start_time

    # Get time between each interval (adjust if necessary)
    y_time = np.linspace(0, time_elapsed, tStep)  # Use actual number of time steps tStep

    # ---------------- Subplot 1: drone's traj -------------------------------------------------------

    # Extract estimated target's positions from pT_hat
    pT_hat_x = pT_hat[0, :tStep]  # Estimated x-coordinates over time
    pT_hat_y = pT_hat[1, :tStep]  # Estimated y-coordinates over time

    # Extract virtual target's position history from p_T
    p_T_x_history = p_T[0, :tStep]  # Target x-coordinate history
    p_T_y_history = p_T[1, :tStep]  # Target y-coordinate history


    # Combine all points into arrays for easy processing
    all_x = np.concatenate((Agent_x_data, pT_hat_x, p_T_x_history))
    all_y = np.concatenate((Agent_y_data, pT_hat_y, p_T_y_history))

    # Calculate distances from the last target position to each point
    p_T_x = p_T[0, tStep]  # Current x position of the target
    p_T_y = p_T[1, tStep]  # Current y position of the target

    # Calculate distances from (p_T_x, p_T_y) to each point
    distances = np.sqrt((all_x - p_T_x) ** 2 + (all_y - p_T_y) ** 2)

    # Get the maximum distance for scaling the plot
    max_diag_range = max(distances)

    # Calculate the padding for the plot limits
    padding = 0.2 * max_diag_range  

    # Set aspect ratio and axis limits for the trajectory plot
    axs[0, 0].set_aspect('equal', adjustable='box')
    new_x_min = p_T_x - max_diag_range - padding
    new_x_max = p_T_x + max_diag_range + padding
    axs[0, 0].set_xlim(new_x_min, new_x_max)
    new_y_min = p_T_y - max_diag_range - padding
    new_y_max = p_T_y + max_diag_range + padding
    axs[0, 0].set_ylim(new_y_min, new_y_max)

    # Plot drone's trajectory, estimated target's trajectory, and final target position
    axs[0, 0].plot(Agent_x_data, Agent_y_data, 'r-', markersize=4, linewidth=1, label='Drone')  # drone trajectory
    axs[0, 0].plot(pT_hat_x, pT_hat_y, 'k-', markersize=4, linewidth=1, label='Estimated Target')  # estimated target trajectory
    axs[0, 0].plot(p_T_x_history, p_T_y_history, 'b-', markersize=4, linewidth=1, label='Target')  # virtual target trajectory

    # Set plot titles, labels, and legends
    axs[0, 0].set_title('(a) Trajectory Plot')
    axs[0, 0].legend(loc='upper left')
    axs[0, 0].set_xlabel('x-axis (m)')
    axs[0, 0].set_ylabel('y-axis (m)')
    axs[0, 0].grid()

    # ---------------- subplot 2: target estimation error & tracking error --------------------------
    # Assuming varrho_data and delta_data are available for plotting the errors
    axs[0, 1].plot(y_time, varrho_data, 'r-', markersize=4, linewidth=1, label=r'$\varrho(t) = d(t) - \hat{d}(t)$') 
    axs[0, 1].plot(y_time, delta_data, 'b-', markersize=4, linewidth=1, label=r'$\delta(t) = d(t) - d^{*}$') 

    # Plot estimation & tracking errors
    axs[0, 1].legend(loc='upper right')
    axs[0, 1].set_title('(b) Estimation & Tracking Errors Plot')
    axs[0, 1].set_xlabel('time (sec)')
    axs[0, 1].set_ylabel('Error (m)')
    axs[0, 1].grid()

    # ---------------- subplot 3: target estimation error -------------------------------------------
    # Assuming x_tilde_norm_data is available to plot the norm of estimation error
    x_tilde_norm_0 = x_tilde_norm_data[0]  # Initial norm of the error
    axs[1, 0].axhline(y=x_tilde_norm_0, color='k', linestyle='-.', label=r'$||\tilde{x}(0)||$')  # Initial error line
    axs[1, 0].plot(y_time, x_tilde_norm_data, 'b-', markersize=4, linewidth=1, label=r'$||\tilde{x}(t)||$')  # Error norm over time

    # Set plot titles, labels, and legends
    axs[1, 0].set_title('(c) Target Estimation Error Plot')
    axs[1, 0].legend(loc='upper right')
    axs[1, 0].set_xlabel('time (sec)')
    axs[1, 0].set_ylabel('Error (m)')
    axs[1, 0].grid()

    # ---------------- subplot 4: integral terms -------------------------------------------
    f_int_time = np.linspace(0,time_elapsed,len(f_int))
    axs[1, 1].plot(f_int_time, f_int, 'b-',markersize=4,linewidth=1,label=r'$f_{int} = \int_0^t \tilde{d}(\tau) d\tau$') 
    axs[1, 1].legend(loc='upper right')
    axs[1, 1].set_title('(d) SSE integral term')
    axs[1, 1].set_xlabel('time (sec)')
    axs[1, 1].set_ylabel('Error (m)')
    axs[1, 1].grid()



    plt.show()

