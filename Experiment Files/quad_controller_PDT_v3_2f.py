''' 
This code implements the control algorithm for 
a single agent to perform the Bearing-only Target Localization 
and Circumnavigation (BoTLC) task within predefined-time for multiple targets.

This code is intended to be used to produce experimental results
for the corresponding paper.

Author: Aiden Bergheim
Date:   06/11/2025
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


def minEnclosingEllipseStat(points):
    """
    Calculates the minimum enclosing statistical ellipse (based on covariance)
    for a set of 2D points.
    
    Args:
        points (np.array): An [n x 2] array of n points.
        
    Returns:
        tuple: (center, semi_axes, rotation_matrix)
            center (np.array): [2x1] center of the ellipse.
            semi_axes (np.array): [2x1] lengths of semi-axes (a, b).
            rotation_matrix (np.array): [2x2] rotation matrix.
    """
    n, m = points.shape
    
    # --- Handle inputs and edge cases ---
    unique_points = np.unique(points, axis=0)
    if unique_points.shape[0] <= m or np.linalg.matrix_rank(points - np.mean(points, axis=0)) < m:
        # Not enough unique points to calculate a full-rank covariance ellipse.
        center = np.mean(points, axis=0).reshape(-1, 1)
        semi_axes = np.zeros(m)
        rotation_matrix = np.eye(m)
        return center, semi_axes, rotation_matrix
        
    # --- Step 1: Find the Center (Mean/Centroid) ---
    center = np.mean(points, axis=0).reshape(-1, 1) # [2 x 1] column vector

    # --- Step 2: Calculate Covariance Matrix ---
    # rowvar=False because our input is [n x 2] (observations in rows)
    covariance_matrix = np.cov(points, rowvar=False) # [2 x 2] matrix
    
    # --- Step 3: Get Eigenvectors and Eigenvalues ---
    # Eigenvectors (rotation_matrix) are columns
    eigenvalues, rotation_matrix = np.linalg.eig(covariance_matrix)
    
    # Ensure eigenvalues are positive
    if np.any(eigenvalues <= 0):
        center = np.mean(points, axis=0).reshape(-1, 1)
        semi_axes = np.zeros(m)
        rotation_matrix = np.eye(m)
        return center, semi_axes, rotation_matrix
        
    # --- Step 4: Scale Ellipse to Enclose All Points ---
    centered_points = points - center.T # [n x 2] matrix
    inv_covariance = np.linalg.inv(covariance_matrix)
    
    # Calculate squared Mahalanobis distance for all points
    mahala_dist_sq = np.sum((centered_points @ inv_covariance) * centered_points, axis=1)
    
    s_sq_max = np.max(mahala_dist_sq)
    s = np.sqrt(s_sq_max)
    
    # --- Step 5: Calculate Semi-Axis Lengths ---
    semi_axes = s * np.sqrt(eigenvalues)
    
    # Sort axes from largest to smallest
    sort_idx = np.argsort(semi_axes)[::-1]
    semi_axes = semi_axes[sort_idx]
    rotation_matrix = rotation_matrix[:, sort_idx]
    
    return center, semi_axes, rotation_matrix



# --------------- Main Function -----------------------------------------------
def my_PDT_BoTLC_control():

    print("------------ RUNNING CONTROLLER -------------")
    agent_index = 0         # Crazyfly index, legacy from multi-agent implementation
                            # Don't change!
                            # This means that the first agent is viewed as the agent
                            # undertaking the WD BoTLC mission
    target_index = np.array([1, 2, 3, 4])        # The second agent in Agents is the stationary target

    num_targets = len(target_index)

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
    x_tilde_norm_data = [[] for _ in range(num_targets)]
    Agent_y_data = []
    Agent_x_data = []
    Target_y_data = [[] for _ in range(num_targets)]
    Target_x_data = [[] for _ in range(num_targets)] 

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

    q = np.zeros((2, n, num_targets))         # Regressor vector q
    P = np.zeros((2, 2, n, num_targets))      # Matrix P history
    q_dot = np.zeros((2, n, num_targets))     # Time derivative of q
    P_dot = np.zeros((2, 2, n, num_targets))  # Time derivative of P
    xi = np.zeros((2, n, num_targets))        # History of xi

    r = np.zeros(num_targets)
    varphi_x = np.zeros(num_targets)
    varphi_y = np.zeros(num_targets)
    varphi = np.zeros((2, num_targets))

    bar_varphi_x = np.zeros(num_targets)
    bar_varphi_y = np.zeros(num_targets)
    bar_varphi = np.zeros((2, num_targets))

    
    # Initialize pT_hat with a user-defined initial condition
    pT_hat_x0 = 1/4 * np.array([7.7, 7.812, 7.7, 7.703])
    pT_hat_y0 = 1/4 * np.array([0, 0.234, 0, 0.042])
    pT_hat_x = np.zeros(num_targets)
    pT_hat_y = np.zeros(num_targets)
    d_des = np.zeros(n)

    pT_hat_initial = np.array([pT_hat_x0, pT_hat_y0])  # User-defined initial condition
    pT_hat = np.zeros((2, n, num_targets))            # Estimated state \hat{x}
    pT_hat[:, 0, :] = pT_hat_initial        # Set initial condition for \hat{x}

    pT_hat_dot = np.zeros((2, n, num_targets))  # Time derivative of \hat{x}

    # Singular threshold for the determinant of P
    det_threshold = 1e-6  # Adjust this value as needed


    # //////////////////////////////////////////////////////////////////////////////
    # //               PDT Circumnavigation Controller Initialization             //
    # //////////////////////////////////////////////////////////////////////////////
    alpha_2 = 0.5  # Control gain, alpha_2 ∈ (0, 1]
    T_c2 = 2       # Predefined time constant, T_c2 > T_c1
    k_omega = 1    # Control gain, k_omega > 0
    
    


    # Initialize the target position history array
    p_T_x_0 = np.zeros(num_targets)
    p_T_y_0 = np.zeros(num_targets)

    for i, target_idx in enumerate(target_index):
        p_T_x_0[i], p_T_y_0[i] = qu.get_2D_pos(target_idx)

    # CVT
    # v_T_x = -0.02
    # v_T_y = -0.01
    p_T = np.zeros((2, n, num_targets))  # Store the target's x and y positions
    # Set the initial position of the target
    p_T[0, 0, :] = p_T_x_0  # Initial x-coordinates for all targets
    p_T[1, 0, :] = p_T_y_0  # Initial y-coordinates for all targets

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
    myFileName = 'PDT_BOTLC_MultiTarget' + str(current_date.hour) + 'h_' + str(current_date.minute) + 'm_' + str(current_date.day) + '_' + str(current_date.month) + '_' + str(current_date.year)
    myFile = open(os.path.join(cwd, myFileName+'.csv'),"w+")
    csvWriter = csv.writer(myFile)

    # --------------- Record control gains and parameters ---------------------
    # Build header with all target positions
    param_header = ['k_omega', 'd_des', 'alpha_1', 'T_c1', 'alpha_2', 'T_c2']
    param_values = [k_omega, d_des, alpha_1, T_c1, alpha_2, T_c2]

    # Add initial positions for each target
    for i in range(num_targets):
        param_header.extend([f'p_T{i+1}_x_0', f'p_T{i+1}_y_0'])
        param_values.extend([p_T_x_0[i], p_T_y_0[i]])

    param_header.append('z')
    param_values.append(z)

    csvWriter.writerow(param_header)
    csvWriter.writerow(param_values)

    # --------------- set up header for recorded data -------------------------
    data_header = [
        't',                                                # time in secs
        'tStep',                                            # time step
        'p_a_x(t)', 'p_a_y(t)',                             # Agent's current position
    ]

    # Add columns for each target
    for i in range(num_targets):
        data_header.extend([
            f'p_T{i+1}_x(t)', f'p_T{i+1}_y(t)',                 # Target i's current position
            f'pT_hat{i+1}_x(t)', f'pT_hat{i+1}_y(t)',           # Estimated target i position
            f'P{i+1}_11(t)', f'P{i+1}_12(t)', f'P{i+1}_21(t)', f'P{i+1}_22(t)',  # regressor P(t) for target i
            f'q{i+1}_1(t)', f'q{i+1}_2(t)',                     # regressor q(t) for target i
            f'xi{i+1}_1(t)', f'xi{i+1}_2(t)',                   # aux variable xi for target i
            f'varphi{i+1}_x(t)', f'varphi{i+1}_y(t)',           # unit bearing vector varphi(t) for target i
        ])

    # Add remaining columns that are shared across all targets
    data_header.extend([
        'u_x(t)', 'u_y(t)',                                  # control input
        'z'                                                  # current altitude
    ])

    csvWriter.writerow(data_header)
    
    #####################################################################################
    ########################## Main Loop ################################################
    #####################################################################################
    # integer time step
    tStep = 0
    control_start_time = time.time()
    

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
        p_T_x = p_T[0, tStep, :]
        p_T_y = p_T[1, tStep, :]
        pC_x = np.mean(p_T_x)
        pC_y = np.mean(p_T_y)

        # Get the actual agent-to-target distances
        for i in range(num_targets):
            r[i] = math.dist([p_a_x, p_a_y], [p_T_x[i], p_T_y[i]]) 
            
            # Calculate unit bearing vector from the agent to the target
            varphi_x[i] = 1/r[i] * ( - p_a_x + p_T_x[i] )
            varphi_y[i] = 1/r[i] * ( - p_a_y + p_T_y[i] )

            varphi[:, i] = np.array([varphi_x[i], varphi_y[i]])  # \varphi(t)

            # Calculate the orthogonal bearing vector bar_varphi
            bar_varphi_x[i] = - varphi_y[i]
            bar_varphi_y[i] =   varphi_x[i]

            # Bar \varphi(t) vector
            bar_varphi[:, i] = np.array([bar_varphi_x[i], bar_varphi_y[i]])


        

        psi_x = np.mean(varphi_x)
        psi_y = np.mean(varphi_y)
        
        # Normalize the psi vector
        psi_norm = np.sqrt(psi_x**2 + psi_y**2)
        psi_x = psi_x / psi_norm
        psi_y = psi_y / psi_norm

        psi_vec = np.array([psi_x, psi_y])

        bar_psi_x = -psi_y
        bar_psi_y = psi_x

        bar_psi_vec = np.array([bar_psi_x, bar_psi_y])


        # //////////////////////////////////////////////////////////////////////////////
        # //                Calculate PDT Target Estimator Initialization             //
        # //////////////////////////////////////////////////////////////////////////////

        y_t = np.array([p_a_x, p_a_y])  # y(t)

        elapsed_time = time.time() - start_time
        
        for i in range(num_targets):
            # Update P_dot (matrix)
            if (elapsed_time) <= T_c1:
                P_dot[:, :, tStep, i] = -P[:, :, tStep, i] + np.outer(bar_varphi[:, i], bar_varphi[:, i])

                # Update q_dot (vector)
                q_dot[:, tStep, i] = -q[:, tStep, i] + np.outer(bar_varphi[:, i], bar_varphi[:, i]) @ y_t

                # Euler integration to update P and q
                P[:, :, tStep+1, i] = P[:, :, tStep, i] + P_dot[:, :, tStep, i] * dt
                q[:, tStep+1, i] = q[:, tStep, i] + q_dot[:, tStep, i] * dt

                # Check the determinant of P to handle singularity
                det_P = np.linalg.det(P[:, :, tStep, i])

                if det_P > det_threshold:
                    # Calculate \xi(t) only if the determinant is large enough
                    xi[:, tStep, i] = np.linalg.inv(P[:, :, tStep, i]) @ (P[:, :, tStep, i] @ pT_hat[:, tStep, i] - q[:, tStep, i])

                    # Calculate \hat{x} dot
                    norm_xi = np.linalg.norm(xi[:, tStep, i])
                    if norm_xi > 1e-6:  # Avoid division by zero
                        psi = norm_xi ** (-alpha_1) * xi[:, tStep, i]
                    else:
                        psi = np.zeros(2)

                    pT_hat_dot[:, tStep, i] = -(1 / (alpha_1 * T_c1)) * np.exp(norm_xi ** alpha_1) * psi

                else:
                    # If the determinant is too small, set \dot{\hat{x}} = 0
                    xi[:, tStep, i] = np.zeros(2)  # Set \xi(t) to zero to reflect that we're not using P inverse
                    pT_hat_dot[:, tStep, i] = np.zeros(2)

                # Euler integration to update \hat{x}
                pT_hat[:, tStep+1, i] = pT_hat[:, tStep, i] + pT_hat_dot[:, tStep, i] * dt

                pT_hat_x[i] = pT_hat[0, tStep, i]  # Estimated x-coordinate
                pT_hat_y[i] = pT_hat[1, tStep, i]  # Estimated y-coordinate
            else:
                q[:, tStep+1, i] = q[:, tStep, i]
                P[:, :, tStep+1, i] = P[:, :, tStep, i]
                pT_hat[:, tStep+1, i] = pT_hat[:, tStep, i]
                pT_hat_x[i] = pT_hat[0, tStep, i]  # Estimated x-coordinate
                pT_hat_y[i] = pT_hat[1, tStep, i]  # Estimated y-coordinate


        pC_hat_x = np.mean(pT_hat[0, tStep, :])
        pC_hat_y = np.mean(pT_hat[1, tStep, :])


        d_hat = math.dist([p_a_x, p_a_y],[pC_hat_x,pC_hat_y])


        # ---------------- Define Path ----------------


        # ---------------------------------------------------
        # ------------ Circle with Safe Distance ------------
        # -----------------------------------------------

        #r_s = 0.2
        #d_des[tStep] = np.max(np.sqrt((pT_hat_x - pC_hat_x) ** 2 + (pT_hat_y - pC_hat_y) ** 2)) + r_s


        # -----------------------------------------------
        # ----------- Approx. Min Ellipse ---------------
        # -----------------------------------------------

        r_s = 0.2 # Safe distance
        
        # Stack estimated positions into an [n x 2] array
        x_hat_positions = np.stack((pT_hat_x, pT_hat_y), axis=1)
        
        # 1. Get the Ellipse Properties
        center_stat, semi_axes, R = minEnclosingEllipseStat(x_hat_positions)
        
        # 2. Define the Scaled Ellipse Matrix 'A'
        a = semi_axes[0] + r_s
        b = semi_axes[1] + r_s
        
        # Handle degenerate case (e.g., all points collinear or identical)
        if a <= r_s or b <= r_s:
            d_des[tStep] = r_s # Fallback to a circle
        else:
            D_inv_sq = np.diag([1/a**2, 1/b**2])
            A = R @ D_inv_sq @ R.T
            
            # 3. Calculate agent's angle relative to the centroid
            theta = np.arctan2(p_a_y - pC_hat_y, p_a_x - pC_hat_x)
            u_theta = np.array([np.cos(theta), np.sin(theta)])
            
            # 4. Calculate radius 'd' for that angle
            # d = 1 / sqrt(u_theta' * A * u_theta)
            denominator = u_theta.T @ A @ u_theta
            
            if denominator <= 1e-10: # Safety check
                d_des[tStep] = r_s
            else:
                d_des[tStep] = 1 / np.sqrt(denominator)


        
        
        if dt > 0:
            d_des_dot = (d_des[tStep] - d_des[tStep-1])/dt
        else:
            d_des_dot = 0



        d_tilde = d_hat - d_des[tStep]


        # --------------- Define auxiliary variables for plotting ----------------------
        d = math.dist([p_a_x, p_a_y], [pC_x, pC_y])
        varrho = d - d_hat
        delta = d - d_des[tStep]
        varrho_data.append(varrho)
        delta_data.append(delta)

        for i in range(num_targets):
            x_tilde_norm = math.dist([p_T_x[i], p_T_y[i]],[pT_hat_x[i],pT_hat_y[i]])
            x_tilde_norm_data[i].append(x_tilde_norm)

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
                
                # Before time constant 1 T_c1
                if (elapsed_time) <= T_c1:
                    # Tangential term: k_omega * bar_varphi
                    tangential_term = k_omega * bar_psi_vec
                    
                    # Control input u(t) is only the second tangential term
                    u_a = tangential_term

                # After time constant 1 T_c1
                else:
                    sig_term = np.sign(d_tilde) * np.abs(d_tilde) ** (1 - alpha_2)

                    # Compute the first term of the control law
                    centripetal_term = ((np.exp(np.abs(d_tilde) ** alpha_2) / (alpha_2 * T_c2)) * sig_term + f_int[tStep] - d_des_dot) * psi_vec

                    # Tangential term: k_omega * bar_varphi
                    tangential_term = k_omega * bar_psi_vec

                    # Control input u(t) is the sum of the first and second terms
                    u_a = centripetal_term + tangential_term

                # u_t is the control input at time t
                v_a_x = u_a[0]  # x-component of the control input
                v_a_y = u_a[1]  # y-component of the control input

                qp.set_2D_velocity(agent_index, v_a_x, v_a_y, z_des)

                if tStep + 1 < n: # Check bounds
                    p_T[:, tStep+1, :] = p_T[:, tStep, :]

                # //////////////////////////////////////////////////////////////////////////////
                # //                Update target's position using the virtual velocity       //
                # //////////////////////////////////////////////////////////////////////////////
                # Calculate time t relative to the simulation start time
                elapsed_time = time.time() - start_time

                if tStep + 1 < n:
                    p_T[:, tStep+1, :] = p_T[:, tStep, :]

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
                p_T[:, tStep+1, :] = p_T[:, tStep, :]

        else:
            v_a_x = 0 
            v_a_y = 0

            # //////////////////////////////////////////////////////////////////////////////
            # //                Update target's position using the virtual velocity       //
            # //////////////////////////////////////////////////////////////////////////////
            # If not, keep the position unchanged
            if tStep + 1 < n:
                p_T[:, tStep+1, :] = p_T[:, tStep, :]

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
        row_data = [
            t - start_time,  # Time (t)
            tStep,           # Current time step
            p_a_x, p_a_y,    # Agent's current position
        ]

        # Add data for each target
        for i in range(num_targets):
            row_data.extend([
                p_T[0, tStep, i], p_T[1, tStep, i],           # Target i's position
                pT_hat[0, tStep, i], pT_hat[1, tStep, i],     # Estimated target i position
                P[0, 0, tStep, i], P[0, 1, tStep, i],         # P matrix for target i
                P[1, 0, tStep, i], P[1, 1, tStep, i],
                q[0, tStep, i], q[1, tStep, i],               # q vector for target i
                xi[0, tStep, i], xi[1, tStep, i],             # xi for target i
                varphi_x[i], varphi_y[i],                     # varphi for target i
            ])

        # Add shared data
        row_data.extend([
            v_a_x, v_a_y,    # Control input
            z                # Altitude
        ])

        csvWriter.writerow(row_data)

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

    # Combine all points for scaling
    all_x = [Agent_x_data]
    all_y = [Agent_y_data]
    
    for i in range(num_targets):
        # Extract estimated target's positions from pT_hat
        pT_hat_x_i = pT_hat[0, :tStep, i]  # Estimated x-coordinates over time for target i
        pT_hat_y_i = pT_hat[1, :tStep, i]  # Estimated y-coordinates over time for target i
        
        # Extract virtual target's position history from p_T
        p_T_x_history_i = p_T[0, :tStep, i]  # Target x-coordinate history for target i
        p_T_y_history_i = p_T[1, :tStep, i]  # Target y-coordinate history for target i
        
        all_x.extend([pT_hat_x_i, p_T_x_history_i])
        all_y.extend([pT_hat_y_i, p_T_y_history_i])

    # Combine all points into arrays for easy processing
    all_x = np.concatenate(all_x)
    all_y = np.concatenate(all_y)

    # Calculate the center point (average of all final target positions)
    center_x = np.mean([p_T[0, tStep, i] for i in range(num_targets)])
    center_y = np.mean([p_T[1, tStep, i] for i in range(num_targets)])

    # Calculate distances from the center to each point
    distances = np.sqrt((all_x - center_x) ** 2 + (all_y - center_y) ** 2)

    # Get the maximum distance for scaling the plot
    max_diag_range = max(distances)

    # Calculate the padding for the plot limits
    padding = 0.2 * max_diag_range  

    # Set aspect ratio and axis limits for the trajectory plot
    axs[0, 0].set_aspect('equal', adjustable='box')
    new_x_min = center_x - max_diag_range - padding
    new_x_max = center_x + max_diag_range + padding
    axs[0, 0].set_xlim(new_x_min, new_x_max)
    new_y_min = center_y - max_diag_range - padding
    new_y_max = center_y + max_diag_range + padding
    axs[0, 0].set_ylim(new_y_min, new_y_max)

    # Plot drone's trajectory
    axs[0, 0].plot(Agent_x_data, Agent_y_data, 'r-', markersize=4, linewidth=2, label='Drone')
    
    # Define colors for each target
    colors = ['b', 'g', 'c', 'm', 'y', 'orange', 'purple', 'brown']
    
    # Plot each target's trajectory and estimate
    for i in range(num_targets):
        color = colors[i % len(colors)]

        # Extract data for this target
        pT_hat_x_i = pT_hat[0, :tStep, i]
        pT_hat_y_i = pT_hat[1, :tStep, i]
        p_T_x_history_i = p_T[0, :tStep, i]
        p_T_y_history_i = p_T[1, :tStep, i]
        
        # Plot actual target trajectory (solid line)
        axs[0, 0].plot(p_T_x_history_i, p_T_y_history_i, color=color, linestyle='-', 
                       linewidth=1, label=f'Target {i+1}')
        
        # Plot estimated target trajectory (dashed line)
        axs[0, 0].plot(pT_hat_x_i, pT_hat_y_i, color=color, linestyle='--', 
                       linewidth=1, label=f'Est. Target {i+1}')

    # Set plot titles, labels, and legends
    axs[0, 0].set_title('(a) Trajectory Plot')
    axs[0, 0].legend(loc='upper left', fontsize='small')
    axs[0, 0].set_xlabel('x-axis (m)')
    axs[0, 0].set_ylabel('y-axis (m)')
    axs[0, 0].grid()

    # ---------------- subplot 2: target estimation error & tracking error --------------------------
    # Plot varrho and delta for each target

    axs[0, 1].plot(y_time, varrho_data, 'r-', linewidth=1, 
            label=r'$\varrho_c(t)$') # c for centroid
    axs[0, 1].plot(y_time, delta_data, 'b--', linewidth=1, 
                label=r'$\delta_c(t)$') # c for centroid

    axs[0, 1].legend(loc='upper right', fontsize='small')

    # Plot estimation & tracking errors
    axs[0, 1].set_title('(b) Estimation & Tracking Errors Plot')
    axs[0, 1].set_xlabel('time (sec)')
    axs[0, 1].set_ylabel('Error (m)')
    axs[0, 1].grid()

    # ---------------- subplot 3: target estimation error -------------------------------------------
    # Plot estimation error for each target
    for i in range(num_targets):
        color = colors[i % len(colors)]
        x_tilde_norm_0_i = x_tilde_norm_data[i][0]  # Initial norm of the error for target i
        
        axs[1, 0].axhline(y=x_tilde_norm_0_i, color=color, linestyle='-.', 
                          label=rf'$||\tilde{{x}}_{i+1}(0)||$')
        axs[1, 0].plot(y_time, x_tilde_norm_data[i], color=color, linestyle='-', 
                       linewidth=1, label=rf'$||\tilde{{x}}_{i+1}(t)||$')

    # Set plot titles, labels, and legends
    axs[1, 0].set_title('(c) Target Estimation Error Plot')
    axs[1, 0].legend(loc='upper right', fontsize='small')
    axs[1, 0].set_xlabel('time (sec)')
    axs[1, 0].set_ylabel('Error (m)')
    axs[1, 0].grid()

    # ---------------- subplot 4: integral terms -------------------------------------------
    f_int_time = np.linspace(0, time_elapsed, len(f_int))
    axs[1, 1].plot(f_int_time, f_int, 'b-', markersize=4, linewidth=1, 
                   label=r'$f_{int} = \int_0^t \tilde{d}(\tau) d\tau$') 
    axs[1, 1].legend(loc='upper right')
    axs[1, 1].set_title('(d) SSE integral term')
    axs[1, 1].set_xlabel('time (sec)')
    axs[1, 1].set_ylabel('Error (m)')
    axs[1, 1].grid()

    plt.show()

