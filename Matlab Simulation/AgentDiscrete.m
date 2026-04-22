classdef AgentDiscrete

    properties (Access = public)

        % ---- State variables ----
            p                   % 2x1 vector: current agent position [x; y]
            p_dot               % 2x1 vector: current agent velocity [vx; vy]
            p_traj              % 2xT matrix: agent position history over time
            u_traj
            heading             % Scalar: agent heading angle (rad)
    
        % ---- Target-related measurements ----
            d                   % 1xT cell: each cell contains 1xN vector of true distances to N targets
            varphi              % 1xN cell: each cell contains 2x1 vector of bearing components to target
            bar_varphi          % 1xN cell: each cell contains 2x1 vector of bearing components perpendicular to target
            varphi_traj         % 1xN cell: each cell contains 2xT matrix of bearing components over time
            psi_hat                 % Scalar: current bearing angle to targets centroid (rad)
            bar_psi_hat             % Scalar: estimated bearing angle to targets centroid (rad)
            projPoint           % 2x1 vector: coordinates of current projection point
    
        % ---- Target state estimators ----
            x_hat               % 1xN cell: each cell contains 2xT matrix of estimated target positions over time
            x_hat_curr
            x_hat_dot           % 1xN cell: each cell contains 2xT matrix of target position derivatives over time
            P                   % 1xN cell: each cell contains 1xT cell of 2x2 covariance matrices
            q                   % 1xN cell: each cell contains 1xT cell of 2x1 process noise vectors
        
        % ---- Estimated distances and errors ----
            d_hat               % 1xN vector: estimated distances to N targets
            d_convex_hat        % Scalar: estimated distance to convex hull of targets
            d_tilde             % 1xN vector: distance estimation errors (d_hat - true distance)
            u                   % 2x1 vector: current velocity command
            c                   % 2x1 vector: Actual targets centroid position
            c_hat               % 2x1 vector: Estimated targets centroid position
            localization_heading

    
        % ---- Control gains and parameters ----
            k_omega             % Scalar: control gain for angular velocity
            d_des               % Scalar: desired distance to target
            d_des_dot
            d_des_func
            d_des_traj          
            alpha_1             % Scalar: control parameter for distance error dynamics
            alpha_2             % Scalar: control parameter for bearing error dynamics
            Tc1                 % Scalar: control time constant
            Tc2                 % Scalar: control time constant
            controller_frequency
    
        % ---- Logging / diagnostics ----
            delta_traj          % 1xT vector: history of bearing angle offsets
            d_convex_true       % Scalar: true distance to convex hull
            varrho_traj         % 1xT vector: history of distances to projection point
            x_tilde_traj        % 1xT vector: history of target position estimation errors
            projPoint_traj      % 2xT matrix: history of projection point coordinates
            true_hull_vertices  % Mx2 matrix: coordinates of true convex hull vertices
            d_hat_dot           % 1xN vector: time derivatives of estimated distances
            theta_traj
            theta
            integral
            integralss
            curr_control_time

        % ---- 65 Hz rate limiting ----
            last_d_des_update_time      % Scalar: last simulation time updateDesiredDistance executed
            last_control_update_time    % Scalar: last simulation time controlInputPDT executed
            last_bearing_update_time    % Scalar: last simulation time getBearings executed
            last_estimator_update_time  % Scalar: last simulation time estimateTargetPDT executed
            control_rate_period         % Scalar: 1/65 seconds
            prev_d_des                  % Scalar: previous d_des value for finite difference
            
        % ---- Trajectory type for delta calculation ----
            trajectory_type     % String: 'hull', 'circle', or 'ellipse'
            velocity_saturation
            latency

    end

    methods

        % Initialisation of agent
        function obj = AgentDiscrete(p_0, x_hat_0_cell, k_omega, Tc1, Tc2, alpha_1, alpha_2, d_des_function_handle, tSteps, numTargets, targets, heading, velocity_saturation, latency)
           
            % Initial agent state
            obj.p = p_0;
            obj.p_dot = zeros(2, 1);
            obj.p_traj = zeros(2, tSteps);
            obj.heading = heading;
        
            % Control parameters
            obj.k_omega = k_omega;
            obj.Tc1 = Tc1;
            obj.Tc2 = Tc2;
            obj.alpha_1 = alpha_1;
            obj.alpha_2 = alpha_2;
            obj.d_des_func = d_des_function_handle;
            obj.d_des = 2;
            obj.prev_d_des = 2;
            obj.d_des_dot = zeros(1, tSteps);
            obj.d_des_traj = zeros(1, tSteps);
            obj.theta_traj = zeros(1, tSteps);
            obj.integral = zeros(1, tSteps);
            obj.integralss = zeros(1, tSteps);
            obj.velocity_saturation = velocity_saturation;
            obj.latency = latency;
        
            % Distance measurements
            obj.d = cell(1, tSteps);
            obj.d_tilde = 0;
            obj.c = mean(targets, 2);
            obj.curr_control_time = 0;
        
            % Bearing vectors
            obj.varphi = cell(1, numTargets);
            obj.bar_varphi = cell(1, numTargets);
            obj.varphi_traj = cell(1, numTargets);
        
            % Target state estimates
            obj.x_hat = cell(1, numTargets);
            obj.x_hat_dot = cell(1, numTargets);
            obj.d_hat = cell(1, numTargets);
            obj.d_hat_dot = zeros(1, numTargets);
        
            % Projection and auxiliary variables
            obj.psi_hat = zeros(2, 1);
            obj.bar_psi_hat = zeros(2, 1);
            obj.projPoint = zeros(2, 1);
        
            % Estimation parameters
            obj.P = cell(1, numTargets);
            obj.q = cell(1, numTargets);
        
            % Trajectory tracking variables
            obj.delta_traj = zeros(1, tSteps);
            obj.d_convex_true = zeros(1, tSteps);
            obj.varrho_traj = cell(1, numTargets);
            obj.x_tilde_traj = cell(1, numTargets);
            obj.projPoint_traj = zeros(2, tSteps);
            obj.localization_heading = zeros(2,1);

            % 65 Hz rate limiting initialisation
            obj.control_rate_period = 1/65;
            obj.last_d_des_update_time = -inf;
            obj.last_control_update_time = -inf;
            obj.last_bearing_update_time = -inf;
            obj.last_estimator_update_time = -inf;
        
            % Initialise per-target variables
            for i = 1:numTargets
                obj.varphi_traj{i} = zeros(2, tSteps);
                obj.x_hat{i} = zeros(2, tSteps);
                obj.x_hat{i}(:, 1) = x_hat_0_cell(:, i);
                obj.x_hat_dot{i} = zeros(2, tSteps);
        
                obj.P{i} = cell(1, tSteps);
                obj.P{i}{1} = zeros(2, 2);
                obj.q{i} = cell(1, tSteps);
                obj.q{i}{1} = zeros(2, 1);
        
                obj.varrho_traj{i} = zeros(1, tSteps);
                obj.x_tilde_traj{i} = zeros(2, tSteps);
            end

            initial_estimates = zeros(2, numTargets);
            for i = 1:numTargets
                initial_estimates(:, i) = obj.x_hat{i}(:, 1);
            end
            obj.c_hat = mean(initial_estimates, 2);
            obj.theta = atan2(obj.p(2) - obj.c_hat(2), obj.p(1) - obj.c_hat(1));
            obj.theta_traj(1,:) = obj.theta;
        end



        function obj = updateDesiredDistance(obj, cur_tStep, dT)
            t = cur_tStep;
            current_time = t * dT;

            % --- 65 Hz rate limiter ---
            if (current_time - obj.last_d_des_update_time) < obj.control_rate_period
                obj.d_des_traj(t) = obj.d_des;
                if t > 1
                    obj.d_des_dot(t) = obj.d_des_dot(t-1);
                end
                return;
            end
            obj.last_d_des_update_time = current_time;
            % --------------------------
            
            numTargets = length(obj.x_hat);
            
            x_hat_positions = zeros(2, numTargets);
            for i = 1:numTargets
                x_hat_positions(:, i) = obj.x_hat{i}(:, cur_tStep);
            end
        
            theta = atan2(obj.p(2) - obj.c_hat(2), obj.p(1) - obj.c_hat(1));
            obj.theta_traj(t) = theta;
        
            % Store previous value before updating
            obj.prev_d_des = obj.d_des;

            obj.d_des = obj.d_des_func(current_time, theta, x_hat_positions, obj.c_hat, obj.p);
            
            obj.d_des_traj(t) = obj.d_des;

            % Finite difference derivative (matches experimental code)
            elapsed_since_last = obj.control_rate_period; % approximately 1/65
            obj.d_des_dot(t) = (obj.d_des - obj.prev_d_des) / elapsed_since_last;
        end

        % Getting bearing measurement to each target
        function obj = getBearings(obj, cur_tStep, targets)
            t = cur_tStep;
            current_time = t * 0.001; % assumes dT=0.001; pass dT if different
            
            % --- 65 Hz rate limiter ---
            if (current_time - obj.last_bearing_update_time) < obj.control_rate_period
                % Hold previous values, still log delta
                
                return;
            end
            obj.last_bearing_update_time = current_time;
            % --------------------------

            numTargets = size(targets, 2);
            distances = zeros(1, numTargets);
            varphi_matrix = zeros(2, numTargets);
            % Bearings to each target
            for i = 1:numTargets
                target_pos = targets(:, i);  
                d_i = norm(target_pos - obj.p);
                distances(i) = d_i;
                obj.varphi{i} = (target_pos - obj.p) / d_i;
                varphi_matrix(:, i) = obj.varphi{i};
                obj.bar_varphi{i} = [ cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.varphi{i};
            end
            % Bearings to centroid
            obj.psi_hat = (obj.c_hat - obj.p) / norm((obj.c_hat - obj.p));
            obj.bar_psi_hat = [ cos(pi/2), sin(pi/2);
                            -sin(pi/2), cos(pi/2)] * obj.psi_hat;
            
            
            % --- Localization heading (computed once) ---
            if obj.localization_heading == zeros(2,1)
                angles = atan2(varphi_matrix(2, :), varphi_matrix(1, :));
                min_angle = min(angles);
                max_angle = max(angles);
                bisect_angle = (min_angle + max_angle) / 2;
                if (max_angle - min_angle) > pi
                    bisect_angle = bisect_angle + pi;
                end
                perp_angle = bisect_angle - (pi/2);
                perp_angle = wrapToPi(perp_angle);
                obj.localization_heading = [cos(perp_angle); sin(perp_angle)];
            end
        end


        function obj = getBearingsWithNoise(obj, cur_tStep, targets)
            t = cur_tStep;
            current_time = t * 0.001;

            % --- 65 Hz rate limiter ---
            if (current_time - obj.last_bearing_update_time) < obj.control_rate_period
                
                return;
            end
            obj.last_bearing_update_time = current_time;
            % --------------------------

            numTargets = size(targets, 2);
            varphi_matrix = zeros(2, numTargets);
            for i = 1:numTargets
                target_pos = targets(:, i);  
                sigma = deg2rad(0.5);
                v = (target_pos - obj.p);
                v = v / norm(v);
                theta = atan2(v(2), v(1));
                theta = theta + sigma * randn;
                obj.varphi{i} = [cos(theta); sin(theta)];
                varphi_matrix(:, i) = obj.varphi{i};
                obj.bar_varphi{i} = [ cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.varphi{i};
            end
            obj.psi_hat = (obj.c_hat - obj.p) / norm((obj.c_hat - obj.p));
            obj.bar_psi_hat = [ cos(pi/2), sin(pi/2);
                            -sin(pi/2), cos(pi/2)] * obj.psi_hat;
         
            
            if obj.localization_heading == zeros(2,1)
                angles = atan2(varphi_matrix(2, :), varphi_matrix(1, :));
                min_angle = min(angles);
                max_angle = max(angles);
                bisect_angle = (min_angle + max_angle) / 2;
                if (max_angle - min_angle) > pi
                    bisect_angle = bisect_angle + pi;
                end
                perp_angle = bisect_angle - (pi/2);
                perp_angle = wrapToPi(perp_angle);
                obj.localization_heading = [cos(perp_angle); sin(perp_angle)];
            end
        end



        % Individual target estimator from Sui et al. (2025)
        function obj = estimateTargetPDT(obj, cur_tStep, dT, targets, Tc1)
            t = cur_tStep;
            current_time = t * dT;
            numTargets = size(targets, 2);
            latest_x_hats = zeros(2, numTargets);

            % --- 65 Hz rate limiter ---
            if (current_time - obj.last_estimator_update_time) < obj.control_rate_period
                % Hold: propagate previous values forward
                for i = 1:numTargets
                    if t + 1 <= size(obj.x_hat{i}, 2)
                        obj.x_hat{i}(:, t+1) = obj.x_hat{i}(:, t);
                    end
                    obj.P{i}{t+1} = obj.P{i}{t};
                    obj.q{i}{t+1} = obj.q{i}{t};
                    latest_x_hats(:, i) = obj.x_hat{i}(:, t);
                    obj.x_tilde_traj{i}(:, t) = obj.x_hat{i}(:, t) - targets(:, i);
                end
                obj.c_hat = mean(latest_x_hats, 2);
                return;
            end
            % Use the actual elapsed time since last estimator tick for integration
            dt_estimator = current_time - obj.last_estimator_update_time;
            if dt_estimator <= 0 || isinf(dt_estimator)
                dt_estimator = obj.control_rate_period;
            end
            obj.last_estimator_update_time = current_time;
            % --------------------------
            
            for i = 1:numTargets

                if t*dT < Tc1
                    if t <= 2
                        obj.x_hat_dot{i}(:, t) = zeros(2, 1);
                    else
                        xi = inv(obj.P{i}{t}) * (obj.P{i}{t} * obj.x_hat{i}(:, t) - obj.q{i}{t});
                        if norm(xi) == 0
                            Psi_a = zeros(2, 1);
                        else
                            Psi_a = xi ./ (norm(xi) ^ obj.alpha_1);
                        end
                        obj.x_hat_dot{i}(:, t) = -1 / (Tc1 * obj.alpha_1) * exp(norm(xi) ^ obj.alpha_1) * Psi_a;
                    end
                    % Updating kreisselmeier's regressor with 65Hz dt
                    P_dot = -obj.P{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.';
                    q_dot = -obj.q{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.' * obj.p;
                    obj.P{i}{t+1} = obj.P{i}{t} + P_dot * dt_estimator;
                    obj.q{i}{t+1} = obj.q{i}{t} + q_dot * dt_estimator;
                else
                    obj.x_hat_dot{i}(:, t) = zeros(2, 1);
                    obj.P{i}{t+1} = obj.P{i}{t};
                    obj.q{i}{t+1} = obj.q{i}{t};
                end

                if t + 1 <= size(obj.x_hat{i}, 2)
                    obj.x_hat{i}(:, t+1) = obj.x_hat{i}(:, t) + obj.x_hat_dot{i}(:, t) * dt_estimator;
                end
                latest_x_hats(:, i) = obj.x_hat{i}(:, t);
                obj.x_tilde_traj{i}(:, t) = obj.x_hat{i}(:, t) - targets(:, i);

            end
            
            obj.c_hat = mean(latest_x_hats, 2);
        end

        function obj = estimateTargetPDTIndefinite(obj, cur_tStep, dT, targets, Tc1)
            t = cur_tStep;
            current_time = t * dT;
            numTargets = size(targets, 2);
            latest_x_hats = zeros(2, numTargets);

            % --- 65 Hz rate limiter ---
            if (current_time - obj.last_estimator_update_time) < obj.control_rate_period
                for i = 1:numTargets
                    if t + 1 <= size(obj.x_hat{i}, 2)
                        obj.x_hat{i}(:, t+1) = obj.x_hat{i}(:, t);
                    end
                    obj.P{i}{t+1} = obj.P{i}{t};
                    obj.q{i}{t+1} = obj.q{i}{t};
                    latest_x_hats(:, i) = obj.x_hat{i}(:, t);
                    obj.x_tilde_traj{i}(:, t) = obj.x_hat{i}(:, t) - targets(:, i);
                end
                obj.c_hat = mean(latest_x_hats, 2);
                return;
            end
            dt_estimator = current_time - obj.last_estimator_update_time;
            if dt_estimator <= 0 || isinf(dt_estimator)
                dt_estimator = obj.control_rate_period;
            end
            obj.last_estimator_update_time = current_time;
            % --------------------------
            
            for i = 1:numTargets
                if t <= 2
                    obj.x_hat_dot{i}(:, t) = zeros(2, 1);
                else
                    xi = inv(obj.P{i}{t}) * (obj.P{i}{t} * obj.x_hat{i}(:, t) - obj.q{i}{t});
                    if norm(xi) == 0
                        Psi_a = zeros(2, 1);
                    else
                        Psi_a = xi ./ (norm(xi) ^ obj.alpha_1);
                    end
                    obj.x_hat_dot{i}(:, t) = -1 / (Tc1 * obj.alpha_1) * exp(norm(xi) ^ obj.alpha_1) * Psi_a;
                end
                P_dot = -obj.P{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.';
                q_dot = -obj.q{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.' * obj.p;
                obj.P{i}{t+1} = obj.P{i}{t} + P_dot * dt_estimator;
                obj.q{i}{t+1} = obj.q{i}{t} + q_dot * dt_estimator;

                if t + 1 <= size(obj.x_hat{i}, 2)
                    obj.x_hat{i}(:, t+1) = obj.x_hat{i}(:, t) + obj.x_hat_dot{i}(:, t) * dt_estimator;
                end
                latest_x_hats(:, i) = obj.x_hat{i}(:, t);
                obj.x_tilde_traj{i}(:, t) = obj.x_hat{i}(:, t) - targets(:, i);
            end
            
            obj.c_hat = mean(latest_x_hats, 2);
        end

        % Calculating Control input for controller inspired by Sui et al. (2025)
        function obj = controlInputPDT(obj, t, Tc1, Tc2, dT)

            current_time = t * dT;

            % --- 65 Hz rate limiter ---
            if (current_time - obj.last_control_update_time) < obj.control_rate_period
                if t > 1
                    obj.u_traj(:, t) = obj.u_traj(:, t-1);
                end
                return;
            end
            obj.last_control_update_time = current_time;
            % --------------------------

            % Error in distance to shape
            obj.d_tilde = norm(obj.c_hat - obj.p) - obj.d_des;
            
        
            if t*dT < Tc1
                obj.u = obj.k_omega  * obj.localization_heading;
            else
                % Use tanh-based sig to match experimental code
                sig_term = tanh(12 * obj.d_tilde) * abs(obj.d_tilde) ^ (1 - obj.alpha_2);

                v_cen = 1/(obj.alpha_2 * obj.Tc2) * exp(abs(obj.d_tilde) ^ obj.alpha_2) * sig_term - obj.d_des_dot(t);

                obj.u = v_cen * obj.psi_hat + obj.k_omega  * obj.bar_psi_hat;

            end

            if norm(obj.u) > obj.velocity_saturation
                obj.u = obj.velocity_saturation * obj.u / norm(obj.u);
            end
            obj.u_traj(:, t) = obj.u;
        end

        % Calculating Control input for controller inspired by Sui et al. (2025)
        function obj = controlInputPDTUpdated(obj, t, Tc1, Tc2, dT)

            % Error in distance to shape
            obj.d_tilde = norm(obj.c_hat - obj.p) - obj.d_des;
            
            
            if t*dT < Tc1
                obj.u = 2  * obj.localization_heading;
            else
               
                v_cen = 1/(obj.alpha_2 * obj.Tc2) * exp(abs(obj.d_tilde) ^ obj.alpha_2) * sig(obj.d_tilde, 1 - obj.alpha_2) - obj.d_des_dot(t);
                
                obj.u = v_cen * obj.psi_hat + obj.k_omega  * obj.bar_psi_hat;

            end

            obj.u_traj(:, t) = obj.u;
        end

        % Moving agent with holonomic agent constraints
        function obj = move(obj, dT, t)
            if t*dT >= obj.Tc1
                obj.p_dot = obj.u_traj(:, t - obj.latency/dT);
            else
                obj.p_dot = obj.u;
            end
            obj.p = obj.p + obj.p_dot * dT;
            obj.p_traj(:, t) = obj.p;
            obj.delta_traj(t) = norm(obj.c - obj.p) - obj.d_des;
        end

        
    end
end