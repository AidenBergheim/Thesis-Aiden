
classdef Agent

    properties (Access = public)

        % ---- State variables ----
            p                   % 2x1 vector: current agent position [x; y]
            p_dot               % 2x1 vector: current agent velocity [vx; vy]
            p_traj              % 2xT matrix: agent position history over time
            u_traj              % 2xT matrix: control effort history over time
    
        % ---- Target-related measurements ----
            d                   % 1xT cell: each cell contains 1xN vector of true distances to N targets
            varphi              % 1xN cell: each cell contains 2x1 vector of bearing components to target
            bar_varphi          % 1xN cell: each cell contains 2x1 vector of bearing components perpendicular to target
            varphi_traj         % 1xN cell: each cell contains 2xT matrix of bearing components over time
            psi_hat                 % Scalar: current bearing angle to targets centroid (rad)
            bar_psi_hat             % Scalar: estimated bearing angle to targets centroid (rad)
            bar_Chi             % 2x1 vector: unit vector representing agent direction during localization
    
        % ---- Target state estimators ----
            x_hat               % 1xN cell: each cell contains 2xT matrix of estimated target positions over time
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
    
        % ---- Control gains and parameters ----
            k_s                 % Scalar: control gain for speed during localization
            k_d                 % Scalar: control gain for angular velocity
            d_des               % Scalar: desired distance to target
            d_des_dot           % Scalar: time derivative of desired distance to target
            d_des_func          % Function handle: computes desired distance to centroid
            d_des_traj          % 1xN cell: each cell contains desired trajectory over time
            alpha_1             % Scalar: control parameter for distance error dynamics
            alpha_2             % Scalar: control parameter for bearing error dynamics
            Tc1                 % Scalar: control time constant
            Tc2                 % Scalar: control time constant
    
        % ---- Logging / diagnostics ----
            delta_traj          % 1xT vector: history of bearing angle offsets
            d_convex_true       % Scalar: true distance to convex hull
            varrho_traj         % 1xT vector: history of distances to projection point
            x_tilde_traj        % 1xT vector: history of target position estimation errors
            projPoint_traj      % 2xT matrix: history of projection point coordinates
            true_hull_vertices  % Mx2 matrix: coordinates of true convex hull vertices
            d_hat_dot           % 1xN vector: time derivatives of estimated distances
            
    end

    methods

        % Initialisation of agent
        function obj = Agent(p_0, x_hat_0_cell, k_s, k_d, Tc1, Tc2, alpha_1, alpha_2, d_des_function_handle, tSteps, numTargets, targets)
           
            % Initial agent state
            obj.p = p_0;                                 % Initial position (2x1)
            obj.p_dot = zeros(2, 1);                     % Velocity (2x1)
            obj.p_traj = zeros(2, tSteps);               % Position trajectory (2xT)
        
            % Control parameters
            obj.k_s = k_s;                       % Angular velocity gain
            obj.k_d = k_d;
            obj.Tc1 = Tc1;                               % Time constant 1
            obj.Tc2 = Tc2;                               % Time constant 2
            obj.alpha_1 = alpha_1;                       % Control gain α₁
            obj.alpha_2 = alpha_2;                       % Control gain α₂
            obj.d_des_func = d_des_function_handle;      % Function handle for desired distance
            obj.d_des = 0;                               % Desired agent-to-centroid distance
            obj.d_des_dot = zeros(1, tSteps);            % Time derivative of desired agent-to-centroid distance
            obj.d_des_traj = zeros(1, tSteps);           % Time history of desired agent-to-centroid distance
        
            % Distance measurements
            obj.d = cell(1, tSteps);                     % True distances to each target
            obj.d_tilde = 0;                             % Distance error
            obj.c = mean(targets, 2);
        
            % Bearing vectors
            obj.varphi = cell(1, numTargets);            % Bearing vector to target
            obj.bar_varphi = cell(1, numTargets);        % Normalised bearing vector
            obj.varphi_traj = cell(1, numTargets);       % Bearing trajectory over time
        
            % Target state estimates
            obj.x_hat = cell(1, numTargets);             % Estimated target position
            obj.x_hat_dot = cell(1, numTargets);         % Estimated target velocity
            obj.d_hat = cell(1, numTargets);             % Estimated agent-target distance
            obj.d_hat_dot = zeros(1, numTargets);        % Estimated distance rate
        
            % Projection and auxiliary variables
            obj.psi_hat = zeros(2, 1);                       % Auxiliary vector ψ
            obj.bar_psi_hat = zeros(2, 1);                   % Estimated ψ
        
            % Estimation parameters
            obj.P = cell(1, numTargets);                 % Covariance matrices
            obj.q = cell(1, numTargets);                 % Filter bias terms
            initial_estimates = zeros(2, numTargets);
        
            % Trajectory tracking variables
            obj.delta_traj = zeros(1, tSteps);           % Control error δ trajectory
            obj.d_convex_true = zeros(1, tSteps);        % True convex hull distance
            obj.varrho_traj = cell(1, numTargets);       % Varrho trajectory per target
            obj.x_tilde_traj = cell(1, numTargets);      % Target position estimation error
            obj.bar_Chi = zeros(2,1);
        
            % Initialise per-target variables
            for i = 1:numTargets
                obj.varphi_traj{i} = zeros(2, tSteps);           % Bearing trajectory
                obj.x_hat{i} = zeros(2, tSteps);                 % Target position estimate
                obj.x_hat{i}(:, 1) = x_hat_0_cell(:, i);         % Initial position estimate
                obj.x_hat_dot{i} = zeros(2, tSteps);             % Target velocity estimate
        
                obj.P{i} = cell(1, tSteps);
                obj.P{i}{1} = zeros(2, 2);
                obj.q{i} = cell(1, tSteps);
                obj.q{i}{1} = zeros(2, 1);
        
                obj.varrho_traj{i} = zeros(1, tSteps);           % Varrho trajectory
                obj.x_tilde_traj{i} = zeros(2, tSteps);          % Position estimation error

                initial_estimates(:, i) = obj.x_hat{i}(:, 1);
            end

            % Initial estimate of centroid position
            obj.c_hat = mean(initial_estimates, 2);
        end

        % Updating desired agent-to-centroid distance
        function obj = updateDesiredDistance(obj, t, dT)
            
            current_time = t * dT;
            
            % Extract estimated positions (2 x numTargets)
            x_hat_positions = zeros(2, length(obj.x_hat));
            for i = 1:length(obj.x_hat)
                x_hat_positions(:, i) = obj.x_hat{i}(:, t);
            end
        
            % Calculating theta value
            theta = atan2(obj.p(2) - obj.c_hat(2), obj.p(1) - obj.c_hat(1));
        
            % Computing desired agent-to-centroid distance
            obj.d_des = obj.d_des_func(current_time, theta, x_hat_positions, obj.c_hat, obj.p);
            obj.d_des_traj(t) = obj.d_des;
            obj.d_des_dot(t) = 0;   % Derivative is zero for circular case
        end

        % Getting bearing measurement to each target
        function obj = getBearings(obj, cur_tStep, targets)
            t = cur_tStep;
            numTargets = size(targets, 2);
            distances = zeros(1, numTargets);
            varphi_matrix = zeros(2, numTargets);
            
            % Finding bearing measurement for each target
            for i = 1:numTargets
                
                % Only using distance to target for plotting and to
                % calculate bearing, not actually used by agent algorithms
                target_pos = targets(:, i);  
                d_i = norm(target_pos - obj.p);
                distances(i) = d_i;
                obj.varphi{i} = (target_pos - obj.p) / d_i;
                varphi_matrix(:, i) = obj.varphi{i};
                
                % Finding bar_varphi, the rotated pi/2 vector of varphi
                obj.bar_varphi{i} = [ cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.varphi{i};
            end

            % Computing estimated bearing to centroid and the rotated pi/2 
            % vector of this bearing 
            obj.psi_hat = (obj.c_hat - obj.p) / norm((obj.c_hat - obj.p));
            obj.bar_psi_hat = [ cos(pi/2), sin(pi/2);
                            -sin(pi/2), cos(pi/2)] * obj.psi_hat;

            % Used for plotting, not by controller
            obj.delta_traj(t) = norm(obj.c - obj.p) - obj.d_des;

            % Calculating Chi Bar if not already calculated
            if obj.bar_Chi == zeros(2,1)
                
                % Finding min and max of angles
                angles = atan2(varphi_matrix(2, :), varphi_matrix(1, :));
                min_angle = min(angles);
                max_angle = max(angles);
                
                % Finding the bisecting angle and ensuring wrapped
                bisect_angle = (min_angle + max_angle) / 2;
                if (max_angle - min_angle) > pi
                    bisect_angle = bisect_angle + pi;
                end
                
                % Calculating and normalising perpendicular to bisectng
                % angle
                perp_angle = wrapToPi(bisect_angle - (pi/2));
                
                % Generating relevant unit vector
                obj.bar_Chi = [cos(perp_angle); sin(perp_angle)];
            end
            
        end


        % Estimating the position of each target
        function obj = estimateTargetPDT(obj, cur_tStep, dT, targets)
            t = cur_tStep;
            numTargets = size(targets, 2);
            latest_x_hats = zeros(2, numTargets);
            
            for i = 1:numTargets

                % Estimate each target
                if t*dT < obj.Tc1
                    if t <= 2
                        obj.x_hat_dot{i}(:, t) = zeros(2, 1);
                    else
                        % Finding x_hat_dot
                        xi = inv(obj.P{i}{t}) * (obj.P{i}{t} * obj.x_hat{i}(:, t) - obj.q{i}{t});
                        if norm(xi) == 0
                            Psi_a = zeros(2, 1);
                        else
                            Psi_a = xi ./ (norm(xi) ^ obj.alpha_1);
                        end
                        obj.x_hat_dot{i}(:, t) = -1 / (obj.Tc1 * obj.alpha_1) * exp(norm(xi) ^ obj.alpha_1) * Psi_a;
                    end

                    % Updating kreisselmeier's regressor
                    P_dot = -obj.P{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.';
                    q_dot = -obj.q{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.' * obj.p;
                    obj.P{i}{t+1} = obj.P{i}{t} + P_dot * dT;
                    obj.q{i}{t+1} = obj.q{i}{t} + q_dot * dT;

                % Just continue propogating values if past Tc1
                else
                    obj.x_hat_dot{i}(:, t) = zeros(2, 1);   % Assume static target
                    obj.P{i}{t+1} = obj.P{i}{t};            % Propagate P
                    obj.q{i}{t+1} = obj.q{i}{t};            % Propagate q
                end

                % Euler approximation of new x_hat
                if t + 1 <= size(obj.x_hat{i}, 2)
                    obj.x_hat{i}(:, t+1) = obj.x_hat{i}(:, t) + obj.x_hat_dot{i}(:, t) * dT;
                end
                latest_x_hats(:, i) = obj.x_hat{i}(:, t);
                obj.x_tilde_traj{i}(:, t) = obj.x_hat{i}(:, t) - targets(:, i);

            end
            
            % This must also always run, as updateDesiredDistance depends on it
            obj.c_hat = mean(latest_x_hats, 2);
        end

        % Calculating Control input
        function obj = controlInputPDT(obj, t, dT)

            % Computing tracking error
            obj.d_tilde = norm(obj.c_hat - obj.p) - obj.d_des;
            
            % Computing control effort based on stage of circumnavigation
                % Phase 1 - Localization stage
                if t*dT < obj.Tc1
                    obj.u = obj.k_s  * obj.bar_Chi;
                
                % Phase 2 - Cricumnavigation stage
                else
                    v_cen = 1/(obj.alpha_2 * obj.Tc2) * exp(abs(obj.d_tilde) ^ obj.alpha_2)...
                        * sig(obj.d_tilde, 1 - obj.alpha_2) - obj.d_des_dot(t);
                    
                    obj.u = v_cen * obj.psi_hat + obj.k_d  * obj.bar_psi_hat;
                end

            obj.u_traj(:, t) = obj.u;
        end

        % Moving agent with holonomic agent constraints by performing Euler
        % integration
        function obj = move(obj, dT, t)
            obj.p_dot = obj.u;
            obj.p = obj.p + obj.p_dot * dT;
            obj.p_traj(:, t) = obj.p;
        end
    end
end

% sig function used for algorithms in Sui et al. (2025)
function out = sig(z,alpha)
    out = zeros(length(z),1);
    for i = 1:length(z)
        out(i) = sign(z(i))*abs(z(i))^alpha;
    end
end
