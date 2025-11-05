% Agent class that implements algorithms from and inspired by: Cao et al. (2021), 
% Sui et at. (2025) and Zhao et al. (2018)
% Class is inspired by Donglin Sui: https://github.com/Gloogger

classdef Agent

    properties (Access = public)

        % ---- State variables ----
            p                   % 2x1 vector: current agent position [x; y]
            p_dot               % 2x1 vector: current agent velocity [vx; vy]
            p_traj              % 2xT matrix: agent position history over time
            heading             % Scalar: agent heading angle (rad)
    
        % ---- Target-related measurements ----
            d                   % 1xT cell: each cell contains 1xN vector of true distances to N targets
            varphi              % 1xN cell: each cell contains 2x1 vector of bearing components to target
            bar_varphi          % 1xN cell: each cell contains 2x1 vector of bearing components to projection point
            varphi_traj         % 1xN cell: each cell contains 2xT matrix of bearing components over time
            psi                 % Scalar: current bearing angle to projection point (rad)
            psi_hat             % Scalar: estimated bearing angle to projection point (rad)
            projPoint           % 2x1 vector: coordinates of current projection point
    
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
            rs                  % Scalar: desired circumnavigation radius
    
        % ---- Control gains and parameters ----
            k_omega             % Scalar: control gain for angular velocity
            d_des               % Scalar: desired distance to target
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
            
        % ---- Trajectory type for delta calculation ----
            trajectory_type     % String: 'hull', 'circle', or 'ellipse'

    end

    methods

        % Initialisation of agent
        function obj = Agent(p_0, x_hat_0_cell, k_omega, Tc1, Tc2, alpha_1, alpha_2, d_des, rs, tSteps, numTargets, hullVertices, heading)
           
            % Initial agent state
            obj.p = p_0;                                 % Initial position (2x1)
            obj.p_dot = zeros(2, 1);                     % Velocity (2x1)
            obj.p_traj = zeros(2, tSteps);               % Position trajectory (2xT)
            obj.heading = heading;                       % Initial heading (scalar)
        
            % Control parameters
            obj.k_omega = k_omega;                       % Angular velocity gain
            obj.Tc1 = Tc1;                               % Time constant 1
            obj.Tc2 = Tc2;                               % Time constant 2
            obj.alpha_1 = alpha_1;                       % Control gain α₁
            obj.alpha_2 = alpha_2;                       % Control gain α₂
            obj.d_des = d_des;                           % Desired distance to targets
        
            % Distance measurements
            obj.d = cell(1, tSteps);                     % True distances to each target
            obj.d_convex_hat = 0;                        % Estimated convex hull distance
            obj.d_tilde = 0;                             % Distance error
        
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
            obj.psi = zeros(2, 1);                       % Auxiliary vector ψ
            obj.psi_hat = zeros(2, 1);                   % Estimated ψ
            obj.projPoint = zeros(2, 1);                 % Projection point on hull
        
            % Estimation parameters
            obj.P = cell(1, numTargets);                 % Covariance matrices
            obj.q = cell(1, numTargets);                 % Filter bias terms
        
            % Trajectory tracking variables
            obj.delta_traj = zeros(1, tSteps);           % Control error δ trajectory
            obj.d_convex_true = zeros(1, tSteps);        % True convex hull distance
            obj.varrho_traj = cell(1, numTargets);       % Varrho trajectory per target
            obj.x_tilde_traj = cell(1, numTargets);      % Target position estimation error
            obj.projPoint_traj = zeros(2, tSteps);       % Projection point trajectory
        
            % Initialise per-target variables
            for i = 1:numTargets
                obj.varphi_traj{i} = zeros(2, tSteps);           % Bearing trajectory
                obj.x_hat{i} = zeros(2, tSteps);                 % Target position estimate
                obj.x_hat{i}(:, 1) = x_hat_0_cell(:, i);         % Initial position estimate
                obj.x_hat_dot{i} = zeros(2, tSteps);             % Target velocity estimate
        
                obj.P{i} = cell(1, tSteps);                      % Covariance trajectory
                obj.P{i}{1} = zeros(2, 2);                       % Initial covariance
                obj.q{i} = cell(1, tSteps);                      % Bias vector trajectory
                obj.q{i}{1} = zeros(2, 1);                       % Initial bias
        
                obj.varrho_traj{i} = zeros(1, tSteps);           % Varrho trajectory
                obj.x_tilde_traj{i} = zeros(2, tSteps);          % Position estimation error
            end
        
            % Convex hull and sensing parameters
            obj.true_hull_vertices = hullVertices;               % True convex hull vertices
            obj.rs = rs;                                         % Sensor range
            obj.trajectory_type = 'hull';                        % Default trajectory type
        end

        % Getting bearing measurement to each target
        function obj = getBearings(obj, cur_tStep, targets)
            t = cur_tStep;
            numTargets = size(targets, 2);
            distances = zeros(1, numTargets);

            % Finding measurement for each target
            for i = 1:numTargets

                % Only using distance to target for plotting and to
                % calculate bearing, not actually used by agent algorithms
                target_pos = targets(:, i);  
                d_i = norm(target_pos - obj.p);
                distances(i) = d_i;
                obj.varphi{i} = (target_pos - obj.p) / d_i;

                % Finding bar_varphi, the rotated pi/2 vector of varphi
                obj.bar_varphi{i} = [ cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.varphi{i};

                % Recording data for plotting
                obj.varphi_traj{i}(:, t) = obj.varphi{i};
                obj.d_hat{i}(:, t) = norm(obj.x_hat{i}(:, t) - obj.p);
            end

            % NOT USED BY CONTROLLER - for plotting
            obj.d{t} = distances;
            
            % Calculate true distance based on trajectory type
            if strcmp(obj.trajectory_type, 'ellipse')
                obj.d_convex_true(t) = obj.computeTrueDistanceToEllipse(targets);
            else
                obj.d_convex_true(t) = obj.computeTrueDistanceToHull();
            end
            
            obj.delta_traj(t) = obj.d_convex_true(t) - obj.d_des;
        end

        % Individual target estimator from Sui et al. (2025)
        function obj = estimateTargetPDT(obj, cur_tStep, dT, targets)
            t = cur_tStep;
            numTargets = size(targets, 2);

            % Estimating each target
            for i = 1:numTargets

                % Ensuring not out of bound of x_hat_dot object 
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

                % Euler approximate of new x_hat
                if t + 1 <= size(obj.x_hat{i}, 2)
                    obj.x_hat{i}(:, t+1) = obj.x_hat{i}(:, t) + obj.x_hat_dot{i}(:, t) * dT;
                end
            end
        end

        % Calculating psi and psi_hat unit vectors directed from agent to 
        % closest point on convex hull, and perpendicular to it. 
        % Using methods from Holmberg (2016)
        function obj = getPsiAndProjectionHull(obj, cur_tStep, numTargets)
            obj.trajectory_type = 'hull';
            agentPos = obj.p;
            
            % Extract estimated positions
            est_positions = zeros(numTargets, 2);
            for i = 1:numTargets
                est_positions(i, :) = obj.x_hat{i}(:, cur_tStep)';
            end

            % Finding convex hull around points
            K = convhull(est_positions(:,1), est_positions(:,2));
            hull = est_positions(K, :);
            
            min_distance = inf;
            for i = 1:(size(hull,1)-1)

                % Inline projection for hull edges
                A = hull(i,:)';
                B = hull(i+1,:)';
                AB = B - A;
                AP = agentPos - A;
                t = max(0, min(1, dot(AP, AB) / dot(AB, AB)));
                proj = A + t * AB;
                distance = norm(agentPos - proj);
                if  distance < min_distance
                    min_distance = distance;
                    projPoint = proj;
                end
            end

            % Compute psi and psi_hat
            obj.psi = projPoint - agentPos;
            norm_psi = norm(obj.psi);
            obj.psi = obj.psi / norm_psi;
            obj.psi_hat = [ cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.psi;
            
            % Store results
            obj.projPoint = projPoint;
            obj.projPoint_traj(:, cur_tStep) = obj.projPoint;
            obj.d_convex_hat = norm_psi;
        end

        % Calculating psi and psi_hat unit vectors directed from agent to 
        % centre of minimum enclosing circle. From Wang et al. (2024)
        function obj = getPsiAndProjectionMinCircle(obj, cur_tStep, numTargets)
            obj.trajectory_type = 'circle';
            agentPos = obj.p;
        
            % Extract estimated target positions
            est_positions = zeros(numTargets, 2);
            for i = 1:numTargets
                est_positions(i, :) = obj.x_hat{i}(:, cur_tStep)';
            end
        
            % Compute minimum enclosing circle according rule of
            % Propisition 2.3 in Wang et al. (2024)
            [center, radius] = minEnclosingCircleProp(est_positions);
            obj.d_des = radius + 0.5;
        
            % Finding vectors towards and normal to centre of circle from
            % agent
            vec_to_center = center - agentPos;
            norm_vec = norm(vec_to_center);
        
            % Normalize vectors
            obj.psi = vec_to_center / norm_vec;
            obj.psi_hat = [cos(pi/2), sin(pi/2);
                           -sin(pi/2), cos(pi/2)] * obj.psi;
        
            % Store results
            obj.projPoint = center - obj.d_des * obj.psi;
            obj.projPoint_traj(:, cur_tStep) = obj.projPoint;
            obj.d_convex_hat = norm_vec;  % Distance to circle center
        end

        % Calculating psi and psi_hat unit vectors directed from agent to 
        % closest point on minimum ellipse hull, and perpendicular to it. 
        function obj = getPsiAndProjectionMinEllipse(obj, cur_tStep, numTargets)
            obj.trajectory_type = 'ellipse';
            agentPos = obj.p;
        
            % Extract estimated target positions
            est_positions = zeros(numTargets, 2);
            for i = 1:numTargets
                est_positions(i, :) = obj.x_hat{i}(:, cur_tStep)';
            end
        
            % Compute minimum enclosing ellipse
            [center, A_matrix] = obj.minEnclosingEllipse(est_positions);
            
            % Find closest point on ellipse to agent
            projPoint = obj.closestPointOnEllipse(agentPos, center, A_matrix);
            
            % Compute psi and psi_hat
            obj.psi = projPoint - agentPos;
            norm_psi = norm(obj.psi);
            if norm_psi > 0
                obj.psi = obj.psi / norm_psi;
            else
                obj.psi = [1; 0]; % Default direction if agent is at projection point
            end
            obj.psi_hat = [cos(pi/2), sin(pi/2);
                           -sin(pi/2), cos(pi/2)] * obj.psi;
            
            % Store results
            obj.projPoint = projPoint;
            obj.projPoint_traj(:, cur_tStep) = obj.projPoint;
            obj.d_convex_hat = norm_psi;
        end

        % Calculating Control input for controller inspired by Sui et al. (2025)
        function obj = controlInputPDT(obj, t)

            % Error in distance to convex hull
            obj.d_tilde = obj.d_convex_hat - obj.d_des;

            % Adapted centrepetal component, using vectors towards and
            % normal to convex hull rather than a single target
            v_cen = 1/(obj.alpha_2 * obj.Tc2) * exp(abs(obj.d_tilde) ^ obj.alpha_2) * sig(obj.d_tilde, 1 - obj.alpha_2);

            obj.u = v_cen * obj.psi + obj.k_omega  * obj.psi_hat;
        end

        % Calculating Control input for controller from Cao et al. (2021)
        function obj = controlInputCao(obj, t)

            obj.d_tilde = obj.d_convex_hat - obj.d_des;
            v_cen = (obj.d_convex_hat - obj.d_des);

            % Ensuring safe distance
            u_safe_val = (obj.d_convex_hat > obj.rs);
            v_tan = 5 * u_safe_val;

            obj.u = v_cen * obj.psi + v_tan * obj.psi_hat;
        end

        % Moving agent with holonomic agent constraints
        function obj = move(obj, dT, t)
            obj.p_dot = obj.u;
            obj.p = obj.p + obj.p_dot * dT;
            obj.p_traj(:, t) = obj.p;
        end

        % Moving agent with non-holonomic constraints
        % Using the control input projection method from Zhao et al. (2019)
        % To control a planar unicycle model
        function obj = moveNonHolonomic(obj, dT, t)
            
            % Heading vectors
            h_i = [cos(obj.heading); sin(obj.heading)];
            h_perp = [-sin(obj.heading); cos(obj.heading)];

            % Defining f_i parameter
            f_i = obj.u;
            f_perp = f_i - (h_i' * f_i) * h_i; % perpendicular component of f_i
            
            % Calculating velocity and rotation rate inputs
            v = h_i' * f_i;
            heading_dot = h_perp' * f_perp;
           
            % Updating non-holonomic model
            x_dot = v * cos(obj.heading);
            y_dot = v * sin(obj.heading);
            theta_dot = heading_dot;
            
            % Store velocity
            obj.p_dot = [x_dot; y_dot];
            
            % Integration to find new agent pose
            obj.p(1) = obj.p(1) + x_dot * dT;
            obj.p(2) = obj.p(2) + y_dot * dT;
            obj.heading = obj.heading + theta_dot * dT;
            
            % Normalize heading to [-pi, pi]
            obj.heading = atan2(sin(obj.heading), cos(obj.heading));
            
            % Store trajectories
            obj.p_traj(:, t) = obj.p;
        end

        % Helper function to compute minimum enclosing ellipse
        function [center, A_matrix] = minEnclosingEllipse(obj, points)
            % Simple implementation using covariance-based approach
            % For a more robust solution, consider using Khachiyan's algorithm
            
            n = size(points, 1);
            if n < 2
                center = points(1, :)';
                A_matrix = eye(2);
                return;
            end
            
            % Center the points
            center = mean(points, 1)';
            centered_points = points - center';
            
            % Compute covariance matrix
            C = (centered_points' * centered_points) / (n - 1);
            
            % Scale factor to ensure all points are inside
            % Use maximum Mahalanobis distance
            max_dist = 0;
            for i = 1:n
                p = centered_points(i, :)';
                dist = sqrt(p' * inv(C) * p);
                max_dist = max(max_dist, dist);
            end
            
            % Scale the ellipse to enclose all points
            A_matrix = C * (max_dist^2 + 0.1); % Small buffer to ensure enclosure
        end
        
        % Helper function to find closest point on ellipse to external point
        function closest_point = closestPointOnEllipse(obj, external_point, center, A_matrix)
            % Iterative method to find closest point on ellipse
            % Using parametric representation and Newton's method
            
            % Initial guess - point on line from center to external point
            direction = external_point - center;
            if norm(direction) == 0
                closest_point = center;
                return;
            end
            
            % Use eigendecomposition for ellipse parameters
            [V, D] = eig(A_matrix);
            a = sqrt(D(1,1)); % Semi-axis length 1
            b = sqrt(D(2,2)); % Semi-axis length 2
            
            % Transform to ellipse coordinate system
            transformed_point = V' * direction;
            
            % Find parameter t for closest point using iterative method
            t = atan2(transformed_point(2)/b, transformed_point(1)/a);
            
            % Iterative refinement (simplified Newton's method)
            for iter = 1:10
                % Point on ellipse
                ellipse_point = [a * cos(t); b * sin(t)];
                
                % Derivative
                ellipse_deriv = [-a * sin(t); b * cos(t)];
                
                % Vector from ellipse point to external point
                diff_vec = transformed_point - ellipse_point;
                
                % Update parameter
                if norm(ellipse_deriv) > 0
                    dt = dot(diff_vec, ellipse_deriv) / dot(ellipse_deriv, ellipse_deriv);
                    t = t + 0.5 * dt; % Damped update for stability
                end
                
                % Check convergence
                if abs(dt) < 1e-6
                    break;
                end
            end
            
            % Final point on ellipse
            ellipse_point = [a * cos(t); b * sin(t)];
            
            % Transform back to original coordinate system
            closest_point = center + V * ellipse_point;
        end

        % Function to calculate true distance to ellipse for delta_traj calculation
        function true_distance = computeTrueDistanceToEllipse(obj, targets)
            % Calculate minimum enclosing ellipse for true target positions
            target_positions = targets';  % Convert to Nx2 format
            [center, A_matrix] = obj.minEnclosingEllipse(target_positions);
            
            % Find closest point on true ellipse to agent
            closest_point = obj.closestPointOnEllipse(obj.p, center, A_matrix);
            
            % Calculate distance
            true_distance = norm(obj.p - closest_point);
        end

        % Function to calculate true distance to convex hull
        function true_distance = computeTrueDistanceToHull(obj)
            hull = obj.true_hull_vertices;
            numVertices = size(hull, 1);

            min_distance = inf;

            % Going through each of the vertices to find minimum distance
            for i = 1:numVertices

                % Wrap around from last to first
                A = hull(i, :)';
                B = hull(mod(i, numVertices) + 1, :)';

                AB = B - A;
                AP = obj.p - A;

                % Project point onto edge segment
                t_param = max(0, min(1, dot(AP, AB) / dot(AB, AB)));
                proj = A + t_param * AB;
                distance = norm(obj.p - proj);

                % Finding if distance is less than min previously achieved
                if distance < min_distance
                    min_distance = distance;
                end
            end

            true_distance = min_distance;
        end
        
    end
end