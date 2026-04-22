classdef CaoAgent

    properties (Access = public)

        % ---- Agent state ----
            p                   % (2x1) current agent position y(t)
            p_dot               % (2x1) current agent velocity y_dot(t)
            p_traj              % (2xT) position history
            u_traj              % (2xT) control input history
            heading             % scalar: heading angle (rad)
            velocity_saturation % scalar: max speed (m/s)
            latency             % scalar: control latency (s)

        % ---- Bearing vectors (Notation 1) ----
            varphi              % (1xN cell) phi_i: unit vector from agent to target i
            bar_varphi          % (1xN cell) phi_bar_i: phi_i rotated pi/2 CW

        % ---- Scalar estimator state (Eq. 1) ----
            rho_hat             % (1xN) estimated distance to each target
            rho_true            % (1xN) true distance (for logging only)
            rho_tilde           % (1xN) estimation error rho_hat - rho_true

        % ---- Estimated target positions ----
            x_hat               % (1xN cell) each cell is (2xT) estimated positions over time

        % ---- Convex hull variables (Notation 6) ----
            psi_hat             % (2x1) unit vector from agent to est. hull (psi_hat)
            bar_psi_hat         % (2x1) psi_hat rotated pi/2 CW (psi_bar_hat)
            D_hat               % scalar: distance from agent to est. hull
            projPoint           % (2x1) closest point on est. hull

        % ---- True convex hull variables (for logging) ----
            D_true              % scalar: distance from agent to TRUE hull
            psi_true            % (2x1) unit vector from agent to true hull

        % ---- Control parameters (Eq. 2) ----
            k_gain              % scalar: radial gain k > 0
            d_des               % scalar: desired enclosing distance d
            alpha_speed         % scalar: tangential speed alpha
            rs                  % scalar: safety distance rs
            u                   % (2x1) current velocity command

        % ---- True targets info ----
            c                   % (2x1) true centroid (for reference)

        % ---- Logging ----
            delta_traj          % (1xT) tracking error: D_true - d_des
            D_hat_traj          % (1xT) estimated hull distance over time
            D_true_traj         % (1xT) true hull distance over time
            x_tilde_traj        % (1xN cell) each (2xT) position estimation error
            rho_tilde_traj      % (NxT) scalar estimation errors over time
            projPoint_traj      % (2xT) projection point history
            localization_heading % (2x1) initial heading for localisation phase

    end

    methods

        % Constructor
        function obj = CaoAgent(p_0, x_hat_0, k_gain, d_des, alpha_speed, rs, ...
                                  tSteps, numTargets, targets, heading, ...
                                  velocity_saturation, latency)


            obj.p                   = p_0(:);
            obj.p_dot               = zeros(2, 1);
            obj.heading             = heading;
            obj.velocity_saturation = velocity_saturation;
            obj.latency             = latency;
            obj.u                   = zeros(2, 1);

            % Control parameters
            obj.k_gain      = k_gain;
            obj.d_des       = d_des;
            obj.alpha_speed = alpha_speed;
            obj.rs          = rs;

            % True centroid
            obj.c = mean(targets, 2);

            % Initialise bearing vectors
            obj.varphi     = cell(1, numTargets);
            obj.bar_varphi = cell(1, numTargets);

            % Initialise scalar estimator (Assumption 19: rs <= rho_hat_i(0) <= rho_i(0))
            obj.rho_hat   = zeros(1, numTargets);
            obj.rho_true  = zeros(1, numTargets);
            obj.rho_tilde = zeros(1, numTargets);

            for i = 1:numTargets
                obj.rho_true(i)  = norm(targets(:, i) - p_0);
                % Use distance from p_0 to initial estimate, clamped to [rs, rho_true]
                rho_init = norm(x_hat_0(:, i) - p_0);
                obj.rho_hat(i) = max(rs, min(rho_init, obj.rho_true(i)));
            end

            % Trajectory logs
            obj.p_traj         = zeros(2, tSteps);
            obj.u_traj         = zeros(2, tSteps);
            obj.delta_traj     = zeros(1, tSteps);
            obj.D_hat_traj     = zeros(1, tSteps);
            obj.D_true_traj    = zeros(1, tSteps);
            obj.projPoint_traj = zeros(2, tSteps);
            obj.rho_tilde_traj = zeros(numTargets, tSteps);

            % Per-target estimate logs
            obj.x_hat        = cell(1, numTargets);
            obj.x_tilde_traj = cell(1, numTargets);
            for i = 1:numTargets
                obj.x_hat{i}        = zeros(2, tSteps);
                obj.x_tilde_traj{i} = zeros(2, tSteps);
                % Store initial estimate: x_hat_i = y + rho_hat_i * phi_i
                diff = targets(:, i) - p_0;
                phi_i = diff / norm(diff);
                obj.x_hat{i}(:, 1) = p_0 + obj.rho_hat(i) * phi_i;
            end

            % Hull variables
            obj.D_hat       = 0;
            obj.D_true      = 0;
            obj.psi_hat     = zeros(2, 1);
            obj.bar_psi_hat = zeros(2, 1);
            obj.psi_true    = zeros(2, 1);
            obj.projPoint   = zeros(2, 1);

            % Localisation heading (for compatibility)
            obj.localization_heading = zeros(2, 1);
        end


        % Bearing measurements (Notation 1)
        function obj = getBearings(obj, cur_tStep, targets)

            numTargets = size(targets, 2);
            for i = 1:numTargets
                diff = targets(:, i) - obj.p;
                d_i  = norm(diff);
                obj.rho_true(i) = d_i;
                if d_i < 1e-9
                    continue
                end
                phi_i = diff / d_i;
                obj.varphi{i}     = phi_i;
                % Clockwise pi/2 rotation:
                obj.bar_varphi{i} = [phi_i(2); -phi_i(1)];
            end
        end


        % Scalar distance estimator (Eq. 1)
        function obj = estimateTargetsCao(obj, cur_tStep, dT, targets)

            t = cur_tStep;
            numTargets = length(obj.varphi);
            y_dot = obj.p_dot;  % velocity from previous step

            for i = 1:numTargets
                phi_i     = obj.varphi{i};
                phi_bar_i = obj.bar_varphi{i};

                % Components of agent velocity along bearing directions
                v_i     = phi_i' * y_dot;       % radial component
                v_bar_i = phi_bar_i' * y_dot;   % tangential component

                rho_true_i = obj.rho_true(i);
                if rho_true_i < 1e-9
                    continue
                end
                
                % Eq. (4): rho_hat_dot = -v_i - |v_bar_i| * rho_tilde / rho_true
                rho_tilde_i = obj.rho_hat(i) - rho_true_i;
                rho_hat_dot = -v_i - abs(v_bar_i) * rho_tilde_i / rho_true_i;
                
                % Integrate rho_hat directly
                obj.rho_hat(i) = obj.rho_hat(i) + rho_hat_dot * dT;
                obj.rho_tilde(i) = obj.rho_hat(i) - rho_true_i;

                % Reconstruct estimated position: x_hat_i = y + rho_hat_i * phi_i
                obj.x_hat{i}(:, t) = obj.p + obj.rho_hat(i) * obj.varphi{i};

                % Log estimation error
                obj.x_tilde_traj{i}(:, t) = obj.x_hat{i}(:, t) - targets(:, i);
                obj.rho_tilde_traj(i, t) = obj.rho_tilde(i);
            end
        end

        % Compute psi_hat and D_hat (Notation 6, Definition 5)
        function obj = getPsiAndProjectionHull(obj, cur_tStep, numTargets, targets)
            t = cur_tStep;

            % --- Estimated hull projection ---
            est_positions = zeros(numTargets, 2);
            for i = 1:numTargets
                est_positions(i, :) = obj.x_hat{i}(:, t)';
            end

            [obj.D_hat, obj.projPoint, obj.psi_hat, obj.bar_psi_hat] = ...
                projectOntoHull(obj.p, est_positions);

            obj.projPoint_traj(:, t) = obj.projPoint;
            obj.D_hat_traj(t) = obj.D_hat;

            % --- True hull projection (for logging only) ---
            true_positions = zeros(numTargets, 2);
            for i = 1:numTargets
                true_positions(i, :) = targets(:, i)';
            end

            [D_true_val, ~, psi_true_val, ~] = projectOntoHull(obj.p, true_positions);
            obj.D_true = D_true_val;
            obj.psi_true = psi_true_val;
            obj.D_true_traj(t) = D_true_val;

            % Log tracking error: D_true - d_des
            obj.delta_traj(t) = obj.D_true - obj.d_des;
        end


        % Control algorithm (Eq. 2)

        function obj = controlInputCao(obj, t)
            % Radial component: drives D_hat toward d_des
            v_radial = obj.k_gain * (obj.D_hat - obj.d_des) * obj.psi_hat;

            % Tangential component: only active when safely outside hull
            % u(D_hat - rs) = 1 if D_hat >= rs, else 0
            if obj.D_hat >= obj.rs
                u_safe = 1;
            else
                u_safe = 0;
            end
            v_tangential = obj.alpha_speed * u_safe * obj.bar_psi_hat;

            % Combined control
            obj.u = v_radial + v_tangential;

            % Velocity saturation
            if norm(obj.u) > obj.velocity_saturation
                obj.u = obj.velocity_saturation * obj.u / norm(obj.u);
            end

            obj.u_traj(:, t) = obj.u;
        end

        % Move: 
        function obj = move(obj, dT, t)
            obj.p_dot = obj.u;
            obj.p     = obj.p + obj.p_dot * dT;
            obj.p_traj(:, t) = obj.p;
        end

    end
end


% project point onto convex hull of a point set
function [dist_val, proj_point, psi_vec, psi_bar_vec] = projectOntoHull(agent_pos, points)
    M = size(points, 1);

    if M == 1
        % Single target: hull is just the point
        proj_point = points(1, :)';
        diff = proj_point - agent_pos;
        dist_val = norm(diff);
        if dist_val < 1e-9
            psi_vec     = [1; 0];
            psi_bar_vec = [0; -1];
        else
            psi_vec     = diff / dist_val;
            psi_bar_vec = [psi_vec(2); -psi_vec(1)];
        end
        return
    end

    if M == 2
        % Two targets
        [proj_point, dist_val] = projectOntoSegment(agent_pos, points(1,:)', points(2,:)');
        diff = proj_point - agent_pos;
        if dist_val < 1e-9
            psi_vec     = [1; 0];
            psi_bar_vec = [0; -1];
        else
            psi_vec     = diff / dist_val;
            psi_bar_vec = [psi_vec(2); -psi_vec(1)];
        end
        return
    end

    % Build convex hull
    K = convhull(points(:,1), points(:,2));
    hull = points(K, :);

    % Find closest point on hull boundary to agent
    min_distance = inf;
    best_proj = hull(1, :)';

    for i = 1:(size(hull, 1) - 1)
        A = hull(i, :)';
        B = hull(i+1, :)';
        [proj, d] = projectOntoSegment(agent_pos, A, B);
        if d < min_distance
            min_distance = d;
            best_proj = proj;
        end
    end

    % Check if agent is INSIDE the hull (distance = 0)
    if inpolygon(agent_pos(1), agent_pos(2), hull(:,1), hull(:,2))
        dist_val = min_distance;
        proj_point = best_proj;
    else
        dist_val = min_distance;
        proj_point = best_proj;
    end

    % Compute unit vectors
    diff = proj_point - agent_pos;
    if dist_val < 1e-9
        psi_vec     = [1; 0];
        psi_bar_vec = [0; -1];
    else
        psi_vec     = diff / dist_val;
        % Clockwise pi/2 rotation
        psi_bar_vec = [psi_vec(2); -psi_vec(1)];
    end
end


function [proj, dist_val] = projectOntoSegment(point, A, B)
    % Project a point onto a line segment [A, B].
    AB = B - A;
    AP = point - A;
    t = dot(AP, AB) / (dot(AB, AB) + 1e-15);
    t = max(0, min(1, t));
    proj = A + t * AB;
    dist_val = norm(point - proj);
end