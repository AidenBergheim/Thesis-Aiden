classdef AgentDeghat
 
    properties (Access = public)

        % ---- Agent state ------------------------------------------------
        p               % 2x1   current position [x; y]
        p_dot           % 2x1   current velocity
        p_traj          % 2xT   position history
        heading         % scalar heading angle (rad) — for non-holonomic model
        u               % 2x1   current control input
        u_traj          % 2xT   control input history

        % ---- Bearing measurements  (phi_i and phi_bar_i in the paper) ---
        varphi          % 1xN cell  unit bearing vectors from agent to each target
        bar_varphi      % 1xN cell  varphi{i} rotated pi/2 CCW
        varphi_traj     % 1xN cell  2xT bearing history per target

        % ---- Per-target estimators (Eq. 9) ------------------------------
        x_hat           % 1xN cell  2xT estimated target positions
        x_hat_curr      % 2xN       current-step estimates (snapshot)
        x_hat_dot       % 1xN cell  2xT estimated target velocity (dx_hat/dt)
        x_tilde_traj    % 1xN cell  2xT estimation errors  (x_hat - x_true)

        % ---- Virtual target / centroid  (Eq. 8) -------------------------
        c_hat           % 2x1  estimated centroid  p_hat_T
        c               % 2x1  TRUE centroid  (for logging / plotting only)

        % ---- Desired radius  (Eq. 4) -------------------------------------
        rho_d           % scalar  current desired radius
        rho_d_traj      % 1xT    desired radius history
        d_hat           % 1xN    || p_hat_T - p_hat_Ti || per target

        % ---- Control / estimator parameters (Deghat 2015) ---------------
        k_est           % scalar  estimator gain  (k_est > 0)
        alpha           % scalar  tangential speed (alpha in paper)
                        %         > 0 -> counterclockwise; < 0 -> clockwise
        d_buffer        % scalar  buffer distance d in Eq. (4)

        % ---- Diagnostics / logging --------------------------------------
        delta_traj      % 1xT   rho_hat(t) - rho_d(t) history
        d_tilde         % scalar current distance error
        d_convex_true   % placeholder kept for framework compatibility
        varrho_traj


    end % properties

    methods

        % =================================================================
        %  CONSTRUCTOR
        % =================================================================
        function obj = AgentDeghat(p_0, x_hat_0, k_est, alpha, d_buffer, ...
                                   tSteps, numTargets, targets, heading, ...
                                   velocity_saturation, latency)

            % --- Agent state ---
            obj.p       = p_0;
            obj.p_dot   = zeros(2, 1);
            obj.p_traj  = zeros(2, tSteps);
            obj.p_traj(:, 1) = p_0;   % FIX: seed initial position so t=1 is correct
            obj.heading = heading;
            obj.u       = zeros(2, 1);
            obj.u_traj  = zeros(2, tSteps);

            % --- Deghat 2015 parameters ---
            obj.k_est             = k_est;
            obj.alpha             = alpha;
            obj.d_buffer          = d_buffer;


            % --- True centroid (logging only) ---
            obj.c = mean(targets, 2);

            % --- Radius / control error logging ---
            obj.rho_d      = 0;
            obj.rho_d_traj = zeros(1, tSteps);
            obj.delta_traj = zeros(1, tSteps);
            obj.d_tilde    = 0;
            obj.d_convex_true = zeros(1, tSteps);

            % --- Bearing measurement storage ---
            obj.varphi      = cell(1, numTargets);
            obj.bar_varphi  = cell(1, numTargets);
            obj.varphi_traj = cell(1, numTargets);

            % --- Per-target estimator storage ---
            obj.x_hat        = cell(1, numTargets);
            obj.x_hat_dot    = cell(1, numTargets);
            obj.x_tilde_traj = cell(1, numTargets);
            obj.d_hat        = zeros(1, numTargets);
            obj.x_hat_curr   = zeros(2, numTargets);

            % --- Initialise per-target cells ---
            for i = 1:numTargets
                obj.varphi_traj{i}  = zeros(2, tSteps);
                obj.x_hat{i}        = zeros(2, tSteps);
                obj.x_hat{i}(:, 1)  = x_hat_0(:, i);
                obj.x_hat_dot{i}    = zeros(2, tSteps);
                obj.x_tilde_traj{i} = zeros(2, tSteps);
            end

            % --- Seed centroid estimate from initial target estimates ---
            obj.c_hat = mean(x_hat_0, 2);

        end 


        % Finding bearing measurements
        function obj = getBearings(obj, cur_tStep, targets)

            t          = cur_tStep;
            numTargets = size(targets, 2);

            % CCW pi/2 rotation matrix
            R_cw = [0, 1;
                      -1,  0];

            for i = 1:numTargets
                d_i = norm(targets(:, i) - obj.p);
                % phi_i — Eq. (1):
                obj.varphi{i}            = (targets(:, i) - obj.p) / d_i;
                obj.varphi_traj{i}(:, t) = obj.varphi{i};
                obj.bar_varphi{i}        = R_cw * obj.varphi{i};
            end


        end 

        %  Target estimator from Deghat et al. (Eq. 9)
        function obj = estimateTargetDeghat(obj, cur_tStep, dT, targets)
                        t          = cur_tStep;
            numTargets = size(targets, 2);
            I          = eye(2);

            latest_x_hats = zeros(2, numTargets);

            for i = 1:numTargets

                x_hat_i = obj.x_hat{i}(:, t);
                phi_i   = obj.varphi{i};

                proj        = I - phi_i * phi_i';
                x_hat_dot_i = obj.k_est * proj * (obj.p - x_hat_i);

                % Store derivative
                obj.x_hat_dot{i}(:, t) = x_hat_dot_i;

                % Forward Euler integration -> estimate at t+1
                if t + 1 <= size(obj.x_hat{i}, 2)
                    obj.x_hat{i}(:, t + 1) = x_hat_i + x_hat_dot_i * dT;
                end

                latest_x_hats(:, i) = x_hat_i;

                % Logging 
                obj.x_tilde_traj{i}(:, t) = x_hat_i - targets(:, i);
                obj.varrho_traj{i}(t) = norm(obj.p - x_hat_i);

            end

            % centroid estimate 
            obj.c_hat      = mean(latest_x_hats, 2);
            obj.x_hat_curr = latest_x_hats;

            % Desired radius — Eq. (4)
            max_dist = 0;
            for i = 1:numTargets
                dist_i = norm(obj.c_hat - latest_x_hats(:, i));
                obj.d_hat(i) = dist_i;
                if dist_i > max_dist
                    max_dist = dist_i;
                end
            end
            obj.rho_d         = max_dist + obj.d_buffer;
            obj.rho_d_traj(t) = obj.rho_d;


        end


        %  Deghat et al. controller 2015 (Eq. 13)
        function obj = controlInputDeghat(obj, t, dT)
            
            % Bearing phi to estimated virtual target — Eq. (12)
            diff    = obj.c_hat - obj.p;
            rho_hat = norm(diff);
            phi = diff / rho_hat;


            % phi rotated pi/2 counterclockwise
            R_cw   = [0, 1;
                        -1,  0];
            phi_bar = R_cw * phi;

            % Control law — Eq. (13)
            obj.d_tilde       = rho_hat - obj.rho_d;
            obj.u = obj.d_tilde * phi + obj.alpha * phi_bar;

            obj.u_traj(:, t) = obj.u;
            obj.delta_traj(t) = norm(obj.c - obj.p) - obj.rho_d;

        end


        % Move function
        function obj = move(obj, dT, t)
            idx = max(1, t);
            obj.p_dot        = obj.u_traj(:, idx);
            obj.p            = obj.p + obj.p_dot * dT;
            obj.p_traj(:, t) = obj.p;

        end



    end 

end 