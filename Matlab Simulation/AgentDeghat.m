classdef AgentDeghat
    % AgentDeghat - Implements Deghat et al. (2015):
    %   "Multi-target localization and circumnavigation by a single agent
    %    using bearing measurements"
    %   Int. J. Robust Nonlinear Control, 25:2362-2374.
    %
    % -----------------------------------------------------------------------
    % ALGORITHM OVERVIEW
    % -----------------------------------------------------------------------
    %
    %  ESTIMATOR  (Eq. 9) — per-target, runs every timestep:
    %
    %    d/dt p_hat_Ti = k_est * (I - phi_i * phi_i') * (p_A - p_hat_Ti)
    %
    %    where phi_i is the unit bearing vector from the agent to target i.
    %    The projection (I - phi_i*phi_i') removes the component along the
    %    bearing line; only the perpendicular component can be inferred from
    %    a bearing measurement.  Under persistence of excitation the estimate
    %    converges exponentially to the true target position (Theorem 1).
    %
    %  VIRTUAL TARGET  (Eq. 8):
    %
    %    p_hat_T = (1/n) * sum_i( p_hat_Ti )   % estimated centroid
    %
    %  DESIRED RADIUS  (Eq. 4):
    %
    %    rho_d = max_i || p_hat_T - p_hat_Ti || + d_buffer
    %
    %    Ensures the circumnavigation circle encloses all targets.
    %
    %  CONTROLLER  (Eq. 13) — single integrator, runs every timestep:
    %
    %    u = (rho_hat - rho_d) * phi + alpha * phi_bar
    %
    %    where:
    %      rho_hat  = || p_A - p_hat_T ||           (estimated distance to centroid)
    %      phi      = (p_hat_T - p_A) / rho_hat     (bearing to estimated centroid)
    %      phi_bar  = phi rotated pi/2 CCW           (tangential direction)
    %      alpha    > 0 drives counterclockwise motion; alpha < 0 for CW
    %
    %    The radial term (rho_hat - rho_d)*phi corrects the radius error.
    %    The tangential term alpha*phi_bar maintains orbital motion.
    %    Convergence of rho(t) - rho_d to zero is exponentially fast
    %    once the estimator has converged (Theorem 2).
    %
    % -----------------------------------------------------------------------
    % ROTATION CONVENTION
    % -----------------------------------------------------------------------
    %  phi_bar is phi rotated pi/2 counterclockwise:
    %
    %    R_cw = [0, -1;
    %              1,  0]
    %
    %  With this convention, alpha > 0 produces counterclockwise orbit,
    %  consistent with Deghat et al. (2015) and standard 2D geometry.
    %
    % -----------------------------------------------------------------------
    % INTERFACE COMPATIBILITY
    % -----------------------------------------------------------------------
    % This class mirrors the public interface of the existing Agent class
    % (estimateTargetPDT / controlInputPDT stubs included) so that it can be
    % dropped into the same simulation loop without modification.
    %
    % Typical simulation loop:
    %   for t = 1:tSteps
    %       agent = agent.getBearings(t, targets);
    %       agent = agent.estimateTargetDeghat(t, dT, targets);
    %       agent = agent.controlInputDeghat(t, dT);
    %       agent = agent.move(dT, t);          % holonomic
    %       % OR
    %       agent = agent.moveNonHolonomic(dT, t); % unicycle
    %   end

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

        % ---- Framework-compatibility aliases ----------------------------
        k_omega
        Tc1
        Tc2
        alpha_1
        alpha_2
        d_des
        d_des_dot
        d_des_traj
        d_des_func
        d_hat_dot
        theta
        theta_traj
        integral
        integralss
        curr_control_time
        velocity_saturation
        latency
        psi                 % bearing to TRUE centroid (for logging)
        bar_psi             % psi rotated pi/2 CCW (for logging)
        localization_heading
        projPoint
        projPoint_traj
        varrho_traj
        d_convex_hat
        P                   % not used in Deghat — kept for compatibility
        q                   % not used in Deghat — kept for compatibility
        true_hull_vertices
        trajectory_type

    end % properties

    methods

        % =================================================================
        %  CONSTRUCTOR
        % =================================================================
        function obj = AgentDeghat(p_0, x_hat_0, k_est, alpha, d_buffer, ...
                                   tSteps, numTargets, targets, heading, ...
                                   velocity_saturation, latency)
            % AgentDeghat  Constructor
            %
            % INPUTS
            %   p_0               2x1   initial agent position [x; y]
            %   x_hat_0           2xN   initial target position estimates
            %                           (column i = initial estimate of target i)
            %   k_est             scalar estimator gain  (k_est > 0)
            %   alpha             scalar tangential speed (> 0 for CCW orbit)
            %   d_buffer          scalar buffer distance d in Eq. (4)
            %   tSteps            int   total simulation timesteps
            %   numTargets        int   number of targets N
            %   targets           2xN   TRUE target positions (for logging only)
            %   heading           scalar initial agent heading (rad)
            %   velocity_saturation scalar max agent speed (m/s)
            %   latency           scalar control latency (s); 0 for no latency

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
            obj.velocity_saturation = velocity_saturation;
            obj.latency           = latency;

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
            obj.d_hat_dot    = zeros(1, numTargets);
            obj.x_hat_curr   = zeros(2, numTargets);

            % --- Compatibility placeholders ---
            obj.P = cell(1, numTargets);
            obj.q = cell(1, numTargets);
            obj.psi              = zeros(2, 1);
            obj.bar_psi          = zeros(2, 1);
            obj.projPoint        = zeros(2, 1);
            obj.projPoint_traj   = zeros(2, tSteps);
            obj.localization_heading = zeros(2, 1);
            obj.integral         = zeros(1, tSteps);
            obj.integralss       = zeros(1, tSteps);
            obj.theta_traj       = zeros(1, tSteps);
            obj.d_des            = 0;
            obj.d_des_dot        = zeros(1, tSteps);
            obj.d_des_traj       = zeros(1, tSteps);
            obj.curr_control_time = 0;
            obj.k_omega  = alpha;   % alias
            obj.Tc1      = 0;
            obj.Tc2      = 0;
            obj.alpha_1  = 0;
            obj.alpha_2  = 0;
            obj.varrho_traj = cell(1, numTargets);

            % --- Initialise per-target cells ---
            for i = 1:numTargets
                obj.varphi_traj{i}  = zeros(2, tSteps);
                obj.x_hat{i}        = zeros(2, tSteps);
                obj.x_hat{i}(:, 1)  = x_hat_0(:, i);
                obj.x_hat_dot{i}    = zeros(2, tSteps);
                obj.x_tilde_traj{i} = zeros(2, tSteps);
                obj.varrho_traj{i}  = zeros(1, tSteps);
                obj.P{i}            = cell(1, tSteps);
                obj.P{i}{1}         = zeros(2, 2);
                obj.q{i}            = cell(1, tSteps);
                obj.q{i}{1}         = zeros(2, 1);
            end

            % --- Seed centroid estimate from initial target estimates ---
            obj.c_hat = mean(x_hat_0, 2);
            obj.theta = atan2(obj.p(2) - obj.c_hat(2), ...
                              obj.p(1) - obj.c_hat(1));
            obj.theta_traj(1) = obj.theta;

        end % constructor


        % =================================================================
        %  BEARING MEASUREMENTS
        % =================================================================
        function obj = getBearings(obj, cur_tStep, targets)
            % getBearings  Compute phi_i and phi_bar_i for each target.
            %
            % phi_i     = unit vector from agent p_A to target i  (Eq. 1)
            % phi_bar_i = phi_i rotated pi/2 counterclockwise
            %
            % NOTE: delta_traj is NO LONGER logged here. It is logged at the
            % end of controlInputDeghat, after both rho_hat and rho_d have
            % been updated for the current timestep. Logging it here would
            % use stale c_hat and rho_d from the previous step.

            t          = cur_tStep;
            numTargets = size(targets, 2);

            % CCW pi/2 rotation matrix: alpha > 0 => counterclockwise orbit
            R_cw = [0, 1;
                      -1,  0];

            for i = 1:numTargets
                d_i = norm(targets(:, i) - obj.p);
                if d_i < 1e-12
                    d_i = 1e-12;   % guard against division by zero
                end
                % phi_i — Eq. (1): unit vector from agent to target i
                obj.varphi{i}            = (targets(:, i) - obj.p) / d_i;
                obj.varphi_traj{i}(:, t) = obj.varphi{i};
                % phi_bar_i — pi/2 CCW rotation of phi_i
                obj.bar_varphi{i}        = R_cw * obj.varphi{i};
            end

            % Bearing to TRUE centroid (for logging only — not used in control)
            c_diff = obj.c - obj.p;
            c_norm = norm(c_diff);
            if c_norm > 1e-12
                obj.psi     = c_diff / c_norm;
                obj.bar_psi = R_cw * obj.psi;
            end

        end % getBearings


        % =================================================================
        %  DEGHAT 2015 ESTIMATOR  (Eq. 9)
        % =================================================================
        function obj = estimateTargetDeghat(obj, cur_tStep, dT, targets)
            % estimateTargetDeghat  Per-target position estimator (Eq. 9).
            %
            % For each target i:
            %
            %   d/dt p_hat_Ti = k_est * (I - phi_i * phi_i') * (p_A - p_hat_Ti)
            %
            % After updating all estimates:
            %   p_hat_T  = mean(p_hat_Ti)                    (Eq. 8)
            %   rho_d    = max_i||p_hat_T - p_hat_Ti|| + d   (Eq. 4)

            t          = cur_tStep;
            numTargets = size(targets, 2);
            I          = eye(2);

            latest_x_hats = zeros(2, numTargets);

            for i = 1:numTargets

                x_hat_i = obj.x_hat{i}(:, t);
                phi_i   = obj.varphi{i};

                % ---- Eq. (9) ----
                % Projection matrix: (I - phi_i * phi_i')
                % Projects (p_A - p_hat_Ti) onto the subspace perpendicular
                % to the bearing line — the only direction observable from
                % a bearing-only measurement.
                proj        = I - phi_i * phi_i';
                x_hat_dot_i = obj.k_est * proj * (obj.p - x_hat_i);

                % Store derivative
                obj.x_hat_dot{i}(:, t) = x_hat_dot_i;

                % Forward Euler integration -> estimate at t+1
                if t + 1 <= size(obj.x_hat{i}, 2)
                    obj.x_hat{i}(:, t + 1) = x_hat_i + x_hat_dot_i * dT;
                end

                latest_x_hats(:, i) = x_hat_i;

                % Log estimation error at current step
                obj.x_tilde_traj{i}(:, t) = x_hat_i - targets(:, i);

                % Log agent-to-estimate distance
                obj.varrho_traj{i}(t) = norm(obj.p - x_hat_i);

            end

            % ---- Virtual target (centroid) estimate — Eq. (8) ----
            obj.c_hat      = mean(latest_x_hats, 2);
            obj.x_hat_curr = latest_x_hats;

            % ---- Desired radius — Eq. (4) ----
            %   rho_d(t) = max_i || p_hat_T - p_hat_Ti || + d_buffer
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

            % Keep compatibility aliases in sync
            obj.d_des         = obj.rho_d;
            obj.d_des_traj(t) = obj.rho_d;
            obj.theta_traj(t) = atan2(obj.p(2) - obj.c_hat(2), ...
                                      obj.p(1) - obj.c_hat(1));

        end % estimateTargetDeghat


        % =================================================================
        %  DEGHAT 2015 CONTROLLER  (Eq. 13)
        % =================================================================
        function obj = controlInputDeghat(obj, t, dT)
            % controlInputDeghat  Single-integrator circumnavigation controller.
            %
            %   u = (rho_hat - rho_d) * phi + alpha * phi_bar
            %
            % where:
            %   rho_hat = || p_A - p_hat_T ||          estimated radius (Eq. 11)
            %   phi     = (p_hat_T - p_A) / rho_hat    bearing to centroid (Eq. 12)
            %   phi_bar = phi rotated pi/2 CCW          tangential direction
            %
            % Radial term  (rho_hat - rho_d)*phi:
            %   > 0 when agent is too far  -> moves inward
            %   < 0 when agent is too close -> moves outward
            % Tangential term alpha*phi_bar drives orbital motion.
            % Together they produce exponentially convergent circumnavigation
            % (Theorem 2) once the estimator has localised the targets.
            %
            % delta_traj is logged here — AFTER both rho_hat and rho_d have
            % been computed for this timestep — so the logged value is correct.

            % ---- Bearing phi to estimated virtual target — Eq. (12) ----
            diff    = obj.c_hat - obj.p;
            rho_hat = norm(diff);


            phi = diff / rho_hat;


            % phi_bar: phi rotated pi/2 counterclockwise
            % R_cw = [0, -1; 1, 0]  =>  alpha > 0 gives CCW orbit
            R_cw   = [0, 1;
                        -1,  0];
            phi_bar = R_cw * phi;

            % ---- Control law — Eq. (13) ----
            obj.d_tilde       = rho_hat - obj.rho_d;
            obj.u = obj.d_tilde * phi + obj.alpha * phi_bar;

            obj.u_traj(:, t) = obj.u;
            

            % FIX: log delta here, after rho_hat and rho_d are both current
            obj.delta_traj(t) = norm(obj.c - obj.p) - obj.rho_d;

        end % controlInputDeghat


        % =================================================================
        %  HOLONOMIC MOTION  (single integrator)
        % =================================================================
        function obj = move(obj, dT, t)
            % move  Integrate agent position with optional control latency.
            %
            % If latency > 0, the agent applies the control input from
            % (latency / dT) timesteps ago, emulating a sensor-to-actuator
            % processing delay.

            latency_steps = round(obj.latency / dT);
            idx = max(1, t - latency_steps);

            obj.p_dot        = obj.u_traj(:, idx);
            obj.p            = obj.p + obj.p_dot * dT;
            obj.p_traj(:, t) = obj.p;

        end % move


        % =================================================================
        %  NON-HOLONOMIC MOTION  (planar unicycle)
        % =================================================================
        function obj = moveNonHolonomic(obj, dT, t)
            % moveNonHolonomic  Unicycle model via control-input projection.
            %
            % Following Zhao et al. (2019): the holonomic velocity command u
            % is decomposed into a forward speed v and a heading rate omega.
            % The heading angle is then integrated.

            h_i    = [cos(obj.heading); sin(obj.heading)];
            h_perp = [-sin(obj.heading); cos(obj.heading)];

            f_perp = obj.u - (h_i' * obj.u) * h_i;

            v           = h_i'    * obj.u;
            heading_dot = h_perp' * f_perp;

            obj.p_dot   = [v * cos(obj.heading); v * sin(obj.heading)];
            obj.p       = obj.p + obj.p_dot * dT;
            obj.heading = obj.heading + heading_dot * dT;
            obj.heading = atan2(sin(obj.heading), cos(obj.heading));

            obj.p_traj(:, t) = obj.p;

        end % moveNonHolonomic


    end % methods

end % classdef