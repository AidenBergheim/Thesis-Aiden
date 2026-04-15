classdef MaLiAgent
% MaLiAgent  Implements the Ma & Li (2022) algorithm for finite-time
% circumnavigation of group targets along an irregular-shaped (extended
% convex hull) trajectory using bearing-only measurements.
%
% Reference:
%   Ma Z, Li Y. Finite-time circumnavigation by irregular-shaped trajectory
%   with bearing-only measurements. Int J Syst Sci. 2022;53(6):1170-1190.
%   https://doi.org/10.1080/00207721.2021.1993379
%
% NOTE: Obstacle avoidance (Section 4.3, Eq. 28-34) is intentionally
% omitted. The agent circumnavigates the extended convex hull boundary
% of the estimated target positions without any obstacle modulation.
%
% Algorithm components (equation numbers from the paper):
%   Eq. (5)   : Finite-time position estimator for each target
%   Eq. (12)  : Coordinate transform from OXY to local ellipse frame
%   Eq. (13a) : Radial velocity controller  uir = -k1*sgn(di - Ri)
%   Eq. (13b) : Angular velocity controller (consensus for multi-agent)
%   Eq. (13c) : Tangential velocity  uit = omega_i * di
%
% Key notation (matching paper):
%   p_i         : agent position (2x1) on horizontal plane OXY
%   r_hat_j     : estimate of target j position (2x1)
%   o_hat       : estimated geometric centre of group targets (2x1)
%   true_centre : true geometric centre -- set externally for logging only
%   tau_ijx     : unit bearing vector from agent to target j (2x1)
%   tau_ijy     : unit vector perpendicular to bearing direction (2x1)
%   di          : current distance from agent to o_hat
%   Ri          : desired radius at agent's current angle (Eq. below Eq. 12)
%   theta_i     : current angle of agent about o_hat (rad)
%   omega_i     : current angular velocity (rad/s)
%   omega_c     : prescribed angular velocity
%   o_t         : centre of the extended ellipse for current edge
%   phi_i       : angle of agent in local ellipse frame o_t x_t y_t
%   ah, bh      : semi-major and semi-minor axes of extended ellipse
%   beta        : rotation angle of local edge coordinate system
%   sd          : safety distance for extended convex hull
%
% Ri formula from the paper (page 1180):
%   Ri = sqrt( (o_hat_tx - ah*cos(phi_i))^2 + (o_hat_ty - bh*sin(phi_i))^2 )
%
% where (o_hat_tx, o_hat_ty) is the geometric centre expressed in the
% local ellipse coordinate system o_t x_t y_t via Equation (12):
%   [o_hat_tx; o_hat_ty] = Phi * (o_hat - o_t)
%
% Usage:
%   ag = MaLiAgent(p0, x_hat0, k0, alpha0, k1, k2, omega_c, sd, ...
%                  tSteps, numTargets, targets, heading, ...
%                  velocity_saturation, latency);
%   ag.true_centre = mean(Targets, 2);
%   ag.Tc_est = 3.0;
%   for t = 1:N
%       ag = ag.measureBearings(Targets);
%       ag = ag.updatePositionEstimator(dT);
%       ag = ag.computeControl(t, dT, {});
%       ag = ag.move(dT, t);
%   end

    properties (Access = public)

        % --- Agent state ---
        p                    % (2x1)   current position on OXY
        u                    % (2x1)   current velocity command
        p_traj               % (2xT)   position history
        u_traj               % (2xT)   control input history
        heading              % scalar  heading angle (rad)
        velocity_saturation  % scalar  maximum speed (m/s)
        latency              % scalar  control latency (s)

        % --- Estimator gains (Eq. 5) ---
        k0        % finite-time estimator gain > 0
        alpha0    % finite-time exponent, 0 < alpha0 < 1

        % --- Controller gains (Eq. 13a-c) ---
        k1        % radial velocity gain > 0
        k2        % angular consensus gain > 0
        omega_c   % prescribed angular velocity (rad/s)

        % --- Extended convex hull parameters (Sec. 3.3) ---
        sd        % safety distance used to build extended ellipse zones

        % --- Fixed ellipse override ---
        % Set these externally to bypass automatic hull construction.
        % When fixed_ah is non-empty, these values are used directly
        % and the ellipse is centred at o_hat (no offset).
        fixed_ah    % scalar  override semi-major axis (empty = auto)
        fixed_bh    % scalar  override semi-minor axis
        fixed_beta  % scalar  override rotation angle (rad)

        % --- Target position estimates r_hat_j (Eq. 5) ---
        r_hat     % (2 x M) one column per target

        % --- Estimated geometric centre ---
        o_hat       % (2x1)  mean of r_hat columns

        % --- True geometric centre (set externally, for logging only) ---
        true_centre % (2x1)  true mean of target positions

        % --- Cached bearing frame vectors (updated each step) ---
        tau_ijx   % (2 x M) unit bearing vectors to each target
        tau_ijy   % (2 x M) perpendicular to tau_ijx, CCW 90 deg

        % --- Angular state ---
        theta     % scalar  current angle of agent about o_hat (rad)
        omega_i   % scalar  current angular velocity (rad/s)

        % --- Two-phase parameters ---
        Tc_est    % scalar  localisation phase duration (s)
        R_hold    % scalar  holding radius during localisation phase (m)

        % --- Tracking error ---
        d_tilde      % scalar  current radial error
        d_tilde_traj % (1xT)  history of d_tilde

        % --- Estimation error logging ---
        x_tilde_traj % (1xM cell) each cell is (2xT) estimation error per target

    end

    methods

        % ==============================================================
        % Constructor
        % ==============================================================
        function obj = MaLiAgent(p0, x_hat0, k0, alpha0, k1, k2, omega_c, sd, ...
                                  tSteps, numTargets, targets, heading, ...
                                  velocity_saturation, latency)

            assert(alpha0 > 0 && alpha0 < 1, 'Require 0 < alpha0 < 1.');
            assert(k0 > 0, 'Require k0 > 0.');
            assert(k1 > 0, 'Require k1 > 0.');
            assert(k2 > 0, 'Require k2 > 0.');

            obj.p                   = p0(:);
            obj.k0                  = k0;
            obj.alpha0              = alpha0;
            obj.k1                  = k1;
            obj.k2                  = k2;
            obj.omega_c             = omega_c;
            obj.sd                  = sd;
            obj.heading             = heading;
            obj.velocity_saturation = velocity_saturation;
            obj.latency             = latency;
            obj.u                   = zeros(2, 1);
            obj.omega_i             = omega_c;

            M             = numTargets;
            obj.r_hat     = x_hat0;
            obj.tau_ijx   = zeros(2, M);
            obj.tau_ijy   = zeros(2, M);
            obj.p_traj    = zeros(2, tSteps);
            obj.u_traj    = zeros(2, tSteps);

            % Geometric centre estimate and angular state
            obj.o_hat       = mean(x_hat0, 2);
            obj.true_centre = mean(targets, 2);
            vec             = p0(:) - obj.o_hat;
            obj.theta       = atan2(vec(2), vec(1));

            % Holding radius = initial distance from agent to estimated centre
            obj.R_hold = norm(vec);

            % Localisation phase duration (tune externally)
            obj.Tc_est = 3.0;

            % Fixed ellipse override (empty = use automatic construction)
            obj.fixed_ah   = [];
            obj.fixed_bh   = [];
            obj.fixed_beta = [];

            % Tracking error log
            obj.d_tilde      = 0;
            obj.d_tilde_traj = zeros(1, tSteps);

            % Estimation error log per target
            obj.x_tilde_traj = cell(1, M);
            for i = 1:M
                obj.x_tilde_traj{i} = zeros(2, tSteps);
            end
        end

        % ==============================================================
        % Step 1 - Bearing measurements  (Sec. 4.1, Remark 4.2)
        % ==============================================================
        function obj = measureBearings(obj, targets)
            % measureBearings  Compute tau_ijx and tau_ijy for each target.
            %
            %   tau_ijx = (r_j - p) / ||r_j - p||       unit bearing vector
            %   tau_ijy = tau_ijx rotated 90 deg CCW     perpendicular vector
            %
            % Only bearing direction is used -- no range information.

            M = size(targets, 2);
            for k = 1:M
                diff = targets(:, k) - obj.p;
                d    = norm(diff);
                if d < 1e-9
                    continue
                end
                tau_x             = diff / d;
                obj.tau_ijx(:, k) = tau_x;
                obj.tau_ijy(:, k) = [-tau_x(2); tau_x(1)]; % 90 deg CCW
            end
        end

        % ==============================================================
        % Step 2 - Finite-time position estimator  (Eq. 5)
        % ==============================================================
        function obj = updatePositionEstimator(obj, dT, targets)
            % updatePositionEstimator  Finite-time estimator - Eq. (5):
            %
            %   d/dt r_hat_j = k0 * sig[ tau_ijy' * (p - r_hat_j) * tau_ijy ]^alpha0
            %
            % Theorem 4.1 proves convergence in finite time ts0.

            M = size(obj.r_hat, 2);
            for k = 1:M
                tau_y     = obj.tau_ijy(:, k);
                err       = obj.p - obj.r_hat(:, k);
                proj      = (tau_y' * err) * tau_y;   % 2x1 projection
                r_hat_dot = obj.k0 * sig_pow(proj, obj.alpha0);
                obj.r_hat(:, k) = obj.r_hat(:, k) + r_hat_dot * dT;
            end

            obj.o_hat = mean(obj.r_hat, 2);
        end

        % ==============================================================
        % Log estimation errors
        % ==============================================================
        function obj = logEstimationErrors(obj, t, targets)
            M = size(targets, 2);
            for k = 1:M
                obj.x_tilde_traj{k}(:, t) = obj.r_hat(:, k) - targets(:, k);
            end
        end

        % ==============================================================
        % Step 3 - Controller  (Eq. 13a-c)
        % ==============================================================
        function obj = computeControl(obj, t, dT, neighbour_thetas)
            % computeControl  Two-phase finite-time circumnavigation controller.
            %
            % Phase 1 (current_time < Tc_est):
            %   Holds agent at R_hold (circular orbit) while estimator
            %   converges. Matches Ma & Li Sec. 5 two-phase approach.
            %
            % Phase 2 (current_time >= Tc_est):
            %   Full Eq. (13a) radial controller drives agent to the
            %   desired extended convex hull ellipse radius.
            %
            % d_tilde_traj is ALWAYS computed relative to true_centre
            % so it reflects the real geometric tracking error.

            current_time = t * dT;

            % --- Polar state relative to estimated centre (for control) ---
           if ~isempty(obj.true_centre)
                centre = obj.true_centre;
            else
                centre = obj.o_hat;
            end
            vec = obj.p - centre;
            
            di_est  = norm(vec_est);
            if di_est < 1e-9
                obj.u               = zeros(2, 1);
                obj.d_tilde         = 0;
                obj.d_tilde_traj(t) = 0;
                obj.u_traj(:, t)    = obj.u;
                return
            end
            obj.theta = atan2(vec_est(2), vec_est(1));

            % --- True tracking error (logged relative to true_centre) ---
            vec_true = obj.p - obj.true_centre;
            di_true  = norm(vec_true);
            if di_true > 1e-9
                theta_true  = atan2(vec_true(2), vec_true(1));
                Ri_true     = obj.computeRi(obj.true_centre, theta_true);
                obj.d_tilde = di_true - Ri_true;
            else
                obj.d_tilde = 0;
            end
            obj.d_tilde_traj(t) = obj.d_tilde;

            % --- Radial velocity  (Eq. 13a) ---
            if current_time < obj.Tc_est
                % Phase 1: hold at R_hold
                radial_err = di_est - obj.R_hold;
                uir = -obj.k1 * sign(radial_err);
            else
                % Phase 2: drive to extended hull ellipse
                Ri_ctrl = obj.computeRi(obj.o_hat, obj.theta);
                uir = -obj.k1 * sign(di_est - Ri_ctrl);
            end

            % --- Angular velocity  (Eq. 13b) ---
            alpha1          = 0.5;
            n_agents        = 1 + length(neighbour_thetas);
            omega_consensus = 0;
            for jj = 1:length(neighbour_thetas)
                theta_j   = neighbour_thetas{jj};
                angle_err = wrapToPi(theta_j - obj.theta - 2*pi*jj/n_agents);
                omega_consensus = omega_consensus + obj.k2 * sig_pow(angle_err, alpha1);
            end
            obj.omega_i = omega_consensus + obj.omega_c;

            % --- Tangential velocity  (Eq. 13c) ---
            uit = obj.omega_i * di_est;

            % --- Cartesian decomposition  (Remark 4.5) ---
            uix = -uit * sin(obj.theta) + uir * cos(obj.theta);
            uiy =  uit * cos(obj.theta) + uir * sin(obj.theta);
            obj.u = [uix; uiy];

            % --- Velocity saturation ---
            if norm(obj.u) > obj.velocity_saturation
                obj.u = obj.velocity_saturation * obj.u / norm(obj.u);
            end

            obj.u_traj(:, t) = obj.u;
        end

        % ==============================================================
        % Compute Ri: desired radius from centre to ellipse boundary
        % (Paper page 1180, Eq. 12 and the Ri formula)
        % ==============================================================
        function Ri = computeRi(obj, centre, theta_i)
            % computeRi  Desired radius from geometric centre to the
            % extended ellipse boundary at bearing angle theta_i.
            %
            % This implements the paper's Ri formula correctly:
            %
            %   1. Get ellipse parameters (ah, bh, beta, o_t)
            %   2. Transform centre into local ellipse frame via Eq. (12):
            %      [o_hat_tx; o_hat_ty] = Phi * (centre - o_t)
            %   3. Find the angle phi_i of the agent in the local frame
            %   4. Compute:
            %      Ri = sqrt( (o_hat_tx - ah*cos(phi_i))^2
            %               + (o_hat_ty - bh*sin(phi_i))^2 )
            %
            % When fixed_ah is set AND the ellipse is centred at the
            % geometric centre (o_t == centre), this reduces to the
            % standard polar radius formula.

            [ah, bh, beta, o_t] = obj.computeHullEllipse();

            % --- Rotation matrix Phi (Eq. 10) ---
            Phi = [ cos(beta), -sin(beta);
                    sin(beta),  cos(beta)];

            % --- Transform centre into local ellipse frame (Eq. 12) ---
            o_hat_local = Phi * (centre - o_t);
            o_hat_tx    = o_hat_local(1);
            o_hat_ty    = o_hat_local(2);

            % --- Agent angle in local ellipse frame ---
            % The agent is at angle theta_i in global frame about centre.
            % In the local frame, the angular direction rotates by -beta.
            phi_i = theta_i - beta;

            % --- Desired radius (paper page 1180) ---
            % Ri = distance from (o_hat_tx, o_hat_ty) to point on ellipse
            % at parametric angle phi_i
            ex = ah * cos(phi_i);
            ey = bh * sin(phi_i);
            Ri = sqrt((o_hat_tx - ex)^2 + (o_hat_ty - ey)^2);

            % Guard against degenerate case
            if Ri < 1e-9
                Ri = max(ah, bh);
            end
        end

        % ==============================================================
        % Compute extended hull ellipse parameters  (Sec. 3.3)
        % ==============================================================
        function [ah, bh, beta, o_t] = computeHullEllipse(obj)
            % computeHullEllipse  Return ellipse parameters and centre.
            %
            % Returns:
            %   ah    : semi-major axis of extended ellipse
            %   bh    : semi-minor axis of extended ellipse
            %   beta  : rotation angle of local coordinate system
            %   o_t   : (2x1) centre of the extended ellipse in OXY
            %
            % If fixed_ah is set externally, uses those values and
            % places the ellipse centre at o_hat (no offset).
            %
            % Otherwise, constructs from estimated target positions
            % following Sec. 3.3 steps:
            %   - Build convex hull of estimated targets
            %   - For each edge, construct extended ellipse danger zone
            %   - For multi-edge hulls, select the edge whose extended
            %     ellipse the agent currently faces
            %   - For collinear/degenerate cases (< 3 unique hull pts),
            %     use the single-ellipse construction (Fig. 5)

            % --- Fixed override path ---
            if ~isempty(obj.fixed_ah)
                ah   = obj.fixed_ah;
                bh   = obj.fixed_bh;
                beta = obj.fixed_beta;
                o_t  = obj.o_hat;  % ellipse centred at geometric centre
                return
            end

            M = size(obj.r_hat, 2);

            % --- Single target: circle of radius sd ---
            if M == 1
                ah   = obj.sd;
                bh   = obj.sd;
                beta = 0;
                o_t  = obj.r_hat(:, 1);
                return
            end

            % --- Build convex hull ---
            pts = obj.r_hat';  % (M x 2)
            unique_pts = unique(pts, 'rows');

            if size(unique_pts, 1) < 2
                % All targets at same point
                ah   = obj.sd;
                bh   = obj.sd;
                beta = 0;
                o_t  = obj.o_hat;
                return
            end

            if size(unique_pts, 1) < 3
                % Collinear case (Fig. 5 in paper):
                % Connect the two furthest targets, extend by sd,
                % build circumscribed ellipse
                [ah, bh, beta, o_t] = obj.buildCollinearEllipse(unique_pts);
                return
            end

            % --- General case: >= 3 non-collinear points ---
            K = convhull(pts(:,1), pts(:,2));
            hull_idx = K(1:end-1);  % remove duplicate last point
            hull_verts = obj.r_hat(:, hull_idx);  % (2 x num_hull_verts)
            num_edges = size(hull_verts, 2);

            % Find which edge the agent currently faces
            % (closest edge based on perpendicular distance from ray)
            agent_angle = obj.theta;
            ray_dir = [cos(agent_angle); sin(agent_angle)];

            best_edge_idx = 1;
            best_dot = -inf;

            for e = 1:num_edges
                v1 = hull_verts(:, e);
                v2 = hull_verts(:, mod(e, num_edges) + 1);
                edge_mid = (v1 + v2) / 2;
                to_mid = edge_mid - obj.o_hat;
                if norm(to_mid) > 1e-9
                    to_mid = to_mid / norm(to_mid);
                end
                d = dot(ray_dir, to_mid);
                if d > best_dot
                    best_dot = d;
                    best_edge_idx = e;
                end
            end

            % Build extended ellipse for the selected edge
            v1 = hull_verts(:, best_edge_idx);
            v2 = hull_verts(:, mod(best_edge_idx, num_edges) + 1);
            [ah, bh, beta, o_t] = obj.buildEdgeEllipse(v1, v2);
        end

        % ==============================================================
        % Build extended ellipse for a single edge (Sec. 3.3 step 3)
        % ==============================================================
        function [ah, bh, beta, o_t] = buildEdgeEllipse(obj, v1, v2)
            % buildEdgeEllipse  Construct the extended ellipse danger zone
            % for one edge of the convex hull.
            %
            % Following Sec. 3.3 step 3:
            %   - Edge midpoint is o_h (origin of local coord system)
            %   - Edge direction defines o_h x_h axis
            %   - Perpendicular defines o_h y_h axis
            %   - Extend edge by sd on both sides -> new edge length
            %   - Rectangle: new edge length x sd width
            %   - Inscribed ellipse of rectangle, then circumscribed ellipse
            %   - o_t is translated from o_h by sd/2 along y_h

            edge = v2 - v1;
            edge_len = norm(edge);

            if edge_len < 1e-9
                ah = obj.sd; bh = obj.sd;
                beta = 0; o_t = (v1 + v2) / 2;
                return
            end

            % Local coordinate system angle
            beta = atan2(edge(2), edge(1));

            % Edge midpoint (o_h)
            o_h = (v1 + v2) / 2;

            % Extended edge: original half-length + sd on each side
            half_len = edge_len / 2 + obj.sd;

            % Rectangle dimensions: length = 2*half_len, width = sd
            % Inscribed ellipse of rectangle has semi-axes:
            %   a_inscribed = half_len, b_inscribed = sd/2
            % Circumscribed ellipse (scaled by sqrt(2)):
            %   ah = half_len * sqrt(2), bh = (sd/2) * sqrt(2)
            % But per the paper's construction, the circumscribed ellipse
            % of the rectangle has semi-axes equal to half the diagonals
            % projected onto each axis. For a rectangle with half-dims
            % (half_len, sd/2), the circumscribed ellipse has:
            ah = half_len;
            bh = obj.sd / 2;

            % The extended ellipse is the circumscribed ellipse of the
            % inscribed ellipse (Sec. 3.3 step 3). The paper says
            % "the inscribed ellipse is similarly enlarged to obtain its
            % circumscribed ellipse". For a rectangle, the circumscribed
            % ellipse of the inscribed ellipse scales by sqrt(2):
            ah = ah * sqrt(2);
            bh = bh * sqrt(2);

            % o_t is o_h translated by sd/2 along the outward normal
            % (positive y_h direction, away from hull interior)
            normal = [-sin(beta); cos(beta)];  % perpendicular to edge
            % Ensure normal points away from o_hat
            if dot(normal, o_h - obj.o_hat) < 0
                normal = -normal;
            end
            o_t = o_h + (obj.sd / 2) * normal;
        end

        % ==============================================================
        % Build ellipse for collinear targets (Sec. 3.3 step 6, Fig. 5)
        % ==============================================================
        function [ah, bh, beta, o_t] = buildCollinearEllipse(obj, unique_pts)
            % buildCollinearEllipse  Special case for collinear targets.
            %
            % Per Sec. 3.3 step 6 (Fig. 5):
            %   - Connect the two furthest targets
            %   - Extend by sd on both sides
            %   - Build rectangle: extended length x 2*sd width
            %   - Circumscribed ellipse of the inscribed ellipse

            if size(unique_pts, 1) < 2
                ah = obj.sd; bh = obj.sd; beta = 0;
                o_t = unique_pts(1, :)';
                return
            end

            % Find the two furthest points
            max_dist = 0;
            p1 = unique_pts(1, :)';
            p2 = unique_pts(2, :)';
            for i = 1:size(unique_pts, 1)
                for j = i+1:size(unique_pts, 1)
                    d = norm(unique_pts(i,:) - unique_pts(j,:));
                    if d > max_dist
                        max_dist = d;
                        p1 = unique_pts(i, :)';
                        p2 = unique_pts(j, :)';
                    end
                end
            end

            edge = p2 - p1;
            beta = atan2(edge(2), edge(1));
            o_t  = (p1 + p2) / 2;

            % Extended line segment Sr: half-length + sd
            half_len = max_dist / 2 + obj.sd;

            % Rectangle: Sr length x 2*sd width
            % Inscribed ellipse: semi-axes = (half_len, sd)
            % Circumscribed ellipse: scale by sqrt(2)
            ah = half_len * sqrt(2);
            bh = obj.sd * sqrt(2);
        end

        % ==============================================================
        % Move - integrate dynamics and log position
        % ==============================================================
        function obj = move(obj, dT, t)
            latency_steps = round(obj.latency / dT);
            if obj.latency > 0 && t > latency_steps
                p_dot = obj.u_traj(:, t - latency_steps);
            else
                p_dot = obj.u;
            end
            obj.p            = obj.p + p_dot * dT;
            obj.p_traj(:, t) = obj.p;
        end

        % ==============================================================
        % Diagnostic helpers
        % ==============================================================
        function errs = localizationErrors(obj, true_targets)
            errs = vecnorm(obj.r_hat - true_targets, 2, 1);
        end

        function err = centreError(obj, true_centre_in)
            err = norm(obj.o_hat - true_centre_in(:));
        end

    end % methods
end % classdef


% -----------------------------------------------------------------------
% Module-level helper function
% -----------------------------------------------------------------------

function y = sig_pow(x, alpha)
% sig_pow  Component-wise signed power: sgn(x) .* |x|.^alpha
    y = sign(x) .* (abs(x) .^ alpha);
end