classdef ChunTianAgent
% ChunTianAgent  Implements the Chun & Tian (2020) algorithm for
% multi-target localization and elliptical circumnavigation using
% bearing-only measurements.
%
% Reference:
%   Chun S, Tian Y-P. Multi-targets localization and elliptical
%   circumnavigation by multi-agents using bearing-only measurements in
%   two-dimensional space. Int J Robust Nonlinear Control. 2020;30:3250-3268.
%   https://doi.org/10.1002/rnc.4932
%
% Algorithm components (equation numbers from the paper):
%   Eq. (3)  : Deghat position estimator for each target
%   Eq. (4)  : Shao-Tian geometric centre estimator (distributed consensus)
%   Eq. (5)  : Elliptical circumnavigation controller
%   Eq. (7)  : Desired ellipse radius rho_d(vartheta)
%   Eq. (11-12): Tangential bearing angle psi_i1 from axial bearing psi_i
%   Eq. (14) : rho_d expressed in terms of measurable psi_i
%
% Notation follows the paper directly:
%   p         : agent position (2x1)
%   xi_k      : position of target k (2x1)
%   phi_k     : unit bearing vector from agent to target k  (varphi^k_i)
%   phibar_k  : phi_k rotated 3*pi/2 CCW = 90 CW  (bar_varphi^k_i)
%   r         : this agent's estimate of the geometric centre xi*(t)
%   omega     : auxiliary consensus state variable
%   psi_i     : bearing angle of vector from agent to r
%   phi_i     : unit vector in direction psi_i
%   phi_i1    : unit vector in tangential direction (Eq. 11-12)
%   rho_hat   : ||r - p||  (estimated distance to centre, rho_hat*_i)
%   rho_d     : desired ellipse radius at psi_i  (Eq. 14)
%   a, b      : semi-major and semi-minor axes of desired ellipse
%   alpha     : CCW rotation angle of ellipse major axis
%   eta       : tangential velocity gain
%               Stability requires  0 < |eta| < (a^2+b^2)/(a^2-b^2)
%               Negative eta gives clockwise circumnavigation (Remark 1)
%   k_gain    : position estimator gain k_i > 0  (Eq. 3)
%   eps       : geometric centre consensus gain epsilon > 0  (Eq. 4)
%
% Usage:
%   ag = ChunTianAgent(p0, x_hat0, a, b, alpha, k_gain, eta, eps);
%   for t = 1:N
%       ag = ag.step(targets, neighbour_rs, dt);
%   end
%
% See also: simulate_chun_tian.m

    properties (Access = public)

        % --- Agent state ---
        p                    % (2x1)   current position
        u                    % (2x1)   current velocity command
        p_traj               % (2xT)   position history
        u_traj               % (2xT)   control input history
        heading              % scalar  current heading angle (rad)
        velocity_saturation  % scalar  maximum speed (m/s)
        latency              % scalar  control latency (s)

        % --- Ellipse parameters ---
        a           % semi-major axis (m)
        b           % semi-minor axis (m)
        alpha       % CCW rotation angle of ellipse (rad)

        % --- Gains ---
        k_gain      % position estimator gain k_i   (Eq. 3)
        eta         % tangential velocity gain       (Eq. 5)
        eps         % consensus gain epsilon         (Eq. 4)

        % --- Target position estimates  xi_hat^i_k  (Eq. 3) ---
        xi_hat      % (2 x M) one column per observed target

        % --- Geometric centre estimator state  (Eq. 4) ---
        r           % (2x1) estimate of xi*(t)
        omega       % (2x1) auxiliary consensus variable

        % --- Cached bearings (updated each step) ---
        phi         % (2 x M) unit bearing vectors to each target
        phibar      % (2 x M) 90-deg CW rotations of phi columns

        % --- Tracking error ---
        d_tilde      % scalar  rho_hat - rho_d(psi_i): distance error to desired ellipse (Eq. 5)
        d_tilde_traj % (1xT)  history of d_tilde over time
    end

    methods

        % ==============================================================
        % Constructor
        % ==============================================================
        function obj = ChunTianAgent(p0, x_hat0, a, b, alpha, k_gain, eta, eps, tSteps, numTargets, targets, heading, velocity_saturation, latency)
            % ChunTianAgent  Construct a Chun-Tian agent.
            %
            % Inputs (matching existing Agent class signature)
            %   p0                   (2x1)   initial agent position
            %   x_hat0               (2xM)   initial target position estimates
            %   a                    scalar  semi-major axis (a >= b > 0)
            %   b                    scalar  semi-minor axis
            %   alpha                scalar  ellipse rotation angle (rad)
            %   k_gain               scalar  position estimator gain k_i > 0  (Eq. 3)
            %   eta                  scalar  tangential velocity gain          (Eq. 5)
            %                                |eta| < (a^2+b^2)/(a^2-b^2)
            %   eps                  scalar  consensus gain epsilon > 0        (Eq. 4)
            %   tSteps               scalar  number of simulation time steps
            %   numTargets           scalar  number of targets
            %   targets              (2xM)   initial true target positions
            %   heading              scalar  initial heading angle (rad)
            %   velocity_saturation  scalar  maximum speed (m/s)
            %   latency              scalar  control latency (s)
            %
            % Paper Section 5, Agent 1 values:
            %   a=8, b=4, alpha=0, k_gain=3, eta=1.5, eps=3

            % Validate ellipse axes
            assert(a >= b && b > 0, 'Require a >= b > 0.');

            % Validate eta stability bound (Proposition 1)
            if a > b
                eta_max = (a^2 + b^2) / (a^2 - b^2);
                assert(abs(eta) < eta_max, ...
                    'eta = %.4f violates stability bound |eta| < %.4f', ...
                    eta, eta_max);
            end

            obj.p                   = p0(:);
            obj.a                   = a;
            obj.b                   = b;
            obj.alpha               = alpha;
            obj.k_gain              = k_gain;
            obj.eta                 = eta;
            obj.eps                 = eps;
            obj.heading             = heading;
            obj.velocity_saturation = velocity_saturation;
            obj.latency             = latency;
            obj.u                   = zeros(2, 1);

            M              = numTargets;
            obj.xi_hat     = x_hat0;
            obj.phi        = zeros(2, M);
            obj.phibar     = zeros(2, M);
            obj.p_traj     = zeros(2, tSteps);
            obj.u_traj     = zeros(2, tSteps);

            % Initialise geometric centre estimate and consensus variable
            obj.r            = mean(x_hat0, 2);
            obj.omega        = zeros(2, 1);

            % Initialise tracking error log
            obj.d_tilde      = 0;
            obj.d_tilde_traj = zeros(1, tSteps);
        end

        % ==============================================================
        % Main step function
        % ==============================================================
        function obj = step(obj, targets, neighbour_rs, dt)
            % step  Advance the agent by one time step dt.
            %
            % Inputs
            %   targets       (2 x M) true target positions (used only to
            %                 compute bearing unit vectors – no range used)
            %   neighbour_rs  cell array of (2x1) r estimates from neighbours
            %   dt            scalar time step (s)
            %
            % The method executes in the order specified by the paper:
            %   1. Measure bearings  (passive – bearing only, Sec. 2)
            %   2. Update position estimator  (Eq. 3)
            %   3. Update geometric centre estimator  (Eq. 4)
            %   4. Compute circumnavigation control input  (Eq. 5)
            %   5. Integrate holonomic agent dynamics  x_dot = u

            obj = obj.measureBearings(targets);
            obj = obj.updatePositionEstimator(dt);
            obj = obj.updateGeometricCentreEstimator(neighbour_rs, dt);
            obj = obj.computeControl();
            obj.p = obj.p + obj.u * dt;
        end

        % ==============================================================
        % Step 1 – Bearing measurements
        % ==============================================================
        function obj = measureBearings(obj, targets)
            % measureBearings  Compute phi^k_i and bar_phi^k_i (Sec. 2).
            %
            %   phi^k_i    = (xi_k - p) / ||xi_k - p||
            %   bar_phi^k_i = phi^k_i rotated 3*pi/2 CCW  (= 90 deg CW)
            %
            % Only the direction is used – no range information.

            M = size(targets, 2);
            for k = 1:M
                diff = targets(:, k) - obj.p;
                d    = norm(diff);
                if d < 1e-9
                    continue   % agent on target – bearing undefined
                end
                phi_k            = diff / d;
                obj.phi(:, k)    = phi_k;
                obj.phibar(:, k) = rot270ccw(phi_k);   % 3*pi/2 CCW = 90 CW
            end
        end

        % ==============================================================
        % Step 2 – Position estimator  (Eq. 3)
        % ==============================================================
        function obj = updatePositionEstimator(obj, dt)
            % updatePositionEstimator  Deghat et al. estimator – Eq. (3):
            %
            %   d/dt xi_hat^i_k = k_i * (I - phi^k_i * phi^k_i') * (p - xi_hat^i_k)
            %
            % Because  I - phi*phi' = phibar*phibar'  for unit vectors, we use:
            %   d/dt xi_hat^i_k = k_i * phibar^k_i * (phibar^k_i' * (p - xi_hat^i_k))

            M = size(obj.xi_hat, 2);
            for k = 1:M
                phibar_k    = obj.phibar(:, k);
                err         = obj.p - obj.xi_hat(:, k);
                xi_hat_dot  = obj.k_gain * phibar_k * (phibar_k' * err);
                obj.xi_hat(:, k) = obj.xi_hat(:, k) + xi_hat_dot * dt;
            end
        end

        % ==============================================================
        % Step 3 – Geometric centre estimator  (Eq. 4)
        % ==============================================================
        function obj = updateGeometricCentreEstimator(obj, neighbour_rs, dt)
            % updateGeometricCentreEstimator  Shao-Tian estimator – Eq. (4):
            %
            %   omega_dot_i = eps * sum_{j in Ni} sgn(r_j - r_i)
            %   r_i         = omega_i + (n/m) * sum_{k in Ni^O} (1/|Nk^V|) * xi_hat^i_k
            %
            % When a single agent observes all M targets with equal weight,
            % the local contribution simplifies to  mean(xi_hat).

            % Consensus correction via component-wise signum
            omega_dot = zeros(2, 1);
            for jj = 1:length(neighbour_rs)
                r_j       = neighbour_rs{jj};
                omega_dot = omega_dot + obj.eps * sign(r_j - obj.r);
            end
            obj.omega = obj.omega + omega_dot * dt;

            % Local geometric centre estimate
            local_mean = mean(obj.xi_hat, 2);
            obj.r      = obj.omega + local_mean;
        end

        % ==============================================================
        % Step 4 – Circumnavigation controller  (Eq. 5)
        % ==============================================================
        function obj = computeControl(obj, t)
            % computeControl  Elliptical circumnavigation controller – Eq. (5):
            %
            %   u_i = (rho_hat*_i - rho_d_i(psi_i)) * phi_i
            %         + eta_i * rho_hat*_i * phi_i1
            %
            % where:
            %   phi_i   = unit vector from p to r  (axial direction)
            %   phi_i1  = tangential unit vector at current ellipse point
            %   rho_hat = ||r - p||
            %   rho_d   = desired ellipse radius at bearing psi_i  (Eq. 14)
            %   d_tilde = rho_hat - rho_d  (tracking error to desired ellipse)

            vec_to_r = obj.r - obj.p;
            rho_hat  = norm(vec_to_r);

            if rho_hat < 1e-9
                obj.u                  = zeros(2, 1);
                obj.d_tilde            = 0;
                obj.d_tilde_traj(t)    = 0;
                obj.u_traj(:, t)       = obj.u;
                return
            end

            phi_i  = vec_to_r / rho_hat;
            psi_i  = atan2(phi_i(2), phi_i(1));

            rho_d  = obj.ellipseRadius(psi_i);        % Eq. (14)
            phi_i1 = obj.tangentialUnitVector(psi_i); % Eq. (11-12)

            % Tracking error: how far agent is from the desired ellipse
            obj.d_tilde         = rho_hat - rho_d;
            obj.d_tilde_traj(t) = obj.d_tilde;

            obj.u = obj.d_tilde * phi_i + obj.eta * rho_hat * phi_i1;

            % Apply velocity saturation
            if norm(obj.u) > obj.velocity_saturation
                obj.u = obj.velocity_saturation * obj.u / norm(obj.u);
            end

            obj.u_traj(:, t) = obj.u;
        end

        % ==============================================================
        % Ellipse radius at bearing psi  (Eq. 14)
        % ==============================================================
        function rho_d = ellipseRadius(obj, psi)
            % ellipseRadius  Desired ellipse radius at axial bearing psi.
            %
            % Eq. (14):
            %   rho_d(psi) = a*b / sqrt( a^2*sin^2(psi-alpha) + b^2*cos^2(psi-alpha) )
            %
            % Note: vartheta - psi = pi  (Eq. 8), so sin^2(vartheta-alpha)
            % = sin^2(psi-alpha+pi) = sin^2(psi-alpha) – sign drops out.

            theta = psi - obj.alpha;
            denom = sqrt(obj.a^2 * sin(theta)^2 + obj.b^2 * cos(theta)^2);
            if denom < 1e-12
                rho_d = obj.a;
                return
            end
            rho_d = (obj.a * obj.b) / denom;
        end

        % ==============================================================
        % Tangential unit vector phi_i1  (Eq. 11-12)
        % ==============================================================
        function phi_i1 = tangentialUnitVector(obj, psi)
            % tangentialUnitVector  Unit tangent to the ellipse at the point
            % corresponding to axial bearing psi_i, from Eq. (11)-(12):
            %
            %   cos(psi_i1) = ( a^2*sin(psi-alpha)*cos(alpha)
            %                  + b^2*cos(psi-alpha)*sin(alpha) )
            %                / sqrt( a^4*sin^2(psi-alpha) + b^4*cos^2(psi-alpha) )
            %
            %   sin(psi_i1) = ( a^2*sin(psi-alpha)*sin(alpha)
            %                  - b^2*cos(psi-alpha)*cos(alpha) )
            %                / sqrt( a^4*sin^2(psi-alpha) + b^4*cos^2(psi-alpha) )

            a  = obj.a;  b = obj.b;  alpha = obj.alpha;
            theta = psi - alpha;
            s  = sin(theta);  c = cos(theta);
            sa = sin(alpha);  ca = cos(alpha);

            denom = sqrt(a^4 * s^2 + b^4 * c^2);
            if denom < 1e-12
                % Degenerate – fall back to CCW-rotated axial direction
                phi_i = [cos(psi); sin(psi)];
                phi_i1 = rot90ccw(phi_i);
                return
            end

            cos_psi1 = (a^2 * s * ca + b^2 * c * sa) / denom;
            sin_psi1 = (a^2 * s * sa - b^2 * c * ca) / denom;
            phi_i1   = [cos_psi1; sin_psi1];
        end

        % ==============================================================
        % Diagnostic helpers
        % ==============================================================
        function errs = localizationErrors(obj, true_targets)
            % localizationErrors  ||xi_hat^i_k - xi_k|| for each target k.
            % Returns (1 x M) vector.
            errs = vecnorm(obj.xi_hat - true_targets, 2, 1);
        end

        function err = centreError(obj, true_centre)
            % centreError  ||r_i - xi*||
            err = norm(obj.r - true_centre(:));
        end

        function err = circumnavigationError(obj, true_centre)
            % circumnavigationError  ||xi - xi*|| - rho_d(psi_i)
            % Should converge to 0 (Theorem 1).
            vec    = obj.p - true_centre(:);
            rho_st = norm(vec);
            vec_r  = obj.r - obj.p;
            psi_i  = atan2(vec_r(2), vec_r(1));
            rho_d  = obj.ellipseRadius(psi_i);
            err    = rho_st - rho_d;
        end

        % ==============================================================
        % Move – integrate dynamics and log position
        % ==============================================================
        function obj = move(obj, dT, t)
            % move  Integrate holonomic agent dynamics and log position.
            %
            %   p(t+1) = p(t) + u(t) * dT
            %
            % Latency is handled by indexing back into u_traj if a delay
            % has been specified and enough steps have elapsed.

            latency_steps = round(obj.latency / dT);
            if obj.latency > 0 && t > latency_steps
                p_dot = obj.u_traj(:, t - latency_steps);
            else
                p_dot = obj.u;
            end

            obj.p            = obj.p + p_dot * dT;
            obj.p_traj(:, t) = obj.p;
        end

    end % methods
end % classdef


% -----------------------------------------------------------------------
% Module-level helper functions (not class methods)
% -----------------------------------------------------------------------

function v = rot270ccw(u)
% rot270ccw  Rotate a 2-vector by 3*pi/2 CCW (= 90 deg CW).
% Paper uses this rotation for bar_varphi (Section 2, Eq. after (2)).
    v = [u(2); -u(1)];
end

function v = rot90ccw(u)
% rot90ccw  Rotate a 2-vector by pi/2 CCW.
    v = [-u(2); u(1)];
end