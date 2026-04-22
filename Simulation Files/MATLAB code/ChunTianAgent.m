classdef ChunTianAgent

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


        function obj = ChunTianAgent(p0, x_hat0, a, b, alpha, k_gain, eta, eps, tSteps, numTargets, targets, heading, velocity_saturation, latency)


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

        % Main step function
        function obj = step(obj, targets, neighbour_rs, dt)
           
            obj = obj.measureBearings(targets);
            obj = obj.updatePositionEstimator(dt);
            obj = obj.updateGeometricCentreEstimator(neighbour_rs, dt);
            obj = obj.computeControl();
            obj.p = obj.p + obj.u * dt;
        end

        % Bearing measurements
        function obj = measureBearings(obj, targets)

            M = size(targets, 2);
            for k = 1:M
                diff = targets(:, k) - obj.p;
                d    = norm(diff);
                phi_k            = diff / d;
                obj.phi(:, k)    = phi_k;
                obj.phibar(:, k) = rot270ccw(phi_k);
            end
        end

        % Position estimator  (Eq. 3)
        function obj = updatePositionEstimator(obj, dt)
            M = size(obj.xi_hat, 2);
            for k = 1:M
                phibar_k    = obj.phibar(:, k);
                err         = obj.p - obj.xi_hat(:, k);
                xi_hat_dot  = obj.k_gain * phibar_k * (phibar_k' * err);
                obj.xi_hat(:, k) = obj.xi_hat(:, k) + xi_hat_dot * dt;
            end
        end

        % Geometric centre estimator  (Eq. 4)
        function obj = updateGeometricCentreEstimator(obj, neighbour_rs, dt)
            
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

        % Circumnavigation controller  (Eq. 5)
        function obj = computeControl(obj, t)

            vec_to_r = obj.r - obj.p;
            rho_hat  = norm(vec_to_r);

            phi_i  = vec_to_r / rho_hat;
            psi_i  = atan2(phi_i(2), phi_i(1));

            rho_d  = obj.ellipseRadius(psi_i);        % Eq. (14)
            phi_i1 = obj.tangentialUnitVector(psi_i); % Eq. (11-12)

            % Tracking error
            obj.d_tilde         = rho_hat - rho_d;
            obj.d_tilde_traj(t) = obj.d_tilde;

            obj.u = obj.d_tilde * phi_i + obj.eta * rho_hat * phi_i1;


            obj.u_traj(:, t) = obj.u;
        end

        % Ellipse radius at bearing psi
        function rho_d = ellipseRadius(obj, psi)

            theta = psi - obj.alpha;
            denom = sqrt(obj.a^2 * sin(theta)^2 + obj.b^2 * cos(theta)^2);
            if denom < 1e-12
                rho_d = obj.a;
                return
            end
            rho_d = (obj.a * obj.b) / denom;
        end

        % Tangential unit vector phi_i1  (Eq. 11-12)
        function phi_i1 = tangentialUnitVector(obj, psi)
            

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

        % Diagnostic helpers
        function errs = localizationErrors(obj, true_targets)
            errs = vecnorm(obj.xi_hat - true_targets, 2, 1);
        end

        function err = centreError(obj, true_centre)
            err = norm(obj.r - true_centre(:));
        end

        function err = circumnavigationError(obj, true_centre)
            vec    = obj.p - true_centre(:);
            rho_st = norm(vec);
            vec_r  = obj.r - obj.p;
            psi_i  = atan2(vec_r(2), vec_r(1));
            rho_d  = obj.ellipseRadius(psi_i);
            err    = rho_st - rho_d;
        end

        % Move the agent according to control effort
        function obj = move(obj, dT, t)
            p_dot = obj.u;

            obj.p            = obj.p + p_dot * dT;
            obj.p_traj(:, t) = obj.p;
        end

    end 
end

function v = rot270ccw(u)
    v = [u(2); -u(1)];
end

function v = rot90ccw(u)
    v = [-u(2); u(1)];
end