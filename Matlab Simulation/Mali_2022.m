%% run_MaLi_2022.m
% =========================================================================
% Implementation of Ma & Li (2022):
%   "Finite-time circumnavigation by irregular-shaped trajectory
%    with bearing-only measurements"
%   International Journal of Systems Science, 53:6, 1170-1190
%
% Adapted for a SINGLE AGENT circumnavigating MULTIPLE STATIC TARGETS in 2D.
%
% Algorithms implemented:
%   - Finite-time bearing-only estimator  (Section 4.1, Eq. 5)
%   - Irregular-shaped trajectory model    (Section 3.3)
%   - Circumnavigation control protocol    (Section 4.2, Eq. 13)
%
% Sign conventions and notation follow the paper throughout.
% =========================================================================
clear; clc; close all;


%% =======================================================================
%  USER PARAMETERS
%  =======================================================================

% ---- Simulation ----
dT    = 0.01;   % Time step (s)
T_end = 60;     % Total simulation time (s)
N     = round(T_end / dT);

% ---- Target positions (2 x M matrix, static, 2-D) ----
%  Change these to any 2xM set of points.
targets = [2.0,  4.5,  3.0,  5.5,  2.5;
           1.0,  1.0,  3.5,  2.5,  4.5];
M       = size(targets, 2);
c_true  = mean(targets, 2);   % True geometric centre

% ---- Agent initial pose ----
p0 = [-4; -3];   % 2x1 starting position

% ---- Estimator (Section 4.1, Eq. 5) ----
k0     = 1.2;    % Convergence coefficient   (k0 > 0)
alpha0 = 0.85;   % Smoothness exponent        (0 < alpha0 < 1)
%  The estimator is "switched off" after Tc1 seconds (steady-state):
Tc1    = 10.0;   % Estimation phase duration (s)

% ---- Controller (Section 4.2, Eq. 13) ----
k1      = 2.5;   % Radial gain (must satisfy k1 > |Ṙ|_max; tune conservatively)
omega_c = 0.25;  % Prescribed circumnavigation angular velocity (rad/s)
sd      = 0.6;   % Safety stand-off distance for extended convex hull (m)

% ---- Misc ----
v_max  = 3.0;    % Velocity saturation (m/s)
noise_init = 1.5; % Std-dev of initial estimate error (m)


%% =======================================================================
%  INITIALISATION
%  =======================================================================
rng(0);   % Reproducible randomness

p     = p0;
x_hat = targets + noise_init * randn(2, M);   % Initial estimates
c_hat = mean(x_hat, 2);

% ---- Pre-allocate storage ----
p_traj     = zeros(2, N);
c_hat_traj = zeros(2, N);
d_traj     = zeros(1, N);
R_traj     = zeros(1, N);
u_traj     = zeros(2, N);
est_err    = zeros(M, N);
x_hat_traj = zeros(2, M, N);

fprintf('Starting Ma & Li (2022) simulation ...\n');
fprintf('  Targets: %d   |   Agent start: [%.1f, %.1f]\n', M, p0(1), p0(2));
fprintf('  Tc1 = %.1f s  |  k0 = %.2f  |  k1 = %.2f  |  omega_c = %.3f\n', ...
        Tc1, k0, k1, omega_c);


%% =======================================================================
%  MAIN SIMULATION LOOP
%  =======================================================================
for t = 1:N
    curr_t = t * dT;

    % ------------------------------------------------------------------ %
    % STEP 1 — Bearing measurements                                       %
    %   Agent measures unit bearing vector phi_j = (r_j - p)/||r_j - p|| %
    %   to every target j.  No range information is available.            %
    %   tau_ijx = phi_j,   tau_ijy = R(+90°) * tau_ijx   (⊥ direction)  %
    % ------------------------------------------------------------------ %
    tau_x = zeros(2, M);
    tau_y = zeros(2, M);
    for j = 1:M
        diff       = targets(:,j) - p;
        tau_x(:,j) = diff / norm(diff);
        % Rotate +90° (CCW) for perpendicular:  [-sin; cos]
        tau_y(:,j) = [-tau_x(2,j);  tau_x(1,j)];
    end

    % ------------------------------------------------------------------ %
    % STEP 2 — Finite-time target position estimator  (Eq. 5)            %
    %                                                                      %
    %   ṙ̂_j = k0 · sig[ τ_ijy · (τ_ijy^T (p_i − r̂_j)) ]^α₀           %
    %                                                                      %
    %   Proof (Theorem 4.1): because τ_ijy ⊥ (p - r_j),                 %
    %     τ_ijy^T(p − r̂_j) = −τ_ijy^T(r̂_j − r_j)                     %
    %   so the estimator drives the perpendicular error to zero.          %
    %   Active for t ∈ [0, Tc1], then frozen.                            %
    % ------------------------------------------------------------------ %
    if curr_t <= Tc1
        for j = 1:M
            % Scalar projection of error onto perpendicular bearing
            s = tau_y(:,j)' * (p - x_hat(:,j));    % τ_ijy^T(p − r̂_j)

            % Vectorised update (2x1), then apply sig(·)^alpha0
            update_vec = tau_y(:,j) * s;            % τ_ijy · s
            x_hat_dot  = k0 * sig_func(update_vec, alpha0);

            x_hat(:,j) = x_hat(:,j) + x_hat_dot * dT;   % Euler step
        end
    end
    % Estimated geometric centre (ô_i in the paper)
    c_hat = mean(x_hat, 2);

    % ------------------------------------------------------------------ %
    % STEP 3 — Circumnavigation geometry                                  %
    %   Decompose agent motion into radial (e_r) and tangential (e_t)    %
    %   components w.r.t. the estimated centre c_hat.                    %
    % ------------------------------------------------------------------ %
    dp = p - c_hat;
    d  = norm(dp);          % Current distance from c_hat  (d_i in paper)

    if d < 1e-6
        theta = 0;  e_r = [1; 0];
    else
        theta = atan2(dp(2), dp(1));
        e_r   = dp / d;
    end
    e_t = [-e_r(2); e_r(1)];   % Tangential unit vector (CCW positive)

    % Desired radius R_i: distance from c_hat to the boundary of the
    % extended convex hull Ed[co(r̂_j)] in direction theta (Section 3.3)
    R = computeHullRadius(x_hat, c_hat, theta, sd);

    % ------------------------------------------------------------------ %
    % STEP 4 — Control protocol  (Eqs. 13a and 13c)                      %
    %                                                                      %
    %   For a SINGLE agent the inter-agent consensus term in (13b) is     %
    %   absent, so ω_i = ω_c directly.                                   %
    %                                                                      %
    %   Radial:       u_ir = −k₁ · sgn(d_i − R_i)        [Eq. 13a]     %
    %   Tangential:   u_it = ω_c · d_i                    [Eq. 13c]     %
    %   Full velocity: u_i  = u_ir · e_r + u_it · e_t     [Eq. 8a]      %
    % ------------------------------------------------------------------ %
    e_dist = d - R;
    u_ir   = -k1 * sign(e_dist);    % Radial velocity
    u_it   =  omega_c * d;          % Tangential velocity (arc-speed)

    u = u_ir * e_r + u_it * e_t;

    % Velocity saturation
    if norm(u) > v_max
        u = v_max * u / norm(u);
    end

    % ------------------------------------------------------------------ %
    % STEP 5 — Holonomic agent dynamics  (Eq. 8a)                        %
    %   ṗ = u   →   Euler integration                                    %
    % ------------------------------------------------------------------ %
    p = p + u * dT;

    % ------------------------------------------------------------------ %
    % Record                                                               %
    % ------------------------------------------------------------------ %
    p_traj(:,t)        = p;
    c_hat_traj(:,t)    = c_hat;
    d_traj(t)          = d;
    R_traj(t)          = R;
    u_traj(:,t)        = u;
    x_hat_traj(:,:,t)  = x_hat;
    for j = 1:M
        est_err(j,t) = norm(x_hat(:,j) - targets(:,j));
    end
end

fprintf('Simulation complete.\n');
fprintf('  Final d_i      = %.3f m\n', d_traj(end));
fprintf('  Final R_i      = %.3f m\n', R_traj(end));
fprintf('  Final |d-R|    = %.3f m\n', abs(d_traj(end) - R_traj(end)));


%% =======================================================================
%  PLOTTING
%  =======================================================================
t_vec = (1:N) * dT;
cmap  = cool(256);

figure('Name', 'Ma & Li (2022) — Single-Agent Circumnavigation', ...
       'Position', [80 60 1300 900], 'Color', 'w');

% ---- (1) Trajectory ----
ax1 = subplot(2,3,[1,4]);
hold on; grid on; axis equal;

% Agent path (colour-coded by time)
n_pts = size(p_traj, 2);
for ii = 1:n_pts-1
    cidx = max(1, round(ii/n_pts * 255) + 1);
    plot(p_traj(1,ii:ii+1), p_traj(2,ii:ii+1), ...
         'Color', cmap(cidx,:), 'LineWidth', 1.2);
end

% Extended hull (final estimate)
drawHull(x_hat, c_hat, sd);

% Target markers
plot(targets(1,:), targets(2,:), 'r*', 'MarkerSize',14, 'LineWidth',2.5, ...
     'DisplayName','Targets');
% True centroid
plot(c_true(1), c_true(2), 'k^', 'MarkerSize',11, 'LineWidth',2.5, ...
     'MarkerFaceColor','k', 'DisplayName','True centroid');
% Estimated centroid (final)
plot(c_hat(1), c_hat(2), 'b^', 'MarkerSize',11, 'LineWidth',2.5, ...
     'MarkerFaceColor','b', 'DisplayName','Est. centroid (final)');
% Initial estimates
plot(x_hat_traj(1,:,1), x_hat_traj(2,:,1), 'ms', 'MarkerSize',9, ...
     'LineWidth',2, 'DisplayName','Init. estimates');
% Start / end
plot(p_traj(1,1),   p_traj(2,1),   'go', 'MarkerSize',12, 'LineWidth',3, ...
     'DisplayName','Start');
plot(p_traj(1,end), p_traj(2,end), 'bs', 'MarkerSize',12, 'LineWidth',3, ...
     'DisplayName','End');

xlabel('X (m)'); ylabel('Y (m)');
title({'Agent Trajectory', '(cool = early, warm = late)'}, 'FontSize',11);
legend('Location','northwest','FontSize',8);

% ---- (2) Radial error ----
subplot(2,3,2);
hold on; grid on;
plot(t_vec, d_traj, 'b-',  'LineWidth',1.8, 'DisplayName','d_i  (actual)');
plot(t_vec, R_traj, 'r--', 'LineWidth',1.8, 'DisplayName','R_i  (desired)');
xline(Tc1, 'k:', 'LineWidth',1.5, 'Label','T_{c1}', ...
      'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
xlabel('Time (s)'); ylabel('Radius (m)');
title('Radial Distance vs Desired Radius', 'FontSize',11);
legend('FontSize',9); ylim([0, max(R_traj)*1.5 + 0.5]);

% ---- (3) Radial error (d - R) ----
subplot(2,3,5);
hold on; grid on;
plot(t_vec, d_traj - R_traj, 'Color',[0.2 0.6 0.2], 'LineWidth',1.8);
yline(0, 'k--', 'LineWidth',1.2);
xline(Tc1, 'k:', 'LineWidth',1.5, 'Label','T_{c1}', ...
      'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
xlabel('Time (s)'); ylabel('d_i - R_i  (m)');
title('Circumnavigation Radius Error', 'FontSize',11);

% ---- (4) Estimation errors ----
subplot(2,3,3);
hold on; grid on;
for j = 1:M
    plot(t_vec, est_err(j,:), 'LineWidth',1.5, ...
         'DisplayName', sprintf('Target %d', j));
end
xline(Tc1, 'k:', 'LineWidth',1.5, 'Label','T_{c1}', ...
      'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
xlabel('Time (s)'); ylabel('||r̂_j − r_j||  (m)');
title('Target Estimation Error (Eq. 5)', 'FontSize',11);
legend('FontSize',9, 'Location','northeast');

% ---- (5) Speed profile ----
subplot(2,3,6);
speeds = vecnorm(u_traj);
hold on; grid on;
plot(t_vec, speeds, 'Color',[0.7 0.2 0.8], 'LineWidth',1.5);
yline(v_max, 'r--', 'v_{max}', 'LabelHorizontalAlignment','right');
xline(Tc1, 'k:', 'LineWidth',1.5, 'Label','T_{c1}', ...
      'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
xlabel('Time (s)'); ylabel('||u_i||  (m/s)');
title('Agent Speed', 'FontSize',11);


%% =======================================================================
%  HELPER FUNCTIONS
%  =======================================================================

function y = sig_func(x, kappa)
    % sig(x)^κ as defined in Notation 1.1 of Ma & Li (2022):
    %   sig(ξ)^κ = sgn(ξ_i) · |ξ_i|^κ   (element-wise)
    y = sign(x) .* (abs(x) .^ kappa);
end


function R = computeHullRadius(x_hat, c_hat, theta, sd)
    % ------------------------------------------------------------------ %
    % Compute the expected radius R_i = distance from c_hat to the       %
    % boundary of the extended convex hull Ed[co(r̂_j)] at angle theta.  %
    %                                                                      %
    % Construction follows Section 3.3 of Ma & Li (2022):                %
    %   1. Build convex hull of estimated target positions.               %
    %   2. Expand each edge outward by safety distance sd.                %
    %   3. Cast a ray from c_hat at angle theta; return intersection.    %
    %                                                                      %
    % Special case (collinear / M ≤ 2): returns circumscribed ellipse    %
    % radius (Section 3.3, step 6).                                       %
    % ------------------------------------------------------------------ %
    M   = size(x_hat, 2);
    pts = x_hat';   % M × 2

    % ---- Collinear / two-point fallback: ellipse (Section 3.3 step 6) ----
    if M < 3
        R = ellipseRadius(x_hat, c_hat, theta, sd);
        return;
    end
    % Check for near-collinearity via rank of centred matrix
    cen = pts - c_hat';
    if rank(cen, 1e-6) < 2
        R = ellipseRadius(x_hat, c_hat, theta, sd);
        return;
    end

    % ---- Build convex hull ----
    try
        k = convhull(pts(:,1), pts(:,2), 'Simplify', true);
    catch
        R = ellipseRadius(x_hat, c_hat, theta, sd);
        return;
    end
    hull_v = pts(k(1:end-1), :);   % nv × 2  (last vertex = first, removed)
    nv = size(hull_v, 1);

    % ---- Expand hull: offset each edge outward by sd ----
    %  For each edge, move it outward by sd along the edge normal.
    %  Recompute vertices as intersections of adjacent offset lines.
    expanded = expandHull(hull_v, c_hat, sd);

    % ---- Ray-polygon intersection ----
    ray_dir = [cos(theta); sin(theta)];
    R = rayPolyDist(c_hat, ray_dir, expanded);

    % ---- Fallback if no valid intersection found ----
    if ~isfinite(R) || R <= 0
        R = max(vecnorm(x_hat - c_hat)) + sd;
    end
end


function expanded = expandHull(hull_v, c_hat, sd)
    % Expand each edge of convex hull outward by sd.
    % Returns new polygon vertices as intersections of offset edges.
    nv = size(hull_v, 1);
    
    % Compute offset lines for each edge
    %   Each edge: A→B with outward normal n (pointing away from c_hat)
    %   Offset line: passes through A + sd*n, direction B - A
    A_off = zeros(nv, 2);
    dirs  = zeros(nv, 2);
    
    for i = 1:nv
        A = hull_v(i, :)';
        B = hull_v(mod(i, nv) + 1, :)';
        edge = B - A;
        edge_len = norm(edge);
        if edge_len < 1e-10
            dirs(i,:)  = edge';
            A_off(i,:) = A';
            continue;
        end
        edge_u = edge / edge_len;
        % Candidate outward normal (one of the two perpendiculars)
        n_cand = [-edge_u(2); edge_u(1)];
        mid = (A + B) / 2;
        if dot(n_cand, mid - c_hat) < 0
            n_cand = -n_cand;
        end
        A_off(i,:) = (A + sd * n_cand)';
        dirs(i,:)  = edge_u';
    end
    
    % Vertices of expanded polygon = intersection of consecutive offset lines
    expanded = zeros(nv, 2);
    for i = 1:nv
        i_next = mod(i, nv) + 1;
        P = A_off(i,:)';
        d1 = dirs(i,:)';
        Q = A_off(i_next,:)';
        d2 = dirs(i_next,:)';
        
        % Solve P + t*d1 = Q + s*d2
        denom = d1(1)*d2(2) - d1(2)*d2(1);
        if abs(denom) < 1e-10
            % Parallel: use midpoint fallback
            expanded(i,:) = ((P + Q)/2)';
        else
            diff = Q - P;
            t = (diff(1)*d2(2) - diff(2)*d2(1)) / denom;
            expanded(i,:) = (P + t*d1)';
        end
    end
end


function t_out = rayPolyDist(origin, dir, polygon)
    % Minimum positive distance from origin along dir to polygon boundary.
    nv    = size(polygon, 1);
    t_out = inf;
    for i = 1:nv
        A    = polygon(i, :)';
        B    = polygon(mod(i, nv) + 1, :)';
        edge = B - A;
        % Solve: origin + t·dir = A + s·edge
        denom = dir(1)*edge(2) - dir(2)*edge(1);
        if abs(denom) < 1e-12, continue; end
        diff = A - origin;
        t = (diff(1)*edge(2) - diff(2)*edge(1)) / denom;
        s = (diff(1)*dir(2)  - diff(2)*dir(1))  / denom;
        if t > 1e-9 && s >= -1e-9 && s <= 1 + 1e-9
            if t < t_out, t_out = t; end
        end
    end
end


function R = ellipseRadius(x_hat, c_hat, theta, sd)
    % Extended ellipse for collinear / two-target case (Section 3.3, step 6)
    % Semi-major axis: max spread along principal direction + sd
    % Semi-minor axis: sd
    M = size(x_hat, 2);
    if M < 2
        R = sd + 0.3;
        return;
    end
    cen = (x_hat - c_hat);
    [~, ~, V] = svd(cen', 'econ');
    major_dir = V(:,1);     % Principal axis direction
    projs = major_dir' * cen;
    a = max(abs(projs)) + sd;   % Semi-major axis
    b = sd;                      % Semi-minor axis
    beta = atan2(major_dir(2), major_dir(1));  % Orientation angle
    phi  = theta - beta;         % Angle in ellipse frame
    % Polar equation of ellipse: r = ab / sqrt((b cos φ)² + (a sin φ)²)
    R = (a * b) / sqrt((b*cos(phi))^2 + (a*sin(phi))^2);
    R = max(R, sd * 0.5);
end


function drawHull(x_hat, c_hat, sd)
    % Draw convex hull and expanded hull on current axes.
    M   = size(x_hat, 2);
    pts = x_hat';
    if M < 3 || rank(pts - pts(1,:), 1e-6) < 2
        % Draw ellipse for collinear case
        ths = linspace(0, 2*pi, 200);
        for k = 1:length(ths)-1
            R1 = ellipseRadius(x_hat, c_hat, ths(k),   sd);
            R2 = ellipseRadius(x_hat, c_hat, ths(k+1), sd);
            p1 = c_hat + R1 * [cos(ths(k));   sin(ths(k))];
            p2 = c_hat + R2 * [cos(ths(k+1)); sin(ths(k+1))];
            plot([p1(1) p2(1)], [p1(2) p2(2)], 'g-', 'LineWidth', 2);
        end
        return;
    end
    try
        k = convhull(pts(:,1), pts(:,2), 'Simplify', true);
    catch; return; end
    hull_v = pts(k, :);
    raw_v  = pts(k(1:end-1), :);
    exp_raw = expandHull(raw_v, c_hat, sd);
    exp_v   = [exp_raw; exp_raw(1,:)];
    plot(hull_v(:,1), hull_v(:,2), 'r-',  'LineWidth', 2, ...
         'DisplayName', 'Convex hull co(r̂_j)');
    plot(exp_v(:,1),  exp_v(:,2),  'g--', 'LineWidth', 2.5, ...
         'DisplayName', sprintf('Extended hull E_d (s_d=%.1f)', sd));
end


