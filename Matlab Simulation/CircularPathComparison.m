
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MATLAB Simulation of Multi-Target Bearing 
% only Target Localization and Circumnavigation (BoTLC),
% Using controllers inspired by Cao et al. (2021) Sui et al. (2025) Zhao et al. (2018)

% Written by Aiden Bergheim For Research Thesis A and inspired by
% the MATLAB simulation written by Donglin Sui: https://github.com/Gloogger

clear
clf
close all
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Initial Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    tBegin = 0;             % (seconds)
    tFinal = 50;            % (seconds)
    dT     = 0.001;         % (seconds)
    
    tSteps = (tFinal-tBegin)/dT;   % force tSteps to be an integer

    % ----------------------------------------
    %        Define Multiple Targets
    % ----------------------------------------
    
    Targets       = [2, 1, 3; 
                     4, 2, 3];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);
    

    % ----------------------------------------
    %              Define Agent
    % ----------------------------------------
    
    % ---- Initial Conditions ----
    
        p_0             = [9; 8];      % initial location of the agent 
       
        x_hat_0 = [3, 3, 7;
                    2, 0, 3];

        

    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_omega = 5;           
       
        % PDT algorthm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 0.2;
            Tc2 = 0.4;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 1.91;
        
        

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    % Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, 10, 0);

    AgentMD = AgentDeghat(p_0, x_hat_0, 5, 5, 0.5, ...
                        tSteps, qtyTargets, Targets, initial_heading, 10, 0);



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT = AgentPDT.updateDesiredDistance(t, dT);
    AgentPDT = AgentPDT.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

    AgentMD = AgentMD.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentMD = AgentMD.estimateTargetDeghat(t, dT, Targets);          % --- Run estimator
    AgentMD = AgentMD.controlInputDeghat(t, dT);                % --- Run control law
    AgentMD = AgentMD.move(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps));
targetColors = lines(qtyTargets);
t_vec = (0:mytStepFinal-1) * dT;
t_vec_2 = (0:mytStepFinal*10-1) * 0.001;

figure;
tl = tiledlayout(1, 2);

% =========================================================================
%  (a) Agent Trajectories  —  spans both rows of column 1
% =========================================================================
ax1 = nexttile(1);
hold(ax1, 'on');

% --- True target positions ---
h_targets = gobjects(qtyTargets, 1);
for i = 1:qtyTargets
    h_targets(i) = plot(ax1, Targets(1,i), Targets(2,i), '+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(i,:), ...
        'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
end

% --- Common start position (both agents share the same p_0) ---
h_start = plot(ax1, p_0(1), p_0(2), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'none', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');

% --- Agent trajectories ---
h_path_MD = plot(ax1, [p_0(1), AgentMD.p_traj(1,1:mytStepFinal)], [p_0(2), AgentMD.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Deghat et al. (2014)]');

h_path_PDT = plot(ax1, [p_0(1), AgentPDT.p_traj(1,1:mytStepFinal)], [p_0(2), AgentPDT.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Proposed Method]');

% --- Final positions ---
h_final_MD = plot(ax1, AgentMD.p_traj(1,mytStepFinal), AgentMD.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0, 0.9], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Deghat et al. (2014)]']);

h_final_PDT = plot(ax1, AgentPDT.p_traj(1,mytStepFinal), AgentPDT.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0.9, 0, 0], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Proposed Method]']);

axis(ax1, 'equal');
box(ax1, 'on');
title(ax1, '(a) Agent Trajectories', 'Interpreter', 'latex')
xlabel(ax1, 'x (m)', 'Interpreter', 'latex')
ylabel(ax1, 'y (m)', 'Interpreter', 'latex')
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
xlim(ax1, [-6 11]);
ylim(ax1, [0 14]);
legend(ax1, [h_path_MD, h_path_PDT, h_final_MD, h_final_PDT, h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'best', 'NumColumns', 2);
hold(ax1, 'off');




% =========================================================================
%  (b) Tracking Errors  —  column 3, top row
% =========================================================================
ax3 = nexttile(2);
hold(ax3, 'on');

plot(ax3, t_vec, real(AgentMD.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Deghat et al. (2014)]', ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9]);

plot(ax3, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Proposed Method]', ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0]);

yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');

xlim(ax3, [0, tFinal]);
box(ax3, 'on');
title(ax3, '(b) Tracking Errors', 'Interpreter', 'latex')
xlabel(ax3, 'time (sec)', 'Interpreter', 'latex')
ylabel(ax3, 'Errors (m)', 'Interpreter', 'latex')
grid(ax3, 'on');
set(ax3, 'FontSize', 10);
legend(ax3, 'Interpreter', 'latex', 'Location', 'best');
pbaspect(ax3, [2 1 1]);
hold(ax3, 'off');
ylim([-1 8])


% =========================================================================
%  Final layout
% =========================================================================
tl.TileSpacing = 'compact';
tl.Padding = 'compact';

%{

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps));
targetColors = lines(qtyTargets);
t_vec = (0:mytStepFinal-1) * dT;

figure;
tl = tiledlayout(2, 3);

% =========================================================================
%  (a) Agent Trajectories  —  spans both rows of column 1
% =========================================================================
ax1 = nexttile(1, [2 1]);
hold(ax1, 'on');

% --- True target positions ---
h_targets = gobjects(qtyTargets, 1);
for i = 1:qtyTargets
    h_targets(i) = plot(ax1, Targets(1,i), Targets(2,i), '+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(i,:), ...
        'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
end

% --- Common start position (both agents share the same p_0) ---
h_start = plot(ax1, AgentMD.p_traj(1,1), AgentMD.p_traj(2,1), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'none', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');

% --- Agent trajectories ---
h_path_MD = plot(ax1, AgentMD.p_traj(1,1:mytStepFinal), AgentMD.p_traj(2,1:mytStepFinal), ...
    'LineWidth', 1, ...
    'Color', [0, 0.5, 0], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Deghat]');

h_path_PDT = plot(ax1, AgentPDT.p_traj(1,1:mytStepFinal), AgentPDT.p_traj(2,1:mytStepFinal), ...
    'LineWidth', 1, ...
    'Color', 'b', ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [PDT]');

% --- Final positions ---
h_final_MD = plot(ax1, AgentMD.p_traj(1,mytStepFinal), AgentMD.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0.5, 0], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Deghat]']);

h_final_PDT = plot(ax1, AgentPDT.p_traj(1,mytStepFinal), AgentPDT.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'b', ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [PDT]']);

axis(ax1, 'equal');
box(ax1, 'on');
title(ax1, '(a) Agent Trajectories', 'Interpreter', 'latex')
xlabel(ax1, 'x (m)', 'Interpreter', 'latex')
ylabel(ax1, 'y (m)', 'Interpreter', 'latex')
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
xlim(ax1, [-6 11]);
ylim(ax1, [-6 14]);
legend(ax1, [h_path_MD, h_path_PDT, h_final_MD, h_final_PDT, h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'best', 'NumColumns', 2);
hold(ax1, 'off');


% =========================================================================
%  (b) Estimate Trajectories  —  spans both rows of column 2
% =========================================================================
ax2 = nexttile(2, [2 1]);
hold(ax2, 'on');

% --- Deghat estimate trajectories (solid) ---
h_est_MD = gobjects(qtyTargets, 1);
for i = 1:qtyTargets
    h_est_MD(i) = plot(ax2, ...
        AgentMD.x_hat{i}(1, 1:mytStepFinal), ...
        AgentMD.x_hat{i}(2, 1:mytStepFinal), ...
        'LineWidth', 1, ...
        'Color', targetColors(i,:), ...
        'LineStyle', '-', ...
        'DisplayName', ['$$\mbox{\boldmath$\hat{x}$}^{MD}_' num2str(i) '(t)$$']);
end

% --- PDT estimate trajectories (dashed) ---
h_est_PDT = gobjects(qtyTargets, 1);
for i = 1:qtyTargets
    h_est_PDT(i) = plot(ax2, ...
        AgentPDT.x_hat{i}(1, 1:mytStepFinal), ...
        AgentPDT.x_hat{i}(2, 1:mytStepFinal), ...
        'LineWidth', 1, ...
        'Color', targetColors(i,:), ...
        'LineStyle', '--', ...
        'DisplayName', ['$$\mbox{\boldmath$\hat{x}$}^{PDT}_' num2str(i) '(t)$$']);
end

% --- Shared initial estimate positions (circles) ---
for i = 1:qtyTargets
    plot(ax2, AgentMD.x_hat{i}(1,1), AgentMD.x_hat{i}(2,1), 'o', ...
        'MarkerSize', 6, ...
        'MarkerEdgeColor', targetColors(i,:), ...
        'MarkerFaceColor', 'none', ...
        'LineWidth', 1.5, ...
        'HandleVisibility', 'off');
end

% --- True target positions ---
for i = 1:qtyTargets
    plot(ax2, Targets(1,i), Targets(2,i), '+', ...
        'Color', targetColors(i,:), ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'HandleVisibility', 'off');
end

axis(ax2, 'equal');
box(ax2, 'on');
title(ax2, '(b) Estimate Trajectories', 'Interpreter', 'latex')
xlabel(ax2, 'x (m)', 'Interpreter', 'latex')
ylabel(ax2, 'y (m)', 'Interpreter', 'latex')
grid(ax2, 'on');
set(ax2, 'FontSize', 10);
xlim(ax2, [-6 11]);
ylim(ax2, [-6 14]);
legend(ax2, [h_est_MD; h_est_PDT], ...
    'Interpreter', 'latex', 'Location', 'best', 'NumColumns', 2);
hold(ax2, 'off');


% =========================================================================
%  (c) Tracking Errors  —  column 3, top row
% =========================================================================
ax3 = nexttile(3);
hold(ax3, 'on');

plot(ax3, t_vec, real(AgentMD.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Deghat]', ...
    'LineWidth', 1, ...
    'Color', [0, 0.5, 0]);

plot(ax3, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [PDT]', ...
    'LineWidth', 1, ...
    'Color', 'r');

yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');

xlim(ax3, [0, tFinal]);
box(ax3, 'on');
title(ax3, '(c) Tracking Errors', 'Interpreter', 'latex')
xlabel(ax3, 'time (sec)', 'Interpreter', 'latex')
ylabel(ax3, '$\delta$ (m)', 'Interpreter', 'latex')
grid(ax3, 'on');
set(ax3, 'FontSize', 10);
legend(ax3, 'Interpreter', 'latex', 'Location', 'best');
pbaspect(ax3, [2 1 1]);
hold(ax3, 'off');


% =========================================================================
%  (d) Estimation Errors  —  column 3, bottom row
%      Shows BOTH Deghat and PDT estimation errors for each target
% =========================================================================
ax4 = nexttile(6);
hold(ax4, 'on');

% --- Deghat estimation errors (solid lines) ---
for i = 1:qtyTargets
    err_norm = vecnorm(AgentMD.x_tilde_traj{i}(:, 1:mytStepFinal));
    plot(ax4, t_vec, err_norm, ...
        'LineWidth', 1.3, ...
        'LineStyle', '-', ...
        'Color', targetColors(i,:), ...
        'DisplayName', ['$$\|\mbox{\boldmath$\tilde{x}$}^{MD}_' num2str(i) '\|$$']);
end

% --- PDT estimation errors (dashed lines) ---
for i = 1:qtyTargets
    err_norm = vecnorm(AgentPDT.x_tilde_traj{i}(:, 1:mytStepFinal));
    plot(ax4, t_vec, err_norm, ...
        'LineWidth', 1.3, ...
        'LineStyle', '--', ...
        'Color', targetColors(i,:), ...
        'DisplayName', ['$$\|\mbox{\boldmath$\tilde{x}$}^{PDT}_' num2str(i) '\|$$']);
end

xlim(ax4, [0, tFinal]);
box(ax4, 'on');
title(ax4, '(d) Estimation Errors', 'Interpreter', 'latex')
xlabel(ax4, 'time (sec)', 'Interpreter', 'latex')
ylabel(ax4, 'Error (m)', 'Interpreter', 'latex')
grid(ax4, 'on');
set(ax4, 'FontSize', 10);
legend(ax4, 'Interpreter', 'latex', 'Location', 'best', 'NumColumns', 2);
pbaspect(ax4, [2 1 1]);
hold(ax4, 'off');


% =========================================================================
%  Final layout
% =========================================================================
tl.TileSpacing = 'compact';
tl.Padding = 'compact';

%}

% sig function used for algorithms in Sui et al. (2025)
function out = sig(z,alpha)
    out = zeros(length(z),1);
    for i = 1:length(z)
        out(i) = sign(z(i))*abs(z(i))^alpha;
    end
end


function closest_point = closestPointOnConvexHullAtAngle(theta, c_hat, x_hat_positions, r_s)
% Calculates the point on a "safe" path offset from the convex hull
% of the targets, as seen from the centroid 'c_hat' at a given angle 'theta'.

    % --- Step 1: Compute Convex Hull ---
    % Ensure at least 3 unique points for convhull
    unique_pts = unique(x_hat_positions', 'rows');
    if size(unique_pts, 1) < 3
        % Fallback for 1 or 2 points: just offset from centroid
        ray_direction = [cos(theta); sin(theta)];
        closest_point = c_hat + r_s * ray_direction;
        return;
    end
    
    K = convhull(x_hat_positions(1,:), x_hat_positions(2,:));
    hull_indices = K(1:end-1);  % Remove duplicate last point
    hull_vertices = x_hat_positions(:, hull_indices);
    num_hull_vertices = size(hull_vertices, 2);
    
    % --- Step 2: Find Ray-Hull Intersection ---
    ray_direction = [cos(theta); sin(theta)];
    
    max_distance = 0;
    intersection_point = c_hat; 
    best_edge = [0; 0]; % Store the edge vector of the correct intersection

    for i = 1:num_hull_vertices
        v1 = hull_vertices(:, i);
        v2 = hull_vertices(:, mod(i, num_hull_vertices) + 1);
        
        % Edge vector (world coordinates)
        edge = v2 - v1; 
        
        % Solve for intersection: c_hat + t*ray_direction = v1 + s*edge
        % [ray_direction, -edge] * [t; s] = v1 - c_hat
        A = [ray_direction, -edge];
        b_vec = v1 - c_hat;
        
        det_A = det(A);
        
        % Check if ray is parallel to edge
        if abs(det_A) > 1e-10
            params = A \ b_vec;
            t = params(1);  % Distance along ray from centroid
            s = params(2);  % Position along edge (0 to 1)
            
            % Check if intersection is valid (forward ray, on edge segment)
            if t > 0 && s >= 0 && s <= 1
                if t > max_distance
                    max_distance = t;
                    % This is the point on the hull boundary
                    intersection_point = c_hat + t * ray_direction; 
                    best_edge = edge; % Store this edge
                end
            end
        end
    end
    
    % --- Step 3: Offset Intersection Point Perpendicularly ---
    
    if max_distance == 0
        % No intersection found (e.g., c_hat is outside hull)
        % Fallback: just offset along the ray
        closest_point = c_hat + r_s * ray_direction;
        return;
    end

    % Calculate the outward normal vector to the 'best_edge'
    edge_dir = best_edge;
    % Get the perpendicular vector (rotate 90 degrees)
    normal = [edge_dir(2); -edge_dir(1)]; 
    normal = normal / (norm(normal) + 1e-9); % Normalize
    
    % Ensure the normal points "outward" relative to the centroid
    % (i.e., in the same general direction as the ray from the centroid)
    if dot(normal, ray_direction) < 0
        normal = -normal;
    end
    
    % Offset the intersection point outward by r_s in the *normal direction*
    closest_point = intersection_point + r_s * normal;
end


function distance = computeConvexHullRadius(theta, c_hat, x_hat_positions, y)
    r_s = 0.3;

    closest_point = closestPointOnConvexHullAtAngle(theta, c_hat, x_hat_positions, r_s);

    distance = norm(closest_point - c_hat);
end

function distance = computeMinCircleRadius(theta, c_hat, x_hat_positions, y)
    % NOTE: 'y' is unused, but kept to match the function handle
    
    r_s = 0.3;
    [center, radius] = minEnclosingCircleProp(x_hat_positions');
    R_safe = radius + r_s; % Radius of the "safe" circle
    
    % Vector from the "safe" circle's center to the centroid
    CO = c_hat - center;
    
    % Unit vector from the centroid in the direction of the angle theta
    u_theta = [cos(theta); sin(theta)];

    
    a = 1;
    b = 2 * dot(CO, u_theta);
    c = norm(CO)^2 - R_safe^2;
    
    % Solve the quadratic formula: d = (-b ± sqrt(b^2 - 4ac)) / 2a
    % We take the positive root, as distance must be positive.
    distance = (-b + sqrt(b^2 - 4*a*c)) / (2*a);
end


function distance = computeMinEllipseRadius(theta, c_hat, x_hat_positions, y)
    % NOTE: 'y' is unused, but kept to match the function handle
    
    r_s = 0.3; % Safe distance
    
    % --- Step 1: Get the Ellipse Properties ---
    % This calculates the ellipse centered at c_hat (the mean of x_hat_positions)
    % that encloses all estimated points.
    [center, semi_axes, rotation_matrix] = minEnclosingEllipseStat(x_hat_positions');
    
    % Check for degenerate case (e.g., single point)
    if all(semi_axes == 0)
        distance = r_s; % Just return the safe distance
        return;
    end

    % --- Step 2: Define the Scaled Ellipse Matrix ---
    % The ellipse is defined by (P-c)' * A * (P-c) = 1
    % A = R * diag(1/a^2, 1/b^2) * R'
    
    % Add safe distance to each semi-axis
    a = semi_axes(1) + r_s;
    b = semi_axes(2) + r_s;
    R = rotation_matrix;
    
    % Create the diagonal matrix of squared inverse semi-axes
    D_inv_sq = diag([1/a^2; 1/b^2]);
    
    % Create the 'A' matrix
    A = R * D_inv_sq * R';

    % --- Step 3: Calculate Radius 'd' for angle 'theta' ---
    % We want to find 'd' such that P = c_hat + d*u_theta is on the ellipse.
    % (P - c_hat)' * A * (P - c_hat) = 1
    % (d*u_theta)' * A * (d*u_theta) = 1
    % d^2 * (u_theta' * A * u_theta) = 1
    % d = 1 / sqrt(u_theta' * A * u_theta)
    
    u_theta = [cos(theta); sin(theta)];
    
    denominator = u_theta' * A * u_theta;
    
    % Add a check to prevent division by zero if denominator is non-positive
    if denominator <= 1e-10
        distance = r_s; % Fallback
    else
        distance = 1 / sqrt(denominator);
    end
end


