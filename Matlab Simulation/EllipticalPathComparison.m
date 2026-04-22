
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
    tFinal = 30;            % (seconds)
    dT     = 0.001;         % (seconds)
    tSteps = (tFinal-tBegin)/dT;   % force tSteps to be an integer
% ----------------------------------------
%        Define Multiple Targets
% ----------------------------------------
    Targets       = [-6, -2, 3, 5;
                      0,  0, 0, 0];    % Chun & Tian (2020) Section 5
    qtyTargets = size(Targets, 2);
% ----------------------------------------
%              Define Agent
% ----------------------------------------
% ---- Initial Conditions ----
        p_0             = [20; 10];      % Agent 1 initial position, Section 5
        x_hat_0 = [-4, 0, 5, 7;
                    -2, 2, 2, 2];          % perturbed initial estimates
% ---- Control Constants ----
% control gain for adjusting tangential speed
            k_omega = 10;
% PDT algorithm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 0.2;
            Tc2 = 0.4;
% Cao algorithm parameters
            alpha = 5;
% Non-holonomic parameters
            initial_heading = pi/2;
% Desired Distance to Targets
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeChunTianEllipseRadius(theta);
% ----------------------------------------
%             Create agents
% ----------------------------------------
% Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);
% Agent utilizing Chun & Tian (2020): a1=8, b1=4, alpha1=0, k1=3, eta1=1.5, eps=3
    AgentChunTian = ChunTianAgent(p_0, x_hat_0, 8, 4, 0, 3, 1.5, 3, ...
                    tSteps, qtyTargets, Targets, initial_heading, 1000, 0);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT = AgentPDT.updateDesiredDistance(t, dT);
    AgentPDT = AgentPDT.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

    AgentChunTian = AgentChunTian.measureBearings(Targets);
    AgentChunTian = AgentChunTian.updatePositionEstimator(dT);
    AgentChunTian = AgentChunTian.updateGeometricCentreEstimator({}, dT);
    AgentChunTian = AgentChunTian.computeControl(t);
    AgentChunTian = AgentChunTian.move(dT, t);

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps));
targetColors = lines(qtyTargets);
t_vec = (0:mytStepFinal-1) * dT;
t_vec_2 = (0:mytStepFinal*10-1) * 0.001;

figure('Position', [50 200 720 320]);
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
h_path_MD = plot(ax1, [p_0(1), AgentChunTian.p_traj(1,1:mytStepFinal)], [p_0(2), AgentChunTian.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Chun \& Tian (2020)]');

h_path_PDT = plot(ax1, [p_0(1), AgentPDT.p_traj(1,1:mytStepFinal)], [p_0(2), AgentPDT.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Proposed Method]');

% --- Final positions ---
h_final_MD = plot(ax1, AgentChunTian.p_traj(1,mytStepFinal), AgentChunTian.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0, 0.9], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Chun \& Tian (2020)]']);

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
xlim([-20 30])
ylim([-10 30])
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
legend(ax1, [h_path_MD, h_path_PDT, h_final_MD, h_final_PDT, h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'north', 'NumColumns', 2);
hold(ax1, 'off');




% =========================================================================
%  (b) Tracking Errors  —  column 3, top row
% =========================================================================
ax2 = nexttile(2);
hold(ax2, 'on');

plot(ax2, t_vec, real(AgentChunTian.d_tilde_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\|x_i - \xi^*\| - \rho_{d_i}(\psi_i)$$ [Chun \& Tian (2020)]', ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9]);

plot(ax2, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Proposed Method]', ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0]);

yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');

xlim(ax2, [0, tFinal]);
box(ax2, 'on');
title(ax2, '(b) Tracking Errors', 'Interpreter', 'latex')
xlabel(ax2, 'time (sec)', 'Interpreter', 'latex')
ylabel(ax2, 'Errors (m)', 'Interpreter', 'latex')
grid(ax2, 'on');
set(ax2, 'FontSize', 10);
legend(ax2, 'Interpreter', 'latex', 'Location', 'northeast');
pbaspect(ax2, [2 1 1]);
hold(ax2, 'off');
ylim([-1 18])


% =========================================================================
%  Final layout
% =========================================================================
tl.TileSpacing = 'compact';
tl.Padding = 'compact';


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

function distance = computeChunTianEllipseRadius(theta)
    % computeChunTianEllipseRadius  Returns the desired ellipse radius at
    % bearing angle theta for Agent 1 from Chun & Tian (2020) Section 5.
    %
    % Implements Eq. (14) from the paper:
    %   rho_d(psi) = a*b / sqrt( a^2*sin^2(psi - alpha) + b^2*cos^2(psi - alpha) )
    %
    % Agent 1 parameters (Section 5):
    %   a = 8, b = 4, alpha = 0
    %
    % Inputs (signature matches d_des_handle convention):
    %   theta            : current bearing angle from agent to estimated centre
    %   ~                : c_hat        (unused)
    %   ~                : x_hat_positions (unused)
    %   ~                : y / agent position (unused)

    a     = 8;
    b     = 4;
    alpha = 0;

    theta_rel = theta - alpha;
    denom     = sqrt(a^2 * sin(theta_rel)^2 + b^2 * cos(theta_rel)^2);

    if denom < 1e-12
        distance = a;  % degenerate fallback
    else
        distance = (a * b) / denom;
    end
end

