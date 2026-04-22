
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
    tFinal = 42;            % (seconds)
    dT     = 0.001;         % (seconds)
    
    tSteps = (tFinal-tBegin)/dT;   % force tSteps to be an integer

    % ----------------------------------------
    %        Define Multiple Targets
    % ----------------------------------------
    
    Targets       = [-2, 4, 2, 1; 
                     0, 5, 0, 1];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);
    

    % ----------------------------------------
    %              Define Agent
    % ----------------------------------------
    
    % ---- Initial Conditions ----
    
        p_0             = [8; 0];      % initial location of the agent 


        x_hat_0         = [7.7, 7.812, 7.7, 7.703; 
                           0, 0.234, 0, 0.042];   % agent's initial guess of target positions

    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_omega = 1;           
       
        % PDT algorthm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 1;
            Tc2 = 1;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 1;
        d_des_handle1 = @(time, theta, x_hat_positions, c_hat, y) 5.7 + 0.2*sin(2.1*time);
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeConvexHullRadius(theta, c_hat, x_hat_positions, y);
        d_des_handle2 = @(time, theta, x_hat_positions, c_hat, y) computeMinCircleRadius(theta, c_hat, x_hat_positions, y);
        d_des_handle3 = @(time, theta, x_hat_positions, c_hat, y) computeMinEllipseRadius(theta, c_hat, x_hat_positions, y);
        

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    % Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT1 = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle1, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);

    AgentPDT2 = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle2, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);

    AgentPDT3 = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle3, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    AgentPDT1 = AgentPDT1.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT1 = AgentPDT1.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT1 = AgentPDT1.updateDesiredDistance(t, dT);
    AgentPDT1 = AgentPDT1.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT1 = AgentPDT1.move(dT, t);                                % --- Execute control law

    AgentPDT2 = AgentPDT2.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT2 = AgentPDT2.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT2 = AgentPDT2.updateDesiredDistance(t, dT);
    AgentPDT2 = AgentPDT2.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT2 = AgentPDT2.move(dT, t);                                % --- Execute control law

    AgentPDT3 = AgentPDT3.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT3 = AgentPDT3.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT3 = AgentPDT3.updateDesiredDistance(t, dT);
    AgentPDT3 = AgentPDT3.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT3 = AgentPDT3.move(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index
targetColors = lines(qtyTargets); 
t_vec = (0:mytStepFinal-1) * dT; % Create a time vector
figure('Position', [50 100 1100 420]); % Create a new figure
t = tiledlayout(2, 3); % Create a 2-row, 3-column grid

%  ---- (a) Agent Trajectory plot -----
ax1 = nexttile(1, [2 1]); 
hold(ax1, 'on');

% 1. PRE-ALLOCATE HANDLES
% We must capture these to force the legend order later
h_targets = gobjects(qtyTargets, 1); 
h_start   = gobjects(1, 1);
h_finals  = gobjects(3, 1);
h_paths   = gobjects(3, 1); % Assuming 3 agents (PDT1, PDT2, PDT3)



% 2. PLOT TARGETS (Left Column of Legend)
for i = 1:qtyTargets
    xT = Targets(1, i);
    yT = Targets(2, i);
    h_targets(i) = plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(i, :),...
        'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
end



% 4. PLOT START POSITION (Bottom Right)
h_start = plot(AgentPDT1.p_traj(1,1), AgentPDT1.p_traj(2,1), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'none', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');

% 5. PLOT FINAL POSITIONS (Right Column of Legend)
% Note: I've updated indices to match 1, 2, 3 for clarity in the handle array
h_finals(1) = plot(AgentPDT1.p_traj(1,mytStepFinal), AgentPDT1.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0.5, 0], ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(42)$$ [Sinusoidal]');

h_finals(2) = plot(AgentPDT2.p_traj(1,mytStepFinal), AgentPDT2.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'b', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(42)$$ [Min. Circle]');    

h_finals(3) = plot(AgentPDT3.p_traj(1,mytStepFinal), AgentPDT3.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'r', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(42)$$ [Approx. Min. Ellipse]');

% 3. PLOT PATHS (Bottom of Left/Right Columns)
h_paths(1) = plot(ax1, AgentPDT1.p_traj(1,:), AgentPDT1.p_traj(2,:), ...
    'LineWidth', 1, ...
    'Color', [0, 0.5, 0], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Sinusoidal]');   

h_paths(2) = plot(ax1, AgentPDT2.p_traj(1,:), AgentPDT2.p_traj(2,:), ...
    'LineWidth', 1, ...
    'Color', 'b', ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Min. Circle]');   

h_paths(3) = plot(ax1, AgentPDT3.p_traj(1,:), AgentPDT3.p_traj(2,:), ...
    'LineWidth', 1, ...
    'Color', 'r', ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Approx. Min. Ellipse]');  

% ==== Axis properties ====
axis(ax1, 'equal');
box(ax1, 'on'); 
title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
xlabel(ax1, 'x (m)', 'Interpreter','latex')
ylabel(ax1, 'y (m)', 'Interpreter','latex')
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
xlim(ax1, [-6 11]);
ylim(ax1, [-6 14]);

% ==== CUSTOM LEGEND ORDERING ====
% MATLAB fills legends Column-by-Column (Top-Down) by default.
% To get Targets Left | Finals Right, we construct two stacks:

% Stack 1 (Left Column): All Targets, then some Paths
stack_left = [h_start; h_targets]; 
stack_middle = [h_finals];

stack_right = [h_paths(1:3)];

% Stack 2 (Right Column): All Finals, then remaining Path + Start


% Combine them
legend_handles = [stack_right; stack_middle; stack_left];

legend(ax1, legend_handles, 'Interpreter','latex', 'Location','north', 'NumColumns', 2);
hold(ax1, 'off');

% ------- (b) Estimate Trajectories plot --------
    % This plot is in column 2 and spans 2 rows
    ax2 = nexttile(2, [2 1]); 
    hold(ax2, 'on');
    
    % Second, plot all trajectories (column 2 of legend)
    for i = 1:qtyTargets
        plot(ax2, AgentPDT1.x_hat{i}(1, 1:mytStepFinal), AgentPDT1.x_hat{i}(2, 1:mytStepFinal), ...
            'LineWidth', 1, ...
            'Color', targetColors(i, :),...
            'LineStyle'  ,  '--', ...
            'DisplayName', ['$$\mbox{\boldmath$\hat{x}$}_' num2str(i) '(t)$$']);
    end

    % First, plot all starting positions (column 1 of legend)
    for i = 1:qtyTargets
        plot(ax2, AgentPDT1.x_hat{i}(1, 1), AgentPDT1.x_hat{i}(2, 1), 'o', ...
            'MarkerSize', 6, ...
            'MarkerEdgeColor', targetColors(i, :), ...
            'MarkerFaceColor', 'none', ...
            'LineWidth', 1.5, ...
            'DisplayName', ['$$\mbox{\boldmath$\hat{x}$}_' num2str(i) '(0)$$']);
    end
    

    
    % Third, plot all actual target positions (column 3 of legend)
    for i = 1:qtyTargets
        xT = Targets(1, i);
        yT = Targets(2, i);
        plot(ax2, xT, yT, '+', ...
            'Color', targetColors(i, :),...
            'LineWidth', 1, ...
            'MarkerSize', 7, ...
            'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
    end
        
    % ==== Axis properties ====
    axis(ax2, 'equal');
    box(ax2, 'on');
    title(ax2, '(b) Estimate Trajectories', 'Interpreter','latex')
    xlabel(ax2, 'x (m)', 'Interpreter','latex')
    ylabel(ax2, 'y (m)', 'Interpreter','latex')
    grid(ax2, 'on');
    set(ax2, 'FontSize', 10);
    xlim(ax2, [-6 11]);
    ylim(ax2, [-6 14]);
    legend(ax2, 'Interpreter','latex', 'Location','north', 'NumColumns', 3);
    hold(ax2, 'off');

%  ---- (c) Tracking Error Plot -----
    % This plot is in column 3, top row
    ax3 = nexttile(3); 
    hold(ax3, 'on');
    
    % Plotting PDT Agent Tracking Error
    plot(ax3, t_vec, real(AgentPDT1.delta_traj(1:mytStepFinal)),...
        'DisplayName', '$$\delta(t)$$ [Sinusoidal]', ...
        'LineWidth', 1, ...
        'Color'      ,  [0, 0.5, 0]);
    hold on
    plot(ax3, t_vec, real(AgentPDT2.delta_traj(1:mytStepFinal)),...
        'DisplayName', '$$\delta(t)$$ [Min. Circle]', ...
        'LineWidth', 1, ...
        'Color'      ,  'r');
    hold on
    plot(ax3, t_vec, real(AgentPDT3.delta_traj(1:mytStepFinal)),...
        'DisplayName', '$$\delta(t)$$ [Approx. Min. Ellipse]', ...
        'LineWidth', 1, ...
        'Color'      ,  'b');
    
    % --- Add Vertical Line for Tracking Convergence (Tc1 + Tc2) ---
    if exist('Tc1', 'var') && exist('Tc2', 'var')
        xline(ax3, Tc1 + Tc2, '-.', {'$T_{c,1} + T_{c,2}$'}, ...
            'Interpreter', 'latex', ...
            'FontSize', 10, ...
            'LabelVerticalAlignment', 'top', ...
            'HandleVisibility', 'off'); 
    end

    % --- Axis Properties ---
    xlim(ax3, [0, tFinal])
    ylim(ax3,[-0.3, 6.5])
    box(ax3, 'on'); % <--- ADDED THIS to close the box
    title(ax3, '(c) Tracking Errors', 'Interpreter','latex')
    xlabel(ax3, 'time (sec)', 'Interpreter','latex')
    ylabel(ax3, 'Errors (m)', 'Interpreter','latex')
    grid(ax3, 'on');
    set(ax3, 'FontSize', 10);
    legend(ax3, 'Interpreter','latex', 'Location','northeast');
    pbaspect(ax3, [2 1 1]); 
    hold(ax3, 'off');


% ------ (c) Estimation Error Plot ---------
    % This plot is in column 3, bottom row (tile 6)
    ax4 = nexttile(6); 
    hold(ax4, 'on');
    
    % Plotting Estimation Errors
    for i = 1:qtyTargets
        % Calculate the norm (magnitude) of the error vector
        err_norm_i = vecnorm(AgentPDT1.x_tilde_traj{i}(:, 1:mytStepFinal));
        
        % Plotting norm against t_vec
        semilogx(ax4, (1:mytStepFinal) * dT, err_norm_i, ...
            'LineWidth', 1.3, ...
            'LineStyle', '--', ...
            'Color', targetColors(i, :),...
            'DisplayName', ['$$\|\mbox{\boldmath$\tilde{x}$}_' num2str(i) '(t)\|$$']);
    end
    % --- Add Vertical Line for Estimation Convergence (Tc1) ---
    if exist('Tc1', 'var')
        xline(ax4, Tc1, '-.', {'$T_{c,1}$'}, ...
            'Interpreter', 'latex', ...
            'FontSize', 10, ...
            'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility', 'off'); 
    end

    % --- Axis Properties ---
    xlim(ax4, [0, tFinal])
    ylim([0 10])
    box(ax4, 'on'); % <--- ADDED THIS to close the box
    title(ax4, '(d) Estimation Errors', 'Interpreter','latex')
    xlabel(ax4, 'time (sec)', 'Interpreter','latex')
    ylabel(ax4, 'Errors (m)', 'Interpreter','latex')
    grid(ax4, 'on');
    set(ax4, 'FontSize', 10);
    legend(ax4, 'Interpreter','latex', 'Location','northeast');
    pbaspect(ax4, [2 1 1]); 
    set(ax4, 'XScale', 'log')
    hold(ax4, 'off');


% --- Final layout adjustments for the whole figure ---
t.TileSpacing = 'compact';
t.Padding = 'compact';


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


