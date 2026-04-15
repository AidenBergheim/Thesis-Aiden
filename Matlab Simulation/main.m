
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
    tFinal = 60;            % (seconds)
    frequency = 60;
    dT     = 1 / frequency;         % (seconds)
    latency = 0;              % (seconds)
    
    tSteps = (tFinal-tBegin)/dT;   % force tSteps to be an integer

    % ----------------------------------------
    %        Define Multiple Targets
    % ----------------------------------------
    
    Targets       = [0.5, 0.2, 0.6; 
                     -0.5, 0.6, 0];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);
    

    % ----------------------------------------
    %              Define Agent
    % ----------------------------------------
    
    % ---- Initial Conditions ----
    
        p_0             = [1.44639504; -1.416090488];      % initial location of the agent 
       	

        x_hat_0         = [1.0, 0.5, 0.4; 
                           0, 0.234/4, 0];   % agent's initial guess of target positions

        

    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_s = 0.2;           
       
        % PDT algorthm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 2;
            Tc2 = 4;
            velocity_saturation = 100;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 1;
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 1.2 + 0.2*sin(time);
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeConvexHullRadius(theta, c_hat, x_hat_positions, y);
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeMinCircleRadius(theta, c_hat, x_hat_positions, y);
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeMinEllipseRadius(theta, c_hat, x_hat_positions, y);
        

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    % Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT = Agent(p_0, x_hat_0, k_s, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, velocity_saturation ,latency);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
curr_time = 0;

for t = 1:tSteps
    AgentPDT = AgentPDT.updateDesiredDistance(t, dT);
    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT = AgentPDT.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index
targetColors = lines(qtyTargets); 
t_vec = (0:mytStepFinal-1) * dT; % Create a time vector

figure; % Create a new figure
t = tiledlayout(2, 3); % Create a 2-row, 3-column grid

%  ---- (a) Agent Trajectory plot -----
    % This plot is in column 1 and spans 2 rows
    ax1 = nexttile(1, [2 1]); 
    
    hold(ax1, 'on');
    
    % Plotting actual target positions
    for i = 1:qtyTargets
        xT = Targets(1, i);
        yT = Targets(2, i);
        plot(ax1, xT, yT, 'k+', ...
            'LineWidth', 1, ...
            'MarkerSize', 7, ...
            'Color', targetColors(i, :),...
            'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
    end
    
    % Plotting PDT Agent
    plot(ax1, AgentPDT.p_traj(1,1:length(AgentPDT.p_traj)), AgentPDT.p_traj(2,1:length(AgentPDT.p_traj)), ...
        'LineWidth', 1, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$',...
        'Color'      ,  'k');   
    
    % ==== Axis properties ====
    axis(ax1, 'equal');
    title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
    xlabel(ax1, 'x (m)', 'Interpreter','latex')
    ylabel(ax1, 'y (m)', 'Interpreter','latex')
    grid(ax1, 'on');
    set(ax1, 'FontSize', 10);
    xlim(ax1, [-2 2]);
    ylim(ax1, [-2 2]);
    legend(ax1, 'Interpreter','latex', 'Location','best');
    hold(ax1, 'off');

% ------- (b) Estimate Trajectories plot --------
    % This plot is in column 2 and spans 2 rows
    ax2 = nexttile(2, [2 1]); 
    hold(ax2, 'on');
    
    % Plotting Target Estimation Errors
    for i = 1:qtyTargets
        plot(ax2, AgentPDT.x_hat{i}(1, 1:mytStepFinal), AgentPDT.x_hat{i}(2, 1:mytStepFinal), ...
            'LineWidth', 1, ...
            'Color', targetColors(i, :),...
            'LineStyle'  ,  '--', ...
            'DisplayName', ['$$\mbox{\boldmath$\hat{x}$}_' num2str(i) '(t)$$']);
    end

    % Plotting actual target positions
    for i = 1:qtyTargets
        xT = Targets(1, i);
        yT = Targets(2, i);
        plot(ax2, xT, yT, 'k+', ...
            'Color', targetColors(i, :),...
            'LineWidth', 1, ...
            'MarkerSize', 7, ...
            'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
    end
        
    % ==== Axis properties ====
    axis(ax2, 'equal');
    title(ax2, '(b) Estimate Trajectories', 'Interpreter','latex')
    xlabel(ax2, 'x (m)', 'Interpreter','latex')
    ylabel(ax2, 'y (m)', 'Interpreter','latex')
    grid(ax2, 'on');
    set(ax2, 'FontSize', 10);
    xlim(ax2, [-2 2]);
    ylim(ax2, [-2 2]);
    legend(ax2, 'Interpreter','latex', 'Location','best');
    hold(ax2, 'off');

%  ---- (c) Tracking Error Plot -----
    % This plot is in column 3, top row
    ax3 = nexttile(3); 
    hold(ax3, 'on');
    
    % Plotting PDT Agent Tracking Error
    plot(ax3, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)),...
        'DisplayName', '$$\delta(t)$$', ...
        'LineWidth', 1, ...
        'Color'      ,  'k');
    hold on;
        % --- Add Vertical Line for Tracking Convergence (Tc1 + Tc2) ---
    if exist('Tc1', 'var') && exist('Tc2', 'var')
        xline(ax3, Tc1 + Tc2, '-.', {'$T_{c,1} + T_{c,2}$'}, ...
            'Interpreter', 'latex', ...
            'FontSize', 10, ...
            'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility', 'off'); 
    end
        
    % --- Axis Properties ---
    xlim(ax3, [0, tFinal])
    title(ax3, '(c) Tracking Errors', 'Interpreter','latex')
    xlabel(ax3, 'time (sec)', 'Interpreter','latex')
    ylabel(ax3, 'Errors (m)', 'Interpreter','latex')
    grid(ax3, 'on');
    set(ax3, 'FontSize', 10);
    legend(ax3, 'Interpreter','latex', 'Location','best');
    pbaspect(ax3, [2 1 1]); % You can still control aspect ratio
    hold(ax3, 'off');

% ------ (d) Estimation Error Plot ---------
    % This plot is in column 3, bottom row (tile 6)
    ax4 = nexttile(6); 
    hold(ax4, 'on');
    
    % Plotting Estimation Errors
    for i = 1:qtyTargets
        % Calculate the norm (magnitude) of the error vector
        err_norm_i = vecnorm(AgentPDT.x_tilde_traj{i}(:, 1:mytStepFinal));
        
        % Plotting norm against t_vec
        plot(ax4, t_vec, err_norm_i, ...
            'LineWidth', 1.3, ...
            'LineStyle', '--', ...
            'Color', targetColors(i, :),...
            'DisplayName', ['$$\|\mbox{\boldmath$\tilde{x}$}_' num2str(i) '(t)\|$$']);
    end

    hold on
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
    title(ax4, '(d) Estimation Errors', 'Interpreter','latex')
    xlabel(ax4, 'time (sec)', 'Interpreter','latex')
    ylabel(ax4, 'Errors (m)', 'Interpreter','latex')
    grid(ax4, 'on');
    set(ax4, 'FontSize', 10);
    legend(ax4, 'Interpreter','latex', 'Location','best');
    pbaspect(ax4, [2 1 1]); % You can still control aspect ratio
    hold(ax4, 'off');


%figure(2)
%    plot(t_vec, real(AgentPDT.integral(1:mytStepFinal)),...
%        'DisplayName', '$$\zeta(t)$$', ...
%        'LineWidth', 1, ...
%        'Color'      ,  'b');
%    hold on
%    plot(t_vec, real(AgentPDT.integralss(1:mytStepFinal)),...
%        'DisplayName', '$$\zeta_s(t)$$', ...
%        'LineWidth', 1, ...
%        'Color'      ,  'r');
%    legend('Interpreter','latex', 'Location','best');
%    grid on

% --- Final layout adjustments for the whole figure ---
t.TileSpacing = 'compact';
t.Padding = 'compact';
% sig function used for algorithms in Sui et al. (2025)
function out = sig(z,alpha)
    out = zeros(length(z),1);
    for i = 1:length(z)
        out(i) = sinh(z(i))*abs(z(i))^alpha;
    end
end


function closest_point = closestPointOnConvexHullAtAngle(theta, c_hat, x_hat_positions, r_s)
    unique_pts = unique(x_hat_positions', 'rows');
    ray_direction = [cos(theta); sin(theta)];
    
    if size(unique_pts, 1) < 3
        max_proj = 0;
        for j = 1:size(x_hat_positions, 2)
            v = x_hat_positions(:, j) - c_hat;
            proj = dot(v, ray_direction);
            if proj > max_proj
                max_proj = proj;
            end
        end
        closest_point = c_hat + (max_proj + r_s) * ray_direction;
        return;
    end
    
    K = convhull(x_hat_positions(1,:), x_hat_positions(2,:));
    hull_indices = K(1:end-1);
    hull_vertices = x_hat_positions(:, hull_indices);
    num_hull_vertices = size(hull_vertices, 2);
    
    max_distance = 0;
    best_edge = [0; 0];
    found = false;

    for i = 1:num_hull_vertices
        v1 = hull_vertices(:, i);
        v2 = hull_vertices(:, mod(i, num_hull_vertices) + 1);
        edge = v2 - v1;
        
        A_mat = [ray_direction, -edge];
        b_vec = v1 - c_hat;
        det_A = det(A_mat);
        
        if abs(det_A) > 1e-10
            params = A_mat \ b_vec;
            t_param = params(1);
            s = params(2);
            
            if t_param > 0 && s >= -1e-10 && s <= 1+1e-10
                if t_param > max_distance
                    max_distance = t_param;
                    best_edge = edge;
                    found = true;
                end
            end
        end
    end
    
    if ~found
        closest_point = c_hat + r_s * ray_direction;
        return;
    end

    % Intersection point on the hull boundary
    intersection_point = c_hat + max_distance * ray_direction;
    
    % Outward normal to the edge
    normal = [best_edge(2); -best_edge(1)];
    normal = normal / (norm(normal) + 1e-9);
    if dot(normal, ray_direction) < 0
        normal = -normal;
    end
    
    % Offset perpendicular to edge, then project back onto the ray
    % to get a consistent radial distance from centroid
    offset_point = intersection_point + r_s * normal;
    
    % Project offset_point back onto the ray from c_hat
    radial_dist = dot(offset_point - c_hat, ray_direction);
    closest_point = c_hat + radial_dist * ray_direction;
end

function distance = computeConvexHullRadius(theta, c_hat, x_hat_positions, y)
    r_s = 0.2;
    ray_direction = [cos(theta); sin(theta)];
    
    unique_pts = unique(x_hat_positions', 'rows');
    if size(unique_pts, 1) < 3
        max_proj = 0;
        for j = 1:size(x_hat_positions, 2)
            v = x_hat_positions(:, j) - c_hat;
            proj = dot(v, ray_direction);
            if proj > max_proj
                max_proj = proj;
            end
        end
        distance = max_proj + r_s;
        return;
    end
    
    K = convhull(x_hat_positions(1,:), x_hat_positions(2,:));
    hull_indices = K(1:end-1);
    hull_vertices = x_hat_positions(:, hull_indices);
    num_hull_vertices = size(hull_vertices, 2);
    
    max_dist = 0;
    
    % --- Offset edges: push each edge outward by r_s ---
    for i = 1:num_hull_vertices
        v1 = hull_vertices(:, i);
        v2 = hull_vertices(:, mod(i, num_hull_vertices) + 1);
        edge = v2 - v1;
        edge_len = norm(edge);
        if edge_len < 1e-12, continue; end
        
        normal = [edge(2); -edge(1)] / edge_len;
        edge_mid = (v1 + v2) / 2;
        if dot(normal, edge_mid - c_hat) < 0
            normal = -normal;
        end
        
        v1_off = v1 + r_s * normal;
        v2_off = v2 + r_s * normal;
        edge_off = v2_off - v1_off;
        
        A_mat = [ray_direction, -edge_off];
        b_vec = v1_off - c_hat;
        det_A = det(A_mat);
        
        if abs(det_A) > 1e-10
            params = A_mat \ b_vec;
            t_param = params(1);
            s = params(2);
            if t_param > 0 && s >= -1e-10 && s <= 1+1e-10
                if t_param > max_dist
                    max_dist = t_param;
                end
            end
        end
    end
    
    % --- Vertex arcs: circle of radius r_s at each vertex ---
    for i = 1:num_hull_vertices
        v = hull_vertices(:, i);
        oc = c_hat - v;
        b_coeff = 2 * dot(ray_direction, oc);
        c_coeff = dot(oc, oc) - r_s^2;
        discriminant = b_coeff^2 - 4*c_coeff;
        
        if discriminant >= 0
            sqrt_disc = sqrt(discriminant);
            t2 = (-b_coeff + sqrt_disc) / 2;  % far intersection only
            if t2 > max_dist
                max_dist = t2;
            end
        end
    end
    
    if max_dist == 0
        distance = r_s;
    else
        distance = max_dist;
    end
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




