
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
    dT     = 0.002;         % (seconds)
    
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

            alpha_1 = 0.8;
            alpha_2 = 0.05;
            Tc1 = 1;
            Tc2 = 1;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 6 + 0.2*sin(30*time) + time;
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeConvexHullRadius(theta, c_hat, x_hat_positions, y);
        d_des_handle1 = @(time, theta, x_hat_positions, c_hat, y) computeMinCircleRadius(theta, c_hat, x_hat_positions, y);
        d_des_handle2 = @(time, theta, x_hat_positions, c_hat, y) computeMinEllipseRadius(theta, c_hat, x_hat_positions, y);
        

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    % Agent utilizing controller and localization from Sui et al. (2025)
    AgentHolonomic = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle2, tSteps, qtyTargets, Targets, initial_heading);

    AgentNonholonomic = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle2, tSteps, qtyTargets, Targets, initial_heading);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    AgentHolonomic = AgentHolonomic.updateDesiredDistance(t, dT);
    AgentHolonomic = AgentHolonomic.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentHolonomic = AgentHolonomic.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentHolonomic = AgentHolonomic.controlInputPDT(t, Tc1, dT);                % --- Run control law
    AgentHolonomic = AgentHolonomic.move(dT, t);                                % --- Execute control law

    AgentNonholonomic = AgentNonholonomic.updateDesiredDistance(t, dT);
    AgentNonholonomic = AgentNonholonomic.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentNonholonomic = AgentNonholonomic.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentNonholonomic = AgentNonholonomic.controlInputPDT(t, Tc1, dT);                % --- Run control law
    AgentNonholonomic = AgentNonholonomic.moveNonHolonomic(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index
targetColors = lines(qtyTargets);
%  ---- Create a single figure for side-by-side subplots -----
figure;
%  ---- First Subplot: Agent Trajectory -----
subplot(1,2,1);
plot_traj = gca;
hold on;
% Plotting actual target positions
for i = 1:qtyTargets
        xT = Targets(1, i);
        yT = Targets(2, i);
if i == 1
            plot(xT, yT, 'b+', ...
'LineWidth', 1, ...
'MarkerSize', 7, ...
'Color', 'b', ...
'DisplayName', 'Target positions $$\mbox{\boldmath$x$}_i$$');
else
            plot(xT, yT, 'b+', ...
'LineWidth', 1, ...
'MarkerSize', 7, ...
'Color', 'b', ...
'HandleVisibility', 'off');
end
end
% Plotting PDT Agent
% plot trajectory of the single agent
    plot(AgentNonholonomic.p_traj(1,1:mytStepFinal), AgentNonholonomic.p_traj(2,1:mytStepFinal), ...
'LineWidth', 1, ...
'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Nonholonomic Agent]',...
'Color'      ,  'k');

% Plot starting position (open circle)
    plot(AgentNonholonomic.p_traj(1,1), AgentNonholonomic.p_traj(2,1), 'o', ...
'MarkerSize', 8, ...
'MarkerEdgeColor', 'k', ...
'MarkerFaceColor', 'none', ...
'LineWidth', 1.5, ...
'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');

% Plot ending position (filled circle)
    plot(AgentNonholonomic.p_traj(1,mytStepFinal), AgentNonholonomic.p_traj(2,mytStepFinal), 'o', ...
'MarkerSize', 8, ...
'MarkerEdgeColor', 'k', ...
'MarkerFaceColor', 'k', ...
'LineWidth', 1.5, ...
'DisplayName', '$$\mbox{\boldmath$y$}(30)$$');

    hold on
    centroid = mean(Targets, 2);
% (Omitted commented-out code for clarity)
    hold on;
% (Omitted commented-out code for clarity)
    hold on;
% (Omitted commented-out code for clarity)
%hold on
% ==== Axis properties ====
axis equal
title('(a) Agent Trajectories', 'Interpreter','latex')
xlabel('x (m)', 'Interpreter','latex')
ylabel('y (m)','Interpreter','latex')
grid on
set(gca,'FontSize', 14);
xlim([-5 10]);
ylim([-5 8]);
box on;
legend('Interpreter','latex', 'Location','best');
%  ---- Second Subplot: Tracking Error -----
subplot(1,2,2)
plot_errors = gca; % Get the axes handle
hold on
% Plotting PDT Agent
    plot(real(AgentNonholonomic.delta_traj),...
'DisplayName', '$$\delta(t)$$ [Nonholonomic Agent]', ...
'LineWidth', 1, ...
'Color'      ,  'k');
% --- Add Vertical Line for Tracking Convergence (Tc1 + Tc2) ---
if exist('Tc1', 'var') && exist('Tc2', 'var')
        xline((Tc1 + Tc2)/dT, '-.', {'$T_{c,1} + T_{c,2}$'}, ...
'Interpreter', 'latex', ...
'FontSize', 10, ...
'LabelVerticalAlignment', 'top', ...
'HandleVisibility', 'off');
end
% --- Configure axes *after* plotting ---
xlim([0,tFinal/dT])
xticks(0 : 5/dT : tFinal/dT);
xt = get(gca, 'XTick');
set(gca, 'XTick',xt, 'XTickLabel',xt*dT);
title('(b) Tracking Errors', 'Interpreter','latex')
xlabel('time (sec)', 'Interpreter','latex')
ylabel('Errors (m)', 'Interpreter','latex')
ylim([-2 5])
grid on
box on;
set(gca,'FontSize', 14);
% --- Set the aspect ratio *after* all other properties ---
pbaspect([3 1 1]);
legend('Interpreter','latex', 'Location','best');

% sig function used for algorithms in Sui et al. (2025)
function out = sig(z,alpha)
    out = zeros(length(z),1);
    for i = 1:length(z)
        out(i) = sign(z(i))*abs(z(i))^alpha;
    end
end


function closest_point = closestPointOnConvexHullAtAngle(theta, c_hat, x_hat_positions, r_s)
    
    % Compute convex hull
    K = convhull(x_hat_positions(1,:), x_hat_positions(2,:));
    hull_indices = K(1:end-1);  % Remove duplicate last point
    hull_vertices = x_hat_positions(:, hull_indices);
    num_hull_vertices = size(hull_vertices, 2);
    
    % Ray direction from centroid at angle theta
    ray_direction = [cos(theta); sin(theta)];
    
    % Find intersection of ray with convex hull edges
    max_distance = 0;
    intersection_point = c_hat;
    
    for i = 1:num_hull_vertices
        v1 = hull_vertices(:, i);
        v2 = hull_vertices(:, mod(i, num_hull_vertices) + 1);
        
        % Express edge relative to centroid
        v1_rel = v1 - c_hat;
        v2_rel = v2 - c_hat;
        edge = v2_rel - v1_rel;
        
        % Solve for intersection: t*ray_direction = v1_rel + s*edge
        % [ray_direction, -edge] * [t; s] = v1_rel
        A = [ray_direction, -edge];
        det_A = det(A);
        
        if abs(det_A) > 1e-10
            params = A \ v1_rel;
            t = params(1);  % Distance along ray from centroid
            s = params(2);  % Position along edge (0 to 1)
            
            % Check if intersection is valid (forward ray, on edge segment)
            if t > 0 && s >= 0 && s <= 1
                if t > max_distance
                    max_distance = t;
                    intersection_point = c_hat + t * ray_direction;
                end
            end
        end
    end
    
    % Offset the intersection point outward by r_s in the direction of the ray
    closest_point = c_hat + (max_distance + r_s) * ray_direction;
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


