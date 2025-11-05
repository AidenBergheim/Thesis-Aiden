
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
    tFinal = 10;            % (seconds)
    dT     = 0.005;         % (seconds)
    
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
            k_omega = 5;           
       
        % PDT algorthm parameters
            beta_1 = 0.5;          
            beta_2 = 0.5;
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 0.1;
            Tc2 = 0.2;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 5 + 0.2*sin(30*time)
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeConvexHullRadius(theta, c_hat, x_hat_positions, y);
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeMinCircleRadius(theta, c_hat, x_hat_positions, y);
        %d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeMinEllipseRadius(theta, c_hat, x_hat_positions, y);
        

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    % Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    AgentPDT = AgentPDT.updateDesiredDistance(t, dT);
    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets);          % --- Run estimator
    AgentPDT = AgentPDT.controlInputPDT(t, Tc1, dT);                % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index

targetColors = lines(qtyTargets); 

       
%  ---- Agent Trajectory plot -----

    figure('Position',[50 100 550 450]);
    plot_traj = gca;
    hold on;
    
    % Plotting actual target positions
    
        for i = 1:qtyTargets
            xT = Targets(1, i);
            yT = Targets(2, i);
        
            if i == 1
                plot(xT, yT, 'r+', ...
                    'LineWidth', 0.7, ...
                    'MarkerSize', 7, ...
                    'Color', 'r', ...
                    'DisplayName', 'Target positions $\mathbf{x}_i$');
            else
                plot(xT, yT, 'r+', ...
                    'LineWidth', 0.7, ...
                    'MarkerSize', 7, ...
                    'Color', 'r', ...
                    'HandleVisibility', 'off');
            end
        end
    
    
    % Plotting PDT Agent
    
        % plot trajectory of the single agent
        plot(AgentPDT.p_traj(1,1:mytStepFinal), AgentPDT.p_traj(2,1:mytStepFinal), ...
            'LineWidth', 0.8, ...
            'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Controller Inspried by Sui et al. (2025)]',...
            'Color'      ,  [0.3, 0.4, 1]);   
        hold on
        centroid = mean(Targets, 2);
        desired_traj = zeros(2, mytStepFinal);
        for t = 1:mytStepFinal
            theta_t = AgentPDT.theta_traj(t);
            d_des_t = AgentPDT.d_des_traj(t);
            % Position on desired trajectory at angle theta_t and distance d_des_t from centroid
            desired_traj(:, t) = centroid + d_des_t * [cos(theta_t); sin(theta_t)];
        end
        plot(desired_traj(1,:), desired_traj(2,:), '--', ...
            'LineWidth', 1, ...
            'DisplayName', 'Desired Trajectory',...
            'Color'      ,  [0.55, 0.45, 0.25]);   
        hold on


        hold on; % Hold on from the start
        
        % Plotting Target Estimation Errors

        for i = 1:qtyTargets
            
            plot(AgentPDT.x_hat{i}(1, 1:mytStepFinal), AgentPDT.x_hat{i}(2, 1:mytStepFinal), ...
                'LineWidth', 0.6, ...
                'Color', [0.1; 0.9; 0.1],...
                'DisplayName', ['Target ' num2str(i) ' Estimated Position']);
        end
    
        hold on;
        
        
        % Plot the centroid by providing its x and y components separately
        plot(centroid(1), centroid(2), 'k+', ...
            'LineWidth', 1.1, ...
            'MarkerSize', 14, ...
            'DisplayName', 'Target centroid position $\mathbf{c}$')
        hold on
    
    % ==== Axis properties ====
    axis equal
    
    title('(a) Agent Trajectories')
    xlabel('x (m)')
    ylabel('y (m)')
    grid on
    set(gca,'FontSize', 10);
    xlim([-5 10]);
    ylim([-5 8]);
    legend('Interpreter','latex', 'Location','best');


%  ---- Tracking Error Plot -----

    figure('Position',[600 100 550 450]);
    plot_errors = gca;
    
    
    % Plotting PDT Agent
    
        plot(real(AgentPDT.delta_traj),...
            'DisplayName', '$$\delta(t)=d(t)-d^*$$ [Controller Inspried by Sui et al. (2025)]', ...
            'LineWidth', 1.6, ...
            'Color'      ,  [0.85, 0.15, 0.15]);
        hold on

    xlim([0,tFinal/dT])
    xticks([0,0.4/dT,2/dT:2/dT:tFinal/dT])
    xt = get(gca, 'XTick');
    set(gca, 'XTick',xt, 'XTickLabel',xt*dT);

    
    title('(b) Tracking Errors')
    xlabel('time (sec)')
    ylabel('Errors (m)')
    
    hold on    
        
    grid on
    set(gca,'FontSize', 10);
    legend('Interpreter','latex', 'Location','best');

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


