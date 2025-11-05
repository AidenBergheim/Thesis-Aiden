
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
    tFinal = 12;            % (seconds)
    dT     = 0.005;         % (seconds)
    
    tSteps = int64((tFinal-tBegin)/dT);   % force tSteps to be an integer

    % ----------------------------------------
    %        Define Multiple Targets
    % ----------------------------------------
    
    Targets       = [-2, 4, 2, 1; 
                     0, 5, 0, 1];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);

    % Getting convex hull vertices
    K = convhull(Targets(1,:)', Targets(2,:)');
    hullVertices = Targets(:, K)';


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
            d_des = 0.5;
            rs = 0.3;

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    % Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

    % Agent utilizing controller and localization from Cao et al. (2021)
    AgentCao = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

    % Agent utilizing Nonholonomic Algorithms inspired by Sui et al. (2025)
    % and Zhao et al. (2018)
    NonHolonomic = Agent(p_0, x_hat_0, k_omega, Tc1, 0.12, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps

    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets);          % --- Run estimator
    AgentPDT = AgentPDT.getPsiAndProjectionMinEllipse(t, qtyTargets);     % --- Estimate Convex Hull
    AgentPDT = AgentPDT.controlInputPDT(t);                          % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

    AgentCao = AgentCao.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentCao = AgentCao.estimateTargetPDT(t, dT, Targets);          % --- Run estimator
    AgentCao = AgentCao.getPsiAndProjectionMinEllipse(t, qtyTargets);     % --- Estimate Convex Hull
    AgentCao = AgentCao.controlInputCao(t);                          % --- Run control law
    AgentCao = AgentCao.move(dT, t);                                % --- Execute control law
    
    NonHolonomic = NonHolonomic.getBearings(t, Targets);                    % --- Take bearing measurements
    NonHolonomic = NonHolonomic.estimateTargetPDT(t, dT, Targets);          % --- Run estimator
    NonHolonomic = NonHolonomic.getPsiAndProjectionHull(t, qtyTargets);  % --- Estimate Convex Hull
    NonHolonomic = NonHolonomic.controlInputPDT(t);                          % --- Run control law
    NonHolonomic = NonHolonomic.move(dT, t);                                % --- Execute control law

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
    
    % Draw approximate Minimum Enclosing Ellipse
    
        % Calculate minimum enclosing ellipse for true target positions
        target_positions = Targets';  % Convert to Nx2 format
        [ellipse_center, A_matrix] = computeMinEnclosingEllipse(target_positions);
        
        % Plot the ellipse
        plotEllipse(ellipse_center, A_matrix, [0.8, 0.7, 0.1], 1.2, 'Approximate Minimum Enclosing Ellipse');
        
        hold on;


        target_positions = Targets';  % Convert to Nx2 format
        [ellipse_center, A_matrix] = computeMinEnclosingEllipseKhachiyan(target_positions);
        
        % Plot the ellipse
        plotEllipse(ellipse_center, A_matrix, [0.4, 0.6, 0.2], 1.2, 'True Minimum Enclosing Ellipse (Khachiyan)');
        
        hold on;
    
    % Plotting actual target positions
    
        for i = 1:qtyTargets
            xT = Targets(1, i);
            yT = Targets(2, i);
        
            if i == 1
                plot(xT, yT, 'r+', ...
                    'LineWidth', 0.7, ...
                    'MarkerSize', 7, ...
                    'Color', 'k', ...
                    'DisplayName', 'Target positions $\mathbf{x}_i$');
            else
                plot(xT, yT, 'r+', ...
                    'LineWidth', 0.7, ...
                    'MarkerSize', 7, ...
                    'Color', 'k', ...
                    'HandleVisibility', 'off');
            end
        end
    
    % Plotting Cao Agent    
    
        % plot trajectory of the single agent
        plot(AgentCao.p_traj(1,1:mytStepFinal), AgentCao.p_traj(2,1:mytStepFinal), ...
            'LineWidth', 1.5, ...
            'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Controller From Cao et al. (2021)]',...
            'Color'      ,  [0.5 0.5 0.8]);   
        hold on
    
    % Plotting PDT Agent
    
        % plot trajectory of the single agent
        plot(AgentPDT.p_traj(1,1:mytStepFinal), AgentPDT.p_traj(2,1:mytStepFinal), ...
            'LineWidth', 1.5, ...
            'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Controller Inspried by Sui et al. (2025)]',...
            'Color'      ,  [0.85, 0.15, 0.15]);   
        hold on
    
    % ==== Axis properties ====
    axis equal
    
    title('(a) Agent Trajectories')
    xlabel('x (m)')
    ylabel('y (m)')
    xlim([-5 10]);
    ylim([-2 9]);
    grid on
    set(gca,'FontSize', 10);
    legend('Interpreter','latex', 'Location','best');


%  ---- Tracking Error Plot -----

    figure('Position',[600 100 550 450]);
    plot_errors = gca;
    
    
    % Plotting PDT Agent
    
        plot(AgentPDT.delta_traj,...
            'DisplayName', '$$\delta(t)=d(t)-d^*$$ [Controller Inspried by Sui et al. (2025)]', ...
            'LineWidth', 1.6, ...
            'Color'      ,  [0.85, 0.15, 0.15]);
        hold on
    
    % Plotting Cao Agent
    
        plot(AgentCao.delta_traj,...
            'DisplayName', '$$\delta(t)=d(t)-d^*$$ [Controller From Cao et al. (2021)]', ...
            'LineWidth', 1.6, ...
            'Color'      ,  [0.5 0.5 0.8]);
        hold on
    
    
    xlim([0,tFinal/dT])
    xticks([0,0.4/dT,2/dT:2/dT:tFinal/dT])
    xt = get(gca, 'XTick');
    set(gca, 'XTick',xt, 'XTickLabel',xt*dT);
    
    ylim([-0.5, 7.5])
    yticks([0:2:6, 7.5])
    
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

% Function to calculate minimum enclosing ellipse (standalone version for plotting)
function [center, A_matrix] = computeMinEnclosingEllipse(points)
    % Simple implementation using covariance-based approach
    % For a more robust solution, consider using Khachiyan's algorithm
    
    n = size(points, 1);
    if n < 2
        center = points(1, :)';
        A_matrix = eye(2);
        return;
    end
    
    % Center the points
    center = mean(points, 1)';
    centered_points = points - center';
    
    % Compute covariance matrix
    C = (centered_points' * centered_points) / (n - 1);
    
    % Add small regularization to avoid singular matrices
    C = C + 1e-6 * eye(2);
    
    % Scale factor to ensure all points are inside
    % Use maximum Mahalanobis distance
    max_dist = 0;
    for i = 1:n
        p = centered_points(i, :)';
        dist = sqrt(p' * (C \ p));
        max_dist = max(max_dist, dist);
    end
    
    % Scale the ellipse to enclose all points
    A_matrix = C * (max_dist^2 + 0.1); % Small buffer to ensure enclosure
end

% Function to plot an ellipse given center and covariance matrix
function plotEllipse(center, A_matrix, color, linewidth, displayname)
    % Generate ellipse points using eigendecomposition
    [V, D] = eig(A_matrix);
    
    % Semi-axis lengths
    a = sqrt(D(1,1));
    b = sqrt(D(2,2));
    
    % Parameter for ellipse
    theta = linspace(0, 2*pi, 100);
    
    % Generate ellipse in standard position
    ellipse_std = [a * cos(theta); b * sin(theta)];
    
    % Transform to actual position and orientation
    ellipse_points = V * ellipse_std + center;
    
    % Plot the ellipse
    plot(ellipse_points(1, :), ellipse_points(2, :), ...
        'Color', color, ...
        'LineWidth', linewidth, ...
        'DisplayName', displayname);
end

% Function to calculate the true minimum distance between agent and convex
% hull for plotting purposes. Logic inspired by Holmberg (2016)
function true_distance = computeTrueDistanceToHull(obj)
    hull = obj.true_hull_vertices;
    numVertices = size(hull, 1);

    min_distance = inf;

    % Going through each of the vertices to find minimum distance
    for i = 1:numVertices

        % Wrap around from last to first
        A = hull(i, :)';
        B = hull(mod(i, numVertices) + 1, :)';

        AB = B - A;
        AP = obj.p - A;

        % Project point onto edge segment
        t_param = max(0, min(1, dot(AP, AB) / dot(AB, AB)));
        proj = A + t_param * AB;
        distance = norm(obj.p - proj);

        % Finding if distance is less than min previously achieved
        if distance < min_distance
            min_distance = distance;
        end
    end

    true_distance = min_distance;
end
% Standalone version for plotting
function [center, A_matrix] = computeMinEnclosingEllipseKhachiyan(points)
    n = size(points, 1);  % number of points
    d = size(points, 2);  % dimension (should be 2)
    
    % Handle edge cases
    if n <= d
        % Not enough points for proper ellipse - use simple bounding
        center = mean(points, 1)';
        ranges = max(points) - min(points);
        A_matrix = diag(max(ranges/2, 0.1).^2);
        return;
    end
    
    % Khachiyan's algorithm
    tolerance = 1e-8;
    max_iterations = 1000;
    
    % Initialize: assume points are columns of Q
    Q = points';  % 2 x n matrix
    u = ones(n, 1) / n;  % uniform weights
    
    for iter = 1:max_iterations
        % Compute weighted center
        center = Q * u;
        
        % Center the points
        Q_centered = Q - center * ones(1, n);
        
        % Compute current ellipse matrix M
        M = Q_centered * diag(u) * Q_centered';
        
        % Ensure M is invertible
        if rcond(M) < 1e-12
            M = M + 1e-10 * eye(d);
        end
        
        % Compute distances squared
        M_inv = M \ eye(d);
        distances_sq = zeros(n, 1);
        for i = 1:n
            diff = Q_centered(:, i);
            distances_sq(i) = diff' * M_inv * diff;
        end
        
        % Find point with maximum distance
        [max_dist_sq, max_idx] = max(distances_sq);
        
        % Check convergence
        if max_dist_sq <= d * (1 + tolerance)
            break;
        end
        
        % Compute step size
        step_size = (max_dist_sq - d - 1) / ((d + 1) * (max_dist_sq - 1));
        step_size = max(0, min(step_size, 1));
        
        % Update weights
        u = u * (1 - step_size);
        u(max_idx) = u(max_idx) + step_size;
        
        % Normalize weights (should sum to 1)
        u = u / sum(u);
    end
    
    % Final computation
    center = Q * u;
    Q_centered = Q - center * ones(1, n);
    M = Q_centered * diag(u) * Q_centered';
    
    % Ensure positive definite and scale appropriately
    [V, D] = eig(M);
    D = diag(max(diag(D), 1e-8));
    M = V * D * V';
    
    % The ellipse matrix should be scaled by dimension factor
    A_matrix = M * d;
    
    % Verify all points are inside and add small buffer if needed
    M_inv = A_matrix \ eye(d);
    max_actual_dist = 0;
    for i = 1:n
        diff = points(i, :)' - center;
        dist = sqrt(diff' * M_inv * diff);
        max_actual_dist = max(max_actual_dist, dist);
    end
    
    if max_actual_dist > 1
        A_matrix = A_matrix * (max_actual_dist^2 * 1.01);
    end
end