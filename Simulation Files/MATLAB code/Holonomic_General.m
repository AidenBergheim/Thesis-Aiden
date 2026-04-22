%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MATLAB Simulation of Multi-Target Bearing 
% only Target Localization and Circumnavigation (BoTLC),

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
    dT     = 0.01;         % (seconds)
    
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
        d_des_handle1 = @(time, theta, x_hat_positions, c_hat, y) 5.7 + 0.2*sin(10*theta);
        d_des_handle2 = @(time, theta, x_hat_positions, c_hat, y) computeMinCircleRadius(theta, c_hat, x_hat_positions, y);
        d_des_handle3 = @(time, theta, x_hat_positions, c_hat, y) computeMinEllipseRadius(theta, c_hat, x_hat_positions, y);
        

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------

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

% PRE-ALLOCATE HANDLES
h_targets = gobjects(qtyTargets, 1); 
h_start   = gobjects(1, 1);
h_finals  = gobjects(3, 1);
h_paths   = gobjects(3, 1); 



% PLOT TARGETS 
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


stack_left = [h_start; h_targets]; 
stack_middle = [h_finals];
stack_right = [h_paths(1:3)];

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


% Computing the radius of the minimum circle around targets with safe
% distance
function distance = computeMinCircleRadius(theta, c_hat, x_hat_positions, y)
    
    r_s = 0.3;
    [center, radius] = minEnclosingCircleProp(x_hat_positions');
    R_safe = radius + r_s;
    
    % Vector from the "safe" circle's center to the centroid
    CO = c_hat - center;
    u_theta = [cos(theta); sin(theta)];
    
    % Coefficients of quadratic formula
    a = 1;
    b = 2 * dot(CO, u_theta);
    c = norm(CO)^2 - R_safe^2;
    
    % Solving
    distance = (-b + sqrt(b^2 - 4*a*c)) / (2*a);
end

% Computing radius of minimum ellipse with safe distance
function distance = computeMinEllipseRadius(theta, c_hat, x_hat_positions, y)
    
    r_s = 0.3; 
    
    % Get the Ellipse Properties
    [center, semi_axes, rotation_matrix] = minEnclosingEllipseStat(x_hat_positions');
    

    % The ellipse is defined by (P-c)' * A * (P-c) = 1
    % A = R * diag(1/a^2, 1/b^2) * R'
    a = semi_axes(1) + r_s;
    b = semi_axes(2) + r_s;
    R = rotation_matrix;
    
    % Create the diagonal matrix of squared inverse semi-axes
    D_inv_sq = diag([1/a^2; 1/b^2]);
    A = R * D_inv_sq * R';

    % finding 'd' such that P = c_hat + d*u_theta is on the ellipse.   
    u_theta = [cos(theta); sin(theta)];
    denominator = u_theta' * A * u_theta;
    
    % preventing division by zero
    distance = 1 / sqrt(denominator);
end


