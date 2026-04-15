
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MATLAB Simulation of Proposed Algorithms for Multi-Target Bearing 
% only Target Localization and Circumnavigation (BoTLC), by a single holonomic agent

% Written by Aiden Bergheim and inspired by the MATLAB simulation
% written by Donglin Sui: https://github.com/Gloogger

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
    
    Targets       = [-1, 3, 2, 1; 
                     0, 2, 0, 1];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);

    % ----------------------------------------
    %              Define Agent
    % ----------------------------------------
    
    % ---- Initial Conditions ----
    
        p_0             = [8; 0];      % initial location of the agent 
       
        x_hat_0         = [7.7, 7.812, 7.7, 7.703; 
                           0, 0.234, 0, 0.042];   % agent's initial guess of target positions

    % ---- Control Constants ----
    
        % PDT algorthm parameters
            k_s = 1;            % Speed during localization
            k_d = 1;            % Tangential speed during circumnavigation
            alpha_1 = 0.5;      % Control gain α₁
            alpha_2 = 0.5;      % Control gain α₂
            Tc1 = 1;            % Time constant 1
            Tc2 = 1;            % Time constant 2
    
        % Desired Distance to Targets
            d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 3.5;
       
    % ----------------------------------------
    %             Create agent
    % ----------------------------------------
    
    Agent = Agent(p_0, x_hat_0, k_s, k_d, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    Agent = Agent.updateDesiredDistance(t, dT);                 % --- Update desired distance to centroid
    Agent = Agent.getBearings(t, Targets);                      % --- Take bearing measurements
    Agent = Agent.estimateTargetPDT(t, dT, Targets);       % --- Run estimator
    Agent = Agent.controlInputPDT(t, dT);             % --- Run control law
    Agent = Agent.move(dT, t);                                  % --- Execute control law
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
    ax1 = nexttile(1, [2 1]); 
    hold(ax1, 'on');

    % Plot targets
    for i = 1:qtyTargets
        xT = Targets(1, i);
        yT = Targets(2, i);
        h_targets(i) = plot(ax1, xT, yT, 'k+', ...
            'LineWidth', 1, ...
            'MarkerSize', 7, ...
            'Color', targetColors(i, :),...
            'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
    end
    
    % Plot initial agent position
    plot(Agent.p_traj(1,1), Agent.p_traj(2,1), 'o', ...
        'MarkerSize', 8, ...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', 'none', ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');
    
    % Plot final agent position
     plot(Agent.p_traj(1,mytStepFinal), Agent.p_traj(2,mytStepFinal), 'o', ...
        'MarkerSize', 8, ...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', [0, 0, 0.5], ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(42)$$ [Circular]');
    
    % Plot agent path
    plot(ax1, Agent.p_traj(1,:), Agent.p_traj(2,:), ...
        'LineWidth', 1, ...
        'Color', [0, 0, 0.5], ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Circular]');   
    
    % Axis properties
    axis(ax1, 'equal');
    box(ax1, 'on'); 
    title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
    xlabel(ax1, 'x (m)', 'Interpreter','latex')
    ylabel(ax1, 'y (m)', 'Interpreter','latex')
    grid(ax1, 'on');
    set(ax1, 'FontSize', 10);
    xlim(ax1, [-6 11]);
    ylim(ax1, [-6 14]);
    legend(ax1, 'Interpreter','latex', 'Location','best', 'NumColumns', 2);
    hold(ax1, 'off');

% ------- (b) Estimate Trajectories plot --------
    ax2 = nexttile(2, [2 1]); 
    hold(ax2, 'on');
    
    % Plot target estimation trajectories
    for i = 1:qtyTargets
        plot(ax2, Agent.x_hat{i}(1, 1:mytStepFinal), Agent.x_hat{i}(2, 1:mytStepFinal), ...
            'LineWidth', 1, ...
            'Color', targetColors(i, :),...
            'LineStyle'  ,  '--', ...
            'DisplayName', ['$$\mbox{\boldmath$\hat{x}$}_' num2str(i) '(t)$$']);
    end

    % Plot initial target position estimates
    for i = 1:qtyTargets
        plot(ax2, Agent.x_hat{i}(1, 1), Agent.x_hat{i}(2, 1), 'o', ...
            'MarkerSize', 6, ...
            'MarkerEdgeColor', targetColors(i, :), ...
            'MarkerFaceColor', 'none', ...
            'LineWidth', 1.5, ...
            'DisplayName', ['$$\mbox{\boldmath$\hat{x}$}_' num2str(i) '(0)$$']);
    end
    
    % Plot actual target positions
    for i = 1:qtyTargets
        xT = Targets(1, i);
        yT = Targets(2, i);
        plot(ax2, xT, yT, '+', ...
            'Color', targetColors(i, :),...
            'LineWidth', 1, ...
            'MarkerSize', 7, ...
            'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
    end
        
    % Axis properties
    axis(ax2, 'equal');
    box(ax2, 'on');
    title(ax2, '(b) Estimate Trajectories', 'Interpreter','latex')
    xlabel(ax2, 'x (m)', 'Interpreter','latex')
    ylabel(ax2, 'y (m)', 'Interpreter','latex')
    grid(ax2, 'on');
    set(ax2, 'FontSize', 10);
    xlim(ax2, [-6 11]);
    ylim(ax2, [-6 14]);
    legend(ax2, 'Interpreter','latex', 'Location','best', 'NumColumns', 3);
    hold(ax2, 'off');

%  ---- (c) Tracking Error Plot -----
    ax3 = nexttile(3); 
    hold(ax3, 'on');
    
    % Plotting PDT Agent Tracking Error
    plot(ax3, t_vec, real(Agent.delta_traj(1:mytStepFinal)),...
        'DisplayName', '$$\delta(t)$$ [Sinusoidal]', ...
        'LineWidth', 1, ...
        'Color'      ,  [0, 0, 0.5]);
    hold on
    
    % Add Vertical Line for Tracking Convergence (Tc1 + Tc2)
    if exist('Tc1', 'var') && exist('Tc2', 'var')
        xline(ax3, Tc1 + Tc2, '-.', {'$T_{c,1} + T_{c,2}$'}, ...
            'Interpreter', 'latex', ...
            'FontSize', 10, ...
            'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility', 'off'); 
    end

    % --- Axis Properties ---
    xlim(ax3, [0, tFinal])
    box(ax3, 'on');
    title(ax3, '(c) Tracking Errors', 'Interpreter','latex')
    xlabel(ax3, 'time (sec)', 'Interpreter','latex')
    ylabel(ax3, 'Errors (m)', 'Interpreter','latex')
    grid(ax3, 'on');
    set(ax3, 'FontSize', 10);
    legend(ax3, 'Interpreter','latex', 'Location','best');
    pbaspect(ax3, [2 1 1]); 
    hold(ax3, 'off');

% ------ (d) Estimation Error Plot ---------
    ax4 = nexttile(6); 
    hold(ax4, 'on');
    
    % Plotting Estimation Errors
    for i = 1:qtyTargets
        err_norm_i = vecnorm(Agent.x_tilde_traj{i}(:, 1:mytStepFinal));
        plot(ax4, t_vec, err_norm_i, ...
            'LineWidth', 1.3, ...
            'LineStyle', '--', ...
            'Color', targetColors(i, :),...
            'DisplayName', ['$$\|\mbox{\boldmath$\tilde{x}$}_' num2str(i) '(t)\|$$']);
    end

    % Add Vertical Line for Estimation Convergence (Tc1)
    if exist('Tc1', 'var')
        xline(ax4, Tc1, '-.', {'$T_{c,1}$'}, ...
            'Interpreter', 'latex', ...
            'FontSize', 10, ...
            'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility', 'off'); 
    end

    % Axis Properties
    xlim(ax4, [0, tFinal])
    box(ax4, 'on'); 
    title(ax4, '(d) Estimation Errors', 'Interpreter','latex')
    xlabel(ax4, 'time (sec)', 'Interpreter','latex')
    ylabel(ax4, 'Errors (m)', 'Interpreter','latex')
    grid(ax4, 'on');
    set(ax4, 'FontSize', 10);
    legend(ax4, 'Interpreter','latex', 'Location','best');
    pbaspect(ax4, [2 1 1]); 
    hold(ax4, 'off');

% --- Final layout adjustments for the whole figure ---
t.TileSpacing = 'compact';
t.Padding = 'compact';
