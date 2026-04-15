
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
    dT     = 0.01;         % (seconds)
    
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
fig1 = figure;        % create a figure
ax1 = axes(fig1);     % create axes inside the figure

    hold(ax1, 'on');

    % Plot targets
    for i = 1:qtyTargets
        xT = Targets(1, i);
        yT = Targets(2, i);
        h_targets(i) = plot(ax1, xT, yT, 'k+', ...
            'LineWidth', 1, ...
            'MarkerSize', 7, ...
            'Color', targetColors(i, :),...
            'DisplayName', ['Target ' num2str(i)]);
    end
    
    % Plot initial agent position
    plot(Agent.p_traj(1,1), Agent.p_traj(2,1), 'o', ...
        'MarkerSize', 8, ...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', 'none', ...
        'LineWidth', 1.5, ...
        'DisplayName', 'Initial Agent Position');
    
    % Plot final agent position
     plot(Agent.p_traj(1,mytStepFinal), Agent.p_traj(2,mytStepFinal), 'o', ...
        'MarkerSize', 8, ...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', [0, 0, 0.5], ...
        'LineWidth', 1.5, ...
        'DisplayName', 'Final Agent Position');
    
    % Plot agent path
    plot(ax1, Agent.p_traj(1,:), Agent.p_traj(2,:), ...
        'LineWidth', 1, ...
        'Color', [0, 0, 0.5], ...
        'DisplayName', 'Agent Trajectory');   
    
    % Axis properties
    axis(ax1, 'equal');
    box(ax1, 'on'); 
    title(ax1, 'Agent Trajectoy', 'Interpreter','latex')
    xlabel(ax1, 'x (m)', 'Interpreter','latex')
    ylabel(ax1, 'y (m)', 'Interpreter','latex')
    grid(ax1, 'on');
    set(ax1, 'FontSize', 10);
    xlim(ax1, [-6 11]);
    ylim(ax1, [0 14]);
    legend(ax1, 'Interpreter','latex', 'Location','best', 'NumColumns', 2);
    hold(ax1, 'off');
