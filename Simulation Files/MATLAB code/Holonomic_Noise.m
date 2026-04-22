
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    tFinal = 30;            % (seconds)
    frequency = 1000;
    dT     = 0.01;         % (seconds)
    
    tSteps = (tFinal-tBegin)/dT;   % force tSteps to be an integer

    % ----------------------------------------
    %        Define Multiple Targets
    % ----------------------------------------
    
    Targets       = [-1, 4, 3, 1; 
                     4, 5, 0, 1];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);
    

    % ----------------------------------------
    %              Define Agent
    % ----------------------------------------
    
    % ---- Initial Conditions ----
    
        p_0             = [16; 0];      % initial location of the agent 
       

        x_hat_0         = [7.7, 7.812, 7.7, 7.703; 
                           0, 0.234, 0, 0.042];

    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_omega = 1;           
       
        % PDT algorthm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 2;
            Tc2 = 2;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 4;

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    Agent = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    
    Agent = Agent.getBearingsWithNoise(t, Targets);                    % --- Take bearing measurements
    Agent = Agent.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    Agent = Agent.updateDesiredDistance(t, dT);
    Agent = Agent.controlInputPDTUpdated(t, Tc1, Tc2, dT);                % --- Run control law
    Agent = Agent.move(dT, t);                                % --- Execute control law

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
        'DisplayName', '$$\mbox{\boldmath$y$}(30)$$ ');
    
    % Plot agent path
    plot(ax1, Agent.p_traj(1,:), Agent.p_traj(2,:), ...
        'LineWidth', 1, ...
        'Color', [0, 0, 0.5], ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ ');   
    
    % Axis properties
    axis(ax1, 'equal');
    box(ax1, 'on'); 
    title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
    xlabel(ax1, 'x (m)', 'Interpreter','latex')
    ylabel(ax1, 'y (m)', 'Interpreter','latex')
    grid(ax1, 'on');
    set(ax1, 'FontSize', 10);
    xlim(ax1, [-4 18]);
    ylim(ax1, [-5 17]);
    legend(ax1, 'Interpreter','latex', 'Location','north', 'NumColumns', 2);
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
    xlim(ax2, [-4 14]);
    ylim(ax2, [-5 13]);
    legend(ax2, 'Interpreter','latex', 'Location','north', 'NumColumns', 3);
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
            'LabelVerticalAlignment', 'top', ...
            'HandleVisibility', 'off'); 
    end

    % --- Axis Properties ---
    xlim(ax3, [0, tFinal])
    ylim(ax3, [-1, 14])
    box(ax3, 'on');
    title(ax3, '(c) Tracking Errors', 'Interpreter','latex')
    xlabel(ax3, 'time (sec)', 'Interpreter','latex')
    ylabel(ax3, 'Errors (m)', 'Interpreter','latex')
    grid(ax3, 'on');
    set(ax3, 'FontSize', 10);
    legend(ax3, 'Interpreter','latex', 'Location','northeast');
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
            'LabelVerticalAlignment', 'top', ...
            'HandleVisibility', 'off'); 
    end

    % Axis Properties
    xlim(ax4, [0, tFinal])
    ylim(ax4, [-1, 14])
    box(ax4, 'on'); 
    title(ax4, '(d) Estimation Errors', 'Interpreter','latex')
    xlabel(ax4, 'time (sec)', 'Interpreter','latex')
    ylabel(ax4, 'Errors (m)', 'Interpreter','latex')
    grid(ax4, 'on');
    set(ax4, 'FontSize', 10);
    legend(ax4, 'Interpreter','latex', 'Location','northeast');
    pbaspect(ax4, [2 1 1]); 
    
    hold(ax4, 'off');

% --- Final layout adjustments for the whole figure ---
t.TileSpacing = 'compact';
t.Padding = 'compact';




% --- Zoomed inset on ax4 (MUST come after all nexttile calls) ---
drawnow; % Force layout to finalise tile positions before reading them

x_zoom = [1, 6];      % x range to zoom into - adjust to your data
y_zoom = [0, 0.5];  % y range to zoom into - adjust to your data

ax4_inset = axes('Position', [0.74, 0.2, 0.1, 0.2]); % [left bottom width height] in figure coords
box(ax4_inset, 'on');

    for i = 1:qtyTargets
        hold(ax4_inset, 'on')
        err_norm_i = vecnorm(Agent.x_tilde_traj{i}(:, 1:mytStepFinal));
        plot(ax4_inset, t_vec, err_norm_i, ...
            'LineWidth', 1, ...
            'LineStyle', '--', ...
            'Color', targetColors(i, :))
    end
hold(ax4_inset, 'off')
xlim(ax4_inset, x_zoom);
ylim(ax4_inset, y_zoom);
set(ax4_inset, 'FontSize', 7);
grid(ax4_inset, 'on');

% Draw green rectangle on ax4 showing the zoomed region
rectangle(ax4, 'Position', [x_zoom(1), y_zoom(1) - 0.4, diff(x_zoom), 2*diff(y_zoom)], ...
    'EdgeColor', [0, 0.6, 0], 'LineWidth', 1.2);

% Helper: convert data coordinates on an axes -> normalised figure coordinates
data2fig = @(ax, xd, yd) [...
    ax.Position(1) + (xd - ax.XLim(1)) / diff(ax.XLim) * ax.Position(3), ...
    ax.Position(2) + (yd - ax.YLim(1)) / diff(ax.YLim) * ax.Position(4)];

% Top-right corner of rectangle -> top-left of inset
p1 = data2fig(ax4, x_zoom(2), y_zoom(2)); % top-right of rectangle
p2 = [ax4_inset.Position(1), ...                          % left
      ax4_inset.Position(2) + ax4_inset.Position(4)];     % top
annotation('line', [p1(1), p2(1)], [p1(2), p2(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);

% Bottom-right corner of rectangle -> bottom-left of inset
p3 = data2fig(ax4, x_zoom(2), y_zoom(1)); % bottom-right of rectangle
p4 = [ax4_inset.Position(1), ...          % left
      ax4_inset.Position(2)];             % bottom
annotation('line', [p3(1), p4(1)], [p3(2), p4(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);


% --- Zoomed inset on ax3 (MUST come after all nexttile calls) ---
drawnow; % Force layout to finalise tile positions before reading them

x_zoom = [6, 26];      % x range to zoom into - adjust to your data
y_zoom = [-0.2, 0.2];  % y range to zoom into - adjust to your data

ax3_inset = axes('Position', [0.75, 0.69, 0.14, 0.08]); % [left bottom width height] in figure coords
box(ax3_inset, 'on');
hold(ax3_inset, 'on')
plot(ax3_inset, t_vec, real(Agent.delta_traj(1:mytStepFinal)),...
        'DisplayName', '$$\delta(t)$$ [Sinusoidal]', ...
        'LineWidth', 1, ...
        'Color'      ,  [0, 0, 0.5]);

    hold(ax3_inset, 'off')
xlim(ax3_inset, x_zoom);
ylim(ax3_inset, y_zoom);
set(ax3_inset, 'FontSize', 7);
grid(ax3_inset, 'on');

% Draw green rectangle on ax3 showing the zoomed region
rectangle(ax3, 'Position', [x_zoom(1), y_zoom(1) - 0.2, diff(x_zoom), 2*diff(y_zoom)], ...
    'EdgeColor', [0, 0.6, 0], 'LineWidth', 1.2);

% Helper: convert data coordinates on an axes -> normalised figure coordinates
data2fig = @(ax, xd, yd) [...
    ax.Position(1) + (xd - ax.XLim(1)) / diff(ax.XLim) * ax.Position(3), ...
    ax.Position(2) + (yd - ax.YLim(1)) / diff(ax.YLim) * ax.Position(4)];

% Top-left corner of rectangle -> bottom-left of inset
p1 = data2fig(ax3, x_zoom(1), y_zoom(2));
p2 = [ax3_inset.Position(1), ax3_inset.Position(2)];
annotation('line', [p1(1), p2(1)], [p1(2), p2(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);

% Top-right corner of rectangle -> bottom-right of inset
p3 = data2fig(ax3, x_zoom(2), y_zoom(2));
p4 = [ax3_inset.Position(1) + ax3_inset.Position(3), ax3_inset.Position(2)];
annotation('line', [p3(1), p4(1)], [p3(2), p4(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);
