
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MATLAB Simulation of Multi-Target Bearing 
% only Target Localization and Circumnavigation (BoTLC),

% Written by Aiden Bergheim For Research Thesis A and inspired by
% the MATLAB simulation written by Donglin Sui: https://github.com/Gloogger

clear
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Initial Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    tBegin = 0;             % (seconds)
    tFinal = 80;            % (seconds)
    frequency = 1000;
    dT     = 0.01;         % (seconds)
    
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
    
        p_0             = [-1.45; -1.55];      % initial location of the agent 
       
        x_hat_0         = [1, 0.5, 0.3; 
                           0, 0.234, 0.3];


    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_s = 0.1;           
       
        % PDT algorthm parameters
            alpha_0 = 0.5;
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc0 = 2;
            Tc1 = 2;
            Tc2 = 4;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        d_des_handle1 = @(time, theta, x_hat_positions, c_hat, y) 1.1 + 0.1*sin(11*theta);

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    

    Agent1 = Agent_Nonholonomic(p_0, x_hat_0, k_s, Tc0, Tc1, Tc2, ...
                        alpha_0, alpha_1, alpha_2, d_des_handle1, tSteps, qtyTargets, Targets, initial_heading);

    Agent2 = Agent_Nonholonomic(p_0, x_hat_0, k_s, Tc0, Tc1, Tc2, ...
                        alpha_0, alpha_1, alpha_2, d_des_handle1, tSteps, qtyTargets, Targets, initial_heading);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    
    Agent1 = Agent1.getBearings(t, Targets);                    % --- Take bearing measurements
    Agent1 = Agent1.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    Agent1 = Agent1.updateDesiredDistance(t, dT);
    Agent1 = Agent1.controlInputPDTv9(t, dT);                % --- Run control law
    Agent1 = Agent1.moveNonHolonomic(dT, t);                                % --- Execute control law

    
    Agent2 = Agent2.getBearings(t, Targets);                    % --- Take bearing measurements
    Agent2 = Agent2.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    Agent2 = Agent2.updateDesiredDistance(t, dT);
    Agent2 = Agent2.controlInputPDTv8(t, dT);                % --- Run control law
    Agent2 = Agent2.moveNonHolonomic(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index
targetColors = lines(qtyTargets); 
t_vec = (0:mytStepFinal-1) * dT; % Create a time vector

% Define agent colors
color_agent1 = [0, 0, 0.8]; % Blue - with saturation
color_agent2 = [0.8, 0, 0]; % Red  - without saturation

figure('Position', [50 200 720 320]);
t = tiledlayout(1, 2); % Create a 2-row, 3-column grid

%  ---- (a) Agent Trajectory plot -----
ax1 = nexttile(1); 
hold(ax1, 'on');

% 1. PLOT TARGETS
h_targets = gobjects(qtyTargets, 1);
for i = 1:qtyTargets
    xT = Targets(1, i);
    yT = Targets(2, i);
    h_targets(i) = plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(i, :),...
        'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
end

% 2. PLOT START POSITION
h_start = plot(Agent1.p_traj(1,1), Agent1.p_traj(2,1), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'none', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');

% 3. PLOT FINAL POSITIONS
h_final_agent1 = plot(Agent1.p_traj(1,mytStepFinal), Agent1.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', color_agent1, ...
    'MarkerFaceColor', color_agent1, ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(80)$$ [With Saturation]');

h_final_agent2 = plot(Agent2.p_traj(1,mytStepFinal), Agent2.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', color_agent2, ...
    'MarkerFaceColor', color_agent2, ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(80)$$ [Without Saturation]');

% 4. PLOT PATHS — Agent2 first, then Agent1 on top
h_path_agent2 = plot(ax1, Agent2.p_traj(1,:), Agent2.p_traj(2,:), ...
    'LineWidth', 1, ...
    'Color', color_agent2, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Without Saturation]');

h_path_agent1 = plot(ax1, Agent1.p_traj(1,:), Agent1.p_traj(2,:), ...
    'LineWidth', 1, ...
    'Color', color_agent1, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [With Saturation]');


% ==== Axis properties ====
axis(ax1, 'equal');
box(ax1, 'on'); 
title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
xlabel(ax1, 'x (m)', 'Interpreter','latex')
ylabel(ax1, 'y (m)', 'Interpreter','latex')
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
legend(ax1, [h_path_agent1, h_path_agent2, h_final_agent1, h_final_agent2,h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'northwest', 'NumColumns', 2);
xlim(ax1, [-2.5 2]);
ylim(ax1, [-2 2.5]);

hold(ax1, 'off');

%  ---- (b) Tracking Error Plot -----
ax2 = nexttile(2); 
hold(ax2, 'on');

% Agent2 first, then Agent1 on top
plot(ax2, t_vec, real(Agent2.delta_traj(1:mytStepFinal)),...
    'DisplayName', '$$\delta(t)$$ [Without Saturation]', ...
    'LineWidth', 1, ...
    'Color', color_agent2);

plot(ax2, t_vec, real(Agent1.delta_traj(1:mytStepFinal)),...
    'DisplayName', '$$\delta(t)$$ [With Saturation]', ...
    'LineWidth', 1, ...
    'Color', color_agent1);

% --- Vertical Line for Tracking Convergence ---
if exist('Tc1', 'var') && exist('Tc2', 'var')
    xline(ax2, Tc1 + Tc2, '-.', {'$T_{c,1} + T_{c,2}$'}, ...
        'Interpreter', 'latex', ...
        'FontSize', 10, ...
        'LabelVerticalAlignment', 'top', ...
        'HandleVisibility', 'off'); 
end

% --- Axis Properties ---
xlim(ax2, [0, tFinal])
ylim(ax2, [-0.2, 1.6])
box(ax2, 'on');
title(ax2, '(b) Tracking Errors', 'Interpreter','latex')
xlabel(ax2, 'time (sec)', 'Interpreter','latex')
ylabel(ax2, 'Errors (m)', 'Interpreter','latex')
grid(ax2, 'on');
set(ax2, 'FontSize', 10);
legend(ax2, 'Interpreter','latex', 'Location','northeast');
pbaspect(ax2, [2 1 1]); 
hold(ax2, 'off');

% --- Final layout adjustments ---
t.TileSpacing = 'compact';
t.Padding = 'compact';

% --- Zoomed inset on ax2 ---
drawnow;

x_zoom = [3.5, 25];
y_zoom = [-0.075, 0.075];

ax2_inset = axes('Position', [0.72, 0.44, 0.16, 0.15]);
box(ax2_inset, 'on');

% Both agents in inset — Agent2 first, Agent1 on top
plot(ax2_inset, t_vec, real(Agent2.delta_traj(1:mytStepFinal)), ...
    'LineWidth', 1, 'Color', color_agent2);
hold(ax2_inset, 'on');
plot(ax2_inset, t_vec, real(Agent1.delta_traj(1:mytStepFinal)), ...
    'LineWidth', 1, 'Color', color_agent1);
if exist('Tc1', 'var') && exist('Tc2', 'var')
    xline(ax2_inset, Tc1 + Tc2, '-.', ...
        'Interpreter', 'latex', ...
        'FontSize', 10, ...
        'LabelVerticalAlignment', 'top', ...
        'HandleVisibility', 'off'); 
end

hold(ax2_inset, 'off');

xlim(ax2_inset, x_zoom);
ylim(ax2_inset, y_zoom);
set(ax2_inset, 'FontSize', 7);
grid(ax2_inset, 'on');

% Green rectangle on ax2 showing zoomed region
rectangle(ax2, 'Position', [x_zoom(1) - 1, y_zoom(1) - 0.01, diff(x_zoom), diff(y_zoom)], ...
    'EdgeColor', [0, 0.6, 0], 'LineWidth', 1.2);

% Connector lines from rectangle to inset
data2fig = @(ax, xd, yd) [...
    ax.Position(1) + (xd - ax.XLim(1)) / diff(ax.XLim) * ax.Position(3), ...
    ax.Position(2) + (yd - ax.YLim(1)) / diff(ax.YLim) * ax.Position(4)];

p1 = data2fig(ax2, x_zoom(2), y_zoom(2));
p2 = [ax2_inset.Position(1), ax2_inset.Position(2) + ax2_inset.Position(4)];
annotation('line', [p1(1), p2(1)], [p1(2), p2(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);

p3 = data2fig(ax2, x_zoom(2), y_zoom(1));
p4 = [ax2_inset.Position(1), ax2_inset.Position(2)];
annotation('line', [p3(1), p4(1)], [p3(2), p4(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);
