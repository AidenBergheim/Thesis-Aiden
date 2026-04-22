
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
    tFinal = 80;            % (seconds)
    dT     = 0.001;         % (seconds)
    
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

        %x_hat_0         = [7.7, 7.812, 7.7, 7.703; 
        %                   0, 0.234, 0, 0.042];   % agent's initial guess of target positions

        

    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_omega = 0.1;           
       
        % PDT algorthm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.9;
            Tc1 = 2;
            Tc2 = 3;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        d_des_handle1 = @(time, theta, x_hat_positions, c_hat, y) 1.1 + 0.1*sin(time);
     

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------

    AgentPDT1 = AgentDiscrete(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle1, tSteps, qtyTargets, Targets, initial_heading, 2, 0.2);



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
   
    AgentPDT1 = AgentPDT1.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT1 = AgentPDT1.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT1 = AgentPDT1.updateDesiredDistance(t, dT);
    AgentPDT1 = AgentPDT1.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT1 = AgentPDT1.move(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index
targetColors = lines(qtyTargets); 
t_vec = (0:mytStepFinal-1) * dT; % Create a time vector
figure('Position', [50 200 720 320]);
t = tiledlayout(1, 2); % Create a 2-row, 3-column grid

%  ---- (a) Agent Trajectory plot -----
ax1 = nexttile(1); 
hold(ax1, 'on');

% 2. PLOT TARGETS (Left Column of Legend)
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



% 4. PLOT START POSITION (Bottom Right)
h_start = plot(AgentPDT1.p_traj(1,1), AgentPDT1.p_traj(2,1), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'none', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');

% 5. PLOT FINAL POSITIONS (Right Column of Legend)
% Note: I've updated indices to match 1, 2, 3 for clarity in the handle array
h_path_PDT = plot(AgentPDT1.p_traj(1,mytStepFinal), AgentPDT1.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'k', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(80)$$');

% 3. PLOT PATHS (Bottom of Left/Right Columns)
h_final_PDT = plot(ax1, AgentPDT1.p_traj(1,:), AgentPDT1.p_traj(2,:), ...
    'LineWidth', 1, ...
    'Color', 'k', ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$');   


% ==== Axis properties ====
axis(ax1, 'equal');
box(ax1, 'on'); 
title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
xlabel(ax1, 'x (m)', 'Interpreter','latex')
ylabel(ax1, 'y (m)', 'Interpreter','latex')
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
legend(ax1, [h_path_PDT, h_final_PDT, h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'northwest', 'NumColumns', 1);
xlim(ax1, [-2.5 2]);
ylim(ax1, [-2 2.5]);

% ==== CUSTOM LEGEND ORDERING ====
% MATLAB fills legends Column-by-Column (Top-Down) by default.
% To get Targets Left | Finals Right, we construct two stacks:


hold(ax1, 'off');

%  ---- (c) Tracking Error Plot -----
    % This plot is in column 3, top row
    ax2 = nexttile(2); 
    hold(ax2, 'on');
    
    % Plotting PDT Agent Tracking Error
    plot(ax2, t_vec, real(AgentPDT1.delta_traj(1:mytStepFinal)),...
        'DisplayName', '$$\delta(t)$$', ...
        'LineWidth', 1, ...
        'Color'      ,  'k');
    hold on
   
    
    % --- Add Vertical Line for Tracking Convergence (Tc1 + Tc2) ---
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
    box(ax2, 'on'); % <--- ADDED THIS to close the box
    title(ax2, '(b) Tracking Errors', 'Interpreter','latex')
    xlabel(ax2, 'time (sec)', 'Interpreter','latex')
    ylabel(ax2, 'Errors (m)', 'Interpreter','latex')
    grid(ax2, 'on');
    set(ax2, 'FontSize', 10);
    legend(ax2, 'Interpreter','latex', 'Location','northeast');
    pbaspect(ax2, [2 1 1]); 
    hold(ax2, 'off');


% --- Final layout adjustments for the whole figure ---
t.TileSpacing = 'compact';
t.Padding = 'compact';


% --- Zoomed inset on ax2 (MUST come after all nexttile calls) ---
drawnow; % Force layout to finalise tile positions before reading them

x_zoom = [30, 50];      % x range to zoom into - adjust to your data
y_zoom = [-0.05, 0.05];  % y range to zoom into - adjust to your data

ax2_inset = axes('Position', [0.66, 0.44, 0.16, 0.2]); % [left bottom width height] in figure coords
box(ax2_inset, 'on');
plot(ax2_inset, t_vec, real(AgentPDT1.delta_traj(1:mytStepFinal)), ...
    'LineWidth', 1, 'Color', 'k');
xlim(ax2_inset, x_zoom);
ylim(ax2_inset, y_zoom);
set(ax2_inset, 'FontSize', 7);
grid(ax2_inset, 'on');

% Draw green rectangle on ax2 showing the zoomed region
rectangle(ax2, 'Position', [x_zoom(1), y_zoom(1) - 0.02, diff(x_zoom), 1.5 * diff(y_zoom)], ...
    'EdgeColor', [0, 0.6, 0], 'LineWidth', 1.2);

% Helper: convert data coordinates on an axes -> normalised figure coordinates
data2fig = @(ax, xd, yd) [...
    ax.Position(1) + (xd - ax.XLim(1)) / diff(ax.XLim) * ax.Position(3), ...
    ax.Position(2) + (yd - ax.YLim(1)) / diff(ax.YLim) * ax.Position(4)];

% Top-left corner of rectangle -> bottom-left of inset
p1 = data2fig(ax2, x_zoom(1), y_zoom(2));
p2 = [ax2_inset.Position(1), ax2_inset.Position(2)];
annotation('line', [p1(1), p2(1)], [p1(2), p2(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);

% Top-right corner of rectangle -> bottom-right of inset
p3 = data2fig(ax2, x_zoom(2), y_zoom(2));
p4 = [ax2_inset.Position(1) + ax2_inset.Position(3), ax2_inset.Position(2)];
annotation('line', [p3(1), p4(1)], [p3(2), p4(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);

