
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
    tFinal = 50;            % (seconds)
    dT     = 0.01;         % (seconds)
    
    tSteps = (tFinal-tBegin)/dT;   % force tSteps to be an integer

    % ----------------------------------------
    %        Define Multiple Targets
    % ----------------------------------------
    
    Targets       = [2, 1, 3; 
                     4, 2, 3];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);
    

    % ----------------------------------------
    %              Define Agent
    % ----------------------------------------
    
    % ---- Initial Conditions ----
    
        p_0             = [9; 8];      % initial location of the agent 
       
        x_hat_0 = [3, 3, 7;
                    2, 0, 3];

        

    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_omega = 5;           
       
        % PDT algorthm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 0.2;
            Tc2 = 0.4;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) 1.91;
        
        

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, 10, 0);

    AgentMD = AgentDeghat(p_0, x_hat_0, 5, 5, 0.5, ...
                        tSteps, qtyTargets, Targets, initial_heading, 10, 0);



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT = AgentPDT.updateDesiredDistance(t, dT);
    AgentPDT = AgentPDT.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

    AgentMD = AgentMD.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentMD = AgentMD.estimateTargetDeghat(t, dT, Targets);          % --- Run estimator
    AgentMD = AgentMD.controlInputDeghat(t, dT);                % --- Run control law
    AgentMD = AgentMD.move(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps));
targetColors = lines(qtyTargets);
t_vec = (0:mytStepFinal-1) * dT;
t_vec_2 = (0:mytStepFinal*10-1) * 0.001;

figure;
tl = tiledlayout(1, 2);

% =========================================================================
%  (a) Agent Trajectories  —  spans both rows of column 1
% =========================================================================
ax1 = nexttile(1);
hold(ax1, 'on');

% --- True target positions ---
h_targets = gobjects(qtyTargets, 1);
for i = 1:qtyTargets
    h_targets(i) = plot(ax1, Targets(1,i), Targets(2,i), '+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(i,:), ...
        'DisplayName', ['$$\mbox{\boldmath$x$}_' num2str(i) '$$']);
end

% --- Common start position (both agents share the same p_0) ---
h_start = plot(ax1, p_0(1), p_0(2), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'none', ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');

% --- Agent trajectories ---
h_path_MD = plot(ax1, [p_0(1), AgentMD.p_traj(1,1:mytStepFinal)], [p_0(2), AgentMD.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Deghat et al. (2014)]');

h_path_PDT = plot(ax1, [p_0(1), AgentPDT.p_traj(1,1:mytStepFinal)], [p_0(2), AgentPDT.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Proposed Method]');

% --- Final positions ---
h_final_MD = plot(ax1, AgentMD.p_traj(1,mytStepFinal), AgentMD.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0, 0.9], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Deghat et al. (2014)]']);

h_final_PDT = plot(ax1, AgentPDT.p_traj(1,mytStepFinal), AgentPDT.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0.9, 0, 0], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Proposed Method]']);

axis(ax1, 'equal');
box(ax1, 'on');
title(ax1, '(a) Agent Trajectories', 'Interpreter', 'latex')
xlabel(ax1, 'x (m)', 'Interpreter', 'latex')
ylabel(ax1, 'y (m)', 'Interpreter', 'latex')
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
xlim(ax1, [-6 11]);
ylim(ax1, [0 14]);
legend(ax1, [h_path_MD, h_path_PDT, h_final_MD, h_final_PDT, h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'best', 'NumColumns', 2);
hold(ax1, 'off');




% =========================================================================
%  (b) Tracking Errors  —  column 3, top row
% =========================================================================
ax3 = nexttile(2);
hold(ax3, 'on');

plot(ax3, t_vec, real(AgentMD.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Deghat et al. (2014)]', ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9]);

plot(ax3, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Proposed Method]', ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0]);

yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');

xlim(ax3, [0, tFinal]);
box(ax3, 'on');
title(ax3, '(b) Tracking Errors', 'Interpreter', 'latex')
xlabel(ax3, 'time (sec)', 'Interpreter', 'latex')
ylabel(ax3, 'Errors (m)', 'Interpreter', 'latex')
grid(ax3, 'on');
set(ax3, 'FontSize', 10);
legend(ax3, 'Interpreter', 'latex', 'Location', 'best');
pbaspect(ax3, [2 1 1]);
hold(ax3, 'off');
ylim([-1 8])

tl.TileSpacing = 'compact';
tl.Padding = 'compact';

