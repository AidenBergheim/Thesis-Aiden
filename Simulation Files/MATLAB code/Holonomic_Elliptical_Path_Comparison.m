
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
    dT     = 0.01;         % (seconds)
    tSteps = (tFinal-tBegin)/dT; 

% ----------------------------------------
%        Define Multiple Targets
% ----------------------------------------
    Targets       = [-6, -2, 3, 5;
                      0,  0, 0, 0];  
    qtyTargets = size(Targets, 2);
% ----------------------------------------
%              Define Agent
% ----------------------------------------

% ---- Initial Conditions ----
        p_0             = [20; 10];      % Agent 1 initial position, Section 5
        x_hat_0 = [-4, 0, 5, 7;
                    -2, 2, 2, 2];          % perturbed initial estimates

% ---- Control Constants ----
% control gain for adjusting tangential speed
            k_omega = 10;
% PDT algorithm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 0.2;
            Tc2 = 0.4;
% Cao algorithm parameters
            alpha = 5;
% Non-holonomic parameters
            initial_heading = pi/2;
% Desired Distance to Targets
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeChunTianEllipseRadius(theta);
% ----------------------------------------
%             Create agents
% ----------------------------------------
% Proposed agent
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);
% Agent utilizing Chun & Tian (2020): a1=8, b1=4, alpha1=0, k1=3, eta1=1.5, eps=3
    AgentChunTian = ChunTianAgent(p_0, x_hat_0, 8, 4, 0, 3, 1.5, 3, ...
                    tSteps, qtyTargets, Targets, initial_heading, 1000, 0);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT = AgentPDT.updateDesiredDistance(t, dT);
    AgentPDT = AgentPDT.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

    AgentChunTian = AgentChunTian.measureBearings(Targets);
    AgentChunTian = AgentChunTian.updatePositionEstimator(dT);
    AgentChunTian = AgentChunTian.updateGeometricCentreEstimator({}, dT);
    AgentChunTian = AgentChunTian.computeControl(t);
    AgentChunTian = AgentChunTian.move(dT, t);

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps));
targetColors = lines(qtyTargets);
t_vec = (0:mytStepFinal-1) * dT;
t_vec_2 = (0:mytStepFinal*10-1) * 0.001;

figure('Position', [50 200 720 320]);
tl = tiledlayout(1, 2);

% =========================================================================
%  (a) Agent Trajectories 
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
h_path_MD = plot(ax1, [p_0(1), AgentChunTian.p_traj(1,1:mytStepFinal)], [p_0(2), AgentChunTian.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Chun \& Tian (2020)]');

h_path_PDT = plot(ax1, [p_0(1), AgentPDT.p_traj(1,1:mytStepFinal)], [p_0(2), AgentPDT.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Proposed Method]');

% --- Final positions ---
h_final_MD = plot(ax1, AgentChunTian.p_traj(1,mytStepFinal), AgentChunTian.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0, 0.9], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Chun \& Tian (2020)]']);

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
xlim([-20 30])
ylim([-10 30])
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
legend(ax1, [h_path_MD, h_path_PDT, h_final_MD, h_final_PDT, h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'north', 'NumColumns', 2);
hold(ax1, 'off');


% =========================================================================
%  (b) Tracking Errors  — 
% =========================================================================
ax2 = nexttile(2);
hold(ax2, 'on');

plot(ax2, t_vec, real(AgentChunTian.d_tilde_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\|x_i - \xi^*\| - \rho_{d_i}(\psi_i)$$ [Chun \& Tian (2020)]', ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9]);

plot(ax2, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Proposed Method]', ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0]);

yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');

xlim(ax2, [0, tFinal]);
box(ax2, 'on');
title(ax2, '(b) Tracking Errors', 'Interpreter', 'latex')
xlabel(ax2, 'time (sec)', 'Interpreter', 'latex')
ylabel(ax2, 'Errors (m)', 'Interpreter', 'latex')
grid(ax2, 'on');
set(ax2, 'FontSize', 10);
legend(ax2, 'Interpreter', 'latex', 'Location', 'northeast');
pbaspect(ax2, [2 1 1]);
hold(ax2, 'off');
ylim([-1 18])


tl.TileSpacing = 'compact';
tl.Padding = 'compact';

% Computes the ellipse for the ellipse from the Chun and Tian paper
function distance = computeChunTianEllipseRadius(theta)

    a     = 8;
    b     = 4;
    alpha = 0;

    theta_rel = theta - alpha;
    denom     = sqrt(a^2 * sin(theta_rel)^2 + b^2 * cos(theta_rel)^2);

    distance = (a * b) / denom;
end

