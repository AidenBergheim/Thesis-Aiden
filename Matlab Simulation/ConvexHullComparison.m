
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
    tFinal = 30;            % (seconds)
    dT     = 0.001;         % (seconds)
    tSteps = (tFinal-tBegin)/dT;   % force tSteps to be an integer
% ----------------------------------------
%        Define Multiple Targets
% ----------------------------------------
    Targets       = [-2, 4, 2, 1;
                      0, 5, 0, 1];    % Chun & Tian (2020) Section 5
    qtyTargets = size(Targets, 2);
% ----------------------------------------
%              Define Agent
% ----------------------------------------
% ---- Initial Conditions ----
        p_0             = [8; 0];      % Agent 1 initial position, Section 5
        x_hat_0 = [7.7, 7.812, 7.7, 7.703
                    0, 0.234, 0, 0.042];          % perturbed initial estimates

    K = convhull(Targets(1,:)', Targets(2,:)');
    hullVertices = Targets(:, K)';


% ---- Control Constants ----
% control gain for adjusting tangential speed
            k_omega = 1;
% PDT algorithm parameters
            alpha_1 = 0.5;
            alpha_2 = 0.2;
            Tc1 = 0.2;
            Tc2 = 0.4;
% Cao algorithm parameters
            alpha = 5;
% Non-holonomic parameters
            initial_heading = pi/2;
% Desired Distance to Targets
        d_des_handle = @(time, theta, x_hat_positions, c_hat, y) computeConvexHullRadius(theta, c_hat, x_hat_positions, y);
% ----------------------------------------
%             Create agents
% ----------------------------------------
% Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);

    rs = 0.3;
    AgentCao = CaoAgent2(p_0, x_hat_0, 1, 0.5, 5, 0.3, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps
    
    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets, Tc1);          % --- Run estimator
    AgentPDT = AgentPDT.updateDesiredDistance(t, dT);
    AgentPDT = AgentPDT.controlInputPDT(t, Tc1, Tc2, dT);                % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

    AgentCao = AgentCao.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentCao = AgentCao.estimateTargetsCao(t, dT, Targets);          % --- Run estimator
    AgentCao = AgentCao.getPsiAndProjectionHull(t, qtyTargets, Targets);
    AgentCao = AgentCao.controlInputCao(t);                % --- Run control law
    AgentCao = AgentCao.move(dT, t);                                % --- Execute control law
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mytStepFinal = max(1, min(round(tFinal/dT), tSteps));
targetColors = lines(qtyTargets);
t_vec = (0:mytStepFinal-1) * dT;
t_vec_2 = (0:mytStepFinal*10-1) * 0.001;

figure('Position', [50 200 720 400]);
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
h_path_MD = plot(ax1, [p_0(1), AgentCao.p_traj(1,1:mytStepFinal)], [p_0(2), AgentCao.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Cao et al. (2021)]');

h_path_PDT = plot(ax1, [p_0(1), AgentPDT.p_traj(1,1:mytStepFinal)], [p_0(2), AgentPDT.p_traj(2,1:mytStepFinal)], ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0], ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Proposed Method]');

% --- Final positions ---
h_final_MD = plot(ax1, AgentCao.p_traj(1,mytStepFinal), AgentCao.p_traj(2,mytStepFinal), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0, 0.9], ...
    'LineWidth', 1.5, ...
    'DisplayName', ['$$\mbox{\boldmath$y$}(' num2str(tFinal) ')$$ [Cao et al. (2021)]']);

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
xlim([-6 11])
ylim([-5 15])
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
legend(ax1, [h_path_MD, h_path_PDT, h_final_MD, h_final_PDT, h_start, h_targets'], ...
    'Interpreter', 'latex', 'Location', 'north', 'NumColumns', 2);
hold(ax1, 'off');




% =========================================================================
%  (b) Tracking Errors  —  column 3, top row
% =========================================================================
ax2 = nexttile(2);
hold(ax2, 'on');

plot(ax2, t_vec, real(AgentCao.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Cao et al. (2021)]', ...
    'LineWidth', 1, ...
    'Color', [0, 0, 0.9]);

plot(ax2, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)), ...
    'DisplayName', '$$\delta(t)$$ [Proposed Method]', ...
    'LineWidth', 1, ...
    'Color', [0.9, 0, 0]);

yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');

xlim(ax2, [0, tFinal]);
ylim(ax2, [-0.5, 6])
box(ax2, 'on');
title(ax2, '(b) Tracking Errors', 'Interpreter', 'latex')
xlabel(ax2, 'time (sec)', 'Interpreter', 'latex')
ylabel(ax2, 'Errors (m)', 'Interpreter', 'latex')
grid(ax2, 'on');
set(ax2, 'FontSize', 10);
legend(ax2, 'Interpreter', 'latex', 'Location', 'northeast');
pbaspect(ax2, [2 1 1]);
hold(ax2, 'off');
%ylim([-1 18])


% =========================================================================
%  Final layout
% =========================================================================
tl.TileSpacing = 'compact';
tl.Padding = 'compact';


% sig function used for algorithms in Sui et al. (2025)
function out = sig(z,alpha)
    out = zeros(length(z),1);
    for i = 1:length(z)
        out(i) = sign(z(i))*abs(z(i))^alpha;
    end
end

function distance = computeConvexHullRadius(theta, c_hat, x_hat_positions, y)
    r_s = 0.5;
    ray_direction = [cos(theta); sin(theta)];
    
    unique_pts = unique(x_hat_positions', 'rows');
    if size(unique_pts, 1) < 3
        max_proj = 0;
        for j = 1:size(x_hat_positions, 2)
            v = x_hat_positions(:, j) - c_hat;
            proj = dot(v, ray_direction);
            if proj > max_proj
                max_proj = proj;
            end
        end
        distance = max_proj + r_s;
        return;
    end
    
    K = convhull(x_hat_positions(1,:), x_hat_positions(2,:));
    hull_indices = K(1:end-1);
    hull_vertices = x_hat_positions(:, hull_indices);
    num_hull_vertices = size(hull_vertices, 2);
    
    max_dist = 0;
    
    % --- Offset edges: push each edge outward by r_s ---
    for i = 1:num_hull_vertices
        v1 = hull_vertices(:, i);
        v2 = hull_vertices(:, mod(i, num_hull_vertices) + 1);
        edge = v2 - v1;
        edge_len = norm(edge);
        if edge_len < 1e-12, continue; end
        
        normal = [edge(2); -edge(1)] / edge_len;
        edge_mid = (v1 + v2) / 2;
        if dot(normal, edge_mid - c_hat) < 0
            normal = -normal;
        end
        
        v1_off = v1 + r_s * normal;
        v2_off = v2 + r_s * normal;
        edge_off = v2_off - v1_off;
        
        A_mat = [ray_direction, -edge_off];
        b_vec = v1_off - c_hat;
        det_A = det(A_mat);
        
        if abs(det_A) > 1e-10
            params = A_mat \ b_vec;
            t_param = params(1);
            s = params(2);
            if t_param > 0 && s >= -1e-10 && s <= 1+1e-10
                if t_param > max_dist
                    max_dist = t_param;
                end
            end
        end
    end
    
    % --- Vertex arcs: circle of radius r_s at each vertex ---
    for i = 1:num_hull_vertices
        v = hull_vertices(:, i);
        oc = c_hat - v;
        b_coeff = 2 * dot(ray_direction, oc);
        c_coeff = dot(oc, oc) - r_s^2;
        discriminant = b_coeff^2 - 4*c_coeff;
        
        if discriminant >= 0
            sqrt_disc = sqrt(discriminant);
            t2 = (-b_coeff + sqrt_disc) / 2;  % far intersection only
            if t2 > max_dist
                max_dist = t2;
            end
        end
    end
    
    if max_dist == 0
        distance = r_s;
    else
        distance = max_dist;
    end
end





%{



    % --- Zoomed inset on ax2 (MUST come after all nexttile calls) ---
% --- Zoomed inset on ax2 ---
drawnow;
x_zoom = [15, 25];
y_zoom = [-0.2, 0.2];

% Get the TRUE position of ax2 in normalised figure coordinates
% (tiledlayout axes report incorrect .Position values)
ax2_pos_px = getpixelposition(ax2);
fig_pos_px = getpixelposition(gcf);
ax2_pos = ax2_pos_px ./ [fig_pos_px(3), fig_pos_px(4), fig_pos_px(3), fig_pos_px(4)];

inset_left   = ax2_pos(1) + ax2_pos(3) * 0.36;
inset_bottom = ax2_pos(2) + ax2_pos(4) * 0.26;
inset_width  = ax2_pos(3) * 0.4;
inset_height = ax2_pos(4) * 0.25;

ax2_inset = axes('Position', [inset_left, inset_bottom, inset_width, inset_height]);
box(ax2_inset, 'on');
hold(ax2_inset, 'on');
plot(ax2_inset, t_vec, real(AgentCao.delta_traj(1:mytStepFinal)), ...
    'LineWidth', 1, 'Color', [0, 0, 0.9]);
plot(ax2_inset, t_vec, real(AgentPDT.delta_traj(1:mytStepFinal)), ...
    'LineWidth', 1, 'Color', [0.9, 0, 0]);
xlim(ax2_inset, x_zoom);
ylim(ax2_inset, y_zoom);
set(ax2_inset, 'FontSize', 7);
grid(ax2_inset, 'on');
hold(ax2_inset, 'off');

% Green rectangle on ax2
rectangle(ax2, 'Position', [x_zoom(1), y_zoom(1), diff(x_zoom), diff(y_zoom)], ...
    'EdgeColor', [0, 0.6, 0], 'LineWidth', 1.2);

% Helper using the TRUE axes position for data -> figure coordinate conversion
data2fig = @(ax_pos, ax_xlim, ax_ylim, xd, yd) [...
    ax_pos(1) + (xd - ax_xlim(1)) / diff(ax_xlim) * ax_pos(3), ...
    ax_pos(2) + (yd - ax_ylim(1)) / diff(ax_ylim) * ax_pos(4)];

% Top-left corner of rectangle -> bottom-left of inset
p1 = data2fig(ax2_pos, ax2.XLim, ax2.YLim, x_zoom(1), y_zoom(2));
p2 = [ax2_inset.Position(1), ax2_inset.Position(2)];
annotation('line', [p1(1), p2(1)], [p1(2), p2(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);

% Top-right corner of rectangle -> bottom-right of inset
p3 = data2fig(ax2_pos, ax2.XLim, ax2.YLim, x_zoom(2), y_zoom(2));
p4 = [ax2_inset.Position(1) + ax2_inset.Position(3), ax2_inset.Position(2)];
annotation('line', [p3(1), p4(1)], [p3(2), p4(2)], ...
    'Color', [0, 0.6, 0], 'LineStyle', '--', 'LineWidth', 1);

%}