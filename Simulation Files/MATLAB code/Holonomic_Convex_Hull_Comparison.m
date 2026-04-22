
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
    % Proposed algorithms
    AgentPDT = Agent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des_handle, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);

    rs = 0.3;
    AgentCao = CaoAgent(p_0, x_hat_0, 1, 0.5, 5, 0.3, tSteps, qtyTargets, Targets, initial_heading, 1000, 0);

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
%  (b) Tracking Errors
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


tl.TileSpacing = 'compact';
tl.Padding = 'compact';



function distance = computeConvexHullRadius(theta, c_hat, x_hat_positions, y)
    r_s = 0.5;  % Offset radius for Minkowski sum expansion
    ray_direction = [cos(theta); sin(theta)];  % Unit ray from c_hat at angle theta

    % Compute convex hull of estimated target positions
    K = convhull(x_hat_positions(1,:), x_hat_positions(2,:));
    hull_indices = K(1:end-1);  % Remove repeated closing vertex
    hull_vertices = x_hat_positions(:, hull_indices);
    num_hull_vertices = size(hull_vertices, 2);
    max_dist = 0;

    % --- Offset edges: push each edge outward by r_s ---
    for i = 1:num_hull_vertices
        v1 = hull_vertices(:, i);
        v2 = hull_vertices(:, mod(i, num_hull_vertices) + 1);
        edge = v2 - v1;
        edge_len = norm(edge);
        if edge_len < 1e-12, continue; end  % Skip degenerate edges

        % Compute outward-facing unit normal
        normal = [edge(2); -edge(1)] / edge_len;
        edge_mid = (v1 + v2) / 2;
        if dot(normal, edge_mid - c_hat) < 0
            normal = -normal;  % Flip to ensure normal points away from centre
        end

        % Offset edge outward by r_s
        v1_off = v1 + r_s * normal;
        v2_off = v2 + r_s * normal;
        edge_off = v2_off - v1_off;

        % Solve ray-edge intersection: c_hat + t*ray = v1_off + s*edge_off
        A_mat = [ray_direction, -edge_off];
        b_vec = v1_off - c_hat;
        det_A = det(A_mat);
        if abs(det_A) > 1e-10  % Skip parallel ray/edge
            params = A_mat \ b_vec;
            t_param = params(1);  % Distance along ray
            s = params(2);        % Position along edge
            if t_param > 0 && s >= -1e-10 && s <= 1+1e-10  % Valid intersection
                if t_param > max_dist
                    max_dist = t_param;
                end
            end
        end
    end

    % --- Vertex arcs: circle of radius r_s at each vertex ---
    for i = 1:num_hull_vertices
        v = hull_vertices(:, i);
        % Solve ray-circle intersection at each hull vertex
        oc = c_hat - v;
        b_coeff = 2 * dot(ray_direction, oc);
        c_coeff = dot(oc, oc) - r_s^2;
        discriminant = b_coeff^2 - 4*c_coeff;
        if discriminant >= 0
            sqrt_disc = sqrt(discriminant);
            t2 = (-b_coeff + sqrt_disc) / 2;  % Far intersection only
            if t2 > max_dist
                max_dist = t2;
            end
        end
    end

    % Return fallback radius if no intersection found
    if max_dist == 0
        distance = r_s;
    else
        distance = max_dist;
    end
end