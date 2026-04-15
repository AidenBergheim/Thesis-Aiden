%%
%close all
clear all
T = readtable('Recording.csv');
takeoff_index = 210;

end_index = 5000;


%%

alpha_2 = 0.9;
Tc1 = 2;
Tc2 = 3;
k_d = 0.1;
saturation_velocity = 1;

    
Targets       = [0.5, 0.2, 0.6; 
                 -0.5, 0.6, 0];    % initial location of the target 

% sig function used for algorithms in Sui et al. (2025)
function out = sig(z, alpha)
    % REMEMBER THIS IS TANH NOT SIGN
    out = tanh(12 * z) .* (abs(z).^alpha);
end



start_time = 0;
start_index = 0;
for i = 1:length(T.t)
    if (T.u_x_t_(i) ~= 0)
        start_time = T.t(i);
        start_index = i;
        break
    end
end


startIndex = start_index;

d = vecnorm([T.pC_hat_x_t_(startIndex:end_index), T.pC_hat_y_t_(startIndex:end_index)] - [T.p_a_x_t_(startIndex:end_index), T.p_a_y_t_(startIndex:end_index)], 2, 2);
d_tilde = d - T.d_des_t_(startIndex:end_index);

psi = ([T.pC_hat_x_t_(startIndex:end_index), T.pC_hat_y_t_(startIndex:end_index)] - [T.p_a_x_t_(startIndex:end_index), T.p_a_y_t_(startIndex:end_index)]) ./ d;
bar_psi = [psi(:,2), -psi(:,1)];

v_cen = 1/(alpha_2 * Tc2) .* exp(abs(d_tilde) .^ alpha_2) .* sig(d_tilde, 1 - alpha_2) - T.d_des_dot_t_(startIndex:end_index);
u = v_cen .* psi + k_d  .* bar_psi;

u_mag = vecnorm(u, 2, 2);
scale = ones(size(u_mag));
idx = u_mag > saturation_velocity;
scale(idx) = saturation_velocity ./ u_mag(idx);

% 4. Apply saturation element-wise
u = u .* scale;


% 1. Calculate time step and velocities
% These vectors will have a length of (N - 1)
dt = T.t(startIndex+1:end_index) - T.t(startIndex:end_index-1);
vx = (T.p_a_x_t_(startIndex+1:end_index) - T.p_a_x_t_(startIndex:end_index-1)) ./ dt;
vy = (T.p_a_y_t_(startIndex+1:end_index) - T.p_a_y_t_(startIndex:end_index-1)) ./ dt;



%%



targetColors = lines(3);
figure('Position', [50 300 800 400]) % Create a new figure
t = tiledlayout(2, 2); % Create a 2-row, 3-column grid

%  ---- (a) Agent Trajectory plot -----
    % This plot is in column 1 and spans 2 rows
    ax1 = nexttile(1, [2 1]); 
    
    hold(ax1, 'on');

    h_targets = gobjects(3, 1); 
    h_targets_final = gobjects(3, 1); 
    h_targets_initial = gobjects(3, 1); 
    h_start   = gobjects(1, 1);
    h_final  = gobjects(1, 1);
    h_path   = gobjects(1, 1);



    % Plotting Target Estimation Errors
    h_targets(1) = plot(ax1, T.pT_hat1_x_t_(1:end_index), T.pT_hat1_y_t_(1:end_index), ...
        'LineWidth', 1, ...
        'Color', targetColors(1, :),...
        'LineStyle'  ,  '--', ...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_1(t)$$');

    h_targets(2) = plot(ax1, T.pT_hat2_x_t_(1:end_index), T.pT_hat2_y_t_(1:end_index), ...
        'LineWidth', 1, ...
        'Color', targetColors(2, :),...
        'LineStyle'  ,  '--', ...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_2(t)$$');

    h_targets(3) = plot(ax1, T.pT_hat3_x_t_(1:end_index), T.pT_hat3_y_t_(1:end_index), ...
        'LineWidth', 1, ...
        'Color', targetColors(3, :),...
        'LineStyle'  ,  '--', ...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_3(t)$$');

    % Plotting actual target positions
    xT = Targets(1, 1);
    yT = Targets(2, 1);
    h_targets_final(1) = plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(1, :),...
        'DisplayName', '$$\mbox{\boldmath$x$}_1$$');

    xT = Targets(1, 2);
    yT = Targets(2, 2);
    h_targets_final(2) = plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(2, :),...
        'DisplayName', '$$\mbox{\boldmath$x$}_2$$');

    xT = Targets(1, 3);
    yT = Targets(2, 3);
    h_targets_final(3) = plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(3, :),...
        'DisplayName', '$$\mbox{\boldmath$x$}_3$$');

    
    % Plotting initial estimate of target positions
    xT = Targets(1, 1);
    yT = Targets(2, 1);
    h_targets_initial(1) = plot(ax1, T.pT_hat1_x_t_(1), T.pT_hat1_y_t_(1), 'o', ...
        'LineWidth', 1.5, ...
        'MarkerSize', 6, ...
        'Color', targetColors(1, :),...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_1(0)$$');

    xT = Targets(1, 2);
    yT = Targets(2, 2);
    h_targets_initial(2) = plot(ax1, T.pT_hat2_x_t_(1), T.pT_hat2_y_t_(1), 'o', ...
        'LineWidth', 1.5, ...
        'MarkerSize', 6, ...
        'Color', targetColors(2, :),...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_2(0)$$');

    xT = Targets(1, 3);
    yT = Targets(2, 3);
    h_targets_initial(3) = plot(ax1,T.pT_hat3_x_t_(1), T.pT_hat3_y_t_(1), 'o', ...
        'LineWidth', 1.5, ...
        'MarkerSize', 6, ...
        'Color', targetColors(3, :),...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_3(0)$$');

    % Plotting PDT Agent

    h_path = plot(ax1, T.p_a_x_t_(takeoff_index:end_index), T.p_a_y_t_(takeoff_index:end_index), ...
        'LineWidth', 1, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$',...
        'Color'      ,  'k');   

    % Final PDT Agent Position
    h_final = plot(T.p_a_x_t_(end_index), T.p_a_y_t_(end_index), 'o', ...
    'MarkerSize', 8, ...
    'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', [0, 0, 0], ...
    'LineWidth', 1.5, ...
    'DisplayName', '$$\mbox{\boldmath$y$}(t_f)$$');

    % 4. PLOT START POSITION (Bottom Right)
    h_start = plot(T.p_a_x_t_(takeoff_index), T.p_a_y_t_(takeoff_index), 'o', ...
        'MarkerSize', 8, ...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', 'none', ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(0)$$');
    
    % ==== Axis properties ====
    axis(ax1, 'equal');
    title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
    xlabel(ax1, 'x (m)', 'Interpreter','latex')
    ylabel(ax1, 'y (m)', 'Interpreter','latex')
    grid(ax1, 'on');
    set(ax1, 'FontSize', 10);
    xlim(ax1, [-1.2 2.2]);
    ylim(ax1, [-1.2 2.2]);
    hold(ax1, 'off');
    box(ax1, 'on')


    stack_left = [h_targets_initial; h_targets]; 
    stack_right = [h_targets_final; h_start; h_path; h_final];
    legend(ax1, [stack_left; stack_right], ...
    'Interpreter', 'latex', ...
    'Location',    'northeast', ...
    'NumColumns',  2);

%  ---- (b) Tracking Error Plot -----
    % This plot is in column 3, top row
    ax2 = nexttile(2); 
    hold(ax2, 'on');
    
    % Plotting PDT Agent Tracking Error
    plot(ax2, T.t(start_index:end_index) - start_time, d_tilde(:),...
        'DisplayName', '$$\delta(t)$$', ...
        'LineWidth', 1, ...
        'Color'      ,  'k');
    hold on;
        % --- Add Vertical Line for Tracking Convergence (Tc1 + Tc2) ---
    if exist('Tc1', 'var') && exist('Tc2', 'var')
        xline(ax2, Tc1 + Tc2, '-.', {'$T_{c,1} + T_{c,2}$'}, ...
            'Interpreter', 'latex', ...
            'FontSize', 10, ...
            'LabelVerticalAlignment', 'top', ...
            'HandleVisibility', 'off'); 
    end
        
    % --- Axis Properties ---
    xlim(ax2, [0, T.t(end_index) - T.t(start_index)])
    ylim(ax2, [-0.12, max(1.2 * max(d_tilde(:)), 0.8)])
    title(ax2, '(b) Tracking Errors', 'Interpreter','latex')
    xlabel(ax2, 'time (sec)', 'Interpreter','latex')
    ylabel(ax2, 'Errors (m)', 'Interpreter','latex')
    grid(ax2, 'on');
    set(ax2, 'FontSize', 10);
    legend(ax2, 'Interpreter','latex', 'Location','northeast');
    pbaspect(ax2, [2 1 1]); % You can still control aspect ratio
    hold(ax2, 'off');
    box(ax2, 'on')


    
% ------ (c) Estimation Error Plot ---------
    % This plot is in column 3, bottom row (tile 6)
    ax3 = nexttile(4); 
    hold(ax3, 'on');
    
    % Plotting Estimation Errors
    pT_hat1 = [T.pT_hat1_x_t_(start_index:end_index)'; T.pT_hat1_y_t_(start_index:end_index)'];
    err_norm_1 = vecnorm(Targets(:,1) - pT_hat1, 2, 1);
    
    % Plotting norm against t_vec
    plot(ax3, T.t(start_index:end_index) - start_time, err_norm_1, ...
        'LineWidth', 1.3, ...
        'LineStyle', '--', ...
        'Color', targetColors(1, :),...
        'DisplayName', '$$\|\mbox{\boldmath$\tilde{x}$}_1(t)\|$$');

    pT_hat2 = [T.pT_hat2_x_t_(start_index:end_index)'; T.pT_hat2_y_t_(start_index:end_index)'];
    err_norm_2 = vecnorm(Targets(:,2) - pT_hat2, 2, 1);
    hold on
    % Plotting norm against t_vec
    plot(ax3, T.t(start_index:end_index) - start_time, err_norm_2, ...
        'LineWidth', 1.3, ...
        'LineStyle', '--', ...
        'Color', targetColors(2, :),...
        'DisplayName', '$$\|\mbox{\boldmath$\tilde{x}$}_2(t)\|$$');


    pT_hat3 = [T.pT_hat3_x_t_(start_index:end_index)'; T.pT_hat3_y_t_(start_index:end_index)'];
    err_norm_3 = vecnorm(Targets(:,3) - pT_hat3, 2, 1);
    hold on
    % Plotting norm against t_vec
    plot(ax3, T.t(start_index:end_index) - start_time, err_norm_3, ...
        'LineWidth', 1.3, ...
        'LineStyle', '--', ...
        'Color', targetColors(3, :),...
        'DisplayName', '$$\|\mbox{\boldmath$\tilde{x}$}_3(t)\|$$');



    hold on
   % --- Add Vertical Line for Estimation Convergence (Tc1) ---
    if exist('Tc1', 'var')
        xline(ax3, Tc1, '-.', {'$T_{c,1}$'}, ...
            'Interpreter', 'latex', ...
            'FontSize', 10, ...
            'LabelVerticalAlignment', 'top', ...
            'HandleVisibility', 'off'); 
    end

    xlim(ax3, [0, T.t(end_index) - T.t(start_index)])
    title(ax3, '(c) Estimation Errors', 'Interpreter','latex')
    xlabel(ax3, 'time (sec)', 'Interpreter','latex')
    ylabel(ax3, 'Errors (m)', 'Interpreter','latex')
    grid(ax3, 'on');
    set(ax3, 'FontSize', 10);
    legend(ax3, 'Interpreter','latex', 'Location','northeast');
    pbaspect(ax3, [2 1 1]);
    set(ax3, 'XScale', 'log')
    hold(ax3, 'off');
    box(ax3, 'on')
    %ylim(ax3, [-0.05, max([max(err_norm_1), max(err_norm_2), max(err_norm_3)])])

% --- Final layout adjustments for the whole figure ---
t.TileSpacing = 'compact';
t.Padding = 'compact';

% --- Zoomed inset on ax2 (MUST come after all nexttile calls) ---
drawnow; % Force layout to finalise tile positions before reading them

x_zoom = [28, 46];      % x range to zoom into - adjust to your data
y_zoom = [-0.05, 0.05];  % y range to zoom into - adjust to your data

ax2_inset = axes('Position', [0.68, 0.74, 0.2, 0.08]); % [left bottom width height] in figure coords
box(ax2_inset, 'on');
plot(ax2_inset, T.t(start_index:end_index) - start_time, d_tilde(:), ...
    'LineWidth', 1, 'Color', 'k');
xlim(ax2_inset, x_zoom);
ylim(ax2_inset, y_zoom);
set(ax2_inset, 'FontSize', 7);
grid(ax2_inset, 'on');

% Draw green rectangle on ax2 showing the zoomed region
rectangle(ax2, 'Position', [x_zoom(1), y_zoom(1) - 0.05, diff(x_zoom), 2 * diff(y_zoom)], ...
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