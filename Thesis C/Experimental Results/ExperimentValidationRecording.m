%%
close all
clear all
%T = readtable('Recording.csv');
% OR
T = readtable('Very good circle.csv');
% NB: Same values for everything can just swap

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
    out = tanh(10 * z) .* (abs(z).^alpha);
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

d = vecnorm([T.pC_hat_x_t_(startIndex:end), T.pC_hat_y_t_(startIndex:end)] - [T.p_a_x_t_(startIndex:end), T.p_a_y_t_(startIndex:end)], 2, 2);
d_tilde = d - T.d_des_t_(startIndex:end);

psi = ([T.pC_hat_x_t_(startIndex:end), T.pC_hat_y_t_(startIndex:end)] - [T.p_a_x_t_(startIndex:end), T.p_a_y_t_(startIndex:end)]) ./ d;
bar_psi = [psi(:,2), -psi(:,1)];

v_cen = 1/(alpha_2 * Tc2) .* exp(abs(d_tilde) .^ alpha_2) .* sig(d_tilde, 1 - alpha_2) - T.d_des_dot_t_(startIndex:end);
u = v_cen .* psi + k_d  .* bar_psi;

u_mag = vecnorm(u, 2, 2);
scale = ones(size(u_mag));
idx = u_mag > saturation_velocity;
scale(idx) = saturation_velocity ./ u_mag(idx);

% 4. Apply saturation element-wise
u = u .* scale;


% 1. Calculate time step and velocities
% These vectors will have a length of (N - 1)
dt = T.t(startIndex+1:end) - T.t(startIndex:end-1);
vx = (T.p_a_x_t_(startIndex+1:end) - T.p_a_x_t_(startIndex:end-1)) ./ dt;
vy = (T.p_a_y_t_(startIndex+1:end) - T.p_a_y_t_(startIndex:end-1)) ./ dt;



%% --- Control decomposition comparison ---

figure
t1 = tiledlayout(1,3);

% ============================================================
% Signals (already trimmed earlier in your script)
u_theory   = u;                 % Nx2
psi_theory = psi;               % Nx2
barpsi     = bar_psi;           % Nx2

u_commanded = 2*[T.u_x_t_, T.u_y_t_];     % Mx2
u_commanded = u_commanded(startIndex:end,:);

t_plot = T.t(startIndex:end) - start_time;

% ---------- FORCE COMMON LENGTH ----------
N = min([ ...
    size(u_theory,1), ...
    size(psi_theory,1), ...
    size(barpsi,1), ...
    size(u_commanded,1), ...
    length(t_plot) ]);

u_theory    = u_theory(1:N,:);
psi_theory  = psi_theory(1:N,:);
barpsi      = barpsi(1:N,:);
u_commanded = u_commanded(1:N,:);
t_plot      = t_plot(1:N);

% ============================================================
% ---------- Theoretical decomposition ----------
u_tangential = ...
    ( sum(u_theory .* barpsi,2) ) .* barpsi;

u_centripetal = ...
    sum(u_theory .* psi_theory,2) .* psi_theory;

% ---------- Commanded decomposition ----------
u_commanded_tangential = ...
    ( sum(u_commanded .* barpsi,2)) .* barpsi;

u_commanded_centripetal = ...
    ( sum(u_commanded .* psi_theory,2)) .* psi_theory;

% Signed scalar components
u_tangential_signed        = sum(u_theory .* barpsi, 2);
u_centripetal_signed       = sum(u_theory .* psi_theory, 2);

u_cmd_tangential_signed   = sum(u_commanded .* barpsi, 2);
u_cmd_centripetal_signed  = sum(u_commanded .* psi_theory, 2);

% ============================================================
% -------------------- Plot 1: Centripetal -------------------
ax1 = nexttile(1); hold on;

plot(ax1, t_plot, u_centripetal_signed - u_cmd_centripetal_signed, ...
    'LineWidth',1,'DisplayName','Theoretical Centripetal u - Commanded Centripetal u');

xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Centripetal Control');
grid on; legend;
hold off;

% -------------------- Plot 2: Tangential --------------------
ax2 = nexttile(2); hold on;

plot(ax2, t_plot, u_tangential_signed - u_cmd_tangential_signed, ...
    'LineWidth',1,'DisplayName','Theoretical Tangential u - Commanded Tangential u');

xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Tangential Control');
grid on; legend;
hold off;

% -------------------- Plot 3: Total Error -------------------
ax3 = nexttile(3); hold on;

plot(ax3, t_plot, ...
    vecnorm(u_commanded,2,2) - vecnorm(u_theory,2,2), ...
    'LineWidth',2,'DisplayName','Error in u');

xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Total Control Error');
grid on; legend;
hold off;

%%



targetColors = lines(3);
figure; % Create a new figure
t = tiledlayout(2, 3); % Create a 2-row, 3-column grid

%  ---- (a) Agent Trajectory plot -----
    % This plot is in column 1 and spans 2 rows
    ax1 = nexttile(1, [2 1]); 
    
    hold(ax1, 'on');
    % Plotting Target Estimation Errors
    plot(ax1, T.pT_hat1_x_t_(1:end), T.pT_hat1_y_t_(1:end), ...
        'LineWidth', 1, ...
        'Color', targetColors(1, :),...
        'LineStyle'  ,  '--', ...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_1(t)$$');

    plot(ax1, T.pT_hat2_x_t_(1:end), T.pT_hat2_y_t_(1:end), ...
        'LineWidth', 1, ...
        'Color', targetColors(2, :),...
        'LineStyle'  ,  '--', ...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_1(t)$$');

    plot(ax1, T.pT_hat3_x_t_(1:end), T.pT_hat3_y_t_(1:end), ...
        'LineWidth', 1, ...
        'Color', targetColors(3, :),...
        'LineStyle'  ,  '--', ...
        'DisplayName', '$$\mbox{\boldmath$\hat{x}$}_1(t)$$');

    % Plotting actual target positions
    xT = Targets(1, 1);
    yT = Targets(2, 1);
    plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(1, :),...
        'DisplayName', '$$\mbox{\boldmath$x$}_1$$');

    xT = Targets(1, 2);
    yT = Targets(2, 2);
    plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(2, :),...
        'DisplayName', '$$\mbox{\boldmath$x$}_2$$');

    xT = Targets(1, 3);
    yT = Targets(2, 3);
    plot(ax1, xT, yT, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 7, ...
        'Color', targetColors(3, :),...
        'DisplayName', '$$\mbox{\boldmath$x$}_3$$');

    
    % Plotting PDT Agent
    plot(ax1, T.p_a_x_t_(startIndex:end), T.p_a_y_t_(startIndex:end), ...
        'LineWidth', 1, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$',...
        'Color'      ,  'k');   
    
    % ==== Axis properties ====
    axis(ax1, 'equal');
    title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
    xlabel(ax1, 'x (m)', 'Interpreter','latex')
    ylabel(ax1, 'y (m)', 'Interpreter','latex')
    grid(ax1, 'on');
    set(ax1, 'FontSize', 10);
    xlim(ax1, [-2 2]);
    ylim(ax1, [-2 2]);
    legend(ax1, 'Interpreter','latex', 'Location','best');
    hold(ax1, 'off');



%  ---- (c) Tracking Error Plot -----
    % This plot is in column 3, top row
    ax2 = nexttile(2); 
    hold(ax2, 'on');
    
    % Plotting PDT Agent Tracking Error
    plot(ax2, T.t(start_index:end) - start_time, d_tilde(:),...
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
    xlim(ax2, [0, T.t(end)])
    title(ax2, '(c) Tracking Errors', 'Interpreter','latex')
    xlabel(ax2, 'time (sec)', 'Interpreter','latex')
    ylabel(ax2, 'Errors (m)', 'Interpreter','latex')
    grid(ax2, 'on');
    set(ax2, 'FontSize', 10);
    legend(ax2, 'Interpreter','latex', 'Location','best');
    pbaspect(ax2, [2 1 1]); % You can still control aspect ratio
    hold(ax2, 'off');
    ylim([-0.1 0.8])


    
% ------ (d) Estimation Error Plot ---------
    % This plot is in column 3, bottom row (tile 6)
    ax3 = nexttile(3); 
    hold(ax3, 'on');
    
    % Plotting Estimation Errors
    pT_hat1 = [T.pT_hat1_x_t_(start_index:end)'; T.pT_hat1_y_t_(start_index:end)'];
    err_norm_1 = vecnorm(Targets(:,1) - pT_hat1, 2, 1);
    
    % Plotting norm against t_vec
    plot(ax3, T.t(start_index:end) - start_time, err_norm_1, ...
        'LineWidth', 1.3, ...
        'LineStyle', '--', ...
        'Color', targetColors(1, :),...
        'DisplayName', '$$\|\mbox{\boldmath$\tilde{x}$}_1(t)\|$$');

    pT_hat2 = [T.pT_hat2_x_t_(start_index:end)'; T.pT_hat2_y_t_(start_index:end)'];
    err_norm_2 = vecnorm(Targets(:,2) - pT_hat2, 2, 1);
    hold on
    % Plotting norm against t_vec
    plot(ax3, T.t(start_index:end) - start_time, err_norm_2, ...
        'LineWidth', 1.3, ...
        'LineStyle', '--', ...
        'Color', targetColors(2, :),...
        'DisplayName', '$$\|\mbox{\boldmath$\tilde{x}$}_2(t)\|$$');


    pT_hat3 = [T.pT_hat3_x_t_(start_index:end)'; T.pT_hat3_y_t_(start_index:end)'];
    err_norm_3 = vecnorm(Targets(:,3) - pT_hat3, 2, 1);
    hold on
    % Plotting norm against t_vec
    plot(ax3, T.t(start_index:end) - start_time, err_norm_3, ...
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
            'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility', 'off'); 
    end
    % --- Axis Properties ---
    xlim(ax3, [0, T.t(end)])
    title(ax3, '(d) Estimation Errors', 'Interpreter','latex')
    xlabel(ax3, 'time (sec)', 'Interpreter','latex')
    ylabel(ax3, 'Errors (m)', 'Interpreter','latex')
    grid(ax3, 'on');
    set(ax3, 'FontSize', 10);
    legend(ax3, 'Interpreter','latex', 'Location','best');
    pbaspect(ax3, [2 1 1]); % You can still control aspect ratio
    hold(ax3, 'off');


    ax4 = nexttile(5, [1 2]); 
    hold(ax4, 'on');
    
    plot(T.t(start_index+1:end) - start_time, movmean(sqrt(vx.^2 + vy.^2), 10), 'DisplayName', 'Theoretical $$v(t)$$', 'LineWidth', 1)
    %hold on;
    %plot(T.t(start_index:end) - start_time, sqrt(T.v_a_x_t_(startIndex:end).^2 + T.v_a_y_t_(startIndex:end).^2),...
    %        'DisplayName', 'Actual $$v(t)$$', ...
    %        'LineWidth', 1)

    hold on;
    plot(T.t(start_index:end) - start_time, sqrt((2.*T.u_x_t_(startIndex:end)).^2 + (2.*T.u_y_t_(startIndex:end)).^2),...
        'DisplayName', '$$u(t)$$', ...
        'LineWidth', 1)



    xlim(ax4, [0, T.t(end)])
    title(ax4, '(d) Actuation', 'Interpreter','latex')
    xlabel(ax4, 'time (sec)', 'Interpreter','latex')
    ylabel(ax4, 'Velocity (m/s)', 'Interpreter','latex')
    grid(ax4, 'on');
    set(ax4, 'FontSize', 10);
    legend(ax4, 'Interpreter','latex', 'Location','best');
    %pbaspect(ax4, [2 1 1]); % You can still control aspect ratio
    hold(ax4, 'off');
        
    % --- Axis Properties ---



    %%

        %