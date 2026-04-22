close all
clear all
T = readtable('Sinusoidal Data.csv');

% Finding start and end indices
start_time = 0;
start_index = 0;
for i = 1:length(T.t)
    if (T.u_x_t_(i) ~= 0)
        start_time = T.t(i);
        start_index = i;
        break
    end
end
end_index = 3800;
startIndex = start_index;


% Estimating velocities
dt = T.t(startIndex+1:end_index) - T.t(startIndex:end_index-1);
vx = (T.p_a_x_t_(startIndex+1:end_index) - T.p_a_x_t_(startIndex:end_index-1)) ./ dt;
vy = (T.p_a_y_t_(startIndex+1:end_index) - T.p_a_y_t_(startIndex:end_index-1)) ./ dt;


%% -------- Plotting --------
close all;
figure('Position', [50 100 700 500]);
t = tiledlayout(3,1);

% --- Plotting in the x - direction --------

ax1 = nexttile(1); 
hold(ax1, 'on');
plot(ax1, T.t(start_index+1:end_index) - start_time, movmean(vx, 10), 'DisplayName', '$$\dot{y}_1(t)$$', 'LineWidth', 1.5, 'Color', [0.7, 0, 0])

 plot(ax1, T.t(start_index:end_index) - start_time, movmean(2*T.u_x_t_(startIndex:end_index), 10),...
        'DisplayName', '$$u_1(t)$$', ...
        'LineWidth', 1.5,...
        'Color', [0, 0, 0.7])

title(ax1, '(a) Actuation Error in the x-Direction', 'Interpreter','latex')
xlim(ax1, [0, 55])
ylim(ax1, [-0.3, 1.1])
xlabel(ax1, 'time (sec)', 'Interpreter','latex')
ylabel(ax1, 'velocity (m/s)', 'Interpreter','latex')
grid(ax1, 'on');
set(ax1, 'FontSize', 10);
legend(ax1, 'Interpreter', 'latex', 'Location', 'northeast')
hold(ax1, 'off');
box(ax1, 'on')

% --- Plotting in the y - direction --------

ax2 = nexttile(2); 
hold(ax2, 'on');
plot(ax2, T.t(start_index+1:end_index) - start_time, movmean(vy, 10),...
    'DisplayName', '$$\dot{y}_2(t)$$',...
    'LineWidth', 1.5, ...
    'Color', [0.7, 0, 0])

plot(ax2, T.t(start_index:end_index) - start_time, movmean(2*T.u_y_t_(startIndex:end_index), 10),...
    'DisplayName', '$$u_2(t)$$', ...
    'LineWidth', 1.5, ...
    'Color', [0, 0, 0.7])


title(ax2, '(b) Actuation Error in the y-Direction', 'Interpreter','latex')
xlabel(ax2, 'time (sec)', 'Interpreter','latex')
ylabel(ax2, 'velocity (m/s)', 'Interpreter','latex')
xlim(ax2, [0, 55])
ylim(ax2, [-0.4, 1])
grid(ax2, 'on');
set(ax2, 'FontSize', 10);
legend(ax2, 'Interpreter', 'latex', 'Location', 'northeast')

hold(ax2, 'off');
box(ax2, 'on')


% --- Plotting overall --------

ax3 = nexttile(3);

% Defining error
error_norm = movmean(sqrt((vx - 2*T.u_x_t_(startIndex:end_index-1)).^2 + (vy - 2*T.u_y_t_(startIndex:end_index-1)).^2), 8);
plot(ax3, T.t(start_index:end_index-1) - start_time, error_norm(:), ...
    'DisplayName', '$$|\dot{y}(t) - u(t)|$$', ...
    'LineWidth', 1.5, ...
    'Color', 'k')

title(ax3, '(c) Overall Actuation Error', 'Interpreter','latex')
xlabel(ax3, 'time (sec)', 'Interpreter','latex')
ylabel(ax3, 'velocity (m/s)', 'Interpreter','latex')
xlim(ax3, [0, 55])
ylim(ax3, [-0.1, 1.4])
grid(ax3, 'on');
set(ax3, 'FontSize', 10);
legend(ax3, 'Interpreter', 'latex', 'Location', 'northeast')
hold(ax3, 'off');
box(ax3, 'on')


t.TileSpacing = 'compact';
t.Padding = 'compact';

% --- Zoomed inset on ax2 (MUST come after all nexttile calls) ---
drawnow; % Force layout to finalise tile positions before reading them

x_zoom = [15, 40];      % x range to zoom into - adjust to your data
y_zoom = [-0.02, 0.18];  % y range to zoom into - adjust to your data

ax3_inset = axes('Position', [0.3, 0.18, 0.42, 0.09]); % [left bottom width height] in figure coords
box(ax3_inset, 'on');
plot(ax3_inset, T.t(start_index:end_index-1) - start_time, error_norm(:), ...
    'LineWidth', 1, 'Color', 'k');
xlim(ax3_inset, x_zoom);
ylim(ax3_inset, y_zoom);
set(ax3_inset, 'FontSize', 7);
grid(ax3_inset, 'on');

% Draw green rectangle on ax2 showing the zoomed region
rectangle(ax3, 'Position', [x_zoom(1), y_zoom(1) - 0.02, diff(x_zoom), 1.5 * diff(y_zoom)], ...
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