%%
close all
clear all
TSign = readtable('With Sign.csv');
TTanh = readtable('With Tanh.csv');
TTanh3 = readtable('With Tanh3.csv');
TTanh5 = readtable('With Tanh5.csv');

Tc1 = 2;
Tc2 = 3;

Targets       = [0.5, 0.2, 0.6; 
                 -0.5, 0.6, 0];    % initial location of the target 
end_index = 3000;
start_time = 0;


start_index = 0;
for i = 1:length(TSign.t)
    if (TSign.u_x_t_(i) ~= 0)
        start_time = TSign.t(i);
        start_index = i;
        break
    end
end
startIndex = start_index;

dSign = vecnorm([TSign.pC_hat_x_t_(startIndex:end_index), TSign.pC_hat_y_t_(startIndex:end_index)] - [TSign.p_a_x_t_(startIndex:end_index), TSign.p_a_y_t_(startIndex:end_index)], 2, 2);
d_tilde_Sign = dSign - TSign.d_des_t_(startIndex:end_index);

dTanh = vecnorm([TTanh.pC_hat_x_t_(startIndex:end_index), TTanh.pC_hat_y_t_(startIndex:end_index)] - [TTanh.p_a_x_t_(startIndex:end_index), TTanh.p_a_y_t_(startIndex:end_index)], 2, 2);
d_tilde_Tanh = dTanh - TTanh.d_des_t_(startIndex:end_index);

dTanh3 = vecnorm([TTanh3.pC_hat_x_t_(startIndex:end_index), TTanh3.pC_hat_y_t_(startIndex:end_index)] - [TTanh3.p_a_x_t_(startIndex:end_index), TTanh3.p_a_y_t_(startIndex:end_index)], 2, 2);
d_tilde_Tanh3 = dTanh3 - TTanh3.d_des_t_(startIndex:end_index);

dTanh5 = vecnorm([TTanh5.pC_hat_x_t_(startIndex:end_index), TTanh5.pC_hat_y_t_(startIndex:end_index)] - [TTanh5.p_a_x_t_(startIndex:end_index), TTanh5.p_a_y_t_(startIndex:end_index)], 2, 2);
d_tilde_Tanh5 = dTanh5 - TTanh5.d_des_t_(startIndex:end_index);

LineColours = lines(4);
figure; % Create a new figure
t = tiledlayout(1, 2); % Create a 2-row, 3-column grid

%  ---- (a) Agent Trajectory plot -----
    % This plot is in column 1 and spans 2 rows
    ax1 = nexttile(1); 
    
    hold(ax1, 'on');
    
    % Plotting Centroid
    plot(ax1, (Targets(1, 1) + Targets(1, 2) + Targets(1, 3))/3, (Targets(2, 1) + Targets(2, 2) + Targets(2, 3))/3, 'k+', ...
        'LineWidth', 1, ...
        'MarkerSize', 10, ...
        'Color', 'k',...
        'DisplayName', '$$\mbox{\boldmath$c$}$$');

    
    % Plotting PDT Agent
    plot(ax1, TSign.p_a_x_t_(startIndex:end_index-300), TSign.p_a_y_t_(startIndex:end_index-300), ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [sign$$(\tilde{d}(t))$$]', ...
        'Color', LineColours(1, :));   
    
    plot(ax1, TTanh.p_a_x_t_(startIndex:end_index), TTanh.p_a_y_t_(startIndex:end_index), ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [$$tanh(\tilde{d}(t))$$]', ...
        'Color', LineColours(2, :));

    plot(ax1, TTanh3.p_a_x_t_(startIndex:end_index-300), TTanh3.p_a_y_t_(startIndex:end_index-300), ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [$$tanh(3\tilde{d}(t))$$]', ...
        'Color', LineColours(3, :));

    plot(ax1, TTanh5.p_a_x_t_(startIndex:end_index), TTanh5.p_a_y_t_(startIndex:end_index), ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [$$tanh(5\tilde{d}(t))$$]', ...
        'Color', LineColours(4, :));

    % ==== Axis properties ====
    axis(ax1, 'equal');
    title(ax1, '(a) Agent Trajectories', 'Interpreter','latex')
    xlabel(ax1, 'x (m)', 'Interpreter','latex')
    ylabel(ax1, 'y (m)', 'Interpreter','latex')
    grid(ax1, 'on');
    set(ax1, 'FontSize', 10);
    xlim(ax1, [-1 2.2]);
    ylim(ax1, [-1.5 1.5]);
    legend(ax1, 'Interpreter','latex', 'Location','best');
    hold(ax1, 'off');



%  ---- (c) Tracking Error Plot -----
    % This plot is in column 3, top row
    ax2 = nexttile(2); 
    hold(ax2, 'on');
    
    % Plotting PDT Agent Tracking Error
    plot(ax2, TSign.t(start_index:end_index) - start_time, d_tilde_Sign(:),...
        'DisplayName', '$$\delta(t)$$ [sign$$(\tilde{d}(t))$$]', ...
        'LineWidth', 1.5, ...
        'Color', LineColours(1, :));
    hold on;


    plot(ax2, TTanh.t(start_index:end_index) - start_time, d_tilde_Tanh(:),...
        'DisplayName', '$$\delta(t)$$ [$$tanh(\tilde{d}(t))$$]', ...
        'LineWidth', 1.5, ...
        'Color', LineColours(2, :));
    hold on;

    plot(ax2, TTanh3.t(start_index:end_index) - start_time, d_tilde_Tanh3(:),...
        'DisplayName', '$$\delta(t)$$ [$$tanh(3\tilde{d}(t))$$]', ...
        'LineWidth', 1.5, ...
        'Color', LineColours(3, :));
    hold on;

    plot(ax2, TTanh5.t(start_index:end_index) - start_time, d_tilde_Tanh5(:),...
        'DisplayName', '$$\delta(t)$$ [$$tanh(5\tilde{d}(t))$$]', ...
        'LineWidth', 1.5, ...
        'Color', LineColours(4, :));
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
    xlim(ax2, [0, TSign.t(end_index)])
    title(ax2, '(b) Tracking Errors', 'Interpreter','latex')
    xlabel(ax2, 'time (sec)', 'Interpreter','latex')
    ylabel(ax2, 'Errors (m)', 'Interpreter','latex')
    grid(ax2, 'on');
    set(ax2, 'FontSize', 10);
    legend(ax2, 'Interpreter','latex', 'Location','best');
    pbaspect(ax2, [2 1 1]); % You can still control aspect ratio
    hold(ax2, 'off');


    


    %%

        %