
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MATLAB Simulation of Multi-Target Bearing 
% only Target Localization and Circumnavigation (BoTLC),
% Using controllers inspired by Cao et al. (2021) Sui et al. (2025) Zhao et al. (2019)

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
    tFinal = 12;            % (seconds)
    dT     = 0.005;         % (seconds)
    
    tSteps = int64((tFinal-tBegin)/dT);   % force tSteps to be an integer

    % ----------------------------------------
    %        Define Multiple Targets
    % ----------------------------------------
    
    Targets       = [-2, 4, 2, 1; 
                     0, 5, 0, 1];    % initial location of the target 
    
    qtyTargets = size(Targets, 2);

    hullVertices = true_hull(Targets);


    % ----------------------------------------
    %              Define Agent
    % ----------------------------------------
    
    % ---- Initial Conditions ----
    
        p_0             = [8; 0];      % initial location of the agent 
        
        x_hat_0         = [7.7, 7.812, 7.7, 7.703; 
                           0, 0.234, 0, 0.042];   % agent's initial guess of target positions
    
    % ---- Control Constants ----
    
        % control gain for adjusting tangential speed  
            k_omega = 5;           
       
        % PDT algorthm parameters
            beta_1 = 0.5;          
            beta_2 = 0.5;
            alpha_1 = 0.5;
            alpha_2 = 0.5;
            Tc1 = 0.1;
            Tc2 = 0.2;

        % Cao algorithm parameters
            alpha = 5;

        % Non-holonomic parameters
            initial_heading = pi/2;
    
        % Desired Distance to Targets
            d_des = 0.5;
            rs = 0.3;

    % ----------------------------------------
    %             Create agents
    % ----------------------------------------
    
    % Agent utilizing controller and localization from Sui et al. (2025)
    AgentPDT = PDTAgent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

    % Agent utilizing controller and localization from Cao et al. (2021)
    AgentCao = PDTAgent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

    % Agent utilizing Nonholonomic Algorithms inspired by Sui et al. (2025)
    % and Zhao et al. (2019)
    NonHolonomic = PDTAgent(p_0, x_hat_0, k_omega, Tc1, 0.12, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps

    AgentPDT = AgentPDT.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets);          % --- Run estimator
    AgentPDT = AgentPDT.getPsiAndProjectionHull(t, qtyTargets);     % --- Estimate Convex Hull
    AgentPDT = AgentPDT.getVelocityPDT(t);                          % --- Run control law
    AgentPDT = AgentPDT.move(dT, t);                                % --- Execute control law

    AgentCao = AgentCao.getBearings(t, Targets);                    % --- Take bearing measurements
    AgentCao = AgentCao.estimateTargetPDT(t, dT, Targets);          % --- Run estimator
    AgentCao = AgentCao.getPsiAndProjectionHull(t, qtyTargets);     % --- Estimate Convex Hull
    AgentCao = AgentCao.getVelocityCao(t);                          % --- Run control law
    AgentCao = AgentCao.move(dT, t);                                % --- Execute control law
    
    NonHolonomic = NonHolonomic.getBearings(t, Targets);                    % --- Take bearing measurements
    NonHolonomic = NonHolonomic.estimateTargetPDT(t, dT, Targets);          % --- Run estimator
    NonHolonomic = NonHolonomic.getPsiAndProjectionEllipse(t, qtyTargets);  % --- Estimate Convex Hull
    NonHolonomic = NonHolonomic.getVelocityPDT(t);                          % --- Run control law
    NonHolonomic = NonHolonomic.move(dT, t);                                % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index

targetColors = lines(qtyTargets); 

       
%  ---- Agent Trajectory plot -----

    figure('Position',[50 100 550 450]);
    plot_traj = gca;
    hold on;
    
    % Draw True Convex Hull
    
        hull = AgentPDT.true_hull_vertices;
        k = convhull(hull(:,1), hull(:,2));
        plot(hull(k,1), hull(k,2), ...
            'Color', [0.8, 0.7, 0.1], ... 
            'LineWidth', 1.2, ...
            'DisplayName', 'True Convex Hull');
        
        hold on;
    
    % Plotting actual target positions
    
        for i = 1:qtyTargets
            xT = Targets(1, i);
            yT = Targets(2, i);
        
            if i == 1
                plot(xT, yT, 'r+', ...
                    'LineWidth', 0.7, ...
                    'MarkerSize', 7, ...
                    'Color', 'k', ...
                    'DisplayName', 'Target positions $\mathbf{x}_i$');
            else
                plot(xT, yT, 'r+', ...
                    'LineWidth', 0.7, ...
                    'MarkerSize', 7, ...
                    'Color', 'k', ...
                    'HandleVisibility', 'off');
            end
        end
    
    % Plotting Cao Agent    
    
        % plot trajectory of the single agent
        plot(AgentCao.p_traj(1,1:mytStepFinal), AgentCao.p_traj(2,1:mytStepFinal), ...
            'LineWidth', 1.5, ...
            'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Controller From Cao et al. (2021)]',...
            'Color'      ,  [0.5 0.5 0.8]);   
        hold on
    
    % Plotting PDT Agent
    
        % plot trajectory of the single agent
        plot(AgentPDT.p_traj(1,1:mytStepFinal), AgentPDT.p_traj(2,1:mytStepFinal), ...
            'LineWidth', 1.5, ...
            'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Controller Inspried by Sui et al. (2025)]',...
            'Color'      ,  [0.85, 0.15, 0.15]);   
        hold on
    
    % ==== Axis properties ====
    axis equal
    
    title('(a) Agent Trajectories')
    xlabel('x (m)')
    ylabel('y (m)')
    xlim([-5 10]);
    ylim([-2 9]);
    grid on
    set(gca,'FontSize', 10);
    legend('Interpreter','latex', 'Location','best');


%  ---- Tracking Error Plot -----

    figure('Position',[600 100 550 450]);
    plot_errors = gca;
    
    
    % Plotting PDT Agent
    
        plot(AgentPDT.delta_traj,...
            'DisplayName', '$$\delta(t)=d(t)-d^*$$ [Controller Inspried by Sui et al. (2025)]', ...
            'LineWidth', 1.6, ...
            'Color'      ,  [0.85, 0.15, 0.15]);
        hold on
    
    % Plotting Cao Agent
    
        plot(AgentCao.delta_traj,...
            'DisplayName', '$$\delta(t)=d(t)-d^*$$ [Controller From Cao et al. (2021)]', ...
            'LineWidth', 1.6, ...
            'Color'      ,  [0.5 0.5 0.8]);
        hold on
    
    
    xlim([0,tFinal/dT])
    xticks([0,0.4/dT,2/dT:2/dT:tFinal/dT])
    xt = get(gca, 'XTick');
    set(gca, 'XTick',xt, 'XTickLabel',xt*dT);
    
    ylim([-0.5, 7.5])
    yticks([0:2:6, 7.5])
    
    title('(b) Tracking Errors')
    xlabel('time (sec)')
    ylabel('Errors (m)')
    
    hold on    
        
    grid on
    set(gca,'FontSize', 10);
    legend('Interpreter','latex', 'Location','best');