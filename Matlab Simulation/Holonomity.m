
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ----- Sim for Multi-Target BoTLC in PDT -----
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clf
close all
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Initial Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    tBegin = 0;             % (seconds)
    tFinal = 12;            % (seconds)
    dT     = 0.001;         % (seconds)
    
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
            alpha_2 = 0.1;
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
    %             Create agent
    % ----------------------------------------
    
    AgentPDT = PDTAgent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

    %d_hat_0 = vecnorm(repmat(p_0, 1, qtyTargets) - Targets) - norm(Targets-x_hat_0);
    %AgentCao = CaoAgent(p_0, d_hat_0, x_hat_0, alpha, k_omega, rs, qtyTargets, d_des, tSteps,hullVertices);
    AgentCao = PDTAgent(p_0, x_hat_0, k_omega, Tc1, Tc2, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

    NonHolonomic = PDTAgent(p_0, x_hat_0, k_omega, Tc1, 0.12, ...
                        alpha_1, alpha_2, d_des, rs, tSteps, qtyTargets, hullVertices, initial_heading);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              Main Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:tSteps

    AgentPDT = AgentPDT.getBearings(t, Targets);                % --- Take bearing measurements
    AgentPDT = AgentPDT.estimateTargetPDT(t, dT, Targets);         % --- Run estimator
    AgentPDT = AgentPDT.getPsiAndProjectionHull(t, qtyTargets);     % --- Estimate Convex Hull
    AgentPDT = AgentPDT.getVelocityPDT(t);                         % --- Run control law
    AgentPDT = AgentPDT.moveNonHolonomic(dT, t);                            % --- Execute control law

    AgentCao = AgentCao.getBearings(t, Targets);                % --- Take bearing measurements
    AgentCao = AgentCao.estimateTargetPDT(t, dT, Targets);         % --- Run estimator
    AgentCao = AgentCao.getPsiAndProjectionMinCircle(t, qtyTargets);     % --- Estimate Convex Hull
    AgentCao = AgentCao.getVelocityPDT(t);                         % --- Run control law
    AgentCao = AgentCao.moveNonHolonomic(dT, t);                            % --- Execute control law
    
    %NonHolonomic = NonHolonomic.getBearings(t, Targets);                % --- Take bearing measurements
    %NonHolonomic = NonHolonomic.estimateTargetPDT(t, dT, Targets);         % --- Run estimator
    %NonHolonomic = NonHolonomic.getPsiAndProjectionEllipse(t, qtyTargets);     % --- Estimate Convex Hull
    %NonHolonomic = NonHolonomic.getVelocityPDT(t);                         % --- Run control law
    %NonHolonomic = NonHolonomic.move(dT, t);                            % --- Execute control law

end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mytStepFinal = max(1, min(round(tFinal/dT), tSteps)); % ensure integer index

targetColors = lines(qtyTargets); 


% ===============================        
%  ==== Agent Trajectory plot ====

figure('Position',[50 100 550 450]);
plt_agent_traj = gca;

hold on; % hold once before loop

% ==== Draw true convex hull ====
hull = AgentPDT.true_hull_vertices;
k = convhull(hull(:,1), hull(:,2));
plot(hull(k,1), hull(k,2), ...
    'Color', [0.8, 0.7, 0.1], ... % bright yellow
    'LineWidth', 1.2, ...
    'DisplayName', 'True Convex Hull');

hold on;

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
            'HandleVisibility', 'off');  % No legend entry
    end
end



% Plotting Cao Agent    

    % plot trajectory of the single agent
    plot(AgentCao.p_traj(1,1:mytStepFinal), AgentCao.p_traj(2,1:mytStepFinal), ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Around Minmum Enclosing Circle]',...
        'Color'      ,  [0.5 0.5 0.8]);   
    hold on
    
    % Plot estimate of targets
    %for i = 1:qtyTargets
    %    plot(AgentCao.x_hat{i}(1,1:mytStepFinal), AgentCao.x_hat{i}(2,1:mytStepFinal), ...
    %        'DisplayName', ['$\hat{\mathbf{x}}_' num2str(i) '(t)$'], ...
    %        'Color', 'r', ...
    %        'LineWidth', 0.9);
    %end
    %hold on;

% Plotting PDT Agent

    % plot trajectory of the single agent
    plot(AgentPDT.p_traj(1,1:mytStepFinal), AgentPDT.p_traj(2,1:mytStepFinal), ...
        'LineWidth', 1.5, ...
        'DisplayName', '$$\mbox{\boldmath$y$}(t)$$ [Around Convex Hull',...
        'Color'      ,  [0.85, 0.15, 0.15]);   
    hold on
    
   % Plot estimate of targets
    %for i = 1:qtyTargets
    %    plot(AgentPDT.x_hat{i}(1,1:mytStepFinal), AgentPDT.x_hat{i}(2,1:mytStepFinal), ...
    %        'DisplayName', ['$\hat{\mathbf{x}}_' num2str(i) '(t)$'], ...
    %        'Color', 'g', ...
    %        'LineWidth', 0.9);
    %end
    %hold on;


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

% ==================================        
% ==== Tracking Error plot ====

figure('Position',[600 100 550 450]);
plt_tracking_errors = gca;

plt_tracking_errors = gca;

% ----------------------------
% ---- plot delta(t) traj ----
% ----------------------------
plot(AgentPDT.delta_traj,...
    'DisplayName', '$$\delta(t)=d(t)-d^*$$ [Controller Inspried by Sui et al. (2025)]', ...
    'LineWidth', 1.6, ...
    'Color'      ,  [0.85, 0.15, 0.15]);
hold on

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