classdef CaoAgent

    properties (Access = public)

        p          % 2x1 vector for agent's location at time t
                   % p = [  x(t) ;
                   %        y(t)  ]

        p_dot      % 2x1 vector for dynamics
                   % p_dot = [  v_x(t) ; 
                   %            v_y(t)  ]

        p_traj     % 2xT matrix for position trajectory (T is tSteps to be
                   % specified)
                   % The index t of this matrix, p_traj(2,t), represents
                   % the agent's location at time t
                   % p_traj = [  x(0) ;  x(1) ; ...
                   %             y(0) ;  y(1) ; ... ]

        label      % scalar for agent's label
                   % not used if the system contains only 1 agent

        d          % 1xT vector for agent-target distance at time t
                   % used ONLY for simulation
                   % the on-board controller does not need this information


        varphi          % 2x1 vector for unit bearing vector pointing 
                        % from p to p_T

        varphi_traj     % 2xT vector for the time history of varphi

        varphi_dot      % 2x1 vector for unit bearing vector pointing 
                        % from p to p_T

        bar_varphi      % 2x1 vector for unit bearing vector rotated 
                        % pi/2 CCW from varphi


        psi
        psi_hat
        projPoint

        x_hat         % 2xT vector for agent i's estimate of the target's 
                      % location at time t
                      % where T is tSteps

        x_hat_dot     % 2x1 vector for tuning law of the target estimator

        d_hat         % 1xT vector for the estimated agent-target distance

        d_hat_dot     % scalar for the tuning law 



        d_convex_hat

        d_des
        d_tilde
        % speed = k * (d_tilde) 
        k

        % ========== Tangential Direction ========== 
        k_eta;        

        % ========== Control Law ==========
        % eqn: u = k * (d_tilde) * varphi + k_eta * bar_varphi 

        u          % 2x1 vector for the circumnavigation control law
        rs

        k_omega
        alpha
        

        delta            % scalar defined as
                         %   delta = d - d_des
                         % delta_i is an auxiliary variable used only in 
                         % the stability analysis and plots

        delta_traj       % 1xT vector for history of delta


        d_convex_true


        varrho           % scalar defined as
                         %   varrho = d - d_hat
                         % varrho is an auxiliary variable used only in 
                         % the stability analysis and plots
        
        varrho_traj      % 1xT vector for history of varrho

        
        x_tilde_traj     % 2xT matrix for history of estimated target 
                         % location 

        projPoint_traj
        true_hull_vertices
    end

    methods

        function obj = CaoAgent(p_0, d_hat_0, x_hat_0, ...
                                      k_eta, ...
                                      k, ...
                                      rs, ...
                                      numTargets, ...
                                      d_des, tSteps, hullVertices)

            obj.p               = p_0;
            obj.p_dot           = zeros(2, 1);
            obj.p_traj          = zeros(2, tSteps);

            obj.d_des             = d_des;

            % =================== Controller Gains =================== %

            % centripetal speed 
            obj.k               = k;

            % tangential speed
            obj.k_eta             = k_eta;        


            obj.d                 = cell(1, tSteps);
            

            obj.x_hat_dot         = zeros(2,1);
            obj.d_hat_dot         = cell(1, numTargets);
            
            obj.delta_traj        = zeros(1,tSteps);
            obj.varrho_traj       = zeros(1,tSteps);
            obj.x_tilde_traj      = zeros(2,tSteps); 

            obj.d_convex_hat = 0;
            obj.d_tilde = 0;

            obj.varphi = cell(1, numTargets);
            obj.bar_varphi = cell(1, numTargets);
            obj.varphi_dot = cell(1, numTargets);
            obj.varphi_traj = cell(1, numTargets);
            obj.d_hat = cell(1, numTargets);
            obj.x_hat = cell(1, numTargets);
            obj.x_hat_dot = cell(1, numTargets);
            obj.psi = zeros(2, 1);
            obj.psi_hat = zeros(2, 1);
            obj.projPoint = zeros(2, 1);

            obj.delta_traj = zeros(1,tSteps);
            obj.d_convex_true = zeros(1,tSteps);
            obj.varrho_traj = cell(1, numTargets);
            obj.x_tilde_traj = cell(1, numTargets);
            obj.projPoint_traj = zeros(2, tSteps);

            for i = 1:numTargets
                obj.varphi_traj{i} = zeros(2, tSteps);
                obj.x_hat{i} = zeros(2, tSteps);
                obj.x_hat{i}(:, 1) = x_hat_0(:, i);  % <-- FIXED
            
                obj.x_hat_dot{i} = zeros(2, tSteps);
                obj.d_hat{i} = zeros(1, tSteps);
                obj.d_hat{i}(1) = d_hat_0(i);
            
                obj.varrho_traj{i} = zeros(1, tSteps);
                obj.x_tilde_traj{i} = zeros(2, tSteps);
            end

            obj.true_hull_vertices = hullVertices;
            obj.rs = rs;
        end



        function obj = getBearings(obj, cur_tStep, targets)
            t = cur_tStep;
            numTargets = size(targets, 2);
            distances = zeros(1, numTargets);
        
            for i = 1:numTargets
                % Get true target position
                target_pos = targets(:, i);
        
                % Compute true distance and bearing
                d_i = norm(target_pos - obj.p);
                distances(i) = d_i;
        
                % Store bearing vector
                obj.varphi{i} = (target_pos - obj.p) / d_i;
        
                % Compute bearing derivative if t > 1
                if t > 1
                    obj.varphi_dot{i} = (obj.varphi{i} - obj.varphi_traj{i}(:, t-1)) / double(1);  % Δt = 1 timestep assumed
                else
                    obj.varphi_dot{i} = zeros(2,1);
                end
        
                % Rotate bearing vector CCW by 90°
                obj.bar_varphi{i} = [cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.varphi{i};
        
                % Store bearing history
                obj.varphi_traj{i}(:, t) = obj.varphi{i};
        
                % Update estimated distance (d_hat) from estimated target to agent
                obj.d_hat{i}(t) = norm(obj.x_hat{i}(:, t) - obj.p);
            end
        
            % Save true distances for analysis only
            obj.d{t} = distances;
        
            % Convex hull error and distance tracking
            obj.d_convex_true(t) = computeTrueDistanceToHull(obj);
            obj.delta_traj(t) = obj.d_convex_true(t) - obj.d_des;
        end

         function obj = estimateTarget(obj, cur_tStep, dT, targets)
            t = cur_tStep;
            numTargets = size(targets, 2);
        
            for i = 1:numTargets
                % Update x_hat at current step based on current estimated distance and bearing
                obj.x_hat{i}(:, t) = obj.d_hat{i}(t) * obj.varphi{i} + obj.p;
        
                if t > 1
                    % Estimate x_hat_dot (target velocity estimate)
                    obj.x_hat_dot{i}(:, t) = (obj.x_hat{i}(:, t) - obj.x_hat{i}(:, t-1)) / dT;
        
                    % Compute d_hat_dot using agent velocity and target estimate
                    obj.d_hat_dot{i}(:, t) = -obj.varphi{i}' * obj.p_dot + ...
                                        sign(obj.bar_varphi{i}' * obj.p_dot) * obj.bar_varphi{i}' * obj.x_hat_dot{i}(:, t);
                else
                    obj.x_hat_dot{i}(:, t) = zeros(2, 1);
                    obj.d_hat_dot{i}(:, t) = 0;
                end
        
                % Euler update of distance estimate
                if t + 1 <= length(obj.d_hat)
                    obj.d_hat{i}(t+1) = obj.d_hat{i}(t) + obj.d_hat_dot{i}(t) * dT;
                end
            end
        
            % Optional: recompute convex distance error after update
            obj.d_tilde = obj.d_convex_hat - obj.d_des;
        end





        function obj = getPsiAndProjection(obj, cur_tStep, numTargets)
            agentPos = obj.p;
            
            % Extract estimated positions
            est_positions = zeros(numTargets, 2);
            for i = 1:numTargets
                est_positions(i, :) = obj.x_hat{i}(:, cur_tStep)';
            end
            
            % Remove duplicates
            est_positions = unique(est_positions, 'rows', 'stable');
            n = size(est_positions, 1);
            
            est_positions
            K = convhull(est_positions(:,1), est_positions(:,2));
            hull = est_positions(K, :);
            
            if inpolygon(agentPos(1), agentPos(2), hull(:,1), hull(:,2))
                projPoint = agentPos;
            else
                min_dist = inf;
                for i = 1:(size(hull,1)-1)
                    % Inline projection for hull edges
                    A = hull(i,:)';
                    B = hull(i+1,:)';
                    AB = B - A;
                    AP = agentPos - A;
                    t = max(0, min(1, dot(AP, AB) / dot(AB, AB)));
                    proj = A + t * AB;
                    
                    dist = norm(agentPos - proj);
                    if dist < min_dist
                        min_dist = dist;
                        projPoint = proj;
                    end
                end
            end

            
            % Compute psi and psi_hat
            obj.psi = projPoint - agentPos;
            norm_psi = norm(obj.psi);
            obj.psi = obj.psi / norm_psi;
            obj.psi_hat = [ cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.psi;
            
            % Store results
            obj.projPoint = projPoint;
            obj.projPoint_traj(:, cur_tStep) = obj.projPoint;
            obj.d_convex_hat = norm_psi;
        end


        function obj = getVelocity(obj, t)

            obj.d_tilde = obj.d_convex_hat - obj.d_des;

            v_cen = obj.k * (obj.d_convex_hat - obj.d_des);

            % Ensuring safe distance
            u_safe_val = (obj.d_convex_hat > obj.rs);
            v_tan = obj.k_eta * u_safe_val;

            obj.u = v_cen * obj.psi ...
                   + v_tan * obj.psi_hat;
        end

        function obj = move(obj, dT, t)
            obj.p_dot = obj.u;
            obj.p = obj.p + obj.p_dot * dT;
            obj.p_traj(:, t) = obj.p;
        end
        

    end
end