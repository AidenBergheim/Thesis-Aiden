classdef PDTAgent

    properties (Access = public)
        p
        p_dot
        p_traj
        label

        d               % 1xT cell, each containing 1xN vector (N = numTargets)
        varphi          % 1xN cell, each containing a 2x1 vector
        bar_varphi      % 1xN cell, each containing a 2x1 vector
        varphi_traj     % 1xN cell, each containing a 2xT matrix
        psi
        psi_hat
        projPoint

        % Estimator for each target
        x_hat           % 1xN cell, each containing 2xT matrix
        x_hat_dot       % 1xN cell, each containing 2xT matrix
        P               % 1xN cell, each containing 1xT cell of 2x2 matrices
        q               % 1xN cell, each containing 1xT cell of 2x1 vectors

        Tc1
        alpha_1

        d_hat           % 1xN vector, estimated agent-target distances
        d_convex_hat
        d_tilde
        u
        rs

        k_omega
        d_des
        alpha_2
        Tc2

        % For animation
        delta_traj
        d_convex_true
        varrho_traj
        x_tilde_traj
        projPoint_traj
        true_hull_vertices
    end

    methods

        function obj = PDTAgent(p_0, x_hat_0_cell, k_omega, Tc1, Tc2, alpha_1, alpha_2, d_des, rs, tSteps, numTargets, hullVertices)
            obj.p = p_0;
            obj.p_dot = zeros(2, 1);
            obj.p_traj = zeros(2, tSteps);

            obj.k_omega = k_omega;
            obj.Tc1 = Tc1;
            obj.Tc2 = Tc2;
            obj.alpha_1 = alpha_1;
            obj.alpha_2 = alpha_2;
            obj.d_des = d_des;

            obj.d = cell(1, tSteps);
            obj.d_hat = zeros(1, numTargets);
            obj.d_convex_hat = 0;
            obj.d_tilde = 0;

            obj.varphi = cell(1, numTargets);
            obj.bar_varphi = cell(1, numTargets);
            obj.varphi_traj = cell(1, numTargets);
            obj.x_hat = cell(1, numTargets);
            obj.x_hat_dot = cell(1, numTargets);
            obj.psi = zeros(2, 1);
            obj.psi_hat = zeros(2, 1);
            obj.projPoint = zeros(2, 1);
            obj.P = cell(1, numTargets);
            obj.q = cell(1, numTargets);

            obj.delta_traj = zeros(1,tSteps);
            obj.d_convex_true = zeros(1,tSteps);
            obj.varrho_traj = cell(1, numTargets);
            obj.x_tilde_traj = cell(1, numTargets);
            obj.projPoint_traj = zeros(2, tSteps);

            for i = 1:numTargets
                obj.varphi_traj{i} = zeros(2, tSteps);
                obj.x_hat{i} = zeros(2, tSteps);
                obj.x_hat{i}(:, 1) = x_hat_0_cell(:, i);

                obj.x_hat_dot{i} = zeros(2, tSteps);
                obj.P{i} = cell(1, tSteps);
                obj.P{i}{1} = zeros(2, 2);
                obj.q{i} = cell(1, tSteps);
                obj.q{i}{1} = zeros(2, 1);

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
                target_pos = targets(:, i);  % Extract target position from matrix
                d_i = norm(target_pos - obj.p);
                distances(i) = d_i;
                obj.varphi{i} = (target_pos - obj.p) / d_i;

                obj.bar_varphi{i} = [ cos(pi/2), sin(pi/2);
                                    -sin(pi/2), cos(pi/2)] * obj.varphi{i};
                obj.varphi_traj{i}(:, t) = obj.varphi{i};
                obj.d_hat(i) = norm(obj.x_hat{i}(:, t) - obj.p);
            end

            obj.d{t} = distances;

            % NOT USED BY CONTROLLER - recording tracking error
            obj.d_convex_true(t) = computeTrueDistanceToHull(obj);
            obj.delta_traj(t) = obj.d_convex_true(t) - obj.d_des;
        end

        function obj = estimateTarget(obj, cur_tStep, dT, targets)
            t = cur_tStep;
            numTargets = size(targets, 2);

            for i = 1:numTargets
                if t <= 2
                    obj.x_hat_dot{i}(:, t) = zeros(2, 1);
                else
                    xi = inv(obj.P{i}{t}) * (obj.P{i}{t} * obj.x_hat{i}(:, t) - obj.q{i}{t});
                    if norm(xi) == 0
                        Psi_a = zeros(2, 1);
                    else
                        Psi_a = xi ./ (norm(xi) ^ obj.alpha_1);
                    end
                    obj.x_hat_dot{i}(:, t) = -1 / (obj.Tc1 * obj.alpha_1) * exp(norm(xi) ^ obj.alpha_1) * Psi_a;
                end

                P_dot = -obj.P{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.';
                q_dot = -obj.q{i}{t} + obj.bar_varphi{i} * obj.bar_varphi{i}.' * obj.p;

                obj.P{i}{t+1} = obj.P{i}{t} + P_dot * dT;
                obj.q{i}{t+1} = obj.q{i}{t} + q_dot * dT;

                if t + 1 <= size(obj.x_hat{i}, 2)
                    obj.x_hat{i}(:, t+1) = obj.x_hat{i}(:, t) + obj.x_hat_dot{i}(:, t) * dT;
                end
            end
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

            v_cen = 1/(obj.alpha_2 * obj.Tc2) * exp(abs(obj.d_tilde) ^ obj.alpha_2) * sig(obj.d_tilde, 1 - obj.alpha_2);
            
            % Ensuring safe distance
            u_safe_val = (obj.d_convex_hat > obj.rs);
            v_tan = obj.k_omega * u_safe_val;

            obj.u = v_cen * obj.psi + v_tan * obj.psi_hat;
        end

        function obj = getVelocityCao(obj, t)

            obj.d_tilde = obj.d_convex_hat - obj.d_des;

            v_cen = (obj.d_convex_hat - obj.d_des);

            % Ensuring safe distance
            u_safe_val = (obj.d_convex_hat > obj.rs);
            v_tan = 5 * u_safe_val;

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