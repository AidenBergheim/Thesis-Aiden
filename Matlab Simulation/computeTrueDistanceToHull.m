function d_true = computeTrueDistanceToHull(obj)
    hull = obj.true_hull_vertices;

    if isempty(hull)
        d_true = NaN;
        return;
    end

    numVertices = size(hull, 1);

    % Convex polygon check
    if inpolygon(obj.p(1), obj.p(2), hull(:,1), hull(:,2))
        d_true = 0;  % Agent is inside the hull
    else
        % Agent is outside — compute closest distance to hull edges
        min_dist = inf;

        for i = 1:numVertices
            % Wrap around from last to first
            A = hull(i, :)';
            B = hull(mod(i, numVertices) + 1, :)';  % Wraps index

            AB = B - A;
            AP = obj.p - A;

            % Skip zero-length edges
            if norm(AB) < 1e-12
                continue;
            end

            % Project point onto edge segment
            t_param = max(0, min(1, dot(AP, AB) / dot(AB, AB)));
            proj = A + t_param * AB;
            dist = norm(obj.p - proj);

            if dist < min_dist
                min_dist = dist;
            end
        end

        d_true = min_dist;
    end
end
