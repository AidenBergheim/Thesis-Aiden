function true_hull_vertices = true_hull(targets)

    numTargets = size(targets, 2);
    target_positions = targets'; % Convert to N×2 matrix
    
    if numTargets == 1
        true_hull_vertices = target_positions;
    elseif numTargets == 2
        true_hull_vertices = target_positions;
    else
        try
            K = convhull(target_positions(:,1), target_positions(:,2));
            true_hull_vertices = target_positions(K, :);
        catch
            % Fallback for collinear points
            true_hull_vertices = target_positions;
        end
    end

end