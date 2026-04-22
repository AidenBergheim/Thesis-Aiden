function [center, semi_axes, rotation_matrix] = minEnclosingEllipseStat(points)

    % Finding the number of points
    [n, m] = size(points); 

    % Finding centroid
    center = mean(points, 1)';

    % Calculate Covariance Matrix
    covariance_matrix = cov(points);

    % Get Eigenvectors and Eigenvalues
    [rotation_matrix, D_matrix] = eig(covariance_matrix);
    eigenvalues = diag(D_matrix);

    % Scale Ellipse to Enclose All Points
    centered_points = points - center'; 
    
    % Calculate the squared Mahalanobis distance for all points
    mahala_dist_sq = zeros(n, 1);
    for i = 1:n
        mahala_dist_sq(i) = centered_points(i,:) * inv(covariance_matrix) * centered_points(i,:)';
    end
    
    % Find the maximum squared distance and scale factor
    s_sq_max = max(mahala_dist_sq);
    s = sqrt(s_sq_max);

    % Calculate Ellipse Semi-Axis Lengths 
    semi_axes = s * sqrt(eigenvalues); 
    
    % Sort axes from largest to smallest for consistency
    [semi_axes, sort_idx] = sort(semi_axes, 'descend');
    rotation_matrix = rotation_matrix(:, sort_idx);
end

