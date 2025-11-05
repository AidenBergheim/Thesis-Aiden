function [center, semi_axes, rotation_matrix] = minEnclosingEllipseStat(points)
% minEnclosingEllipseStat Finds an approximate minimum enclosing ellipse
% for a set of points using the statistical covariance method.
%
% This is an APPROXIMATION and not the true Minimum Volume Enclosing
% Ellipsoid (MVEE). It calculates the confidence ellipse from the data's
% covariance and scales it to enclose all points.
%
% INPUT:
%   points: [n x m] matrix, where 'n' is the number of points and
%           'm' is the dimension (e.g., 2 for 2D points).
%
% OUTPUTS:
%   center: [m x 1] column vector representing the center of the ellipse (the centroid).
%   semi_axes: [m x 1] column vector of the lengths of the ellipse's semi-axes,
%              sorted from largest to smallest.
%   rotation_matrix: [m x m] matrix where each column is an eigenvector
%                    representing the orientation of the corresponding semi-axis.

    % --- Handle inputs and edge cases ---
    [n, m] = size(points); % n = num points, m = num dimensions

    % Check for unique points to avoid rank-deficient matrix
    unique_points = unique(points, 'rows');
    if size(unique_points, 1) <= m || rank(points - mean(points,1)) < m
        % Not enough unique points to calculate a full-rank covariance ellipse.
        % Return a degenerate circle with radius 0.
        center = mean(points, 1)';
        semi_axes = zeros(m, 1);
        rotation_matrix = eye(m);
        return;
    end

    % --- Step 1: Find the Center (Mean/Centroid) ---
    % The center of the statistical ellipse is the mean of the points.
    center = mean(points, 1)'; % [m x 1] column vector

    % --- Step 2: Calculate Covariance Matrix ---
    % This matrix describes the spread and orientation of the data.
    covariance_matrix = cov(points); % [m x m] matrix

    % --- Step 3: Get Eigenvectors and Eigenvalues ---
    % 'eig' returns eigenvectors (V) and a diagonal matrix of eigenvalues (D)
    % Eigenvectors give the direction of the ellipse's axes.
    % Eigenvalues give the variance along those axes.
    [rotation_matrix, D_matrix] = eig(covariance_matrix);
    eigenvalues = diag(D_matrix);
    
    % Ensure eigenvalues are positive to avoid sqrt(negative)
    if any(eigenvalues <= 0)
        % This can happen with collinear points, return a degenerate circle
        center = mean(points, 1)';
        semi_axes = zeros(m, 1);
        rotation_matrix = eye(m);
        return;
    end

    % --- Step 4: Scale Ellipse to Enclose All Points ---
    
    % Center the points (subtract the mean)
    centered_points = points - center'; % [n x m] matrix
    
    % We need to find a scaling factor 's^2' such that for all points 'p':
    %   (p - c)' * inv(C) * (p - c) <= s^2
    % where C is the covariance matrix. This value is the
    % squared Mahalanobis distance.

    inv_covariance = inv(covariance_matrix);
    
    % Calculate the squared Mahalanobis distance for all points
    mahala_dist_sq = zeros(n, 1);
    for i = 1:n
        mahala_dist_sq(i) = centered_points(i,:) * inv_covariance * centered_points(i,:)';
    end
    
    % Find the maximum squared distance
    s_sq_max = max(mahala_dist_sq);
    
    % The scale factor 's' is the sqrt of this
    s = sqrt(s_sq_max);

    % --- Step 5: Calculate Semi-Axis Lengths ---
    % The semi-axes are the scaled standard deviations (sqrt of eigenvalues)
    semi_axes = s * sqrt(eigenvalues); % [m x 1] column vector
    
    % Sort axes from largest to smallest for consistency
    [semi_axes, sort_idx] = sort(semi_axes, 'descend');
    rotation_matrix = rotation_matrix(:, sort_idx);

end