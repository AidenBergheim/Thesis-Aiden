function [center, radius] = minEnclosingCircleProp(points)
    n = size(points,1);

    % Finding centre and min radius
    center = mean(points,1)';
    radius = max(vecnorm(points' - center));

    for i = 1:n-1
        for j = i+1:n
            % Candidate circle from midpoint of each pair of points
            c = (points(i,:)' + points(j,:)') / 2;
            r = norm(points(i,:)' - c);
            % Update if all points are enclosed and circle is smaller
            if all(vecnorm(points' - c) <= r + 1e-12) && r < radius
                center = c;
                radius = r;
            end
        end
    end

    for i = 1:n-2
        for j = i+1:n-1
            for k = j+1:n
                % Extract triangle vertices
                p1 = points(i,:)';
                p2 = points(j,:)';
                p3 = points(k,:)';
    
                % Compute side lengths opposite each vertex
                a = norm(p2 - p3);
                b = norm(p1 - p3);
                c = norm(p1 - p2);
    
                % Identify the longest side
                [longest, idx] = max([a b c]);
    
                % Check if triangle is acute at the vertex opposite the longest side
                switch idx
                    case 1
                        v1 = p2 - p1;
                        v2 = p3 - p1;
                        isAcute = dot(v1,v2) > 0;
                    case 2
                        v1 = p1 - p2;
                        v2 = p3 - p2;
                        isAcute = dot(v1,v2) > 0;
                    case 3
                        v1 = p1 - p3;
                        v2 = p2 - p3;
                        isAcute = dot(v1,v2) > 0;
                end
    
                if isAcute
                    % Acute triangle: minimum enclosing circle is the circumcircle
                    c_center = circumcenter(p1, p2, p3);
                    c_radius = norm(c_center - p1);
                else
                    % Obtuse/right triangle: minimum enclosing circle uses longest side as diameter
                    switch idx
                        case 1, ends = [p2 p3];
                        case 2, ends = [p1 p3];
                        case 3, ends = [p1 p2];
                    end
                    c_center = (ends(:,1) + ends(:,2))/2;
                    c_radius = norm(ends(:,1) - ends(:,2))/2;
                end
    
                % Update if all points are enclosed and circle is smaller
                if all(vecnorm(points' - c_center) <= c_radius + 1e-12) && c_radius < radius
                    center = c_center;
                    radius = c_radius;
                end
            end
        end
    end
end