function [center, radius] = minEnclosingCircleProp(points)
    n = size(points,1);
    assert(n >= 1, 'At least one point required');

    if n == 1
        center = points(1,:)';
        radius = 0;
        return;
    end

    center = mean(points,1)';
    radius = max(vecnorm(points' - center));

    for i = 1:n-1
        for j = i+1:n
            c = (points(i,:)' + points(j,:)') / 2;
            r = norm(points(i,:)' - c);
            if all(vecnorm(points' - c) <= r + 1e-12) && r < radius
                center = c;
                radius = r;
            end
        end
    end

    for i = 1:n-2
        for j = i+1:n-1
            for k = j+1:n
                p1 = points(i,:)';
                p2 = points(j,:)';
                p3 = points(k,:)';

                a = norm(p2 - p3);
                b = norm(p1 - p3);
                c = norm(p1 - p2);

                [longest, idx] = max([a b c]);

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
                    c_center = circumcenter(p1, p2, p3);
                    c_radius = norm(c_center - p1);
                else
                    switch idx
                        case 1, ends = [p2 p3];
                        case 2, ends = [p1 p3];
                        case 3, ends = [p1 p2];
                    end
                    c_center = (ends(:,1) + ends(:,2))/2;
                    c_radius = norm(ends(:,1) - ends(:,2))/2;
                end

                if all(vecnorm(points' - c_center) <= c_radius + 1e-12) && c_radius < radius
                    center = c_center;
                    radius = c_radius;
                end
            end
        end
    end
end