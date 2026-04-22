function center = circumcenter(p1, p2, p3)
    A = p2 - p1;
    B = p3 - p1;
    D = 2*(A(1)*B(2) - A(2)*B(1));
    if abs(D) < 1e-12
        dists = [norm(p2-p3), norm(p1-p3), norm(p1-p2)];
        [~, idx] = max(dists);
        switch idx
            case 1, center = (p2+p3)/2;
            case 2, center = (p1+p3)/2;
            case 3, center = (p1+p2)/2;
        end
        return;
    end
    Ux = ((norm(A)^2)*B(2) - (norm(B)^2)*A(2)) / D;
    Uy = ((norm(B)^2)*A(1) - (norm(A)^2)*B(1)) / D;
    center = p1 + [Ux; Uy];
end