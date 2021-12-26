function points3D = triangulate(points2D_2, points2D_4, P2, P4)

    [row, col] = size(points2D_2);  %--> 3x12
    points3D = ones(row+1, col);  % --> 4x12
    %size(points3D)
    p11T = P2(1,:);  %--> first row Pmat1, size: 1x4
    p12T = P2(2,:);
    p13T = P2(3,:);
    
    p21T = P4(1,:);
    p22T = P4(2,:);
    p23T = P4(3,:);
    
    for i=1:col
        point1 = points2D_2(:,i);
        point2 = points2D_4(:,i);
        
        %A1 = [y*p13T - p12T
        %      p11T - xp13T]
        
        A1 = [point1(2)*p13T - p12T; p11T - point1(1)*p13T];
        A2 = [point2(2)*p23T - p22T; p21T - point2(1)*p23T];
        
        A = [A1; A2];
        [V,D] = eigs(A);
        
        points3D(:,i) = V(:,4)./V(4,4)';
    end
    points3D
end