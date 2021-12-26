function F = Fundamental_Matrix(coordinate2, coordinate4)

A = zeros(8,9);

% 8-point algorithm
%create rows of A matrix using 8 points
for i=1:8
    p1 = coordinate2(:,i);
    x1 = p1(1);
    y1 = p1(2);
    
    p2 = coordinate4(:,i);
    x2 = p2(1);
    y2 = p2(2);
    
    
    temp = [x1*x2, x1*y2, x1, y1*x2, y1*y2, y1, x2, y2, 1];
    A(i,:) = temp;
end

[U S V] = svd(A);
% sanity check: rank(A)==8?  --> least singular value at S(8,8)
eigV = V(:,8)

F = zeros(3,3);
F(1,:) = [eigV(1), eigV(2), eigV(3)];
F(2,:) = [eigV(4), eigV(5), eigV(6)];
F(3,:) = [eigV(7), eigV(8), eigV(9)];

%coordinate2(:,5)' * F * coordinate4(:,5)
%coordinate4(:,5)' * F * coordinate2(:,5)

%rank(F) % --> rank(F) is 3 but we want to enforce 2

[U S V] = svd(F); %--> S is 3x3

S(3,3) = 0;

F = U*S*V';
F= F';
%rank(F) % --> rank(F) is now 2

end