function L2 = Euclidean(original, reconstructed)

[row, col] = size(original);

L2 = zeros(1,col);
for i=1:col 
    p1 = original(:, i);
    p2 = reconstructed(:, i);
    L2(i) = (p1(1)-p2(1))^2 + (p1(2)-p2(2))^2 + (p1(3)-p2(3))^2;
end
