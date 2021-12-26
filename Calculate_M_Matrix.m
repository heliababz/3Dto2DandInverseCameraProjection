function [M, location] = Calculate_M_Matrix(vue)

    M = vue.Kmat*vue.Pmat;
    
    %singular value decomposition
    [U, S, V] = svd(M);
    
    %P2 = U*S*V.'
    %because in S sigma1>sigma2>...>sigmar and we want the vector 
    %correspoinding to the smallest eigenvalue so it is at rth positiom
    r = rank(M);
    
    %eig = S(r,r);
    %eigevector at rth row of the right SV 
    %corresponds to the smallest eigenvalue in S  (B = P.'*P)
    location = V(:,4); % ---> location should be 1x3 but here i'm getting 1x4
    
    location = location/location(4);

    %location = vue.position.' 
    % I don't understand why we are supposed to calculate location when
    % it's already given to us in the vue files
end 