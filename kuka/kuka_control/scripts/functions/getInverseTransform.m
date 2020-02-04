function [T_inv] = getInverseTransform(T)
%GETINVERSETRANSFORM Inverts homogeneous transformation matrix.

    [m,n] = size(T);
    if (n ~= 4) || (m ~= 4)
        error("Matrix must be of shape [4,4].")
    end
    
    R = T(1:3,1:3);
    d = T(1:3,4);
    R_t = transpose(R);
    
    T_inv = zeros(4,4);
    T_inv(1:3,1:3) = R_t;
    T_inv(1:3,4) = - R_t * d;
    T_inv(4,4) = 1;
end

