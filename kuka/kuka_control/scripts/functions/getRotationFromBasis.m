function [R] = getRotationFromBasis(basis_x, basis_y, basis_z)
%GETROTATIONFROMBASIS Calculates rotation between standard basis and given
%orthogonal basis.
%   Standard basis is defined by [1,0,0], [0,1,0] and [0,0,1]. New basis   
%   vectors are given in the standard basis. The new basis vectors need to 
%   be linearly independent and the basis must be invertible.

des_size = size([0;0;0]);

if ~ isequal(size(basis_x), des_size) || ~ isequal(size(basis_y), des_size) || ~ isequal(size(basis_z), des_size)
    error("basis vectors need to be column vectors")
end 

% in case we are given non-orthonormal basis
basis_x = basis_x / norm(basis_x);
basis_y = basis_y / norm(basis_y);
basis_z = basis_z / norm(basis_z);

% rotation from standard basis to new basis is inverse of new basis as 
% shown in standard basis
R = inv([basis_x, basis_y, basis_z]);

end

