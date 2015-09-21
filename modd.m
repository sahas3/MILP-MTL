function y = modd(x,n)
% MODD - Element-wise modulo operator
%   This function returns a vector x modulo n, and if any of the
%   elements are zero, they are replaced with n

y = mod(x,n);
y(y == 0) = n;

