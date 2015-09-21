function [ B ] = normc( A )
%NORMC normalize the columns of a matrix to unit vectors
B = A ./ (ones(size(A,1),1)*sqrt(sum(A.^2)));
end

