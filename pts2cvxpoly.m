function [ A, b, pts ] = pts2cvxpoly( pts )
%PTS2CVXPOLY Converts a set of points to a convex polygon representation
%   A convex polygon can be represented by a set of vectors stored in a
%   matrix "A" (vectors transposed and stacked) each one normal to one of
%   the edges, and a column vectors "b" storing the distance from "center"
%   to the edge specified by the corresponding normal at the same row in
%   "A."

center = mean(pts);

% Order points by angle around center starting from 0 to 2 pi
angles = mod( atan2(pts(:,2)-center(2), pts(:,1)-center(1)), 2*pi );
pts_ordered = sortrows([angles pts], 1);
pts = pts_ordered(:,2:end);

% Check Convexity
n_pts = size(pts,1);        
rp = pts(1:n_pts,:);            % Right points (looking out from inside)
mp = pts([2:n_pts, 1], :);      % Middle points     "       "
lp = pts([3:n_pts, 1, 2], :);   % Left points       "       "
% For each set of three consecutive points, take the dot product between
% the vector from right point to left point and the vector from right
% point to middle point. Will be positive if convex
convex_pts = (mp(:,1) - rp(:,1)).*(lp(:,2) - rp(:,2)) + ...
             (mp(:,2) - rp(:,2)).*(rp(:,1) - lp(:,1));
if ~all(convex_pts >= 0)
    error('Points must define a convex polygon.')
end

% Matrix of norms, which are perpendicular to edges
A = [mp(:,2) - rp(:,2), rp(:,1) - mp(:,1)];
A = normc(A')';

% distance = pt dot product norm of edge
b = diag(A*pts');

end

