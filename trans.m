function new_coords=trans(coords,R,r)
% TRANS - transforms the given coordinates by rotating them by R
%         then translating the coordinates by r
    new_coords = R*coords + ...
                 r*ones(1,size(coords, 2));