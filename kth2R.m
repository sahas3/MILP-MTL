function R = kth2R(k,th)
% kth2r outputs a rotation matrix R corresponding to the rotation of
% theta angle in radian by the axis represented bu the 3x1 vector k
%
% R = kth2R( k, th )
% IN:
%     k - vector around which the rotation will be performed
%     th - rotation angle
%
%
% OUT:
%     R - rotation matrix
%
% BY: Sayan Saha

knorm = norm(k,2);
% if( numel( k ) ~= 3 )
%     fprintf( 'rotationVector should have 3 coordinates!\r' );
%     return;
% end

k = reshape( k, 3, 1 );
% if( knorm > 0 )
    k = k / knorm;
    kcross = getCross(k);
    R = eye(3) + sin(th)*kcross + (1-cos(th))*(kcross)^2;
% else
%     fprintf( 'rotation Vector cannot be 0\n\r' );
% end

end