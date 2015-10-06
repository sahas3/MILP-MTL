function pts = order_pts_center( pts )
% pts = ORDER_PTS_CENTER( pts )
% Order points by angle around center starting from 0 to 2 pi

center = mean(pts);

angles = mod( atan2(pts(:,2)-center(2), pts(:,1)-center(1)), 2*pi );
pts_ordered = sortrows([angles pts], 1);
pts = pts_ordered(:,2:end);

end