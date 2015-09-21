function obj = create_and_draw_car(z_n,R_n,mode,n, varargin)
% CREATE_AND_DRAW_CAR - Creates image of object (car) to be controlled
%   r - Position vector of object
%   R - Rotation matrix of object
%   mode - Called in CREATE_CAR

%% Object : Car

if nargin == 0
    scale_size = 1;
else
    scale_size = varargin{1};
end

for j = 1:n
    
    obj(j) = create_car(mode, scale_size);
    
end

for j = 1:n
    
    r = z_n(1:3,j);
    R = R_n(1:3,(j-1)*3+1:j*3);
    % Position object according to state location and orientation
    % and Plot the components
    
    obj(j).Handle = patch('Faces',obj(j).facs,'Vertices',...
        obj(j).vertices','FaceColor',obj(j).color,...
        'FaceAlpha',obj(j).alpha);
    % handle for the 3D model of the car
    
    obj(j).vertices = obj(j).vertices + r(:,ones(1,length(obj(j).vertices)));
    set(obj(j).Handle,'Vertices',obj(j).vertices');
    
    for i = 1:4
        coords = trans(obj(j).arm(i).wheels.coords,R,r);
        obj(j).arm(i).wheels.plot = ...
            fill3(coords(1,:), coords(2,:), coords(3,:), [0 0 1]);
    end
    
    obj(j).trajHandle = line(r(1,:), r(2,:), r(3,:), 'color', [.4 .4 .8],'LineWidth', 2);
    % handle for the trajectory followed by the car
    
    obj(j).z = z_n(:,j);
    
end