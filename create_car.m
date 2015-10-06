function obj = create_car(varargin)
% CREATE_OBJECT - Creates visual representation of object to be controlled

mode = varargin{1};
if nargin == 1
    scale_size = 1;
else
    scale_size = varargin{2};
end

if mode % for getting trajectory
    obj.L = scale_size*(0.55+0.35)*2;            % Car length
    obj.W = scale_size*(0.55+0.35)*2;            % Car width
else % for representation purposes
    obj.L = scale_size*0.55;            % Car length
    obj.W = scale_size*0.55;            % Car width
end

obj.H = scale_size*0.25;            % Car height
obj.CoM = [0;0;0];       % Center of Mass
obj.gravity = 9.81;      % m-sec^2 - gravity
obj.mass = 0.5;
obj.r_wheel = scale_size*0.15;       % radius of the wheels


obj.facs = [1 2 3 4
    5 8 7 6
    4 3 8 5
    3 2 7 8
    2 1 6 7
    1 4 5 6];

obj.coord.x = 0.5*obj.L*[-1 -1 1 1 1 -1 -1 1]';
obj.coord.y = 0.5*obj.W*[-1 1 1 -1 -1 -1 1 1]';
obj.coord.z = 0.5*obj.H*[1 1 1 1 -1 -1 -1 -1]';

obj.vertices = [obj.coord.x...
    obj.coord.y obj.coord.z]';

obj.vertices = obj.vertices + ...
    obj.CoM(:,ones(1,length(obj.vertices)));

obj.friction = 0.2;

obj.color = 'c';
obj.alpha = 1;
obj.u_max = [1.5 -1.5 1.5];

res = 1/30; % Resolution for drawing wheels

arm.wheels1 = [-obj.L/2  +  0*(0:res:1);
    obj.W/4  +  obj.r_wheel*sin(2*pi*(0:res:1));
    -obj.H/2    +  obj.r_wheel*cos(2*pi*(0:res:1))];

arm.wheels2 = [obj.L/2  +  0*(0:res:1);
    obj.W/4  +  obj.r_wheel*sin(2*pi*(0:res:1));
    -obj.H/2    +  obj.r_wheel*cos(2*pi*(0:res:1))];

for i = 1:2:4
    obj.arm(i).wheels.coords = rot([0 0 1], (i-1)*pi/2) * arm.wheels1;
end
for i = 2:2:4
    obj.arm(i).wheels.coords = rot([0 0 1], (i-2)*pi/2) * arm.wheels2;
end

obj.verts_init = obj.vertices;

end
