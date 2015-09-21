function update_plots(objsC)
% UPDATE_PLOTS(objsC) - updates based on the current state z
%  objsC - car objects


cObjsC = size(objsC,2);


for j = 1:cObjsC
    
    pos_car = objsC(j).z(1:3,:);
    R_car = rot([0 0 1], objsC(j).th);
    
    for i = 1:4
        coords = trans(objsC(j).arm(i).wheels.coords, R_car, pos_car);
        set(objsC(j).arm(i).wheels.plot, 'XData', coords(1,:), ...
            'YData', coords(2,:), ...
            'ZData', coords(3,:));
        
    end
    
    objsC(j).vertices = pos_car(:,ones(1,length(objsC(j).vertices))) + R_car*objsC(j).verts_init;

    set(objsC(j).Handle,'Vertices',objsC(j).vertices');
    
end