function objsC = create_display_vars_car(objsC, lt)
% CREATE_DISPLAY_VARS(objsC, lt) - creates variables for display features
%    objsC - car objects which are to be displayed
%      lt  - trail length of object paths


cObjC = size(objsC,2);
for i=1:cObjC
    objsC(i).handle = plot3(objsC(i).z(1),objsC(i).z(2),objsC(i).z(3),'g--','LineWidth',2); % quad handle
    objsC(i).disp_st = [objsC(i).z(1)*ones(lt,1) objsC(i).z(2)*ones(lt,1) objsC(i).z(3)*ones(lt,1)]; % Trail of the quads
    objsC(i).ptr = 1;
    
end

end