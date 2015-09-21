function [objsC, time_disp] = update_car_paths(objsC, time_disp, count, Ts, lt)

cObjsC = size(objsC,2);

for i = 1:cObjsC
    objsC(i).disp_st(objsC(i).ptr,:)=objsC(i).z(1:3)';
end

% The trajectory data is stored in disp_st, but this is of
% fixed length, and the data keeps overwriting the data in a
% loop. "ptr" stores the index of the oldest recorded point,
% and essentially creates an ordered matrix by
% xd = [ disp_st(old_pt:end,1) disp_st(1:old_pt-1) ]
% and similarly for yd.

for i = 1:cObjsC
    xd = objsC(i).disp_st(modd(objsC(i).ptr-lt+1:objsC(i).ptr,lt),1);
    yd = objsC(i).disp_st(modd(objsC(i).ptr-lt+1:objsC(i).ptr,lt),2);
    zd = objsC(i).disp_st(modd(objsC(i).ptr-lt+1:objsC(i).ptr,lt),3);
    objsC(i).ptr = objsC(i).ptr+1;
    objsC(i).ptr = modd(objsC(i).ptr,lt);
    
    % Erase these plots on before replotting
%     set(objsC(i).handle, 'erasemode', 'normal');
    
    
    % Update the trajectory data
    set(objsC(i).handle,'XData', xd, 'YData', yd, 'ZData', zd);
    
end

% Update the time data
time_now = strcat('TIME = ',num2str(count*Ts), 'sec');
set(time_disp,'String',time_now);

update_plots(objsC);
% drawnow;

end
