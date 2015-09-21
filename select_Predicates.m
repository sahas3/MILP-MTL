function SP = select_Predicates(SP, userInput)
% SELECT_PREDICATES returns predicates in polyhedral form Input is SP, userInput
% Output is SP with the field SP.pred

% this SP.pred are in the form of (AX <= b) and are required in computing
% the signedDistance, called in staliro_distance file.

% predicates are defined in terms of (x,y) position of the mobile robot in the
% 2D plane


% Example set representation represented by
% 1 <= y <= 10, 7 <= x <= 9.
% y <= 10 ---> A(1,2) = 1, b(1) = 1
% 1 <= y ---> -y <= -1 ---> A(3,2) = -1, b(3) = -1
% 7 <= x ---> -x <= -7 ---> A(2,1) = -1, b(2) = -7
% x <= 9 ---> A(4,1) = 1, b(4) = 9
% the above process can be automated with the the PTS2CVXPOLY function

% SP.safe ---> indicates if a negation is before the predicate ---> used in
% modify_SP.pred_rob_ball.m


str_name = ('a':'z').';


if ~strcmp(userInput, 'yes')
    
    
    
    % create new figure
    hfig = figure;
    hold on;
    axis([-0.8 0.8 -0.6 0.6]);
    
    pos_init = [650 -300]/1000;
    
    SP.pred_known_sz = 2; % We know Goal and one Obstacle from the begining
    % of the path planning process
    SP.pred_unknown_sz = 1; % We assume we might encounter one more
    % Obstacle in future
    
    
    SP.pred_hor = 40;
    SP.tf = SP.pred_hor*SP.ds;
    
    % Goal Set
    SP.pred(1).values = [-294 544; -292 189; -557 201; -563 550]/1000;
    SP.pred(1).safe = 1;
    
    % Known Obstacle
    SP.pred(2).values = [299 -46; 309 -394; -6 -391; 3 -50]/1000;
    SP.pred(2).safe = -1;
    
    % UnKnown Obstacle
    SP.pred(3).values = bsxfun(@plus, 0.5*SP.pred(2).values, [-50 250]/1000);
    SP.pred(3).safe = -1;
    
    SP.phi_not_global = strcat('[]_[',num2str((SP.pred_hor-5)*SP.ds), ...
        ',', num2str(SP.tf),']a');
    SP.phi_global = '[]!b';
    SP.phi_global_unknown(1).str = '[]!c';
    
    SP.safe_num_known = 1;
    SP.unsafe_num_known = 1;
    
else
    
    
    
    %% Get information about number and type of predicates
    
    SP.safe_num_known = input('\n\n How many known safe sets do you want to choose? ');
    timeStamps = input('\n\n Enter the time-points during which you want to be in the \n safe sets in ascending manner such that the first value \n is when you enter the set and the second value is when you leave the set. ');
    timeStamps = timeStamps(:)*SP.ds;
    temp = sort(timeStamps, 'ascend');
    
    if size(timeStamps,1) ~= SP.safe_num_known*2
        error('Size of the timeStamps variable should be twice that of the number of safe sets you want to visit.');
    end
    
    if temp ~= timeStamps
        error('Enter the time-points in ascending manner.');
    end
    
    SP.unsafe_num_known = input('\n\n How many known unsafe sets do you want to choose? ');
    SP.pred_unknown_sz = input('\n\n How many unknown unsafe sets do you want to choose? ');
    
    
    % create new figure
    hfig = figure;
    hold on;
    axis([-0.8 0.8 -0.6 0.6]);
    
    %% Select Initial Position
    
    
    h_title = title('Choose the initial position of the mobile robot.');
    %   sprintf('Choose the initial position of the mobile robot. ')
    pos_init = ginput(1);
    
    SP.phi_not_global = '';
    
    
    %% Goal Sets
    for jj = 1:SP.safe_num_known
        
        if jj==1
            h_title.String = 'Select the First Safe Set.';
            %             sprintf('Select the First Safe Set.')
        else
            h_title.String = 'Select the Next Safe Set.';
            %             sprintf('Select the Next Safe Set.')
        end
        
        [xpos, ypos] = ginputc('FigHandle', hfig, 'PointColor',  [0 0.5 0], ...
            'Color',  [0 0.5 0], 'LineWidth', 2, 'ShowPoints', true, ...
            'ConnectPoints', true);
        
        SP.pred(jj).values = [xpos ypos];
        
        SP.pred(jj).safe = 1;
        fill(SP.pred(jj).values(:,1), SP.pred(jj).values(:,2) , [0 0.5 0]);
        
        SP.phi_not_global = strcat(SP.phi_not_global, '[]_[',num2str...
            (timeStamps(2*jj-1)), ',', num2str(timeStamps(2*jj)),']', str_name(jj));
        
        if jj ~= SP.safe_num_known
            SP.phi_not_global = strcat(SP.phi_not_global, ' /\ ');
        end
        SP.pred(jj).safe = 1;
    end
    
    SP.tf = timeStamps(end);
    SP.pred_hor = SP.tf/SP.ds;
    SP.phi_global = '';
    
    
    %% Unsafe Sets
    
    
    for jj = (SP.safe_num_known+1):(SP.safe_num_known + SP.unsafe_num_known)
        
        if jj==SP.safe_num_known+1
            h_title.String = 'Select the first known Unsafe Set.';
            %             sprintf('Select the first known Unsafe Set.')
        else
            h_title.String = 'Select the next known Unsafe Set.';
            %             sprintf('Select the next known unsafe Set.')
        end
        
        [xpos, ypos] = ginputc('FigHandle', hfig, 'PointColor', [0.5 0 0], ...
            'Color', [0.5 0 0], 'LineWidth', 2, 'ShowPoints', true, ...
            'ConnectPoints', true);
        SP.pred(jj).values = [xpos ypos];
        SP.pred(jj).safe = -1;
        fill(SP.pred(jj).values(:,1), SP.pred(jj).values(:,2) ,  [0.5 0 0]);
        
        SP.phi_global = strcat(SP.phi_global, '[]!', str_name(jj));
        if jj ~= (SP.safe_num_known + SP.unsafe_num_known)
            SP.phi_global = strcat(SP.phi_global, ' /\ ');
        end
        SP.pred(jj).safe = -1;
        
    end
    
    SP.pred_known_sz =  jj;
    
    
    %% Future Unsafe Sets
    
    
    for jj = (SP.pred_known_sz+1):(SP.pred_known_sz+SP.pred_unknown_sz)
        
        
        if jj==SP.pred_known_sz+1
            h_title.String = 'Select the first unknown Unsafe Set.';
            %             sprintf('Select the first unknown Unsafe Set.')
        else
            h_title.String = 'Select the next unknown Unsafe Set.';
            %             sprintf('Select the next unknown unsafe Set.')
        end
        
        [xpos, ypos] = ginputc('FigHandle', hfig, 'PointColor', [0.75 0 0], ...
            'Color', [0.75 0 0], 'LineWidth', 2, 'ShowPoints', true, ...
            'ConnectPoints', true);
        
        SP.pred(jj).values = [xpos ypos];
        SP.pred(jj).safe = -1;
        fill(SP.pred(jj).values(:,1), SP.pred(jj).values(:,2) ,  [0.75 0 0]);
        
        SP.phi_global_unknown(jj-SP.pred_known_sz).str = strcat('[]!', ...
            str_name(jj));
        
    end
    
end

for ii = 1:(SP.pred_known_sz+SP.pred_unknown_sz)
    
    SP.pred(ii).str = str_name(ii);
    [SP.pred(ii).A, SP.pred(ii).b] = pts2cvxpoly(SP.pred(ii).values);
    
end

SP.phi_orig = [SP.phi_not_global '/\' SP.phi_global];
SP.x0(1:2) = pos_init; % SP.x0 = [x-pos y-pos x-dot y-dot]'
SP.hfig = hfig;


end