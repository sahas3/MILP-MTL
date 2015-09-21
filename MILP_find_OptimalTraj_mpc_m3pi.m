function argvout = MILP_find_OptimalTraj_mpc_m3pi(SP, plot_flag, hfig)
% argvout = MILP_FIND_OPTIMALTRAJ_MPC_M3PI(SP, plot_flag, hfig) implements the
% receding horizon controller for finding the optimal system trajectory

%% Initialize variables

tIndxs = 1:SP.pred_hor+1;
x_cur = SP.x0;
SP.u0.values = zeros(SP.n_inputs, SP.hor_length + 1);
uvals = zeros(SP.n_inputs, SP.hor_length + 1);
yvals = zeros(SP.n_outputs, SP.hor_length + 1);
solutions{3} = zeros(SP.n_outputs, SP.input_length);
SP.yout = zeros(SP.n_outputs, SP.input_length);

SP.sol_count_vals = zeros;

SP.states_system_prev = SP.states_system;
SP.states_system_estimate = SP.states_system;

unknown_pred_count = 0;

%% Create unicycle agents for plotting and other plot variables initialization

if plot_flag
  
  figure(hfig);
  car_sz = 0.05;
  car = create_and_draw_car([SP.states_system(1:2);0], eye(3), 0 , 1, car_sz);
  time_disp=text(0,0.5,0,'Time = 0.000 sec','Color','b', 'FontSize', 20);
       
  trail_length = 10;
  car = create_display_vars_car(car, trail_length);
  
  SP.predicted_traj_handle = plot(SP.states_system(1), SP.states_system(2), ...
    'b', 'LineWidth', 2);
  
  if ~isempty(SP.movieName)
    f = getframe;
    [im,map] = rgb2ind(f.cdata,256,'nodither');
  end
end


%% RUN MPC OPTIMIZATION

sw = tic;

for ii = 1:SP.hor_length+1
  
  iter_time = tic;
  
  sol_count = 1;
  SP.tIndxs = tIndxs;
    
  SP.states_system_prev = SP.states_system;
  
  SP.yout(:,ii) = x_cur(1:SP.n_outputs);
  
  
  %=============================================================================
  % get new predicate online
  %=============================================================================
  if (strcmpi(get(hfig,'currentch'),'b')) && unknown_pred_count < SP.pred_unknown_sz
    
    unknown_pred_count = unknown_pred_count + 1;
    
    % plot the new predicate
    Z = sdpvar(SP.n_outputs,1);
    plot(SP.pred(SP.pred_known_sz+unknown_pred_count).A*Z <= SP.pred...
      (SP.pred_known_sz+unknown_pred_count).b, [], [1 0 0]);
    fill(SP.pred(SP.pred_known_sz+unknown_pred_count).values(:,1), ...
      SP.pred(SP.pred_known_sz+unknown_pred_count).values(:,2) , [0.5 0 0]);
    
    set(hfig,'currentch','p'); % set the character value to some different key
    
    % update MTL specification
    SP.phi = strcat(SP.phi, ' /\ ', SP.phi_global_unknown(unknown_pred_count).str);    
  end
  
  SP.dmin_con = -Inf;
    
  %=============================================================================
  % Solve for the new trajectory of the system
  %=============================================================================
  
  SP.tInd_con = 0;
  SP.crit_pred = 0;
  SP.constraints_tInd = zeros;
  SP.crit_predVal = zeros;
  
  
  SP.break_loop = 0;
  
  prev_tInd_ind = [];
  
  
  while((isempty(prev_tInd_ind) || (~isempty(prev_tInd_ind) && isempty(find...
      (SP.crit_predVal(prev_tInd_ind) == SP.crit_pred, 1))))  && ~SP.break_loop && toc(iter_time) < 0.5*SP.ds)
    
    SP.constraints_tInd(sol_count) = SP.tInd_con;
    SP.crit_predVal(sol_count) = SP.crit_pred;
    
    [solutions,diagnostics] = SP.controller{{x_cur(:) , SP.con_active , ...
      (SP.hor_length + 1 - (ii-1)), SP.u0.dist_input_values, SP.pred.A, SP.pred.b}};
    
    
    if diagnostics == 1 || diagnostics == 12 || diagnostics == 15
      error('The problem is infeasible');
    end
    
    SP.u0.values = solutions{1};
    SP.xout_disc = solutions{2}';
    SP.yout(:,ii+1:end) = solutions{3}(:,2:end-(ii-1));
    
    
    %===========================================================================
    % Compute Robustness and identify the critical constraint
    %===========================================================================
    
    
    [~, SP.tmin_con, SP.dmin_con, ~, SP.crit_pred] = staliro_distance(SP, ...
      SP.yout', SP.times');
    
    
    if SP.tmin_con > ((ii-1)*SP.ds) && abs(SP.dmin_con) ...
        > 1e-5 && sign(SP.dmin_con) == -1
      
      % add constraints corresponding to the critical predicates
      SP.tInd_con = round(SP.tmin_con/SP.ds)+1 - (ii-1);
      SP.con_active(SP.crit_pred, SP.tInd_con) = 1;
      prev_tInd_ind = find(SP.constraints_tInd == SP.tInd_con);
      
    else
      
      SP.break_loop = 1;
      
    end
    
    sol_count = sol_count + 1;
    
  end
  
  SP.sol_count_vals(ii) = sol_count-1;
  uvals(:,ii) = solutions{1}(:,1);
  yvals(:,ii) = solutions{3}(:,1);
  
  
  if plot_flag
    
    figure(hfig);
    car.z = [x_cur(1:2);0];
    car.th = atan2(x_cur(4), x_cur(3));
    [car, time_disp] = update_car_paths(car, time_disp, ii-1, SP.ds, trail_length);
    
    set(SP.predicted_traj_handle, 'XData', solutions{3}(1,1:size(tIndxs,2)), ...
      'YData', solutions{3}(2,1:size(tIndxs,2)));
    
    drawnow;
    
    if ~isempty(SP.movieName)
      f=getframe;
      im(:,:,1,ii) = rgb2ind(f.cdata,map,'nodither');
    end
  end
    
  tIndxs(1) = [];
  
  x_cur = SP.xout_disc(SP.ctrl_hor+1,:)'; % update x0 for next iteration
  
  SP.con_active = [zeros(size(SP.pred_orig,2),1) SP.con_active(:,3:end) ...
    zeros(size(SP.pred_orig,2),1)];
  
  pause(SP.ds - toc(iter_time))
  
end


sprintf('Average time taken to solve the optimization problem is %d in %d solutions', ...
  toc(sw)/sum(SP.sol_count_vals), sum(SP.sol_count_vals))

if ~isempty(SP.movieName)
  imwrite(im,map, strcat(SP.movieName,'.gif'),'DelayTime', SP.ds,'LoopCount',inf);
  argvout.im = im;
  argvout.map = map;
end

%% Output results
SP.u0.values = uvals;
SP.yout = yvals;
argvout.SP = SP;
argvout.input = SP.u0;


end


