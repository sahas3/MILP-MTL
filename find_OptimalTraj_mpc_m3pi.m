function argvout = find_OptimalTraj_mpc_m3pi()
% argvout = FIND_OPTIMALTRAJ_MPC_M3PI() sets up the problem of finding the 
% optimal trajectory for a m3pi robot with unicycle dynamics, by minimizing 
% the 1-norm of the control input over the prediction horizon using a 
% receding horizon framework such that the resultant trajectory satisfies a 
% given Metric Temporal Logic (MTL) specification with a desired robustness.

dbstop if error;
close all;

userInput = input('Type "yes" to run the user-interactive example. ', 's');
movieName = input('Type in the file name if you want to make a movie in .gif format. ', 's');

%% Set Variable Parameters

% set what is to be optimized
obj_u = 1; % when set minimizes 1-norm of control signal

ctrl_horizon = 1; % control horizon for MPC

rob_ball_des = 0.05; % desired robustness radius

plot_flag = 1; % when set plots the position of the m3pi robot in each iteration

ds = 0.5; % time-step for running simulation

%% Describe system

SP = get_system_description();
SP.t0 = 0; % initial time
SP.ds = ds; % time-step

SP.pred_known_sz = []; % number of known predicates
SP.pred_unknown_sz = []; % number of unknown predicates

SP.pred_unknown_sz_orig = SP.pred_unknown_sz;
SP.unknown_pred_flag = 0;

SP = select_Predicates(SP, userInput);
hfig = SP.hfig; % figure handle

if isempty(SP.pred_unknown_sz)
    SP.pred_num = SP.pred_known_sz;
else
    SP.pred_num = SP.pred_known_sz + SP.pred_unknown_sz; % total number of predicates
end

SP.rad = 0.067/2; % size of the m3pi robot


%% -------------------------- System Parameters --------------------------------


SP.times = SP.t0:SP.ds:SP.tf; % time-points 
SP.u0.times = SP.times;

SP.input_length = size(SP.times,2);

SP.return_flag = 0;

SP.u0.dist_input_values = SP.times*0.; % disturbance input signal
SP.u0.dist_input_values = SP.u0.dist_input_values(ones(SP.n_outputs,1),:);
SP.u0.dist_input_values = 0.0*rand(size(SP.u0.dist_input_values));


SP.u0.input_values_human = zeros(SP.n_inputs, SP.input_length); %ignore this variable ---> needed for some other extensions of this work 

% Initialize state variables

SP.states_system = [SP.x0(1:2); 0]; % [x-pos; y-pos; theta]
SP.controller_input_system = [0 0]'; % control input to the nonlinear 
                                                            % system dynamics


%% ------------------------- Optimization Parameters ---------------------------

SP.rob_ball_des = rob_ball_des;
SP.obj_u = obj_u;
SP.ctrl_hor = ctrl_horizon;
SP.hor_length = SP.pred_hor; % simulation horizon
SP.movieName = movieName;

SP.u0.min = SP.u0.min(:,ones(1,SP.hor_length+1)); % minimum control input ---> set in system description code
SP.u0.max = SP.u0.max(:,ones(1,SP.hor_length+1)); % maximum control input

%% ------------------------------ PLOTTING -------------------------------------

SP.pred_actual = SP.pred; % store the actual predicate size
SP = modify_predicates_rob_ball( SP ); % shrink/bloat predicates depending on the robustness radius
SP.pred_orig = SP.pred; 

% Plot the predicates
delete(hfig);

hfig = figure;
hold on;
axis([-0.8 0.8 -0.6 0.6]);


Z = sdpvar(SP.n_outputs,1);
for ii = 1:SP.pred_known_sz
    
    if ii <= SP.safe_num_known
        fill(SP.pred(ii).values(:,1), SP.pred(ii).values(:,2) , [0 0.5 0]);
    end
    
    if SP.pred(ii).safe == -1
        colorval = [1 0 0];
    else
        colorval = [0 1 0];
    end
    plot(SP.pred(ii).A*Z <= SP.pred(ii).b, [], colorval);
    
    if ii > SP.safe_num_known
        fill(SP.pred(ii).values(:,1), SP.pred(ii).values(:,2) , [0.5 0 0]);
    end
end

pause(0.01);

%% -------------------------- RUN OPTIMIZATION ---------------------------------

sw = tic;
sprintf('Encoding the controller ... ')
argvout_temp = generate_critical_constraints_MILP(SP);

sprintf('Time taken to encode controller is : %d', toc(sw))

sprintf('Hit Enter to run the optimization.')
pause();

sprintf('Running the receding horizon controller now. Press "b" to put new unsafe sets in the environment.')
time_exec = tic;
argvout = MILP_find_OptimalTraj_mpc_m3pi(argvout_temp, plot_flag, hfig);

sprintf('Total Time taken to solve: %d', toc(time_exec))

uvals = argvout.input.values;
yvals = argvout.SP.yout;



%% ---------------------- SIMULATE THE FINAL RESULTING TRAJECTORY --------------
SP = argvout.SP;
SP.u0.values = uvals;
SP.pred = SP.pred_actual;
SP.yout = yvals';


[~, SP.tmin_con, SP.dmin_con] = staliro_distance(SP, SP.yout, SP.times');

figure(hfig);
plot(SP.yout(:,1), SP.yout(:,2), 'k-');
plot(SP.yout(:,1), SP.yout(:,2), 'mo');


sprintf('Minimum Robustness of %d is achieved at %d seconds', SP.dmin_con, ...
  SP.tmin_con)
sprintf('Objective function value is %d', sum(sum(abs(SP.u0.values))))


argvout.SP.u0.values = SP.u0.values;
argvout.SP.yout = SP.yout;
argvout.SP.dmin_con = SP.dmin_con;
argvout.SP.tmin_con = SP.tmin_con;

end
