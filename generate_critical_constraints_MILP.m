function argvout = generate_critical_constraints_MILP(SP)
% argvout = GENERATE_CRITICAL_CONSTRAINTS_MILP(SP) identifies all the critical
% constraints that need to be activated for the known environment in an
% open-loop fashion and encodes the MILP controller

yalmip('clear')

%% Discretize the system
SP.ss_cont= ss(SP.A, SP.B, SP.C, SP.D); % continuous dynamics
SP.tf_disc = c2d(SP.ss_cont, SP.ds); % discrete dynamics

%% Setup the controller and the optimization process

SP.controller = setup_MILP_controller_all_cons(SP);

x_cur = SP.x0;
SP.u0.values = zeros(SP.n_inputs, SP.hor_length + 1); % optimized control signal values to be stored here

%% Initialize other variables
SP.t0 = 0; % initial time
SP.tInd_con = 0; % critical time index
SP.crit_pred = 0; % critical predicate
SP.phi = SP.phi_orig; % MTL specification

SP.tmin_con = -1; % critical time
SP.dmin_con = -Inf; % robustness radius

SP.constraints_tInd = zeros; % stores critical time indices values
SP.crit_predVal = zeros; % stores corresponding critical predicates

prev_tInd_ind = []; % tracks if a critical predicate at a certain time was added already ---> avoids looping 
sol_count = 1; % keeps track of how many iterations were required to solve the problem


% set all constraints to be inactive ---> they are made active as required
SP.con_active = zeros(SP.pred_num, SP.hor_length+1);

%% Start the main open-loop optimization procedure

while((isempty(prev_tInd_ind) || (~isempty(prev_tInd_ind) && isempty(find...
    (SP.crit_predVal(prev_tInd_ind) == SP.crit_pred, 1))))  && ...
    (SP.dmin_con < 0 && SP.tmin_con ~= 0))
  
  
  SP.constraints_tInd(sol_count) = SP.tInd_con;
  SP.crit_predVal(sol_count) = SP.crit_pred;
  
  
  %===========================================================================
  % Compute Robustness and identify the critical constraint
  %===========================================================================
  
  % solve the optimization problem
  [solutions,diagnostics] = SP.controller{{x_cur(:) , SP.con_active , ...
    SP.hor_length+1, SP.u0.dist_input_values, SP.pred.A, SP.pred.b}};
  
  
  % proceed if feasible
  if diagnostics == 1 || diagnostics == 12 || diagnostics == 15
    error('The problem is infeasible');
  end
  SP.u0.values = solutions{1};
  SP.yout_disc = solutions{3}';
  
  %===========================================================================
  % Compute Robustness and identify the critical constraint
  %===========================================================================
  
  % compute critical time, predicate and robustness
  [~, SP.tmin_con, SP.dmin_con, ~, SP.crit_pred] = staliro_distance(SP, ...
                                                  SP.yout_disc, SP.times');
  
  
  % add constraints corresponding to the critical predicates
  SP.tInd_con = round(SP.tmin_con/SP.ds)+1;
  SP.con_active(SP.crit_pred, SP.tInd_con) = 1;
  prev_tInd_ind = find(SP.constraints_tInd == SP.tInd_con);
  
  sol_count = sol_count + 1;
  
end

%% Output results

argvout = SP;

end


