function controller = setup_MILP_controller_all_cons(SP) 
  % controller = setup_MILP_controller_all_cons(SP) encodes the MILP controller
  
  %%  define variables to be optimized
  u = sdpvar(repmat(SP.n_inputs,1, SP.hor_length + 1),ones(1,SP.hor_length + 1)); % control signal variable
  x = sdpvar(repmat(SP.n,1, SP.hor_length+2),ones(1,SP.hor_length+2)); % state space variable
  y = sdpvar(repmat(SP.n_outputs,1, SP.hor_length + 1),ones(1,SP.hor_length + 1)); % output variable
  w = sdpvar(repmat(SP.n_inputs,1, SP.hor_length + 1),ones(1,SP.hor_length + 1)); % disturbance signal variable
  
  con_active = sdpvar(SP.pred_num, SP.hor_length+1); % tracks which constraints to be set to active
  d = {}; % bigM variable required for placing constraints to avoid unsafe sets ---> see below in the constraints section
  M = 1000; % bigM value
  x_cur = sdpvar(SP.n,1);
  rem_hor_length = sdpvar(1,1);
%   hor_length = sdpvar(1,1);

  % account for unknown predicates -----------------------------------
  % ------------------------------------------------------------------
  for ii = 1:SP.pred_num % SP.pred_known_sz
    pred_A(ii).vals = sdpvar(size(SP.pred(ii).A,1), size(SP.pred(ii).A,2));
    pred_b(ii).vals = sdpvar(size(SP.pred(ii).b,1), size(SP.pred(ii).b,2));
  end
    
  %% initialize constraints and objective
  
  uVec = [u{:}];
  objective = sum(sum(abs(uVec))); % norm(uVec,1); % 
  
  constraints = []; 
  
  if isempty(SP.u0.input_values_human)
    SP.u0.input_values_human = zeros(size(u));
  end
  
  constraints = [constraints, x{1} == x_cur];
  
  for k = 1:SP.hor_length+1
    % add system dynamics as constraints
    constraints = [constraints, x{k+1} == SP.tf_disc.A*x{k}+SP.tf_disc.B*[u{k}; w{k}]];
    constraints = [constraints, y{k} == SP.tf_disc.C*x{k}+SP.tf_disc.D*[u{k}; w{k}]];
    
    % add bounds on control input signal as constraints
    constraints = [constraints, SP.u0.min(:,k) <= u{k} <= SP.u0.max(:,k)];
    
    if k > value(rem_hor_length)
      constraints = [constraints, u{k} == zeros(SP.n_inputs,1)];
    end
    
%     if k == 1 && value(rem_hor_length) == (SP.hor_length+1)
%       constraints = [constraints, u{k} ~= zeros(SP.n_inputs,1)];
%     end
    
  end
  
  %% encode all the constraints
  sz_d = 0;
  
  for ii = 1:SP.pred_num
    
    if SP.pred(ii).safe == -1 % predicates to be avoided
      
      for jj = 2:SP.hor_length+1
        
        sz_d = sz_d + 1;
        
        if ii > SP.pred_known_sz
          sz_pred = 4;
        else
          sz_pred = size(SP.pred(ii).b,1); % number of hyperplanes constructing the predicate
        end
        
        d{sz_d} = binvar(sz_pred,1); % update the variable holding the bigM variables
        
        % add constraint such that the state is outside of atleast one of the hyperplanes
        constraints = [constraints, sum(d{sz_d}(:,1))*con_active(ii,jj) <= (sz_pred-1)*con_active(ii,jj)];
        
        for kk = 1:sz_pred
          constraints = [constraints, (pred_A(ii).vals(kk,:)*y{jj} ...
            - pred_b(ii).vals(kk))*con_active(ii,jj) >= -M*d{sz_d}(kk,1)*con_active(ii,jj)];
        end
      end
      
    else % predicates to be visited
      
      for jj = 2:SP.hor_length+1
        constraints = [constraints, pred_A(ii).vals*y{jj}*con_active(ii,jj) <= pred_b(ii).vals*con_active(ii,jj)];
      end
      
    end
  end
  
  
  % define what the controller inputs are
    parameters_in = {x_cur, con_active, rem_hor_length, [w{:}], pred_A.vals, pred_b.vals};
    
  % define what the controller outputs are
  solutions_out = {[u{:}], [x{:}], [y{:}]};
  
  
  %% define settings
  
  % verbose = 2 ---> gives detailed iteration wise results ---> good for debugging
  % change solvername to use a different solver than gurobi
  % if no solver option is provided yalmip uses the default in-built one
  
  ops = sdpsettings('verbose', 1, 'solver', 'gurobi' );
  controller = optimizer(constraints, objective, ops,parameters_in,solutions_out);
  
  
end