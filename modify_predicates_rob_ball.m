function SP = modify_predicates_rob_ball( SP )

  
sz_pred = size(SP.pred,2);

% if sz_pred ~= SP.pred_known_sz
%   indxs = SP.pred_known_sz+1:sz_pred;
% else
  indxs = 1:sz_pred;
% end

for ii = indxs
  SP.pred(ii).b = bsxfun(@minus, SP.pred(ii).b, SP.rob_ball_des*SP.pred(ii).safe);
end

end

