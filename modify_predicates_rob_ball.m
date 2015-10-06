function SP = modify_predicates_rob_ball( SP )
% SP = MODIFY_PREDICATES_ROB_BALL( SP )
% shirnks/bloats predicates according to desired robustness radius

sz_pred = size(SP.pred,2);

indxs = 1:sz_pred;

for ii = indxs
    SP.pred(ii).b = bsxfun(@minus, SP.pred(ii).b, SP.rob_ball_des*SP.pred(ii).safe);
end

end

