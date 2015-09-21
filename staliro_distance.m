function [nearest_point_on_s, tmin, dmin, umin, i_pr, inSet] = staliro_distance(SP,xout,tout)
[rob, aux] = dp_taliro(SP.phi,SP.pred,xout,tout);
dmin = rob;
if aux.i==0
    aux.i=1;
end
tmin = tout(aux.i);
nearest_point_on_s = xout(aux.i,:)';
i_pr = aux.pred;%get_predicate_index(aux.predicate,SP.pred);
% disp(['i_pr = ',num2str(i_pr)])
[~,inSet,umin] = SignedDist(nearest_point_on_s,SP.pred(i_pr).A,SP.pred(i_pr).b);
% umin is the projection of the critical point of the trajectory on the critical predicate of the MTL specification 
end