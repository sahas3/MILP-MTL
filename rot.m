function R=rot(k,theta)
  % R=ROT(k,theta)
  % rotation matrix along vector 'k' by angle 'theta'
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
 