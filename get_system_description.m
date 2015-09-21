function SP = get_system_description()

  SP.u0.max = 0.25*[1 1]'; 
  SP.u0.min = -0.25*[1 1]'; 
  
  SP.A = [0 0 1 0;
    0 0 0 1;
    0 0 0 0;
    0 0 0 0];
  
  SP.Bu = [0 0 1 0;0 0 0 1]';
  SP.Bw = zeros(size(SP.Bu));
  SP.B = [SP.Bu SP.Bw];
  
  SP.C = [1 0 0 0;
    0 1 0 0];
  
  SP.Du = [0 0; 0 0]';
  SP.Dw = zeros(size(SP.Du));
  SP.D = [SP.Du SP.Dw];
  
  SP.u0.A = SP.A;
  SP.u0.B = SP.B;
  %       SP.u0.Bu = SP.Bu;
  %       SP.u0.Bw = SP.Bw;
  SP.u0.C = SP.C;
  SP.u0.D = SP.D;
  %       SP.u0.Du = SP.Du;
  %       SP.u0.Dw = SP.Dw;
  
  SP.u0.input_values_human = [];
  
  SP.n = size(SP.A,1);
  SP.n_inputs = size(SP.Bu,2);
  SP.n_outputs = size(SP.C,1);  
  
  SP.x0 = zeros(SP.n,1);
  
  SP.observer_gains = 0.1*eye(3);
  
end