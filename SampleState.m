function s  = SampleState()

% this function is used to generate a random configuration complying with
% the joint limits, which can be set in the function, ParaInitialize.

  global params;
  
  s = zeros(1,6);
  
  s(1) = RandomReal(params.Q1min, params.Q1max);
  s(2) = RandomReal(params.Q2min, params.Q2max);
  s(3) = RandomReal(params.Q3min, params.Q3max);
  s(4) = RandomReal(params.Q4min, params.Q4max);
  s(5) = RandomReal(params.Q5min, params.Q5max);
  s(6) = RandomReal(params.Q6min, params.Q6max);

end

