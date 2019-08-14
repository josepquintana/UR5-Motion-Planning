function status  = HasRobotReachedGoal()

% this function is used to check if the goal configuration is reached
% within a tolerance range, which is defined by params.goaltolerance in
% ParaInitialize

  global params;
  status = norm( params.robot - params.goal ) <= params.goaltolerance;
end

