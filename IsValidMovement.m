function status  = IsValidMovement(C_ini, C_end)

% this function is used to check if a linear motion from C_ini to C_end is
% feasible (including joint limit and collision constraints)

global params;

status = 1;

dstep     = params.distOneStep;
diff      = C_end - C_ini;
nrSteps   = floor( norm(diff) / dstep);
u         = dstep * diff / norm(diff);

for k = 1 : 1 : nrSteps
    params.robot = C_ini + k * u;
    if IsValidState() == 0
        status = 0;
        return;
    end
end