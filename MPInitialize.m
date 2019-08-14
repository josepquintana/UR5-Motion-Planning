function [] = MPInitialize(C_ini)

% this function is used to initialize the struct mp

global mp;


mp.nodes       = [C_ini];
mp.parents     = [-1];
mp.nchildren   = [0];
mp.vidAtGoal   = -1;
mp.sto         = [Inf Inf Inf Inf Inf Inf];
mp.vidNear     = [Inf Inf Inf Inf Inf Inf];
end

