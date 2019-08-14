function P_smooth = SmoothPath(P) 

% this function is used to perform post-processing on the solution path P
% to make it shorter and smoother

% you do not have to modify this file

global params;


P = P';

[n,m] = size(P);
clearvars n;
l = zeros(m,1);
for k=2:m
    l(k)=norm(P(:,k)-P(:,k-1)) + l(k-1); 
end
iter = 1;
while iter <= params.smoothiters
    s1 = rand(1,1)*l(m); 
    s2 = rand(1,1)*l(m); 
    if s2 < s1
        temps = s1;
        s1 = s2;
        s2 = temps;
    end
    for k=2:m
        if s1 < l(k)
            i = k - 1;
            break;
        end
    end
    for k=(i+1):m
        if s2 < l(k)
            j = k;
            break;
        end
    end
    if (j <= i)
        iter = iter + 1;
        continue;
    end
%     t1 = (s1 - l(i))/(l(i+1)-l(i));
%     gamma1 = (1 - t1)*P(:,i) + t1*P(:,i+1);  
%     t2 = (s2 - l(j))/(l(j+1)-l(j));
%     gamma2 = (1 - t2)*P(:,j) + t2*P(:,j+1); 
    status  = IsValidMovement(P(:,i)', P(:,j)');
    if status == 0
        iter = iter + 1;
        continue;
    end
    newP = [P(:,1:i) P(:,j:m)];
    clearvars P;
    P = newP;    
    [n,m] = size(P);
    clearvars n;
    l = zeros(m,1);
    for k=2:m
        l(k)=norm(P(:,k)-P(:,k-1)) + l(k-1);
    end
    iter = iter + 1;
end

P_smooth = P';

end