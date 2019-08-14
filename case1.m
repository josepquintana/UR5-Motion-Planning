function Dm = case1 ( A, B, C, D )
% 求解线段AB和CD在共线CASE下的最短距离
%%%%%%%%%%%%%%%%%%%%% (输入变量解释)
% A, B, C, D 分别为两条线段的四个顶点坐标 （1*3 维，单位：mm）
%%%%%%%%%%%%%%%%%%%%% (输入变量解释)
% Dm 为两条线段的最短距离 （ 标量，单位：mm ）

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 求解AC、BC、AD、BD、AB、CD向量的长度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AC = C - A; 
BC = C - B;
AD = D - A;
BD = D - B;
AB = B - A;
CD = D - C;

ac = norm(AC);
bc = norm(BC);
ad = norm(AD);
bd = norm(BD);
ab = norm(AB);
cd = norm(CD);

%%%%%%%%%%%%%%%%%%%%%%%
% 判断AB、CD是否相交
%%%%%%%%%%%%%%%%%%%%%%%
if bc + bd == cd | ac + ad ==cd | ac + bc == ab | ad + bd == ab % 判断AB、CD是否有重合的部分，即是否相交
    Dm = 0;
else                                                               % 若不相交则AB、CD最短距离为ac,ad,bc,bd中的最小值
    Dm = min( [ac,ad,bc,bd] );
end
end
    


    
