function Dm = Dmin ( A, B, C, D )
% 求解线段AB和CD之间的最短距离
%%%%%%%%%%%%%%%%%%%%% (输入变量解释)
% A, B, C, D 分别为两条线段的四个顶点坐标 （1*3 维，单位：mm）
%%%%%%%%%%%%%%%%%%%%% (输入变量解释)
% Dm 为两条线段的最短距离 （ 标量，单位：mm ）

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 求解AC、BC、AD、BD单位向量
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AC = C - A; 
BC = C - B;
AD = D - A;
BD = D - B;

AC = AC / norm(AC);
BC = BC / norm(BC);
AD = AD / norm(AD);
BD = BD / norm(BD);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 分情况讨论
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if norm(AC - BC) <  1.0e-10 & norm(AD - BD) <  1.0e-10               % 线段AB、CD共线
    Dm = case1( A, B, C, D );           
elseif norm(AC - BC) < 1.0e-10 | norm(AD - BD) <  1.0e-10            % 由于四个点中有三个点共线而导致线段AB、CD共面
        Dm = case2( A, B, C, D );
else 
    if abs( abs( dot( cross( AC, BC ) / norm( cross( AC, BC ) ), cross( AD, BD ) / norm( cross( AD, BD ) ) ) ) - 1 ) < 1.0e-10
                                      % 线段AB、CD共面
        Dm = case2( A, B, C, D );
    else                              % 线段AB、CD异面
        Dm = case3( A, B, C, D );
    end
end
end
    
