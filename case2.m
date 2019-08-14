function Dm = case2 ( A, B, C, D )
% 求解线段AB和CD在共面CASE下的最短距离
%%%%%%%%%%%%%%%%%%%%% (输入变量解释)
% A, B, C, D 分别为两条线段的四个顶点坐标 （1*3 维，单位：mm）
%%%%%%%%%%%%%%%%%%%%% (输入变量解释)
% Dm 为两条线段的最短距离 （ 标量，单位：mm ）

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 求解AB、CD单位向量
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AB = B - A;
CD = D - C;

ab = norm(AB);
cd = norm(CD);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 分情况讨论
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if abs( abs( dot( AB / ab , CD / cd ) ) - 1 ) < 1.0e-10           % 线段AB、CD平行
    a = minp2l( A, C, D );
    b = minp2l( B, C, D );
    c = minp2l( C, A, B );
    d = minp2l( D, A, B );
    Dm = min( [a,b,c,d] );
else                                                           % 线段AB、CD不平行
    R_v1 = AB' / ab ;                              
    R_v3 = cross( AB',CD' ) / norm( cross( AB',CD' ) );
    R_v2 = cross( R_v3,R_v1 );
    R = [R_v1,R_v2,R_v3];                         % 新坐标系相对于基座标系的旋转矩阵
    AA = R'*(A'-A');                              % 进行坐标变换
    BB = R'*(B'-A');
    CC = R'*(C'-A');
    DD = R'*(D'-A');
    alpha = ( DD - CC ) / cd;                     % 判断AB、CD是否相交的准备工作 
    j = -CC(2)/alpha(2);
    i = CC(1) - ( alpha(1) / alpha(2) ) * CC(2);
    if (i >= 0) && (i <= ab) && (j >= 0) && (j <= cd)     % 线段AB、CD相交
        Dm = 0;
    else                                          % 线段AB、CD不相交
        a = minp2l( A, C, D );
        b = minp2l( B, C, D );
        c = minp2l( C, A, B );
        d = minp2l( D, A, B );
        Dm = min( [a,b,c,d] );
    end
end
end

    


    
