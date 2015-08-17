function [Trans, TREE] = getTransMatrix( robotState )
%GETTRANSMATRIX この関数の概要をここに記述
%   詳細説明をここに記述

[ th1, th2, th3, th4, th5, th6, xb, yb, thb, lb, hb ] = pickUpData( robotState );
d0 = 0.1544*1000;
d1 = -0.1181*1000;
d2 = 0.2900*1000;
d3_offset = -0.0070*1000;
d3 = 0.1233*1000;
d4 =0.0741*1000;
d5 = 0.0741*1000;
d6 = 0.1600*1000;
j5_bend =  degtorad(-55);
j6_bend = degtorad(55);

TREE = eye(4);
Trans(:, :, 1) = [ cos(thb), -sin(thb), 0, xb + lb*cos(thb);
    sin(thb),  cos(thb), 0, yb + lb*sin(thb);
    0,         0, 1,                hb;
    0,         0, 0,                1];

%arm_base to j1
Trans(:, :, 1) = Trans(:, :, 1) * [ cos(th1), -sin(th1), 0, 0;
    -sin(th1),  -cos(th1), 0,       0;
    0,         0, -1,     d0;
    0,         0, 0,       1];

%j1 to j2
Trans(:, :, 2) =[ sin(th2),  cos(th2), 0, 0;
    0,         0, 1, 0;
    cos(th2), -sin(th2), 0, d1;
    0,         0, 0, 1];

% j2 to j3
Trans(:, :, 3)=[ -cos(th3), sin(th3), 0, d2;
    sin(th3),  cos(th3), 0,  0;
    0,         0, -1,  0;
    0,         0, 0,  1];

%j3 to j3.5
Trans(:, :, 4)= [ 1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d3_offset;
    0, 0, 0, 1];

% j3.5 to j4
Trans(:, :, 4) = Trans(:, :, 4) * [0, 0, -1, d3
    sin(th4), cos(th4), 0, 0
    cos(th4), -sin(th4), 0, 0;
    0, 0, 0, 1];

%j4 to j5
Trans(:, :, 5)= [ cos(j5_bend)*cos(th5), cos(j5_bend)*-sin(th5), sin(j5_bend), cos(-j5_bend)*d4;
    sin(th5), cos(th5), 0, 0;
    -sin(j5_bend)*cos(th5), sin(j5_bend)*sin(th5), cos(j5_bend), -sin(-j5_bend)*d4;
    0, 0, 0, 1];

%j5 to j6
Trans(:, :, 6)= [cos(j6_bend) * cos(th6), cos(j6_bend) * -sin(th6), sin(j6_bend), -cos(j6_bend)*d5;
    sin(th6), cos(th6), 0, 0;
    -sin(j6_bend) * cos(th6), sin(j6_bend) * sin(th6), cos(j6_bend), -sin(j6_bend) * d5;
    0, 0, 0, 1];

% j6 to ee
Trans(:, :, 7)= [ -1, 0, 0, 0;
    0, 1, 0,  0;
    0, 0, -1,  -d6;
    0, 0, 0,  1];

for i=1:size(Trans, 3)
    TREE = TREE*Trans(:, :, i);
end

end

function [th1, th2, th3, th4, th5, th6, xb, yb, thb, lb, hb ] = pickUpData( robotState )

[ th1, th2, th3, th4, th5, th6, xb, yb, thb ] = Utility.convertRobotState2Params( robotState );

ic = getInitConfig();
lb = ic.lb; 
hb = ic.Heightb; 


end







