function [Trans, TREE] = getTransMatrix( robotState )
%GETTRANSMATRIX この関数の概要をここに記述
%   詳細説明をここに記述

[ th1, th2, th3, th4, th5, th6, xb, yb, thb, lb, Heightb, d0, d1, d2, d3, d41, d42, d51, d52, d6 ] = pickUpData( robotState );

TREE = eye(4);
%TRA
Trans(:, :, 1) = [ cos(thb), -sin(thb), 0, xb + lb*cos(thb);
    sin(thb),  cos(thb), 0, yb + lb*sin(thb);
    0,         0, 1,                Heightb;
    0,         0, 0,                1];

%TA1
Trans(:, :, 2)=[ cos(th1), -sin(th1), 0,       0;
    sin(th1),  cos(th1), 0,       0;
    0,         0, 1, d0 + d1;
    0,         0, 0,       1];


%T12
Trans(:, :, 3) =[ sin(th2),  cos(th2), 0, 0;
    0,         0, 1, 0;
    cos(th2), -sin(th2), 0, 0;
    0,         0, 0, 1];

%T23
Trans(:, :, 4)=[ cos(th3), -sin(th3), 0, d2;
    sin(th3),  cos(th3), 0,  0;
    0,         0, 1,  0;
    0,         0, 0,  1];

%T34
Trans(:, :, 5)= [ 1,         0,         0, d3;
    0, -sin(th4), -cos(th4),  0;
    0,  cos(th4), -sin(th4),  0;
    0,         0,         0,  1];

%T4-4.5
Temp= [1/2, 3^(1/2)/2, 0, d41;
    0,         0, 1,   0;
    3^(1/2)/2,      -1/2, 0,   0;
    0,         0, 0,   1];

%T4-5
Trans(:, :, 6)= Temp*[ 1,         0,         0, d42;
    0, -sin(th5), -cos(th5),   0;
    0,  cos(th5), -sin(th5),   0;
    0,         0,         0,   1];

%T5-5.5
Temp = [1/2, -3^(1/2)/2, 0, d51;
    0,          0, 1,   0;
    -3^(1/2)/2,       -1/2, 0,   0;
    0,          0, 0,   1];

%T5-6
Trans(:, :, 7)= Temp*[ 1,         0,         0, d52;
    0, -sin(th6), -cos(th6),   0;
    0,  cos(th6), -sin(th6),   0;
    0,         0,         0,   1];

%T6EE
Trans(:, :, 8)= [ 1, 0, 0, d6;
    0, 1, 0,  0;
    0, 0, 1,  0;
    0, 0, 0,  1];

for i=1:size(Trans, 3)
    TREE = TREE*Trans(:, :, i);
end

end

function [ th1, th2, th3, th4, th5, th6, xb, yb, thb, lb, Heightb, d0, d1, d2, d3, d41, d42, d51, d52, d6 ] = pickUpData( robotState )

[ th1, th2, th3, th4, th5, th6, xb, yb, thb ] = Utility.convertRobotState2Params( robotState );

ic = getInitConfig();
lb = ic.lb; 
Heightb = ic.Heightb; 
d0 = ic.d0; 
d1 = ic.d1; 
d2 = ic.d2; 
d3 = ic.d3; 
d41 = ic.d41; 
d42 = ic.d42; 
d51 = ic.d51; 
d52 = ic.d52; 
d6 = ic.d6;

end







