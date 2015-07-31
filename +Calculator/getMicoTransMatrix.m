function [Trans, TREE] = getTransMatrix( robotState )
%GETMICOTRANSMATRIX この関数の概要をここに記述
%   ここに行列書いてくれている
%   https://github.com/Kinovarobotics/kinova-ros/blob/master/jaco_driver/src/jaco_arm_kinematics.cpp


j5_bend =  degtorad(55);
j6_bend = degtorad(35);

TREE = eye(4);
% origin to arm_base_on_mobile
Trans(:, :, 1) = [ cos(thb), -sin(thb), 0, xb + lb*cos(thb);
    sin(thb),  cos(thb), 0, yb + lb*sin(thb);
    0,         0, 1,                Heightb;
    0,         0, 0,                1];

%  arm_base_on_mobile to arm_base
Trans(:, :, 1) = Trans(:, :, 1) * [cos(pi/2), -sin(pi/2), 0,  0;
    sin(pi/2), cos(pi/2), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

%arm_base to j1
Trans(:, :, 1) = Trans(:, :, 1) * [ cos(th1), -sin(th1), 0,       0;
    -sin(th1),  -cos(th1), 0,       0;
    0,         0, -1, d0;
    0,         0, 0,       1];

%j1 to j2
Trans(:, :, 2) =[ sin(th2),  cos(th2), 0, 0;
    0,         0, 1, 0;
    cos(th2), -sin(th2), 0, d1;
    0,         0, 0, 1];

% j2 to j3
Trans(:, :, 3)=[ -cos(th3), sin(th3), 0, d2;
    sin(th3),  cos(th3), 0,  0;
    0,         0, 1,  0;
    0,         0, 0,  1];

%j3 to j3.5
Trans(:, :, 4)= [ 1, 0, 0, d3_offset;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

% j3.5 to j4
Trans(:, :, 4) = Trans(:, :, 4) * [0, 0, -1, d3
    sin(th4), cos(th4), 0, 0
    cos(th4), -sin(th4), 0, 0;
    0, 0, 0, 1];

%j4 to j5
Trans(:, :, 5)= [ cos(j5_bend)*cos(th5), cos(j5_bend)*-sin(th5), sin(j5_bend), cos(j5_bend)*d4;
    sin(th5), cos(th5), 0, 0;
    -sin(j5_bend)*cos(th5), sin(j5_bend)*sin(th5), cos(j5_bend), -sin(j5_bend)*d4;
    0, 0, 0, 1];

%j5 to j6
Trans(:, :, 5)= [cos(j6_bend) * cos(th6), cos(j6_bend) * -sin(th6), sin(j6_bend), -cos(j6_bend)*d5;
    sin(th6), cos(th6), 0, 0;
    -sin(j6_bend) * cos(th6), sin(j6_bend) * sin(th6), cos(j6_bend), -sin(j6_bend) * d5;
    0, 0, 0, 1];

% j6 to ee
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