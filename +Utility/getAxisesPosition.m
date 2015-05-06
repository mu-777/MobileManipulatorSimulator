function [ axisX, axisY, axisZ ] = getAxisesPosition( robotState )
%GETAXISESPOSITION この関数の概要をここに記述
%   詳細説明をここに記述

[Trans, ~]  = Calculator.getTransMatrix(robotState);
curTrans = eye(4,4);
N = size(Trans, 3);

axisX = zeros(N, 1); 
axisY = zeros(N, 1); 
axisZ = zeros(N, 1); 

for i=1:N;
    curTrans =  curTrans * Trans(:, :, i);
    axisX(i) = curTrans(1, 4);
    axisY(i) = curTrans(2, 4);
    axisZ(i) = curTrans(3, 4);
end

end

