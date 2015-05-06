function [ cornerX, cornerY, cornerZ ] = getBaseCornerPosition( robotState )
%GETBASECORNERPOSITION この関数の概要をここに記述
%   詳細説明をここに記述

[ xb, yb, thb, Lengthb, Heightb, Widthb] = pickUpData(robotState);

cornerX = zeros(4, 6);
cornerY = zeros(4, 6);
cornerZ = zeros(4, 6);

TRB = [cos(thb)  -sin(thb) 0 xb;
    sin(thb)  cos(thb) 0 yb;
    0         0        1  0;
    0         0        0  1];

baseX =  (Lengthb/2) * [
    -1, 1, 1, -1, -1, -1;
    1, 1, -1, -1, 1, 1;
    1, 1, -1, -1, 1, 1;
    -1, 1, 1, -1, -1, -1
    ];

baseY = (Widthb/2) * [
    -1, -1, 1, 1, -1, -1;
    -1, 1, 1, -1, -1, -1;
    -1, 1, 1, -1, 1, 1;
    -1, -1, 1, 1, 1, 1
    ];

baseZ = Heightb * [
    0, 0, 0, 0, 0, 1;
    0, 0, 0, 0, 0, 1;
    1, 1, 1, 1, 0, 1;
    1, 1, 1, 1, 0, 1
    ];

for i=1:6
    for j =1:4
        point = TRB*[ baseX(j, i); baseY(j, i); baseZ(j, i); 1 ];
        cornerX(j, i) = point(1);
        cornerY(j, i) = point(2);
        cornerZ(j, i) = point(3);
    end
end

end


function [ xb, yb, thb, Lengthb, Heightb, Widthb] = pickUpData(robotState)

[ th1, th2, th3, th4, th5, th6, xb, yb, thb ] = Utility.convertRobotState2Params( robotState );
ic = getInitConfig();
Lengthb = ic.Lengthb;
Heightb = ic.Heightb;
Widthb = ic.Widthb;

end








