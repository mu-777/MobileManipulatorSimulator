function Tnonholonomic = getTnonholonomic( robotState )
%GETTNONHOLONOMIC この関数の概要をここに記述
%   詳細説明をここに記述

ic = getInitConfig();    
M = ic.M;
Rw = ic.Rw;
T = ic.T;
thb = robotState(M+1);

Tnonholo =  [ cos(thb), 0;  sin(thb), 0; 0, 1;] * [ Rw/2 Rw/2; Rw/T -Rw/T];
Tnonholonomic = [eye(M-2), zeros(M-2, 2); zeros(3, M-2), Tnonholo];

end

