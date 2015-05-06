function robotState = getRobotState( dt, robotState_prev, robotVel_prev )
%GETROBOTSTATE この関数の概要をここに記述
%   詳細説明をここに記述

Tnonholonomic = Utility.getTnonholonomic(robotState_prev);

robotState = ( robotState_prev' + (Tnonholonomic*robotVel_prev')*dt )';

end

