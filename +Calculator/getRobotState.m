function robotState = getRobotState( dt, robotState_prev, robotVel_prev )
%GETROBOTSTATE ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q

Tnonholonomic = Utility.getTnonholonomic(robotState_prev);

robotState = ( robotState_prev' + (Tnonholonomic*robotVel_prev')*dt )';

end

