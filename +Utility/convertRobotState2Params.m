function [ th1, th2, th3, th4, th5, th6, xb, yb, thb ] = convertRobotState2Params( robotState )
%CONVERTROBOTSTATE2PARAMS ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q

th1 = robotState(1);
th2 = robotState(2);
th3 = robotState(3);
th4 = robotState(4);
th5 = robotState(5);
th6 = robotState(6);
xb = robotState(7);
yb = robotState(8);
thb = robotState(9);

end

