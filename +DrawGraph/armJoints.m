function figObj = armJoints( figNum, is, t,  robotState)
%DRAWGRAPH_ORIERROR この関数の概要をここに記述
%   詳細説明をここに記述

if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end

subplot(4,1,1)
plot(t, robotState(:,1)*is.RAD2DEG, t, robotState(:,9)*is.RAD2DEG)
xlabel('time')
ylabel('yaw[deg]')
legend('\theta_1','\theta_b')
grid on

subplot(4,1,2)
plot(t, robotState(:,2)*is.RAD2DEG, t, robotState(:,3)*is.RAD2DEG, t, robotState(:,5)*is.RAD2DEG)
xlabel('time')
ylabel('pitch[deg]')
legend('\theta_2','\theta_3','\theta_5')
grid on

subplot(4,1,3)
plot(t, robotState(:,4)*is.RAD2DEG, t, robotState(:,6)*is.RAD2DEG)
xlabel('time')
ylabel('roll[deg]')
legend('\theta_4','\theta_6')
grid on

subplot(4,1,4)
plot(t, abs(robotState(:,9)-robotState(:,1))*is.RAD2DEG)
xlabel('time')
ylabel('|\theta_b-\theta_0| [deg]')
%legend('\theta_b')
grid on

end

