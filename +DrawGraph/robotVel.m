function figObj = robotVel( figNum, is, t, robotVel )
%DRAWGRAPH_ROBOTVEL この関数の概要をここに記述
%   詳細説明をここに記述

% v = (robotVel(:,7) + robotVel(:,8))*is.Rw/2;
% w = (robotVel(:,7) - robotVel(:,8))*is.Rw/is.T;
v = robotVel(:,7);
w = robotVel(:,8);

if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end

subplot(3,2,1)
plot(t, v*0.001)
xlabel('time')
ylabel('v[m/s]')
legend('v')
grid on 

subplot(3,2,2)
plot(t, w*is.RAD2DEG)
xlabel('time')
ylabel('\omega[deg/s]')
legend('\omega')
grid on  

 subplot(3,2,3)
 plot(t, robotVel(:,1)*is.RAD2DEG)
 xlabel('time')
 ylabel('yaw[deg/s]')
 legend('d\theta_0/dt')
 grid on 

 subplot(3,2,4)
 plot(t, robotVel(:,2)*is.RAD2DEG, t, robotVel(:,3)*is.RAD2DEG, t, robotVel(:,5)*is.RAD2DEG)
 xlabel('time')
 ylabel('pitch[deg/s]')
 legend('d\theta_1/dt','d\theta_2/dt','d\theta_4/dt')
 grid on 

 subplot(3,2,5)
 plot(t, robotVel(:,4)*is.RAD2DEG, t, robotVel(:,6)*is.RAD2DEG)
 xlabel('time')
 ylabel('roll[deg/s]')
 legend('d\theta_3/dt','d\theta_5/dt')
 grid on 
 
 

end

