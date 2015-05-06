function figObj = stateError( figNum,is, t, eeState, eeState_ref)
%DRAWGRAPH_STATEERROR この関数の概要をここに記述
%   詳細説明をここに記述
if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end
subplot(3,2,1)
plot(t, eeState_ref(:,1), t, eeState(:,1))
xlabel('time')
ylabel('x[mm]')
legend('x_{EEref}','x_{EE}')
grid on 

subplot(3,2,2)
plot(t, eeState_ref(:,2), t, eeState(:,2))
legend('y_{EEref}','y_{EE}')
xlabel('time')
ylabel('y[mm]')
grid on 

subplot(3,2,3)
plot(t, eeState_ref(:,3), t, eeState(:,3))
xlabel('time')
ylabel('z[mm]')
legend('z_{EEref}','z_{EE}')
grid on 

subplot(3,2,4)
plot(t, eeState_ref(:,4)*is.RAD2DEG, t, eeState(:,4)*is.RAD2DEG)
xlabel('time')
ylabel('roll[deg]')
legend('roll_{EEref}','roll_{EE}')
grid on 

subplot(3,2,5)
plot(t, eeState_ref(:,5)*is.RAD2DEG, t, eeState(:,5)*is.RAD2DEG)
legend('pitch_{EEref}','pitch_{EE}')
xlabel('time')
ylabel('pitch[deg]')
grid on 

subplot(3,2,6)
plot(t, eeState_ref(:,6)*is.RAD2DEG, t, eeState(:,6)*is.RAD2DEG)
xlabel('time')
ylabel('yaw[deg]')
legend('yaw_{EEref}','yaw_{EE}')
grid on 


end

