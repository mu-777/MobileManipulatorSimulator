function figObj = topView( figNum, is, t, robotState, eeState, eeState_ref )
%DRAWGRAPH_TOPVIEW ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q
if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end
plot(robotState(:,7), robotState(:,8), 'r', eeState(:,1), eeState(:,2), 'b', eeState_ref(:,1), eeState_ref(:,2), 'g--')
xlabel('x[mm]')
ylabel('y[mm]')
legend('base', 'ee', 'ee_{ref}')
grid on 
end

