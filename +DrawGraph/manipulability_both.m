function figObj = manipulability_both( figNum, is, t,  manipulability_total, manipulability_arm)
%DRAWGRAPH_MANIPULABILITY この関数の概要をここに記述
%   詳細説明をここに記述

if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end

plot(t, manipulability_total, t, manipulability_arm)
xlabel('time')
ylabel('Manipulability')
legend('total', 'arm')
grid on 

 
end

