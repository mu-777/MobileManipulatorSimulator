function figObj = manipulability( figNum, is, t,  manipulability, legendName)
%DRAWGRAPH_MANIPULABILITY この関数の概要をここに記述
%   詳細説明をここに記述

if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end

plot(t, manipulability)
xlabel('time')
ylabel('Manipulability')
legend(legendName)
grid on 

 
end

