function figObj = sceneMobileManipulator( figNum, is, T, eeState, eeRefState, robotState,  viewTime, viewPoint)
%sceneMobileManipulator この関数の概要をここに記述
%   詳細説明をここに記述

if isempty(figNum)
    figObj = figure('Position', is.windowPosition);
    figure(figObj);
else
    figObj = figure(figNum);
end
    
%% VIEWER
grid on;    hold on;
view(viewPoint);
targetStep = interp1(T, (1:length(T)), viewTime);
centerAxisPos = [robotState(targetStep, is.M-1), robotState(targetStep, is.M), robotState(targetStep, is.M+1)];
currentAxisSize = is.sceneAxisSize + [ centerAxisPos(1), centerAxisPos(1), centerAxisPos(2), centerAxisPos(2), centerAxisPos(3), centerAxisPos(3) ];
axis(round(currentAxisSize));

DrawScene.drawMobileManipulator( figObj, is, eeState, eeRefState, robotState, targetStep);
DrawScene.drawTimeLabel( figObj, is, T, targetStep);

end

