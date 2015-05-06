function figObj = sceneBVH(  figNum, is, T, bvhT, bvhData, targetBVHNodeName, viewTime, viewPoint )
%SCENEBVH この関数の概要をここに記述
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
targetStep = floor(interp1(bvhT, (1:length(bvhT)), viewTime));
centerAxisPos = [bvhData(1).Dxyz(1, targetStep), bvhData(1).Dxyz(3, targetStep), bvhData(1).Dxyz(2, targetStep)];
currentAxisSize = is.sceneAxisSize + [ centerAxisPos(1), centerAxisPos(1), centerAxisPos(2), centerAxisPos(2), centerAxisPos(3), centerAxisPos(3) ];
axis(round(currentAxisSize));

DrawScene.drawBVH( figObj, is, bvhT, bvhData, targetBVHNodeName, viewTime );
DrawScene.drawTimeLabel( figObj, is, T, targetStep);
end

