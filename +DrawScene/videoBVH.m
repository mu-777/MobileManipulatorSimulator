function figObj = videoBVH( is, T, bvhT, bvhData, targetBVHNodeName, viewPoint )
%videoBVH この関数の概要をここに記述
%   詳細説明をここに記述

figObj = figure('Position', is.windowPosition);
figure(figObj);

%% For Video
fps = 10;
stepPerFrame = round(size(T, 1)/(fps*T(end))); %名前が悪い
video = [];

%% Loop
for step = 1 : stepPerFrame : size(T, 1)
    clf;    % グラフをクリア
    grid on;    hold on;
    view(viewPoint);
    
    targetStep = floor(interp1(bvhT, (1:length(bvhT)), T(step)));
    centerAxisPos = [bvhData(1).Dxyz(1, targetStep), bvhData(1).Dxyz(3, targetStep), bvhData(1).Dxyz(2, targetStep)];
    currentAxisSize = is.sceneAxisSize + [ centerAxisPos(1), centerAxisPos(1), centerAxisPos(2), centerAxisPos(2), centerAxisPos(3), centerAxisPos(3) ];
    axis(round(currentAxisSize));
    
    % draw
    DrawScene.drawBVH( figObj, is, bvhT, bvhData, targetBVHNodeName, T(step) );    
    DrawScene.drawTimeLabel( figObj, is, T, step);
    
    % record
    frame = getframe(figObj);
    video = [video; frame];
end

%% Generate
title = strcat('sim_', datestr(now, 'yyyymmdd_HHMM'), '.mp4');
writerObj = VideoWriter(title, 'mpeg-4');
writerObj.FrameRate = 30;
open(writerObj);
writeVideo(writerObj, video);
close(writerObj);


end

