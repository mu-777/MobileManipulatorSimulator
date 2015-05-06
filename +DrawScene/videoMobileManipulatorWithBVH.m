function figObj = videoMobileManipulatorWithBVH( is, T, eeState, eeRefState, robotState, bvhT, bvhData, targetBVHNodeName,  viewPoint )
%videoMobileManipulatorWithBVH ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q

figObj = figure('Position', is.windowPosition);
figure(figObj);
    
%% For Video
fps = 10;
stepPerFrame = round(size(T, 1)/(fps*T(end))); %���O������
video = [];

%% Loop
for step = 1 : stepPerFrame : size(T, 1)
    clf;    % �O���t���N���A
    grid on;    hold on;
    view(viewPoint);
    
    stepForBVH = floor(interp1(bvhT, (1:length(bvhT)), T(step)));
    centerAxisPos = [bvhData(1).Dxyz(1, stepForBVH), bvhData(1).Dxyz(2, stepForBVH), 0];
    % centerAxisPos = [robotState(step, is.M-1), robotState(step, is.M), 0.0];
    currentAxisSize = is.sceneAxisSize + [ centerAxisPos(1), centerAxisPos(1), centerAxisPos(2), centerAxisPos(2), centerAxisPos(3), centerAxisPos(3) ];
    axis(round(currentAxisSize));
    
    % draw
    DrawScene.drawBVH( figObj, is, bvhT, bvhData, targetBVHNodeName, T(step) );    
    DrawScene.drawMobileManipulator( figObj, is, eeState, eeRefState, robotState, step ); 
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

