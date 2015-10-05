function initConfig = getInitConfig( )
%getInitConfig この関数の概要をここに記述
%   基本的に変えないパラメータはここ

%% Define
ic.N = 6;  % x,y,z,roll,pitch,yaw
ic.M = 8;  % 関節数+thl, thr
ic.DEG2RAD = (pi/180);
ic.RAD2DEG = (180/pi);

%% Viewer Config
windowLeft = 50;
windowBottom = 50;
windowWidth = 640;
windowHeight = 480;
ic.windowPosition = [windowLeft, windowBottom, windowWidth, windowHeight];

fieldUnit = 2000;
sceneXmin = - fieldUnit;
sceneXmax = fieldUnit;
sceneYmin = -fieldUnit;
sceneYmax = fieldUnit;
sceneZmin = 0;
sceneZmax = fieldUnit;
ic.sceneAxisSize = [ sceneXmin, sceneXmax, sceneYmin, sceneYmax, sceneZmin, sceneZmax];

%% Robot Config
ic.Lengthb = 800;
ic.Heightb = 323; % 500;
ic.Widthb = 600;

ic.d0 = 154.4;
ic.d1 = -118.1; 
ic.d2 = 290; 
ic.d3_offset = 7;
ic.d3 = 123.3; 
ic.d4 = 74.1;
ic.d5 = 74.1; 
ic.d6 = -160;

ic.j5_bend = -55 * ic.DEG2RAD;
ic.j6_bend = 55 * ic.DEG2RAD;

ic.Rw = 160; % 50;
ic.T = 340; % 300;
ic.lb = 150; % 0.2*(ic.Lengthb/2);


%% Return
initConfig = ic;

end

