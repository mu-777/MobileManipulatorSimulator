
clear;
close all;

%% Mobile Manipulator Simulator
is = getInitSettings();

%% Declaration
mySystem = CMobileManipulator(is, is.robotInit.state, is.robotInit.vel);
myController = CController(is, is.robotInit.state, is.robotInit.vel, is.eeInit.state, is.eeInit.vel); 
myData = CDataManager(is);

%% Initialize
[uInit, eeRefInit] = myController.generate(0, is.robotInit.state, is.robotInit.vel);
myData.initialize(is.robotInit, is.eeInit, is.manipulabilityInit, uInit, eeRefInit);

%% Main Loop
totalStep = 1;
for ctrlStep = 1 : is.maxCtrlStep
    disp([num2str(100*ctrlStep/is.maxCtrlStep), ' %'])
    [uCurr, eeRefCurr] = myController.generate(myData.T(totalStep),...
                                                                        myData.robot.state(totalStep,:),...
                                                                        myData.robot.vel(totalStep,:));
    
    
    for simStep = 1 : is.maxSimStep
        totalStep = totalStep + 1;
        [robotCurr, eeCurr, manipulabilityCurr] = mySystem.update(uCurr);
        
        myData.record(totalStep, robotCurr, eeCurr, manipulabilityCurr, uCurr, eeRefCurr);
    end
    
end
disp('-------------------------')
disp('----------END----------')
disp('-------------------------')

%% Graph (see:CDataManager) 
myData.graphStateError(1);
myData.graphArmJoints(2);
myData.graphTopView(3);
myData.graphRobotVel(4);
myData.graphManipulability(5, 0);

%% View (see:CDataManager)
% myData.viewBVH(6, 1.0, [20, 60]);
myData.viewMobileManipulator(6, 0.0, [20, 60]);
% myData.viewMobileManipulatorWithBVH(6, 1.0, [20, 60]);

%% Video (see:CDataManager)
% myData.videoBVH([20,60]);
% myData.videoMobileManipulator([20,60]);
myData.videoMobileManipulatorWithBVH([20,60]);
















