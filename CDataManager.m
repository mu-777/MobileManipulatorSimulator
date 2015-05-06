classdef CDataManager < handle
    %CDataManager
    
    properties(GetAccess = public, SetAccess = private)
        T;   
        robot;  
        ee;    
        manipulability;   
        u;  
        eeRef;        
        bvhTime;
        bvhFullData;
        targetBVHNodeName;
    end
    properties(Access = private)
        is;
    end
    
    %% Constructor
    methods
        function self = CDataManager(initSettings)
            self.is = initSettings;
            self.declare();
            
            load('./+Data/000_bvhData.mat', 'bvhT')
            load('./+Data/000_bvhData.mat', 'bvhData')
            load('./+Data/000_bvhData.mat', 'targetNodeName')
            self.bvhTime = bvhT;
            self.bvhFullData = bvhData;
            self.targetBVHNodeName = targetNodeName;
        end
    end
    
    %% Recorder
    methods
        function initialize(self, robotInit, eeInit, manipulabilityInit, uInit, eeRefInit)
            self.record(1, robotInit, eeInit, manipulabilityInit, uInit, eeRefInit);
        end
        function record(self, step, robotData, eeData, manipulabilityData, uData, eeRefData)
            self.T(step) = (step - 1) * self.is.simStepWidth;
            self.robot.state(step,:) = robotData.state;
            self.robot.vel(step,:) = robotData.vel;
            self.ee.state(step,:) = eeData.state;
            self.ee.vel(step,:) = eeData.vel;
            self.manipulability.total(step) = manipulabilityData.total;
            self.manipulability.arm(step) = manipulabilityData.arm;
            self.u(step,:) = uData;
            self.eeRef.state(step,:) = eeRefData.state;
            self.eeRef.vel(step,:) = eeRefData.vel;
        end
    end
    methods(Access = private)
        function declare(self)
            self.T = zeros(self.is.maxSize, 1);            
            self.robot.state = zeros(self.is.maxSize, self.is.M+1);   % ŠÖß”+xb,yb,thb
            self.robot.vel = zeros(self.is.maxSize, self.is.M); % ŠÖß”+v,w
            self.ee.state = zeros(self.is.maxSize, self.is.N);
            self.ee.vel =  zeros(self.is.maxSize, self.is.N);
            self.manipulability.total = zeros(self.is.maxSize, 1);
            self.manipulability.arm = zeros(self.is.maxSize, 1);
            self.u = zeros(self.is.maxSize, self.is.M);
            self.eeRef.state = zeros(self.is.maxSize, self.is.N);
            self.eeRef.vel = zeros(self.is.maxSize, self.is.N);
        end
    end
    
    
    %% DrawGraph
    methods
        function graphArmJoints(self, figNum)
            DrawGraph.armJoints(figNum, self.is, self.T, self.robot.state);
        end
        function graphManipulability(self, figNum, switchNum)
            if switchNum == 0
                DrawGraph.manipulability_both(figNum, self.is, self.T, self.manipulability.total, self.manipulability.arm);
            elseif switchNum == 1
                DrawGraph.manipulability(figNum, self.is, self.T, self.manipulability.total, 'total');
            elseif switchNum == 2
                DrawGraph.manipulability(figNum, self.is, self.T, self.manipulability.arm, 'arm');
            end
        end
        function graphReference(self, figNum)
            DrawGraph.reference(figNum, self.is, self.T, self.eeRef.state);
        end
        function graphStateError(self, figNum)
            DrawGraph.stateError(figNum, self.is, self.T, self.eeRef.state, self.ee.state);
        end
        function graphTopView(self, figNum)
            DrawGraph.topView(figNum, self.is, self.T, self.robot.state, self.ee.state, self.eeRef.state);
        end
        function graphRobotVel(self, figNum)
            DrawGraph.robotVel(figNum, self.is, self.T, self.robot.vel);
        end
    end
    
    %% DrawScene
    methods
        function viewBVH(self, figNum, viewTime, viewPoint)
            DrawScene.sceneBVH(  figNum, self.is, self.T, self.bvhTime, self.bvhFullData, self.targetBVHNodeName, viewTime, viewPoint );
        end
        function viewMobileManipulator(self, figNum, viewTime, viewPoint)
            DrawScene.sceneMobileManipulator( figNum, self.is, self.T, self.ee.state, self.eeRef.state, self.robot.state,  viewTime, viewPoint);
        end
        function viewMobileManipulatorWithBVH(self, figNum, viewTime, viewPoint)
            DrawScene.sceneMobileManipulatorWithBVH( figNum, self.is, self.T, self.ee.state, self.eeRef.state, self.robot.state, self.bvhTime, self.bvhFullData, self.targetBVHNodeName, viewTime, viewPoint);
        end
    end
    
    %% MakeVideo
    methods
        function videoBVH(self, viewPoint)
            DrawScene.videoBVH(self.is, self.T, self.bvhTime, self.bvhFullData, self.targetBVHNodeName, viewPoint );
        end
        function videoMobileManipulator(self, viewPoint)
            DrawScene.videoMobileManipulator( self.is, self.T, self.ee.state, self.eeRef.state, self.robot.state, viewPoint);
        end
        function videoMobileManipulatorWithBVH( self, viewPoint)
            DrawScene.videoMobileManipulatorWithBVH( self.is, self.T, self.ee.state, self.eeRef.state, self.robot.state, self.bvhTime, self.bvhFullData, self.targetBVHNodeName,  viewPoint );
        end
    end
    
end

