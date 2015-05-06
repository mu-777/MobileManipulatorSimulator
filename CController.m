classdef CController < handle
    %CController このクラスの概要をここに記述
    %   詳細説明をここに記述
    
    properties(GetAccess = public, SetAccess = private)
        is;
        bvhTime;
        bvhTargetState;
    end
    properties(Access = private)
        eeRefPrev;
        robotPrev;
    end    
    
    methods
        function self = CController(initSettings, robotState_init, robotVel_init, eeState_init, eeVel_init)
            self.is = initSettings;
            self.robotPrev.state = robotState_init;
            self.robotPrev.vel = robotVel_init;
            self.eeRefPrev.state = eeState_init;
            self.eeRefPrev.vel = eeVel_init;
            
            if strcmp(self.is.eeRefGenerationMethod, 'bvh')
                load('./+Data/000_bvhData.mat', 'bvhT')
                load('./+Data/000_bvhData.mat', 'targetState')
                self.bvhTime = bvhT;
                self.bvhTargetState = targetState;
            end
        end
        
        function [u, eeRef] = generate(self, curTime, curRobotState, curRobotVel)
            eeRef = self.generateEERef(curTime);
            u = self.generateInput(eeRef, curRobotState);
            
            % update internal data
            self.robotPrev.state = curRobotState;
            self.robotPrev.vel = curRobotVel;
            self.eeRefPrev = eeRef;
        end
    end
    
    methods(Access = private)
        function eeRef = generateEERef(self, currTime)
            switch self.is.eeRefGenerationMethod
                case 'stay'
                    [eeState_ref, eeVel_ref] = Method.getEERef_stay(self.eeRefPrev.state);
                case 'myPath'
                    [eeState_ref, eeVel_ref] = Method.getEERef_fromMyPath(self.is.ctrlStepWidth, self.eeRefPrev.state);
                case 'bvh'
                    [eeState_ref, eeVel_ref] = Method.getEERef_fromBVH(self.bvhTime, self.bvhTargetState, self.is.ctrlStepWidth, currTime, self.eeRefPrev.state);
            end
            eeRef.state = eeState_ref;
            eeRef.vel = eeVel_ref;
        end
        % self.eeRef使わなかったのは，関数内で計算済みのeeRefを使うことを明示的に示すため
        function u = generateInput(self, eeRef, curRobotState)
            J = Calculator.getJ(curRobotState);
            pinvJ = pinv(J);            
            eeCurr.state = Calculator.getEEState(curRobotState);
            switch self.is.inputGenerationMethod
                case 'V=0'
                    Kvec = zeros(self.is.M, 1);
                case 'V=sqrt(det(JJ^T))usingDJsqDq'
                    Kvec = Method.getEval_totalManipulabilityUsingDJsqDq(curRobotState);
                case 'V=sqrt(det(JJ^T))usingDJDq'
                    Kvec = Method.getEval_totalManipulabilityUsingDJDq(curRobotState, J);
                case 'V=det(Jarm)'
                    Kvec = Method.getEval_armManipulability(self.is.N, self.is.M, curRobotState, J);
            end
            u = ( pinvJ * (eeRef.vel' - self.is.Kpdgain*(eeCurr.state' - eeRef.state'))...
                    + ( eye(self.is.M) - pinvJ*J) * self.is.Kredundancy * Kvec)';
        end
    end

end












