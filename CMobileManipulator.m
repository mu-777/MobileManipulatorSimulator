classdef CMobileManipulator < handle
    %CMOBILEMANIPULATOR このクラスの概要をここに記述
    %   詳細説明をここに記述
    
    properties(GetAccess = public, SetAccess = private)
        is;
    end
    properties(Access = private)
        robotPrev;
    end
    
    methods
        function self = CMobileManipulator(initSettings, robotState_init, robotVel_init)
            self.is = initSettings;
            self.robotPrev.state = robotState_init;
            self.robotPrev.vel = robotVel_init;
        end
        function  [robotCurr, eeCurr, manipulabilityCurr] = update(self, input)
            robotCurr.state = Calculator.getRobotState( self.is.simStepWidth, self.robotPrev.state, self.robotPrev.vel);
            robotCurr.vel = input;
            
            J = Calculator.getJ( robotCurr.state );
            eeCurr.state = Calculator.getEEState(robotCurr.state);
            eeCurr.vel = Calculator.getEEVel(robotCurr.vel, J);
            
            manipulabilityCurr.total = sqrt(abs(det(J*J')));
            manipulabilityCurr.arm = sqrt(abs(det(J(1:self.is.N, 1:self.is.M-2)*J(1:self.is.N, 1:self.is.M-2)')));
            
            self.robotPrev = robotCurr;
        end
    end
    
%     methods
%         function self = set.robotPrev(self, value), self.robotPrev = value; end
%         function value = get.robotPrev(self), value = self.robotPrev; end
%     end
    
end

