%% シミュレーションパラメタオブジェクトクラス
%  　　参照型で定義してやる
%      シミュレーションにおいて変化させられるであろう変数についてのクラス
%      others :  [dgf dgb Fr Energy]

classdef simulation_parameter < handle
   % handle を継承することで，オブジェクトは参照型で定義される
    
   properties (GetAccess = public, SetAccess = private)
       initialGuess = zeros(1, 8);
       initialStatConst = zeros(1, 8);
       roboParam;
       others = zeros(1, 4); % [dgf dgb Fr Energy]
   end
   
   methods
       % constructor
       function obj = simulation_parameter(roboParam_, iniGuess_, iniStatConst_, othersConst_)
           obj.initialGuess = iniGuess_;
           obj.initialStatConst = iniStatConst_;
           obj.roboParam = roboParam_;
           obj.others = othersConst_;
       end
       
       % get values 
       function [iniGuess_] = get_iniGuess(obj)
           iniGuess_ = obj.initialGuess;
       end
       function [iniStatConst_] = get_iniStatConst(obj)
           iniStatConst_ = obj.initialStatConst;
       end
       function [roboParam_] = get_roboParam(obj)
           roboParam_ = obj.roboParam;
       end
       function [others_] = get_others(obj)
           others_ = obj.others;
       end
       
       % change Parameters
       function [] = set_value(obj, value_, name)
           if strcmp(name, 'iniGuess_y')
               obj.initialGuess(1) = value_;
           elseif strcmp(name, 'iniGuess_th')
               obj.initialGuess(2) = value_;
           elseif strcmp(name,'iniGuess_dx')
               obj.initialGuess(3) = value_;
           elseif strcmp(name ,  'iniGuess_dth')
               obj.initialGuess(4) = value_;
           elseif strcmp(name,  'iniGuess_g1')
               obj.initialGuess(5) = value_;
           elseif strcmp(name,  'iniGuess_g2')
               obj.initialGuess(6) = value_;
           elseif strcmp(name,  'iniGuess_g3')
               obj.initialGuess(7) = value_;
           elseif strcmp(name,  'iniGuess_g4')
               obj.initialGuess(8) = value_;
               
           elseif strcmp(name,  'iniStatus_y')
               obj.initialStatConst(1) = value_;
           elseif strcmp(name,  'iniStatus_th')
               obj.initialStatConst(2) = value_;
           elseif strcmp(name, 'iniStatus_dx')
               obj.initialStatConst(3) = value_;
           elseif strcmp(name,  'iniStatus_dth')
               obj.initialStatConst(4) = value_;
           elseif strcmp(name,  'iniStatus_g1')
               obj.initialStatConst(5) = value_;
           elseif strcmp(name,  'iniStatus_g2')
               obj.initialStatConst(6) = value_;
           elseif strcmp(name,  'iniStatus_g3')
               obj.initialStatConst(7) = value_;
           elseif strcmp(name,  'iniStatus_g4')
               obj.initialStatConst(8) = value_;
               
           elseif strcmp(name,  'k')
               obj.roboParam.k = value_;
           elseif strcmp(name,  'L')
               obj.roboParam.L = value_;
           elseif strcmp(name,  'I')
               obj.roboParam.I = value_;
               
           elseif strcmp(name,  'dgf')
               obj.others(1) = value_;
           elseif strcmp(name,  'dgb')
               obj.others(2) = value_;
           elseif strcmp(name,  'Fr')
                obj.others(3) = value_;
           elseif strcmp(name,  'E')
                obj.others(4) = value_;
           end
       end
       
       % set initialGuess
       function [] = set_initialGuess(obj, value_)
            obj.initialGuess = value_;
       end
   end
      
end