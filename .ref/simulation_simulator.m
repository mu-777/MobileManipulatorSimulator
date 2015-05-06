%% simulation _ simulator を　統括するクラス
%   
% simulation の1周期（周期解を見つけるまで）に責任を持つクラス   
%
%  odeParam : 
%   シミュレート最大時間 maxSimTime　
%   シミュレーション精度関連 accuracy, times_of_refine
%
%  simParam :
%   fixedPointAcc : 固定点の精度
%   jacobianAcc   : ヤコビアンの計算精度
%
% 扱っていく固定点
%   x_p =  [y theta dx dtheta g1 g2 g3 g4]
%
% シミュレーションデータ　simDat
% simDat
%       findFlag : 1 or 0
%       fixedPoint_p : x_p
%       accuracy : 実際の解の精度
%       robotParam : 
%       odeParam : 
%       simParam : 
%       stabDat : 
%       analDat :
%
% stabDat : 
%  stabDat.eigVal = [eigVal1, ....] 
%  stabDat.eigVec(i) = eigVal(i)の横ベクトル
%  stabDat.maxEigVal : 保存則に対応する固有値1を除いた固有値
%
% analDat :
%  analDat.gaits  : [0x0000, 0x1000, ........] 
%  analDat.energy : エネルギー
%  analDat.Fr : フルド数
%  analDat.period : 周期
%  analDat.reactionMax : [leg1, leg2, leg3, leg4, maxOfThem] 床反力の大きさ
%  analDat.forcesMax : [f1x, f1y; f2x, f2y; f3x, f3y; f4x, f4y; fmaxx, fmaxy;]　床反力の成分
%
%
%

classdef simulation_simulator < handle
   
    properties (GetAccess = public, SetAccess = private)
        odeParam; % ODE45のパラメタ
        simParam; % Simulation のパラメタ   
        pFunc_poincreMapGallop_with_constraints; % 拘束付きの1周期のギャロップ関数の関数ポインタ
    end
    
   methods
       % constructor
       function obj = simulation_simulator(odeParam_, simParam_, pFunc_poincreMapGallop_with_constraints_)
           obj.odeParam = odeParam_;
           obj.simParam = simParam_;
           obj.pFunc_poincreMapGallop_with_constraints = pFunc_poincreMapGallop_with_constraints_;
       end
       
       % Functor (main Function)
       function [simDat] = work(obj, parameter)
           % 解の探索の1ステップ
           % 　探索関数の代入
           myFunc = @(x, robotParam, odeParam, ctrParam)  obj.pFunc_poincreMapGallop_with_constraints(x, robotParam, odeParam, ctrParam, parameter.initialStatConst, parameter.others);
           % find fixed point
           [x_p_fix, jacobian_fix, max_fix_error, logDat, exitflag] = find_fixedPoint(parameter.initialGuess, myFunc, parameter.roboParam, obj.odeParam, 0, obj.simParam.fixedPointAcc, obj.simParam.jacobianAcc);
           % Calc analData
           analDat = calc_analDat(x_p_fix, logDat, parameter.roboParam, obj.simParam);
           % FindFlag 
           if exitflag <= 0
               findFlag = 0;
           else
               findFlag = 1;
               % 精度が足りていない場合，捨てる
               if max_fix_error > obj.simParam.fixedPointAcc
                   findFlag = 0;
               end
           end
           % Calc stability 
           stabDat = calc_stabDat_from_jacobian(jacobian_fix, obj.simParam.jacobianAcc);
           if findFlag == 0 || exitflag == 10
               stabDat.error = 1;
           else
               stabDat.error = 0;
           end
           % MATLABのイベント処理が失敗してしまった場合，ここの値がおかしくなってしまうことがあるため，おかしいデータを見つけたら捨てることにする
           if abs (stabDat.eigVal(1) - 1.) > obj.simParam.jacobianAcc && abs (stabDat.eigVal(2) - 1.) > obj.simParam.jacobianAcc && abs (stabDat.eigVal(3) - 1.) > obj.simParam.jacobianAcc && abs (stabDat.eigVal(4) - 1.) > obj.simParam.jacobianAcc
               stabDat.error = 1;
           end 
           if max(abs(stabDat.eigVal)) > 1e3
               stabDat.error = 1;
           end
               
           stabDat.jacobian = jacobian_fix;
           
           % create simDat
           simDat.findFlag = findFlag;
           simDat.fixedPoint_p = x_p_fix
           simDat.accuracy = max_fix_error;
           simDat.robotParam = parameter.roboParam;
           simDat.odeParam = obj.odeParam;
           simDat.simParam = obj.simParam;
           simDat.stabDat = stabDat;
           simDat.analDat = analDat;
            
           % おわり
           return;
       end
       
       
   end
    
    
    
end
