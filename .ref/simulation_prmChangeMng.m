%% simulation _ parmeter change を　統括するクラス
%   
%   シミュレーションのパラメタ変化に責任を持つクラス   
%       1自由度のパラメタを変化させてみる
%       終了時変化させたパラメタは初期値に戻しておく
%       

classdef simulation_prmChangeMng < handle
   
    properties (GetAccess = public, SetAccess = private)
        simNum = 0; % simulation number 
        terminateWay = 1; % 1 : Just change parameters till limits
                          % 2 : change parameters till the fixed point can not be found  
        isUpdateIniGuess = 1; % WHether do you update the initial guess or not
                          
        step = 0.1; % displacement of value
        minVal = 0; % minimum limit of the value 
        maxVal = 0; % maximum limit of the value
        changePrm = 'a'; % the name of the changing parameter
        initialVal = 0; % initial value 
        currentVal = 0; % current value
        
        startIniGuess = zeros(1, 8); % the initial value which we start
        finPlusFlag = 0; % the changing in plus direction was over 
    end
    
   methods
       % constructor
       function obj = simulation_prmChangeMng(changePrmName_, initialVal_, minVal_, maxVal_, step_, isUpdateIniGuess_, terminateWay_)
           obj.terminateWay = terminateWay_;
           obj.isUpdateIniGuess = isUpdateIniGuess_;
           obj.step = step_;
           obj.minVal = minVal_;
           obj.maxVal = maxVal_;
           obj.initialVal = initialVal_;
           obj.currentVal = initialVal_;
           obj.changePrm = changePrmName_;
           obj.simNum = 1;
           obj.finPlusFlag = 0;
       end
       
      % work関数の終了処理
       function [] = finalize(obj, parameter_)
           % parameter 変数の終端化
           parameter_.set_value(obj.initialVal, obj.changePrm);
           parameter_.set_initialGuess(obj.startIniGuess);
           % 自分自身の終端化
           obj.simNum = 1;
           obj.finPlusFlag = 0;
       end
       % Functor (main Function)
       function [isEnd_] = work(obj, isSuccess, prvFixedPoint, parameter)
           % 場合分け 
           if obj.simNum == 1
               % はじめての場合
               % 初期値の推定値を更新してやる
               obj.startIniGuess = parameter.initialGuess;
               % パラメタを初期値にしてやる
               obj.currentVal = obj.initialVal;
               parameter.set_value(obj.currentVal, obj.changePrm);
               % これで終わり
               isEnd_ = 0;
               obj.simNum = 2;
               return;
           end
               
           % パラメターを正方向に変化中
           if obj.finPlusFlag == 0
               % パラメタを更新
               obj.currentVal = obj.currentVal + obj.step;
               
               % パラメタの上限に至った場合
               if  obj.currentVal > obj.maxVal
                   obj.finPlusFlag = 1;
                   obj.currentVal = obj.initialVal-obj.step;
                   parameter.set_initialGuess(obj.startIniGuess);
                   % もしも下もこえてれば
                   if obj.currentVal < obj.minVal
                      obj.finalize(parameter);
                      isEnd_ = 1;
                      return
                   end
                   
               % 解が見つかっていることを確認
               elseif isSuccess
                   % この場合は場合によって初期推定値を変化させる
                   if obj.isUpdateIniGuess
                       parameter.set_initialGuess(prvFixedPoint);
                   end
                   
               else
                   % 解が見つかってない場合は何もしないが，もし解が見つからなければ終えるモードであればPLUSフェーズ終了します
                   if obj.terminateWay == 2
                       obj.finPlusFlag = 1;
                       obj.currentVal = obj.initialVal-obj.step;
                       parameter.set_initialGuess(obj.startIniGuess);
                       % もしも下もこえてれば
                       if obj.currentVal < obj.minVal
                          obj.finalize(parameter);
                          isEnd_ = 1;
                          return
                       end
                   end
               end
                             
               % パラメタを反映
               parameter.set_value(obj.currentVal, obj.changePrm);
               % これで終わり
               isEnd_ = 0;  
               obj.simNum = obj.simNum + 1;
               return;
           end
           
           % パラメタを負方向に変化中
           if obj.finPlusFlag == 1
               % パラメタを更新
               obj.currentVal = obj.currentVal - obj.step;
               
               % パラメタの下限に至った場合
               if  obj.currentVal < obj.minVal
                   % change Parameter はおしまい
                   obj.finalize(parameter);
                   isEnd_ = 1;
                   return
               end
               
               % 解が見つかっていることを確認
               if isSuccess
                   % この場合は場合によって初期推定値を変化させる
                   if obj.isUpdateIniGuess
                       parameter.set_initialGuess(prvFixedPoint);
                   end 
               else
                   % 解が見つかってない場合は何もしないが，もし解が見つからなければ終えるモードであれば終了します
                   if obj.terminateWay == 2
                       obj.finalize(parameter);
                       isEnd_ = 1;
                       return
                   end
               end
               
               % パラメタを変化させる
               parameter.set_value(obj.currentVal, obj.changePrm);
               % これで終わり
               isEnd_ = 0;
               obj.simNum = obj.simNum + 1;
               return;
           end
           
       end
       
       % getSimulationRepeatNum
       function [num] = get_simRepNum(obj)
           if round( (obj.initialVal - obj.minVal)/obj.step - 0.5 ) == -1
                num = round( (obj.maxVal - obj.initialVal)/obj.step - 0.5)  + 1;    
           elseif round( (obj.maxVal - obj.initialVal)/obj.step - 0.5) == -1
               num =  round( (obj.initialVal - obj.minVal)/obj.step - 0.5 ) + 1;
           elseif round( (obj.maxVal - obj.initialVal)/obj.step - 0.5) == -1 && round( (obj.initialVal - obj.minVal)/obj.step - 0.5 ) == -1
               num = 1;
           else
               num = round( (obj.maxVal - obj.initialVal)/obj.step - 0.5) + round( (obj.initialVal - obj.minVal)/obj.step - 0.5 ) + 1;
           end
       end
       
   end
    
    
    
end
