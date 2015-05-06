%% simulation _ parmeter change ���@��������N���X
%   
%   �V�~�����[�V�����̃p�����^�ω��ɐӔC�����N���X   
%       1���R�x�̃p�����^��ω������Ă݂�
%       �I�����ω��������p�����^�͏����l�ɖ߂��Ă���
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
       
      % work�֐��̏I������
       function [] = finalize(obj, parameter_)
           % parameter �ϐ��̏I�[��
           parameter_.set_value(obj.initialVal, obj.changePrm);
           parameter_.set_initialGuess(obj.startIniGuess);
           % �������g�̏I�[��
           obj.simNum = 1;
           obj.finPlusFlag = 0;
       end
       % Functor (main Function)
       function [isEnd_] = work(obj, isSuccess, prvFixedPoint, parameter)
           % �ꍇ���� 
           if obj.simNum == 1
               % �͂��߂Ă̏ꍇ
               % �����l�̐���l���X�V���Ă��
               obj.startIniGuess = parameter.initialGuess;
               % �p�����^�������l�ɂ��Ă��
               obj.currentVal = obj.initialVal;
               parameter.set_value(obj.currentVal, obj.changePrm);
               % ����ŏI���
               isEnd_ = 0;
               obj.simNum = 2;
               return;
           end
               
           % �p�����^�[�𐳕����ɕω���
           if obj.finPlusFlag == 0
               % �p�����^���X�V
               obj.currentVal = obj.currentVal + obj.step;
               
               % �p�����^�̏���Ɏ������ꍇ
               if  obj.currentVal > obj.maxVal
                   obj.finPlusFlag = 1;
                   obj.currentVal = obj.initialVal-obj.step;
                   parameter.set_initialGuess(obj.startIniGuess);
                   % ���������������Ă��
                   if obj.currentVal < obj.minVal
                      obj.finalize(parameter);
                      isEnd_ = 1;
                      return
                   end
                   
               % �����������Ă��邱�Ƃ��m�F
               elseif isSuccess
                   % ���̏ꍇ�͏ꍇ�ɂ���ď�������l��ω�������
                   if obj.isUpdateIniGuess
                       parameter.set_initialGuess(prvFixedPoint);
                   end
                   
               else
                   % �����������ĂȂ��ꍇ�͉������Ȃ����C��������������Ȃ���ΏI���郂�[�h�ł����PLUS�t�F�[�Y�I�����܂�
                   if obj.terminateWay == 2
                       obj.finPlusFlag = 1;
                       obj.currentVal = obj.initialVal-obj.step;
                       parameter.set_initialGuess(obj.startIniGuess);
                       % ���������������Ă��
                       if obj.currentVal < obj.minVal
                          obj.finalize(parameter);
                          isEnd_ = 1;
                          return
                       end
                   end
               end
                             
               % �p�����^�𔽉f
               parameter.set_value(obj.currentVal, obj.changePrm);
               % ����ŏI���
               isEnd_ = 0;  
               obj.simNum = obj.simNum + 1;
               return;
           end
           
           % �p�����^�𕉕����ɕω���
           if obj.finPlusFlag == 1
               % �p�����^���X�V
               obj.currentVal = obj.currentVal - obj.step;
               
               % �p�����^�̉����Ɏ������ꍇ
               if  obj.currentVal < obj.minVal
                   % change Parameter �͂����܂�
                   obj.finalize(parameter);
                   isEnd_ = 1;
                   return
               end
               
               % �����������Ă��邱�Ƃ��m�F
               if isSuccess
                   % ���̏ꍇ�͏ꍇ�ɂ���ď�������l��ω�������
                   if obj.isUpdateIniGuess
                       parameter.set_initialGuess(prvFixedPoint);
                   end 
               else
                   % �����������ĂȂ��ꍇ�͉������Ȃ����C��������������Ȃ���ΏI���郂�[�h�ł���ΏI�����܂�
                   if obj.terminateWay == 2
                       obj.finalize(parameter);
                       isEnd_ = 1;
                       return
                   end
               end
               
               % �p�����^��ω�������
               parameter.set_value(obj.currentVal, obj.changePrm);
               % ����ŏI���
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
