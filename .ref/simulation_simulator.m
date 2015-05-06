%% simulation _ simulator ���@��������N���X
%   
% simulation ��1�����i��������������܂Łj�ɐӔC�����N���X   
%
%  odeParam : 
%   �V�~�����[�g�ő厞�� maxSimTime�@
%   �V�~�����[�V�������x�֘A accuracy, times_of_refine
%
%  simParam :
%   fixedPointAcc : �Œ�_�̐��x
%   jacobianAcc   : ���R�r�A���̌v�Z���x
%
% �����Ă����Œ�_
%   x_p =  [y theta dx dtheta g1 g2 g3 g4]
%
% �V�~�����[�V�����f�[�^�@simDat
% simDat
%       findFlag : 1 or 0
%       fixedPoint_p : x_p
%       accuracy : ���ۂ̉��̐��x
%       robotParam : 
%       odeParam : 
%       simParam : 
%       stabDat : 
%       analDat :
%
% stabDat : 
%  stabDat.eigVal = [eigVal1, ....] 
%  stabDat.eigVec(i) = eigVal(i)�̉��x�N�g��
%  stabDat.maxEigVal : �ۑ����ɑΉ�����ŗL�l1���������ŗL�l
%
% analDat :
%  analDat.gaits  : [0x0000, 0x1000, ........] 
%  analDat.energy : �G�l���M�[
%  analDat.Fr : �t���h��
%  analDat.period : ����
%  analDat.reactionMax : [leg1, leg2, leg3, leg4, maxOfThem] �����͂̑傫��
%  analDat.forcesMax : [f1x, f1y; f2x, f2y; f3x, f3y; f4x, f4y; fmaxx, fmaxy;]�@�����͂̐���
%
%
%

classdef simulation_simulator < handle
   
    properties (GetAccess = public, SetAccess = private)
        odeParam; % ODE45�̃p�����^
        simParam; % Simulation �̃p�����^   
        pFunc_poincreMapGallop_with_constraints; % �S���t����1�����̃M�����b�v�֐��̊֐��|�C���^
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
           % ���̒T����1�X�e�b�v
           % �@�T���֐��̑��
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
               % ���x������Ă��Ȃ��ꍇ�C�̂Ă�
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
           % MATLAB�̃C�x���g���������s���Ă��܂����ꍇ�C�����̒l�����������Ȃ��Ă��܂����Ƃ����邽�߁C���������f�[�^����������̂Ă邱�Ƃɂ���
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
            
           % �����
           return;
       end
       
       
   end
    
    
    
end
