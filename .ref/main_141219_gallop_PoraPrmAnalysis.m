% �Œ�_�T���v���O���� 20140724
%
%   �P�C�݌v���j
%       ��{�I�ɏo������̂̓I�u�W�F�N�g�����āC�e�ӔC���Ɨ�����悤�ɂ���
%       �������Ȃ���C�N���X���g���邱�ƂɋC�t���̂��x�������̂ŁC�^���������Ȃǂ̕����͊֐��x�[�X�ɂȂ��Ă���E�E�E�E�����ύX������
%
%       ��{�I�ȍ\����3��
%
%       A simulation_parameter�N���X
%          �@�V�~�����[�V�����ɕK�v�ȃp�����^���Ǘ�����N���X
%            �V�����p�����^�𑝂₵�����Ȃ����炱�̃N���X�ɑ��₵�Ă���   
%            
%       B simulation_simulator�N���X
%           ��T���̃V�~�����[�V�������s���Č��ʂ�Ԃ��N���X
%           �T�����ׂ�����`���������Q�Ƃ���@�Ȃ̂ŁC����`��������ς������Ȃ����炱�̎Q�Ɗ֐��������������Ă���
%           ��T���̂����iNewton�Ƃ��Ȃ�Ƃ��j��ς������Ȃ����炱�̃N���X���������Ă���
%
%       C�@simulation_prameterManager�N���X
%           �V�~�����[�^�̃p�����^�ω��ɑΉ�����N���X
%           �p�����^�̕ω���������ς������Ȃ�����C���̃N���X���������Ă���
% 
%       D ���C���֐�
%           �V�~�����[�V�����̍\�����`�C����3�K�w�̃��[�v�ɂȂ��Ă���@
%           ���O�p�̕ϐ����Ǘ����Ă���@�i������� A-C�̃I�u�W�F�N�g���Ǘ��j
%
%   �Q�C�傫�ȕω������������̕��j          
%       �^����������ς������@�|���@����`��������ω�������i�傫�ȕύX�C��ԗʂ��ς��Ƃ��E�E�E�ł���΂��̈ˑ��֐����ω����ׂ��j�C�p�����^���ς��̂ł���΁CA���ύX
%       simDat�̒��g�𑝂₵���� -> B�̃N���X��ύX
%       �p�����^�̕ω��̎d����ς������i2��C�ɕς���Ƃ��Ȃ�Ƃ��j -> C�̃N���X��������
%       
% �C�t�������ӓ_�Ȃ�
%   MATLAB�̃C�x���g����̕����ɂ́C�o�O�������āC�ڒn��������Ȃ����Ƃ�����C����ɂ���āC��T�������܂������Ȃ��ꍇ���o�Ă���
%   ��T���œ���������C���S�ɌŒ�_�ɂȂ��Ă��Ȃ����CJacobian�̌v�Z�ɂP�O�O�P�O�I�[�_�[�̌덷���ڂ��Ă��܂����Ƃ�����
%   ����ɁC�悭������Ȃ����CMATLAB�̃C�x���g��������Ȃ��o�O�̉e���ŁC����������Ȃ��Ƃ���C���萫��]���ł��Ȃ��Ƃ������݂���i���͂��̂悤�ȉ��͎̂ĂĂ���j
%   �ǂ����CMATLAB�ŉ���������ɂ������R�́C2�̃C�x���g�������ɋN���Ă��邩�炭�����D�D�D�D�ǂ������̐��x�ȉ��̌덷�͓���Ă�����܂����ꏏ������C���̐��x�ȉ��̈ʑ����̌덷1e-9���Ԃ�����Ōv�Z�������y�����āC�o�O���Ȃ���
%   �@�@��̋L�q�͉��̖�������������������D�D�D�D
%   ����ɁC������ԂŃ|�A���J���f�ʔ��肵�Ă���ƁC���̌��肭�����Ȃ��Ȃ�݂����ȁC�C�x���g�𔻒肵�����Ƃ����ɃC�x���g���N���邱�Ƃ͖��炵���D����
%   �����悭������Ȃ��C�C�x���g�����n�̒��ɂ����Ȗ�肪����ƍl���Ă����������ǂ����낤�D���ǁC���x�Ƃ��́C����܂Œʂ�ɂ��Ă����̂��x�X�g�Ȍ��ʁi�C�x���g�𔻒�ł���j������Ă����炵���D
%   ��T���͂�����x�ׂ͍����s���������悢�ł��傤�I�t�ɂ��̕����X�s�[�h�������ꍇ������D
%   
%
%
% odeParam : 
%   �V�~�����[�g�ő厞�� maxSimTime�@
%   �V�~�����[�V�������x�֘A accuracy, times_of_refine
%
% robotParam : 
%   L, I, k
%
% simParam :
%   fixedPointAcc : �Œ�_�̐��x
%   jacobianAcc   : ���R�r�A���̌v�Z���x
%
%
% �����Ă����Œ�_
%   x_p =  [y theta dx dtheta g1 g2 g3 g4]
%
% �f�[�^���O�p�ϐ� logDat
%   timedStatus: ���Ԃɂ���ԗʂȂǂ̕ω����O 
%     time(N) : ���Ԃ̃x�N�g��  
%     x(N, 6) : ��ԗʂ̃x�N�g��: [x y theta dx dy dtheta] 
%     legAngles : �r�Ɠ��̂̊Ԃ̊p�x�@[g1 g2 g3 g4] 
%     legLengthes : ���̒��� [l1 l2 l3 l4]
%     legContacts: �ڒn�̂���Ȃ�[leg1 leg2 leg3 leg4] 1�Őڒn���Ă���C�O�ŕ����Ă���
%
%   timedEvent: �C�x���g�ɂ���ԗʂȂǂ̕ω����O
%     time(event num) : ���ԃx�N�g��
%     x(event num, 6) : ��ԗʂ̃x�N�g�� [x y theta1 dx dy dtheta]    
%     ie() : �f�o�b�O�p  ���Ԃ�,leg1-4��[�r�̒����̔��ʁC���n�r��y���W�̔���]�Cy���W�Cdy  
%
%   liftOffAngles: 4�̃��t�g�I�t�A���O��  
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
%
% stabDat : 
%  stabDat.eigVal = [eigVal1, ....] 
%  stabDat.eigVec(i) = eigVal(i)�̉��x�N�g��
%  stabDat.maxEig : �ۑ����ɑΉ�����ŗL�l1���������ŗL�l
%  stabDat.error : �v�Z�G���[����������
%
% analDat :
%  analDat.gaits  : [0x0000, 0x1000, ........; time, time2] (��납�珇�ɐڒn�r�̐ڒn�P�C���n�O)
%  analDat.energy : �G�l���M�[
%  analDat.Fr : �t���h��
%  analDat.period : ����
%  analDat.reactionMax : [leg1, leg2, leg3, leg4, maxOfThem] �����͂̑傫��
%  analDat.forcesMax : [f1x, f1y; f2x, f2y; f3x, f3y; f4x, f4y; fmaxx, fmaxy;]�@�����͂̐���
%  analDat.freq : 1�����̎��g��
%  analDat.phases : [leg1_td, leg2_td, leg3_td, leg4_td; 
%                        leg1_lf, leg2_lf, leg3_lf, leg4_lf]�@(0-1�ŋK�i��)
%  analDat.phaseDiffs : [�O���E�̈ʑ���td(2-1)�C��덶�E�̈ʑ���td(4-3), �O��̈ʑ���td(2-4), �O��̈ʑ���td(1-3); , �O���E�̈ʑ���lf(2-1)�C��덶�E�̈ʑ���lf(4-3), �O��̈ʑ���lf(2-4), �O��̈ʑ���lf(1-3)]
%  analDat.legSpeed : [leg1_spd, leg2_spd, leg3_spd, leg4_spd, mean] ���ϑ��x�@[rad]/time (��������) 
%  analDat.duty : [leg1_duty, leg2_duty, leg3_duty, leg4_duty, mean] 
%  analDat.COMamp  : �d�S�̐U�� 
%
%
%
%

%% ����������ъe�ۓ��̐ݒ�

clear
close all force

%% Simulator�̃p�����^�ݒ�
%ODE param
 % ode45�@�̃f�[�^�o�͐��H�H �O���t���e����Α傫�����Ă���(�S�����̂��߂�1�ɂ��Ă邯��)
 odeParam.times_of_refine = 4;
 % ���Ό덷(���̒l�łȂ��Ɠ����ڒn���������Ȃ�)
 odeParam.relativeAccuracy = 1e-11; %��{�I�ɂ��̒l�͂�����Ȃ������ǂ��@���������P��OK�ɂ������Ȃ� 3-14
 % ��Ό덷
 odeParam.absoluteAccuracy = 1e-10; %��{�I�ɂ��̒l�͂�����Ȃ������ǂ� ���������P��OK�ɂ������Ȃ� 3-13
 % �V�~�����[�V�����ő厞�� [normalized]
 odeParam.maxSimulationTime = 10;   

%Simulation parameter
 % fixed point accuracy
 simParam.fixedPointAcc = 1e-7;% ���ꂭ�炢�Ŏ��p��͑Ó��ł��傤
 % jacobian accuracy
 simParam.jacobianAcc = 1e-6; % ��������p��͑Ó��ł��傤
 
%���̒T���֐� �ݒ�
% pFuncGallop = @(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst) func_poincreMapGallop_with_constraints_Fr_y_dth_dgs(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst);
pFuncGallop = @(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst) func_poincreMapGallop_with_constraints_dx_y_dth_dg_scissorsym(x, robotParam, odeParam, ctrParam, iniStatConst, otherConst);

% ���ꂪ�V�~�����[�V�����Ŏg����I�I
% Simulator�I�u�W�F�N�g�̍쐬
% *********************************************************************************************************************************
simulator = simulation_simulator(odeParam, simParam, pFuncGallop); 


%% Simulation�ɂ�����p�����^�ݒ�@(�����l�Ƃ����{�b�g�ϐ��̒l�Ƃ�)
%Robot Param
% Lynx Param
x_ini_p = [0. 0.324 0. 1.39 0. (145.9/180.)* pi];
[robotParam.I, robotParam.L, robotParam.k, x_ini] = conv_normParam(7., 0.2015, 0.376/2., 1733.8, 0.22, x_ini_p);
robotParam.k = 5.56;
%robotParam.k = 6;
%robotParam.k = 7;
%robotParam.k = 5;
%robotParam.k = 4;

% ���̏�������l�ݒ�
%x_p_ini = [1.10000000000000, 0., 0., 0., 0., 0., 0., 0.];%[1,6.73529538680675e-20,1.43881089646273,1.55743331943719e-20,0.413523522801169,0.413523522801169,0.413523522801169,0.413523522801169];
x_p_ini = [1.10000000000000,-3.38539965491732e-12,0.600000000000000,1.23257488903669e-17,0.216792079766339,0.216792069766339,0.216792079751332,0.216792069751332];%[1,6.73529538680675e-20,1.43881089646273,1.55743331943719e-20,0.413523522801169,0.413523522801169,0.413523522801169,0.413523522801169];
%x_p_ini = [0.999999999847460,-9.644265966710694e-11,1.440749540262108,1.499999999920854,0.441259863191393,0.441259863191393,0.490148494387054,0.490148494387054];
%x_p_ini = [x_ini(2), x_ini(3), x_ini(4), x_ini(6), 0.2, 0.2, 0.2, 0.2];

% ���邢�͕ۑ�����Ă���C�V�~�����[�V�������ʂ���ǂ�ł���H�H
%load('141214_initialEst_gallop_dg045.mat')

% ���̏�����Ԃɑ΂���S���ݒ�
iniStatConst = [1.1, 0., 0.6, 0., 0., 0., 0., 0.];

% ���̂��̂ق��̍S���ݒ�
%   dgf,dgb�͊��S��0�ɂ��������C���������MATLAB���C�x���g�𔻕ʂł��Ȃ��ꍇ�����݂���̂ŁC�����炭���x�̖�肾�Ǝv�����C���ȉ��̐��x�ɂ��Ă���
%     <- �|�A���J���f�ʔ��肵�Ȃ��悤�ɂ�������������I�I�@�S���l�O�ɂ��ׂ� 1e-9
%   others :  [dgf dgb Fr Energy]
otherConst = [1e-9, 1e-9, 0., 0.];


% ���ꂪ�V�~�����[�V�����Ŏg����I�I
% parameter �I�u�W�F�N�g�̍쐬
% *********************************************************************************************************************************
parameter = simulation_parameter(robotParam, x_p_ini, iniStatConst, otherConst);


%% Simulation �ɂ�����p�����^�ω��̂����̎w��

% �ŉ��w���[�v�̃p�����^�ω��I�u�W�F�N�g
%  simulation_prmChangeMng(changePrmName_, initialVal_, minVal_, maxVal_, step_, isUpdateIniGuess_, terminateWay_)
%     changePrmName_ :  ��������l��8�� iniGuess_y,_th,_dx,_dth,_g1,_g2,_g3,_g4
%                       �|�A���J���f�ʂ̍S���l�@iniStatus_y,_th,_dx,_dth,_g1,_g2,_g3,_g4
%                       ���{�b�g�̃p�����^�@k, L, I
%                       ���̑��̍S���l�@dgf(�O���̐ڒn�p�x��), dgb, Fr(�t���h��), E(�G�l���M�[)
%     initialVal_�F�����l, minVal_:�T���ŏ��l, maxVal_�F�T���ő�l, step_�F�T���X�e�b�v, isUpdateIniGuess_�F��������l�������I�ɑO��̌Œ�_���g��1����0��, terminateWay_= 1:�T���ő�ŏ��l�܂ŒT�� or 2:�����݂���Ȃ��Ȃ�����I���                 

% dth�Ɋւ��ẮA�ł��邾���ׂ������̂��悢�i�p�x��0.5�x�A0.005[rad]���炢�H�H�j
%parameterManager1 = simulation_prmChangeMng('iniStatus_dth', 0., -0.001, 1.5, 0.005, 1, 1);
%parameterManager1 = simulation_prmChangeMng('iniStatus_dx', 0.6, 0.6, 2.4, 0.02, 1, 1);
parameterManager1 = simulation_prmChangeMng('iniStatus_dth', 0., 0., 5., 0.05, 1, 1);
waitbarNum(1) = parameterManager1.get_simRepNum; % �ǂꂭ�炢�̌��������Ă��邩�����Ă�����ΐi�݂������� 
% �Q�w���[�v�̃p�����^�ω��I�u�W�F�N�g
parameterManager2 = simulation_prmChangeMng('iniStatus_dx', 0.6, 0.0, 5., 0.2, 1, 1);
waitbarNum(2) = parameterManager2.get_simRepNum; % �ǂꂭ�炢�̌��������Ă��邩�����Ă�����ΐi�݂�������
% �R�w���[�v�̃p�����^�ω��I�u�W�F�N�g
parameterManager3 = simulation_prmChangeMng('dgf', 0., 0., 1.0, 0.05, 1, 1);
waitbarNum(3) = parameterManager3.get_simRepNum; % �ǂꂭ�炢�̌��������Ă��邩�����Ă�����ΐi�݂�������


% ���ꂪ�V�~�����[�V�����Ŏg����I�I
% ���ۂ̎��s�֐��Ƀ��W�X�g
parameterChanger_low = @(isSuccess, prvFixedPoint, parameter) parameterManager1.work(isSuccess, prvFixedPoint, parameter);
parameterChanger_mid = @(isSuccess, prvFixedPoint, parameter) parameterManager2.work(isSuccess, prvFixedPoint, parameter);
parameterChanger_high = @(isSuccess, prvFixedPoint, parameter) parameterManager3.work(isSuccess, prvFixedPoint, parameter);

%% Simulation �̎��s
% ��{�I�ɂ��̕����͂�����K�v�͂Ȃ��͂��I�I
% ����ɃV�~�����[�V�������񂵂Ă���镔���C�����l�Ƃ����������̂̓I�u�W�F�N�g������������Ƃ���ŕς��Ă���
% �����F��{�I�ɍċN�֐��Ƃ��g���ăI�u�W�F�N�g���ł����͂������ǂ���Ă��Ȃ��I�I�@�܂�����]�T�����

tic;
time = 0;
max = 0;
strings = {'������Ⴀ�C�͂��߂�ł���', '�����C�ꉞ�v�Z���Ă�ID��', '�c�莞�Ԃ͋����Ă�����l','','**********************************************************************'};
h = waitbar(0, strings);

% �e���[�v�Ō��E�񂷐��i���ɖ��Ȃ���Α傫���ݒ肵�āj
maxLoop = 100000;
% ���O�p�ϐ��̒�`
% LogSimDat(1,1,1);

% ���[�v�Ŏg���ϐ��̒�`
 % �O���simulation�ŉ��������������ۂ� ���ɍŉ��w�C���w�C�ŏ�
 isSuccess = [0, 0, 0];
 % �O�񌩂������� ���ɍŉ��w�C���w�C�ŏ�
 prvFixedPoint = zeros(3,8);

%�ŏ�w���[�v
for i = 1:maxLoop
    % �p�����^�ύX
    if parameterChanger_high(isSuccess(3), prvFixedPoint(3, :), parameter);
        % �I���Ɣ��肳�ꂽ�ꍇ
        prvFixedPoint(3, :) = zeros(1, 8);
        isSuccess(3) = 0;
        break;
    end
    
    % ���w���[�v
    for j = 1:maxLoop
        % �p�����^�ύX
        if parameterChanger_mid(isSuccess(2), prvFixedPoint(2, :), parameter);
            % �I���Ɣ��肳�ꂽ�ꍇ
            prvFixedPoint(2, :) = zeros(1, 8);
            isSuccess(2) = 0;
            break;
        end
        
        % �ŉ��w���[�v
        for k = 1:maxLoop
            % �p�����^�ύX
            if parameterChanger_low(isSuccess(1), prvFixedPoint(1, :), parameter);
                % �I���Ɣ��肳�ꂽ�ꍇ
                prvFixedPoint(1, :) = zeros(1, 8);
                isSuccess(1) = 0;
                break;
            end
            % �V�~�����[�V�������s
            simDat = simulator.work(parameter);
            % �l�X�V
            isSuccess(1) = simDat.findFlag;
            prvFixedPoint(1,:) = simDat.fixedPoint_p;
            % ��̃��[�v�̂��߂̒l�X�V
            if k == 1
                isSuccess(2) = isSuccess(1);
                prvFixedPoint(2,:) = prvFixedPoint(1,:);
            end
            % ���ʂ̃��O
            logSimDat(i).at(j,k) = simDat;
            
            % �󋵂�`��
            duration = toc - time;
            if duration > max
                max = duration;
            end
            time = toc;
            mean = time / (((i-1)*waitbarNum(1)*waitbarNum(2) + (j-1)*waitbarNum(1) + k));
            st = strcat('(i, j, k) =  (', num2str(i), ', ', num2str(j), ', ', num2str(k),')' );
            progress = ((i-1)*waitbarNum(1)*waitbarNum(2) + (j-1)*waitbarNum(1) + k) / (waitbarNum(1)*waitbarNum(2)*waitbarNum(3));
            remainTime = time * (1-progress)/progress;
            st3 = strcat('�\�z���Ϗ��v���ԁF', int2str(remainTime / 60), '��');
            st2 = strcat('����̃X�e�b�v���v���ԁF', int2str(duration), '[s]�C���σX�e�b�v���v����:', int2str(mean), '[s], �ő�X�e�b�v���v����:', int2str(max),'[s]');
            st4 = strcat('�\�z�ő及�v���ԁF', int2str(max * ((waitbarNum(1)*waitbarNum(2)*waitbarNum(3))-((i-1)*waitbarNum(1)*waitbarNum(2) + (j-1)*waitbarNum(1) + k))/60), '��');
            if progress < 0.5 
                strings = {'�܂�����Ă䂤�Ă�˂�E�E�E�܂��C�C���ɑ҂Ă�', st, st2, st3, st4};
            elseif progress < 0.8
                strings = {'�����͂������ł��E�E�E�܂��C�҂��Ă�', st, st2, st3, st4};
            elseif progress < 0.9
                strings = {'�������傢��ł��E�E�E', st, st2, st3, st4};
            else 
                strings = {'����������I�E�E�E�q�q�q', st, st2, st3, st4};
            end
            waitbar(progress, h, strings);
        end
        
        % ��̃��[�v�̂��߂̒l�X�V
        if j == 1
            isSuccess(3) = isSuccess(2);
            prvFixedPoint(3,:) = prvFixedPoint(2,:);
        end        
    end
    % ��̃��[�v�̂��߂̒l�X�V ����͂Ȃ� 
end

time = toc;
st = strcat('�v�Z���� : ', int2str(time/60), '��');
strings = {'�����ƁD�D�D�D�����I����Ă�˂񂯂ǁE�E�E�E�E', st};
waitbar( 1, h, strings);

%% Data ���@�ۑ�
description = {'Lynx�F�v�����N����o�E���h�A�����ăM�����b�v�̏o�� y = 1.1 K = 5.56','iniStatus_dth, 0., 0., 5., 0.05, 1, 1','iniStatus_dx, 0.6, 0.0, 5., 0.2, 1, 1','dgf, 0., 0., 1., 0.05, 1, 1','a'};
save('141219_Gallop_PrmAnal_L085_I059_K556_SimPrm_dx_dth_dgf_Const_dx_y11_dth_dg_scissorsym.mat', 'logSimDat', 'description');

%% ���ʂ̕��͕���
close all

% ���͂������z�̔ԍ�
xNum = 3;
yNum = 1;
zNum = 1;

% 3�����O���t�ŕ���
%plot_logSimData_3d(logSimDat(zNum).at, 'dth', 'dx', 'maxEigVal', [yNum, xNum]);
plot_logSimData_3d(logSimDat(zNum).at, 'dth', 'dx', 'g4', [yNum, xNum]);
plot_logSimData_3d_Zs(logSimDat(zNum).at, 'dth', 'dx', {'duty1', 'duty2', 'duty3', 'duty4', 'duty'}, [yNum, xNum], 'dutys');
plot_logSimData_3d_Zs(logSimDat(zNum).at, 'dth', 'dx', {'g1', 'g2', 'g3', 'g4'}, [yNum, xNum], 'gs');
plot_logSimData_3d_Zs(logSimDat(zNum).at, 'dth', 'dx', {'legSpeed1', 'legSpeed2', 'legSpeed3', 'legSpeed4', 'legSpeed'}, [yNum, xNum], 'legSpeeds');
%plot_logSimData_3d(logSimDat(zNum).at, 'dx', 'g3', 'y', [yNum, xNum]);

% 2�����ŉ��
%plot_logSimData_2d(logSimDat(zNum).at(yNum, :), 'dx', 'g1',  xNum);
plot_logSimData_2d(logSimDat(zNum).at(yNum, :), 'dth', 'g4',  xNum);
plot_logSimData_stability(logSimDat(zNum).at(yNum, :), 'dth', xNum);

% 2�����O���t�ŉ��
%plot_logSimData_2d(logSimDat(zNum).at(:, xNum), 'dx', 'g1', yNum);
%plot_logSimData_2d(logSimDat(zNum).at(:, xNum), 'dx', 'g3', yNum);
%plot_logSimData_2d(logSimDat(1).at(:, xNum), 'dx', 'maxEigVal', yNum);

% ���͂������Ƃ���̂��̎��o��
simDat = logSimDat(zNum).at(yNum, xNum);

% simDat ���� logDat�̍쐬
simDat.odeParam.times_of_refine = 4;
logDat = calc_logDat(simDat);
% Investigate result 
plot_data(logDat, simDat.robotParam, 0);
% make a movie
%visualize_data(logDat, simDat.robotParam, 100, 0.5, 0, 'timed_dth045_dx040_with_dg090.avi');

% ��������l��ۑ�
robotParam = simDat.robotParam
x_p_ini = simDat.fixedPoint_p
iniStatConst = x_p_ini;
otherConst = [x_p_ini(5)-x_p_ini(6), x_p_ini(7)-x_p_ini(8), simDat.analDat.Fr, simDat.analDat.energy];
%save('141214_initialEst_gallop_dg045.mat', 'robotParam', 'x_p_ini', 'iniStatConst', 'otherConst');




