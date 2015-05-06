function is = getInitSettings( )
%getInitSettings ���̊֐��̊T�v�������ɋL�q
%   �V�~�����[�V�����ɂ���ĕς���p�����[�^�͂���

% ��{�I�ɕς��Ȃ��p�����[�^��initConfig�ɓ����
is = getInitConfig();

%% FOR CONTROL
is.Kpdgain = 1*diag([1,1,1,0.5,0.5,0.5]);
is.Kredundancy = 1*10e-7 * diag([1,1,1,1,1,1,1,1]);

% eeRefGenerationMethod�́C+Method�Œ�`
% �����t������������΁CCController��generateEERef�֐����Q��
eeRefGenerationMethodNum = 3;
eeRefGenerationMethodList = {
    'stay';
    'myPath';
    'bvh'};
is.eeRefGenerationMethod = eeRefGenerationMethodList{eeRefGenerationMethodNum};

% inputGenerationMethod�́C+Method�Œ�`
% �����t������������΁CCController��generateInput�֐����Q��
inputGenerationMethodNum = 3;
inputGenerationMethodList = {
    'V=0';                               % �璷���g��Ȃ��B0�̕��񂾃x�N�g��
    'V=sqrt(det(JJ^T))usingDJsqDq';      % det(J*J')�̔�����DJsqDq��͓I�Ɍv�Z���ďo�������(DJsqDq�ԈႦ�Ă邭����)
    'V=sqrt(det(JJ^T))usingDJDq';        % det(J*J')�̔�����DJDq��͓I�Ɍv�Z���ďo�������
    'V=det(Jarm)'};                      % det(Jarm)�̔�����DJarmDq��͓I�Ɍv�Z���ďo�������
is.inputGenerationMethod = inputGenerationMethodList{inputGenerationMethodNum};


%% FOR INITIARIZE
load('./+DATA/000_bvhData.mat', 'targetState')
robotInit.state(1) = 30*is.DEG2RAD;
robotInit.state(2) = acos( (targetState(1,3) - is.Heightb - is.d0 - is.d1)/is.d2 );
robotInit.state(3) = 90*is.DEG2RAD - robotInit.state(2);
robotInit.state(4) =  0*is.DEG2RAD;
robotInit.state(5) =-30*is.DEG2RAD;
robotInit.state(6) =  0*is.DEG2RAD;
robotInit.state(7:9) = [0, 0, 0];
eeState_temp = Calculator.getEEState(robotInit.state);
is.robotInit.state = [robotInit.state(1:6),...
    targetState(1,1) - eeState_temp(1),...
    targetState(1,2) - eeState_temp(2),...
    0];
is.robotInit.vel = [0, 0, 0, 0, 0, 0, 0, 0];

% ----
J = Calculator.getJ(is.robotInit.state);
is.eeInit.state = Calculator.getEEState(is.robotInit.state);
is.eeInit.vel = Calculator.getEEVel(is.robotInit.vel, J);

is.manipulabilityInit.total = sqrt(abs(det(J*J')));
is.manipulabilityInit.arm = sqrt(abs(det(J(1:is.N, 1:is.M-2)*J(1:is.N, 1:is.M-2)')));


%% FOR SIMULATOR
is.simStepWidth = 0.0001;    % [sec] �V�~�����[�V�����̌v�Z���ԕ�
is.ctrlStepWidth = 0.1;   % [sec]�@����1STEP�ɂ����鎞��
is.startTime = 0;
is.endTime = 10;

% ----
is.maxCtrlStep = (is.endTime - is.startTime)/is.ctrlStepWidth;
is.maxSimStep = is.ctrlStepWidth/is.simStepWidth;
is.maxSize = is.maxSimStep*is.maxCtrlStep;


end

