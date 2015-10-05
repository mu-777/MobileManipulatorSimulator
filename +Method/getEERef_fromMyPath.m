function [ eeState_ref, eeVel_ref ] = getEERef_fromMyPath( dt, eeState_prev )
%GETEEREF_FROMMYPATH ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q
    
% [mm]
A = 500;
B = 0;

velocity = [
    A;%A*sin(0.5);
    0;%A*cos(0.5);
    0;
    0;
    0;
    0
];


eeVel_ref = velocity';
eeState_ref = eeState_prev + eeVel_ref*dt;

end

