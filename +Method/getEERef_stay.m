function [eeState_ref, eeVel_ref] = getEERef_stay( eeState_prev )
%GETEEREF_STAY ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q

eeState_ref = eeState_prev;
eeVel_ref = eeState_ref - eeState_prev;

end

