function eeVel = getEEVel( robotVel, J )
%GETEEVEL ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q

eeVel = (J*robotVel')';

end

