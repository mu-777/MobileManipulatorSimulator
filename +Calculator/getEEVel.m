function eeVel = getEEVel( robotVel, J )
%GETEEVEL この関数の概要をここに記述
%   詳細説明をここに記述

eeVel = (J*robotVel')';

end

