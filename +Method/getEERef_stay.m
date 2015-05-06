function [eeState_ref, eeVel_ref] = getEERef_stay( eeState_prev )
%GETEEREF_STAY この関数の概要をここに記述
%   詳細説明をここに記述

eeState_ref = eeState_prev;
eeVel_ref = eeState_ref - eeState_prev;

end

