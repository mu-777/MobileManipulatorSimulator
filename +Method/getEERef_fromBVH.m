function [ eeRefState, eeRefVel ] = getEERef_fromBVH( bvhTime, bvhTargetState, dt, currT,  eeRefState_prev)
%GETEEREF_FROMBVH この関数の概要をここに記述
%   詳細説明をここに記述

eeRefState = Utility.interpState(bvhTime, bvhTargetState', currT);
eeRefState(3) = bvhTargetState(1, 3); % z
eeRefState(4) = 0; % roll
eeRefState(5) = 0; % pitch
eeRefVel = (eeRefState - eeRefState_prev)/dt;

end

