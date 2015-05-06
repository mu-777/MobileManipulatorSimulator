function state = interpState( timeline, stateTimeline, time )
%INTERPMULTI この関数の概要をここに記述
%   詳細説明をここに記述
 
for n = 1:size(stateTimeline,1)
    state(n) = interp1(timeline, stateTimeline(n,:), time);
end
end

