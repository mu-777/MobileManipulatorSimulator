function state = interpState( timeline, stateTimeline, time )
%INTERPMULTI ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q
 
for n = 1:size(stateTimeline,1)
    state(n) = interp1(timeline, stateTimeline(n,:), time);
end
end

