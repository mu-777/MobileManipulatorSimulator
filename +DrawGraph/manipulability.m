function figObj = manipulability( figNum, is, t,  manipulability, legendName)
%DRAWGRAPH_MANIPULABILITY ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q

if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end

plot(t, manipulability)
xlabel('time')
ylabel('Manipulability')
legend(legendName)
grid on 

 
end

