function figObj = drawTimeLabel( figNum, is, T, targetStep)
%drawTimeLabel この関数の概要をここに記述
%   詳細説明をここに記述

if isempty(figNum)
    figObj = figure();
else
    figObj = figure(figNum);
end

s = strcat('time: ', num2str(T(targetStep)), '[sec]' );
uicontrol('style', 'text', 'string', s, 'position', [(is.windowPosition(3)-150-10), 1, 150, 30], 'FontSize', 12, 'HorizontalAlignment', 'right');
% todo: make this flexible
end

