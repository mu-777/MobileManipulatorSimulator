function figObj = drawBVH( figNum, is, bvhT, bvhData, targetBVHNodeName, viewTime )
%DRAWBVH ���̊֐��̊T�v�������ɋL�q
%   �ڍא����������ɋL�q

if isempty(figNum)
    figObj = figure('Position', is.windowPosition);
    figure(figObj);
else
    figObj = figure(figNum);
end

% ��̃��[�v�Ń����N��`�悷��̂�parent�̃f�[�^������̂ŁC��ɂ����Ńf�[�^�ϊ����Ă���
for id = 1:length(bvhData)
    bvhCurrentState(id, :) = Utility.interpState( bvhT, bvhData(id).Dxyz(:, :), viewTime);
end

for id = 1:length(bvhData)
    if strcmp(bvhData(id).name, targetBVHNodeName)
        plot3(bvhCurrentState(id, 1), bvhCurrentState(id, 3), bvhCurrentState(id, 2),'.','markersize',20, 'Color', 'g')
    else
        plot3(bvhCurrentState(id, 1), bvhCurrentState(id, 3), bvhCurrentState(id, 2),'.','markersize',20, 'Color', 'k')
    end
    
    parent = bvhData(id).parent;
    if parent > 0
        plot3([bvhCurrentState(parent, 1) bvhCurrentState(id, 1)],...
            [bvhCurrentState(parent, 3) bvhCurrentState(id, 3)],...
            [bvhCurrentState(parent, 2) bvhCurrentState(id, 2)], 'Color', 'k')
    end
end

end

